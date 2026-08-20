# Architectuur — DShot vluchtcontroller met MPU6050

Werking en opbouw van de vluchtcontroller: een STM32F7 die met een MPU6050 zijn
kantelhoek bepaalt en vier ESC's aanstuurt via het DShot300-protocol.

---

## Bestandsoverzicht

| Bestand | Verantwoordelijkheid |
|---|---|
| `Core/Inc/fc_config.h` | **Alle instelbare waarden.** Throttle-grenzen, PID-gains, IMU-oriëntatie, failsafe, teststanden. Begin hier als je iets wilt aanpassen. |
| `Core/Src/MPU6050.c` | Uitlezen van de sensor over I2C, omrekenen naar °/s en g, en gyro-bias kalibratie. |
| `Core/Src/attitude.c` | Complementair filter: combineert gyro en accelerometer tot een kantelhoek. |
| `Core/Src/AnglePID.c` | PID-regelaar voor hoekregeling, met anti-windup en een filter op de D-term. |
| `Core/Src/dshot.c` | DShot300-frames opbouwen (throttle, telemetriebit, CRC) en via timer + DMA uitsturen naar vier kanalen. |
| `Core/Src/main.c` | Opstartvolgorde, de regellus, de mixer en de testroutines. |

---

## Signaalketen

```
MPU6050          imu_to_frame      Attitude_Update       PID_Angle_UpdateRate
(ruwe gyro  ──>  (assen naar  ──>  (complementair   ──>  (setpoint 0 graden
 en accel)        frame-assen)      filter -> hoek)       vs gemeten hoek)
                                                                 │
                                                                 v
DShot300  <──  send_dshot  <──  Update_Motors  <──  correctie per as
(4 kanalen)   (timer+DMA)      (mixer met tekentabellen)
```

### 1. Uitlezen

`MPU6050_Read_Gyro()` en `MPU6050_Read_Accel()` halen zes bytes per sensor op via
I2C en rekenen die om naar graden per seconde en g. De gyro-offset die bij het
opstarten is gemeten, wordt er meteen afgetrokken.

### 2. Assen omrekenen

`imu_to_frame()` zet sensor-assen om naar frame-assen. Het moduletje zit namelijk
90 graden gedraaid én ondersteboven gemonteerd. Zonder deze stap komt een
pitch-beweging binnen als roll, en wijst de Z-as de verkeerde kant op.

Gestuurd door `IMU_ROTATED_90` en `IMU_UPSIDE_DOWN` in `fc_config.h`.

### 3. Sensorfusie

De accelerometer weet welke kant de zwaartekracht op wijst en heeft dus geen
drift, maar hij is gevoelig voor trillingen en versnellingen. De gyro is snel en
rustig, maar zijn hoek loopt langzaam weg omdat je zijn snelheid integreert.

Het complementaire filter combineert beide:

```
hoek = ALPHA * (hoek + gyro * dt) + (1 - ALPHA) * accel_hoek
```

Met `ALPHA` = 0,98 volgt de hoek dus vooral de gyro, maar wordt hij continu
bijgestuurd naar de accelerometer-referentie. Zo krijg je de snelheid van de
gyro zonder de drift.

**`dt` wordt echt gemeten**, met de DWT cycle counter van de Cortex-M7, en niet
aangenomen. Een vaste `dt` die niet klopt met de werkelijke lustijd laat de
gyro-integratie er faktoren naast zitten.

### 4. Regeling

Per as draait een PID met **setpoint 0 graden** (waterpas):

```
fout = 0 - gemeten_hoek
output = Kp * fout + Ki * integraal(fout) + Kd * (-draaisnelheid)
```

Twee bijzonderheden:

De **D-term komt uit de gyro**, niet uit het differentiëren van de berekende hoek.
De gyro meet de draaisnelheid rechtstreeks, terwijl differentiëren ruis flink
versterkt. Omdat de setpoint constant is, geldt `d(fout)/dt = -draaisnelheid`.

Op die D-ingang zit een **eerste-orde laagdoorlaatfilter** (`PID_D_CUTOFF_HZ`),
want de D-term versterkt hoogfrequente ruis en dat hoor je als trillen in de
motoren.

### 5. Mixer

`Update_Motors()` verdeelt de basis-throttle plus de correcties over vier motoren
via tekentabellen:

```
        VOOR
    M2 -------- M3          LINKS  = M1, M2      VOOR   = M2, M3
     |          |           RECHTS = M3, M4      ACHTER = M1, M4
    M1 -------- M4
        ACHTER

index 0 = M1 (TIM2_CH1, PA0)   achter-links
index 1 = M2 (TIM2_CH3, PA2)   voor-links
index 2 = M3 (TIM4_CH2, PD13)  voor-rechts
index 3 = M4 (TIM4_CH1, PD12)  achter-rechts
```

| Tabel | Waarde | Betekenis |
|---|---|---|
| `pitch_sign` | `{+1,-1,-1,+1}` | achterpaar omhoog, voorpaar omlaag |
| `roll_sign` | `{+1,+1,-1,-1}` | linkerpaar omhoog, rechterpaar omlaag |
| `yaw_sign` | `{0,0,0,0}` | nog niet in gebruik; wordt `{+1,-1,+1,-1}` |

**Regel om de tekens te controleren:** de kant die naar beneden kantelt moet
harder gaan draaien. Klopt dat niet, keer dan alle vier de tekens van die as om —
nooit één of twee, want dan wordt de mixer scheef.

Elke motorwaarde wordt begrensd tussen `THROTTLE_MIN` en `THROTTLE_MAX`.

### 6. DShot300 uitsturen

Een DShot-frame is 16 bits: 11 bits throttle, 1 telemetriebit, 4 bits CRC. De
bitwaarde zit in de púlsbreedte — een 1 is twee keer zo lang hoog als een 0. De
timer draait op 108 MHz met periode 359, wat 300 kbit/s geeft.

Elk kanaal heeft zijn eigen timerkanaal en DMA-stream, zodat alle vier de frames
tegelijk de deur uit gaan.

Aandachtspunten die in de praktijk problemen gaven:

- **`dshot_init()` zet de compare-registers en tellers op nul** vóór het eerste
  frame. Anders heeft de eerste puls een willekeurige lengte en mislukt de
  protocol-detectie van de ESC.
- **De DMA-buffer is opgehoogd naar `[4][24]`** = 384 bytes = 12 cache-lijnen van
  32 byte, zodat `SCB_CleanDCache_by_Addr` precies op lijngrenzen valt.
- **Waarden 1 tot 47 worden nooit als gas verstuurd.** Dat zijn DShot-commando's.

---

## Opstartvolgorde

1. Reset-oorzaak printen (brownout, watchdog, pin- of softwarereset)
2. `DWT_Init()` — microsecondentimer vrijgeven en starten
3. `dshot_init()` — timers op nul, buffers vullen met een geldig throttle-0 frame
4. `MPU6050_Init()` — sensor wekken, bereiken en filter instellen
5. 2 s throttle-0 frames sturen, zodat de ESC schoon kan protocol-detecteren
6. **Gyro kalibreren** — plaat moet stil liggen
7. **Level kalibreren** — frame moet waterpas liggen
8. Armen: 3 s wachten, dan 5 s throttle 0
9. Soft-start: throttle opbouwen van 48 naar `THROTTLE_BASE`
10. Regellus

De twee kalibraties bepalen je nulpunt. Ligt de drone er scheef bij tijdens het
opstarten, dan bak je die scheefstand in als "waterpas".

---

## Testgereedschap

| Functie | Doel |
|---|---|
| `test_motors_identify(spin)` | Draait telkens 3 motoren en laat er 1 stilstaan. De stilstaande is de aangekondigde — zo koppel je M1..M4 aan fysieke posities. |
| `test_motors_sequence(spin)` | Draait elke motor 1,5 s achter elkaar. |
| `test_motor_pair(a,b,spin,label)` | Draait twee gekozen motoren, om voor/achter- of links/rechts-paren te bevestigen. |
| `test_motor_ramp(idx,van,tot,stappen)` | Bouwt één motor langzaam op en print elke stap. Vindt de throttle waarbij iets misgaat. |
| `dshot_beep_test()` | Laat elke ESC piepen. Hoor je het, dan komen DShot-commando's aan. |
| `ANGLE_CHECK_ONLY` | Motoren blijven uit, alleen hoeken worden geprint. Veilig de IMU-oriëntatie controleren. |

---

## Veiligheid

- **Hoek-failsafe** (`FAILSAFE_ANGLE`) — motoren definitief uit boven een grens.
  Vangt vooral een verkeerd mixer-teken op: dan wordt de terugkoppeling positief
  en jaagt de regeling zichzelf op in plaats van terug te regelen.
- **Tijd-failsafe** (`FAILSAFE_TIMEOUT`) — automatische stop na een testduur.
- **Bereikbewaking** in `motors_send()` — commando-waarden worden nooit als gas
  verstuurd.
- **Soft-start** — throttle wordt opgebouwd in plaats van in één sprong gezet.

Eenmaal afgeslagen blijven de motoren uit tot een herstart. Dat is bewust: je
wilt niet dat hij vanzelf weer aanslaat.

---

## Instellingen aanpassen

Alles staat in `Core/Inc/fc_config.h`. De belangrijkste:

| Instelling | Huidig | Opmerking |
|---|---|---|
| `THROTTLE_BASE` | 600 | Moet tussen MIN en MAX liggen, met ruimte naar beide kanten |
| `PID_KP` | 9,0 | |
| `PID_KI` | 0,05 | |
| `PID_KD` | 0,8 | Uit de gyro, met filter op `PID_D_CUTOFF_HZ` |
| `PID_OUTPUT_LIMIT` | 500 | Maximale correctie per motor |
| `FAILSAFE_ANGLE` | 80° | Zet op ~55 bij tunen met propellers |

### Tuningvolgorde die gewerkt heeft

1. Eén as tegelijk, de andere op nul zetten in de tekentabel
2. Alleen `Kp` verhogen tot hij net begint te slingeren, dan terug naar 60–70%
3. `Kd` opbouwen tot hij in één of twee bewegingen tot rust komt
4. `Kp` eventueel weer wat omhoog — demping laat meer versterking toe
5. `Ki` alleen als er een blijvende scheefstand overblijft

Verstoor steeds met dezelfde beweging (ongeveer 10 tot 15 graden en dan loslaten),
anders vergelijk je verschillende dingen met elkaar.

---

## Wat er nog niet is

- **Yaw-regeling.** Zonder magnetometer is de yaw-hoek niet meetbaar; dit wordt
  een rate-loop op `gyro_z` met de stick als snelheids-setpoint. `yaw_sign` moet
  dan `{+1,-1,+1,-1}` worden.
- **Radio-ontvanger (SX1280).** Throttle staat nu vast en de setpoints zijn altijd
  0 graden. Zonder ontvanger is er geen gasregeling en geen noodstop.
- **Accubewaking.** De spanning wordt niet gemeten. Beter is dat in software doen
  en waarschuwen, in plaats van de voeding van de vluchtcontroller hard afkappen.
- **Hoogtemeting.** De BMP384-code staat er wel, maar is niet actief.
