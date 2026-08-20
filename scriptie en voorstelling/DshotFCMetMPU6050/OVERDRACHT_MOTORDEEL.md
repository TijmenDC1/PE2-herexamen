# Overdracht motordeel — resources en integratienotities

Bedoeld om het motor-/regelgedeelte van dit project samen te voegen met het
SX1280-project. Bevat de volledige resourcelijst en de punten die bij integratie
misgaan als je ze niet weet.

Board: STM32F7, systeemklok 216 MHz, APB1-timerklok 108 MHz.

---

## 1. Timers

| Timer | Kanaal | Pin | Functie | Instelling |
|---|---|---|---|---|
| TIM2 | CH1 | PA0 | DShot M1 | PSC 0, ARR 359 |
| TIM2 | CH3 | PA2 | DShot M2 | idem (zelfde timerbasis) |
| TIM4 | CH2 | PD13 | DShot M3 | PSC 0, ARR 359 |
| TIM4 | CH1 | PD12 | DShot M4 | idem |

Beide timers draaien op 108 MHz met periode 359, wat 108 MHz / 360 = 300 kHz
oplevert — dat is precies de bitrate van **DShot300**. Pulsbreedtes: `DS_0 = 135`
(37,5 %) en `DS_1 = 270` (75 %).

> Verander je de systeem- of APB-klok bij het samenvoegen, dan verschuift de
> DShot-timing mee en herkent de ESC het protocol niet meer. Controleer na een
> klokwijziging dat de timerklok gedeeld door (ARR+1) nog steeds 300 kHz is.

---

## 2. DMA-streams

| Stream | Request | Motor | Instellingen |
|---|---|---|---|
| DMA1_Stream5 | TIM2_CH1 | M1 | Mem→Periph, Normal, MemInc aan, Word/Word, Very High |
| DMA1_Stream1 | TIM2_UP/CH3 | M2 | idem |
| DMA1_Stream0 | TIM4_CH1 | M4 | idem |
| DMA1_Stream3 | TIM4_CH2 | M3 | idem |

Alle vier op **DMA_PRIORITY_VERY_HIGH**, FIFO uit, mode Normal (niet circular).

> Deze vier streams zijn bezet. Gebruikt het SX1280-project DMA voor SPI2 of voor
> een UART, controleer dan dat het geen van deze streams claimt. Op de STM32F7
> ligt per stream vast welke requests erop kunnen, dus een botsing is niet altijd
> op te lossen door simpelweg een andere stream te kiezen.

---

## 3. Pinnen die het motordeel gebruikt

| Pin | Functie |
|---|---|
| PA0 | DShot M1 (TIM2_CH1) |
| PA2 | DShot M2 (TIM2_CH3) |
| PD13 | DShot M3 (TIM4_CH2) |
| PD12 | DShot M4 (TIM4_CH1) |
| PB8 / PB9 | I2C1 SCL/SDA — MPU6050 (gedeeld met BMM350) |
| PE0 / PE1 | UART8 — debug-terminal, 115200 baud |
| PA1, PA3, PD11, PD10 | Overcurrent-ingangen M1..M4 (nog niet gebruikt in code) |

Ter info, al in dezelfde `.ioc` aanwezig en dus geen conflict: SX1280 op SPI2
(PB12 CS, PB13 SCK, PB14 MISO, PB15 MOSI, PD8 nRST, PD9 Busy), SD-kaart op
SDMMC1 (PC8–PC12, PD2), BMI330 op SPI1, BMP384 op SPI4, GPS op UART7.

---

## 4. Interruptprioriteiten

Prioriteitsgroep: **NVIC_PRIORITYGROUP_4** (alle 4 bits preemption).

| Interrupt | Prioriteit |
|---|---|
| DMA1_Stream0 / 1 / 3 / 5 | 0 |
| TIM2, TIM4 | 0 |
| EXTI15_10 | 0 |
| EXTI4, EXTI9_5 | 3 |
| **SysTick** | **15 (laagste)** |

> **Belangrijk aandachtspunt bij samenvoegen.** SysTick staat op de laagste
> prioriteit, terwijl `HAL_Delay()` en alle HAL-timeouts daarvan afhangen. Voegt
> het SX1280-project interrupts toe met een hoge prioriteit die lang duren, dan
> kan SysTick verhongeren en blijft `HAL_Delay()` hangen. Geef radio-interrupts
> een prioriteit **numeriek hoger dan 0** (dus lagere urgentie) of verlaag de
> SysTick-prioriteit naar bijvoorbeeld 5.

---

## 5. Lussnelheid — dit is het grootste integratieprobleem

De regellus draait op ongeveer **100 Hz (circa 10 ms per iteratie)**, en dat wordt
bijna volledig bepaald door de debug-printf.

Opbouw per iteratie:

| Onderdeel | Tijd | Blokkerend? |
|---|---|---|
| `printf` van de statusregel (~100 tekens @ 115200 baud) | **circa 8,7 ms** | **ja, volledig** |
| `HAL_Delay(1)` aan het eind | 1–2 ms | ja |
| 2× I2C-lezing van de MPU6050 (elk 6 bytes) | circa 0,4–1,6 ms | ja |
| Wachten op de DShot-DMA (`while (dshot_dma_complete == 0)`) | circa 57 µs | ja, maar kort |
| Sensorfusie + 2× PID + mixer | verwaarloosbaar | nee |

De `_write()` in `main.c` gebruikt `HAL_UART_Transmit(..., HAL_MAX_DELAY)` en
verstuurt **teken voor teken**. Dat is volledig blokkerend.

> **Voor de samenvoeging: haal die printf uit de lus.** Zonder printf en zonder
> `HAL_Delay(1)` wordt de lus door de I2C-lezing bepaald en haal je makkelijk
> 500 Hz tot 1 kHz. Een vluchtcontroller hoort typisch op 1 kHz of hoger te
> draaien; 100 Hz is echt aan de lage kant en is waarschijnlijk medeoorzaak van
> het stabiliteitsprobleem hieronder.

**SD-schrijfacties: die zijn er niet.** De SDMMC1-pinnen zijn wel geconfigureerd,
maar er staat geen SD- of FATFS-code in `Core/Src`. In de lus wordt niets
weggeschreven.

---

## 6. Wat de andere kant moet weten over het motordeel

### Bestanden die mee moeten

`dshot.c/.h`, `MPU6050.c/.h`, `attitude.c/.h`, `AnglePID.c/.h`, `fc_config.h`, en
uit `main.c` de functies `motors_send()`, `Update_Motors()`, `imu_to_frame()`,
`DWT_Init()` plus de tekentabellen.

### Volgorde van initialisatie — hier zit een valkuil

```
1. DWT_Init()          microsecondentimer vrijgeven (LAR-unlock nodig op M7!)
2. dshot_init()        CCR's en tellers op nul, buffers vullen
3. MPU6050_Init()
4. 2 s throttle-0 frames sturen        <-- ESC doet hier protocol-detectie
5. gyro kalibreren (stil)              <-- pas NA stap 4
6. level kalibreren (waterpas)
7. armen: 3 s wachten + 5 s throttle 0
8. soft-start vanaf 48 naar basis
```

Stap 4 moet vóór de kalibraties, zodat de ESC een ononderbroken stroom geldige
frames ziet. Stap 1 vereist de DWT-unlock (`0xC5ACCE55` naar `0xE0001FB0`), anders
blijft `CYCCNT` nul en is elke `dt` nul.

### Vijf dingen die stuk gaan als je ze niet weet

1. **DShot-waarden 1 t/m 47 zijn commando's, geen gas.** Nooit als throttle
   versturen. `motors_send()` heeft daarvoor een vangnet.
2. **De DMA-buffer moet op cache-lijnen uitgelijnd zijn.** `motor_dmabuf[4][24]`
   = 384 byte = 12 lijnen van 32. Bij een niet-veelvoud loopt
   `SCB_CleanDCache_by_Addr` over de buffer heen. D-cache staat aan op de F7.
3. **De compare-registers moeten op nul vóór het eerste frame.** Anders is de
   eerste puls willekeurig lang, mislukt de protocol-detectie van de ESC en
   werkt het "om de andere flash".
4. **Motorvolgorde is niet vanzelfsprekend.** Buffer-index → fysieke positie:
   0 = M1 achter-links, 1 = M2 voor-links, 2 = M3 voor-rechts, 3 = M4 achter-rechts.
5. **M2 en M4 draaien omgekeerd**, gerealiseerd door twee van de drie motordraden
   te verwisselen (DShot-commando 20/21 werkte niet op deze ESC's). Bij vervanging
   van een ESC of motor moet dat opnieuw.

### Regellus in pseudocode

```c
lees accel + gyro (I2C)
dt = (DWT->CYCCNT - vorige) / SystemCoreClock
imu_to_frame(...)                      // 90 graden gedraaid + ondersteboven
Attitude_Update(...)                   // complementair filter, ALPHA 0.98
roll_level  = fused_roll  - roll_offset
pitch_level = fused_pitch - pitch_offset
failsafe-controle (hoek + tijd)
roll_cmd  = PID(0, roll_level,  gyro_roll,  dt)
pitch_cmd = PID(0, pitch_level, gyro_pitch, dt)
Update_Motors(THROTTLE_BASE, roll_cmd, pitch_cmd, 0)
```

De setpoints zijn nu hard 0 graden. **Dat is het aanknopingspunt voor de radio:**
vervang die nullen door de stickwaarden, en `THROTTLE_BASE` door de gasstick.

### Failsafe die behouden moet blijven

Motoren gaan definitief uit boven `FAILSAFE_ANGLE` en na `FAILSAFE_TIMEOUT`, en
blijven uit tot een herstart. De hoekbewaking vangt een verkeerd mixer-teken op —
dan wordt de terugkoppeling positief en jaagt de regeling zichzelf op. Bij een
radio hoort daar een derde afslag bij: **verlies van het radiosignaal**.

---

## 7. Openstaand probleem: instabiliteit met beide assen actief

Bij het gelijktijdig activeren van roll en pitch liep het weg — de beweging werd
steeds sneller in plaats van uit te dempen. Met één as tegelijk werkte het wel
(Kp 9, Ki 0,05, Kd 0,8).

Waarschijnlijke oorzaken, in volgorde van verdenking:

**Lussnelheid van slechts 100 Hz.** Een regellus met veel fasevertraging wordt
instabiel zodra je de versterking opvoert, en juist de D-term is daar gevoelig
voor. Bij 10 ms per iteratie werkt een D-term eerder vertragend dan dempend.
Dit is meteen de reden om die printf eruit te halen.

**Het D-filter doet bij 100 Hz vrijwel niets.** Met een kantelfrequentie van 60 Hz
en een stap van 10 ms wordt de filterfactor ongeveer 0,79 — dat is nauwelijks
filtering. De D-term ziet dus bijna ongefilterde ruis.

**Asymmetrische begrenzing verhoogt de gemiddelde stuwkracht.** Met basis 600 en
een limiet van ±500 wordt de onderkant afgekapt op `THROTTLE_MIN` 200 (in plaats
van 100), terwijl de bovenkant tot 1100 ongehinderd doorloopt. Bij grote
correcties gaat de totale stuwkracht daardoor omhoog, wat op een testopstelling
de beweging kan aanjagen.

**Het complementaire filter is een kleine-hoekbenadering.** De gyrosnelheden
worden rechtstreeks als Euler-hoeksnelheden geïntegreerd. Dat klopt rond
waterpas, maar bij grotere uitslagen — en zeker als beide assen tegelijk
bewegen — lopen de assen niet meer samen en gaat de berekende hoek afwijken.

Voorgestelde aanpak: eerst de printf uit de lus en de lussnelheid omhoog, dan
opnieuw meten. Helpt dat, verlaag dan `PID_D_CUTOFF_HZ` naar iets dat past bij de
nieuwe lussnelheid, en verklein `PID_OUTPUT_LIMIT` tot ongeveer 350 zodat beide
assen samen binnen het throttle-bereik blijven.
