# Probleemoverzicht — DShot FC met MPU6050

Overzicht van alle problemen die tijdens de ontwikkeling zijn opgetreden, met per
probleem het symptoom, de oorzaak en de oplossing.

---

## 1. Sensorfusie was geschreven maar niet aangesloten

**Symptoom:** `attitude.c` bevatte een compleet complementair filter, maar de drone
reageerde alleen op de accelerometer.

**Oorzaak:** `Attitude_Update()` werd nergens aangeroepen. De main-loop berekende
de pitch rechtstreeks uit de accelerometer. De gyro werd wel uitgelezen maar
verder niet gebruikt.

**Oplossing:** `Attitude_Update()` in de regellus aangeroepen en de gefuseerde
hoeken (`fused_roll` / `fused_pitch`) als meting gebruikt.

---

## 2. Vaste dt in plaats van gemeten lustijd

**Symptoom:** De gyro-integratie klopte niet.

**Oorzaak:** `dt` stond hard op 0,02 s (50 Hz), terwijl de lus veel sneller draaide.
De gyro integreerde daardoor met een factor 10 tot 20 ernaast.

**Oplossing:** Werkelijke lustijd meten met de DWT cycle counter van de Cortex-M7.

---

## 3. Gemeten dt bleef 0,000000

**Symptoom:** De nieuwe dt-meting gaf altijd exact nul.

**Oorzaak:** Op de Cortex-M7 moet de DWT eerst via het Lock Access Register
worden vrijgegeven. Zonder die schrijfactie blijft `CYCCNT` op nul staan.

**Oplossing:** In `DWT_Init()` de unlock-sleutel `0xC5ACCE55` naar adres
`0xE0001FB0` schrijven, vóór het inschakelen van de teller.

---

## 4. Geen gyro-bias kalibratie

**Symptoom:** Blijvende hoekafwijking en langzame drift.

**Oorzaak:** De MPU6050 heeft in rust altijd een kleine offset op de gyro.

**Oplossing:** `MPU6050_Calibrate_Gyro()` middelt 1000 metingen bij het opstarten
(plaat stil) en trekt die offset af bij elke uitlezing.

---

## 5. IMU niet waterpas op het frame

**Symptoom:** "Frame waterpas" gaf geen 0 graden.

**Oorzaak:** Het sensormoduletje zit niet perfect vlak gemonteerd.

**Oplossing:** Level-kalibratie bij het opstarten (frame waterpas): 500 samples
middelen en het resultaat als vaste offset aftrekken.

---

## 6. IMU-oriëntatie verkeerd (gedraaid én ondersteboven)

**Symptoom:** Kantelen naar voor/achter veranderde de *roll* in plaats van de
pitch. De level-offset gaf `roll = -172,61°`.

**Oorzaak:** Twee dingen tegelijk. De sensor stond 90 graden gedraaid ten opzichte
van het frame, én hij was ondersteboven gemonteerd — een roll van bijna -180°
betekent dat de Z-as omlaag wijst en de sensor -1 g meet in plaats van +1 g.

**Oplossing:** `imu_to_frame()` met twee vlaggen, `IMU_ROTATED_90` en
`IMU_UPSIDE_DOWN`, die de sensor-assen naar frame-assen omrekenen. Zowel de
regellus als de level-kalibratie gebruiken dezelfde mapping.

---

## 7. Maar één motor werd aangestuurd

**Symptoom:** Alleen M1 draaide.

**Oorzaak:** In `send_dshot()` waren de andere drie kanalen uitgecommentarieerd, en
alleen TIM2_CH1 had een DMA-stream in CubeMX.

**Oplossing:** In CubeMX TIM4 (CH1/CH2), TIM2_CH3 en drie extra DMA-streams
toegevoegd, met dezelfde timing als TIM2 (prescaler 0, periode 359 = DShot300).
`send_dshot()` stuurt nu vier frames tegelijk uit.

---

## 8. Mixer-tekens kwamen niet overeen met het frame

**Symptoom:** Bij kantelen kwamen de verkeerde motoren op.

**Oorzaak:** De mixer ging uit van een andere motorindeling dan de fysieke.

**Oplossing:** Motoren geïdentificeerd met een test waarbij telkens drie motoren
draaien en er één stilstaat. Daaruit bleek: M1 links-onder, verder met de klok mee.
De mixer werkt nu met expliciete tekentabellen (`pitch_sign`, `roll_sign`,
`yaw_sign`) die per motor instelbaar zijn.

---

## 9. Basis-throttle onder het minimum

**Symptoom:** De PID leek geen enkel effect te hebben.

**Oorzaak:** `THROTTLE_BASE` (400) stond lager dan `THROTTLE_MIN` (600). Alle vier
de motoren werden daardoor naar 600 geklemd en het onderlinge verschil verdween.

**Oplossing:** Basis tussen minimum en maximum gelegd, met ruimte naar beide kanten
voor de correcties.

---

## 10. Motoren werkten maar om de andere flash

**Symptoom:** Eén keer flashen: motoren doen niets. Nog eens flashen: ze werken.
Ook de eerste stap van de motortest deed nooit iets, de latere stappen wel.

**Oorzaak:** Twee onafhankelijke problemen.

*Ten eerste:* na een reset stonden de compare-registers (CCR) van de timers nog op
een oude waarde. De eerste PWM-periode na `Start_DMA` gebruikte die waarde,
waardoor de eerste puls van het eerste frame een verkeerde lengte had. De ESC doet
juist op die eerste frames zijn protocol-detectie (DShot300 herkennen). Mislukte
dat, dan negeerde hij daarna alles.

*Ten tweede:* de DMA-buffer was `uint32_t motor_dmabuf[4][17]` = 272 bytes. Dat is
geen veelvoud van 32, dus het einde viel midden in een cache-lijn en
`SCB_CleanDCache_by_Addr` liep over de buffer heen.

**Oplossing:** `dshot_init()` die de CCR-registers en tellers expliciet op nul zet
en de buffers vult met een geldig throttle-0 frame, vóór de eerste verzending. En
de buffer opgehoogd naar `[4][24]` = 384 bytes = exact 12 cache-lijnen.

---

## 11. Arming te kort

**Symptoom:** Het eerste gascommando na het opstarten werd genegeerd.

**Oorzaak:** De ESC accepteert pas gas na een langere periode throttle 0, en had
bovendien tijd nodig om zelf op te starten.

**Oplossing:** 3 s wachten plus 5 s throttle 0. Daarnaast begint de DShot-uitzending
nu meteen na de init, vóór de sensorkalibraties, zodat de ESC vanaf het begin een
ononderbroken stroom geldige frames ziet.

---

## 12. Soft-start stuurde DShot-commando's in plaats van gas

**Symptoom:** Motoren startten niet na het toevoegen van de soft-start.

**Oorzaak:** De ramp liep van 0 omhoog en raakte daarbij de waarden 1 t/m 47. Dat
zijn in DShot geen gaswaarden maar speciale commando's (piepen, draairichting,
instellingen opslaan).

**Oplossing:** De ramp begint nu bij 48, de laagste geldige gaswaarde. Daarnaast een
vangnet in `motors_send()` dat elke waarde tussen 1 en 47 naar 0 forceert.

---

## 13. Te weinig stuurautoriteit

**Symptoom:** Met propellers viel de drone gewoon om; de motoren corrigeerden te zwak.

**Oorzaak:** Basis-throttle te laag (stuwkracht loopt kwadratisch met toerental) en
de output-limiet van de PID te klein (±100 op een basis van 400).

**Oplossing:** Basis-throttle omhoog en de output-limiet verruimd.

---

## 14. Brownout-resets

**Symptoom:** De STM32 herstartte zodra de motoren opspinden. In de terminal kwam de
hele opstartsequentie steeds opnieuw voorbij, met verminkte tekens ertussen.

**Diagnose:** Een uitlezing van de reset-oorzaak via de RCC-vlaggen bevestigde
BROWNOUT (BOR). Omdat het ook optrad bij één motor zonder propellers — ongeveer
één ampère — kon het geen spanningsval door stroom zijn. Onderzocht en uitgesloten:
baanbreedte op de print (4 mm is ruim voldoende bij deze stromen), de accu (1550 mAh
100C), de UVLO-drempel op de EN-pin van de buck, en de ST-Link.

**Werkelijke oorzaak:** De spoel achter de buck converter was fysiek losgekomen. Het
contact was intermitterend, wat verklaart waarom het wisselvallig was en erger werd
bij hoger toerental (meer trilling).

**Oplossing:** Spoel opnieuw vastgesoldeerd.

---

## 15. Alle vier de motoren draaiden dezelfde kant op

**Symptoom:** Vier identieke propellers, alle vier blazend naar beneden.

**Oorzaak:** Een quadcopter heeft tegengesteld draaiende paren nodig. Zonder die
paren tellen de koppelreacties op en gaat het frame om zijn verticale as tollen.
Bovendien is yaw-besturing dan onmogelijk, want die werkt juist door het ene
diagonale paar sneller te laten lopen dan het andere.

**Oplossing:** Bij M2 en M4 twee van de drie motordraden verwisseld, en de
bijbehorende propellers gemonteerd. (De route via DShot-commando 20/21 werkte niet
op deze ESC's.)

---

## 16. PID oscilleerde bij grotere verstoringen

**Symptoom:** Klein duwtje kwam netjes terug, harde duw gaf flinke slingering.

**Oorzaak:** Alleen een P-term. Een zuivere P-regelaar op een hoek is van nature
slingerend; de demping moet van de D-term komen. Bij kleine verstoringen deed de
wrijving in de testopstelling nog het dempende werk.

**Oplossing:** D-term toegevoegd, afgeleid uit de **gemeten gyro-snelheid** in plaats
van uit het differentiëren van de berekende hoek. Dat is een directe meting en dus
veel minder ruisgevoelig. Daarna een laagdoorlaatfilter op de D-ingang tegen het
natrillen.

**Eindwaarden:** Kp 9, Ki 0,05, Kd 0,8.

---

## Veiligheidsvoorzieningen die zijn toegevoegd

- **Failsafe op hoek:** motoren definitief uit boven een instelbare hoek, wat een
  verkeerd mixer-teken opvangt (positieve terugkoppeling laat de regeling anders
  zichzelf opjagen).
- **Failsafe op tijd:** automatische stop na een ingestelde testduur.
- **Reset-oorzaak bij opstarten:** print of het een brownout, watchdog, pin- of
  softwarereset was — onmisbaar bij het opsporen van voedingsproblemen.
- **Bereikbewaking in `motors_send()`:** waarden 1–47 worden nooit verstuurd.
- **Controlestand `ANGLE_CHECK_ONLY`:** motoren blijven uit terwijl alleen de hoeken
  worden geprint, om de IMU-oriëntatie veilig te verifiëren.
- **Soft-start:** throttle wordt opgebouwd in plaats van in één sprong gezet.
