# Vluchtplan testen

Twee manieren. De eerste is de echte test, de tweede is handig om snel iets
uit te proberen zonder te flashen.

---

## 1. Op het bord (de echte test)

Dit is wat je wil zien: het bord schrijft het plan naar de SD-kaart, leest het
weer af, en print het over UART8.

**Stap 1 — zet de vlaggen bovenaan `Core/Src/main.c`:**

```c
#define WRITE_FLIGHTPLAN_TO_SD  1   // schrijven
#define DUMP_FLIGHTPLAN_FROM_SD 1   // uitlezen
#define DUMP_FLIGHTPLAN_RAW     1   // ook de ruwe bytes tonen
#define RUN_FLIGHTPLAN          0   // NIET vliegen, motoren blijven uit
```

Met `RUN_FLIGHTPLAN 0` wordt `FlightRun_Execute()` niet aangeroepen. De ESC's
worden dan nooit gearmd, dus dit is veilig met de accu erop. Zo staat het nu
in de repo.

**Stap 2 — terminal openen.** UART8 zit op **PE1 (TX)** en **PE0 (RX)**,
**115200 baud, 8N1**, geen flow control. Open PuTTY, TeraTerm of de Serial
Monitor van STM32CubeIDE op de juiste COM-poort.

> **Let op — RX/TX-swap.** De TX-kant is stuk, dus de swap moet aan staan.
> Vergeet je dat, dan blijft de terminal helemaal leeg terwijl de firmware
> gewoon draait. Dit was de oorzaak de eerste keer.
>
> De swap zit nu buiten de firmware (niet in `main.c` en niet in de `.ioc`).
> Wil je hem permanent maken, dan kan dat met de ingebouwde pin-swap van de
> STM32: CubeMX → UART8 → *Advanced Parameters* → *Swap TX/RX* = `Enable`.
> **Doe dat dan alleen als je de externe swap tegelijk uitzet** — twee keer
> swappen is hetzelfde als niet swappen, en dan is de lijn weer stil.
> Zet het in de `.ioc` en niet met de hand in `main.c`, anders gooit de
> volgende CubeMX-generatie het er weer uit.

**Stap 3 — SD-kaart erin, flashen, resetten.** Je zou dit moeten zien:

```
===== opgestart, vluchtplan-test =====

[1] SCHRIJVEN
flightplan_tool: plannen in de firmware:
  1: testvlucht 1   -> vluchtplan.txt   (6 commando's)
  2: minimaal       -> plan_min.txt     (2 commando's)
  3: kanteltest     -> plan_tilt.txt    (5 commando's)
flightplan_tool: 3 plannen wegschrijven...
- testvlucht 1 -> vluchtplan.txt
  vluchtplan.txt geschreven en teruggelezen (6 commando's)
- minimaal -> plan_min.txt
  plan_min.txt geschreven en teruggelezen (2 commando's)
- kanteltest -> plan_tilt.txt
  plan_tilt.txt geschreven en teruggelezen (5 commando's)
flightplan_tool: 3 van de 3 plannen staan op de kaart

[2] RUWE INHOUD VAN DE KAART
---- ruwe inhoud van vluchtplan.txt (170 bytes) ----
# vluchtplan - automatisch weggeschreven door flightplan_io.c
# 1 commando per lijn, regels met # zijn commentaar
# formaat: zie flightplan.h
#
Throttle 30 2000
Throttle 40 3000
Left 1000
Right 1000
Throttle 35 1500
Land
---------------------------------------

[3] UITLEZEN EN CONTROLEREN
---- vluchtplan.txt ----
  6 commando's, formaat: tekst
   1: Throttle 30 2000
   2: Throttle 40 3000
   3: Left 1000
   4: Right 1000
   5: Throttle 35 1500
   6: Land
  geplande duur: 8500 ms
------------------------------
flightplan_tool: plan is geldig

RUN_FLIGHTPLAN staat op 0: motoren blijven uit, er wordt niet gevlogen
```

**Als het misloopt:**

| Wat je ziet | Wat er aan de hand is |
|---|---|
| helemaal niets | **eerst de RX/TX-swap checken** (zie hierboven), daarna baudrate en COM-poort. Zet `UART_CHECK` in `main.c` op 1: die stuurt 10× `UART8 werkt` rechtstreeks over UART8, zonder printf. Komt dat ook niet door, dan ligt het aan de lijn en niet aan de code |
| `kon de SD-kaart niet mounten` | kaart zit er niet in, of niet FAT32 geformatteerd |
| `%f` of rare tekens in plaats van getallen | `-u _printf_float` staat niet bij de linker-flags |
| `... inlezen mislukt: SD-kaartfout` | het bestand staat er niet, zet `WRITE_FLIGHTPLAN_TO_SD` op 1 |

**Stap 4 — pas als de dump klopt:** zet `RUN_FLIGHTPLAN` op 1 en test dan nog
altijd eerst **zonder propellers**.

Een eigen plan maken: pas de `FlightCmd_t`-arrays bovenaan
`Core/Src/flightplan_tool.c` aan en flash opnieuw.

---

## 2. Op de PC (snel, zonder hardware)

Dezelfde `sdcard.c`, `flightplan.c` en `flightplan_io.c` worden hier
gecompileerd, alleen praat FATFS met een map `./sdtest/` in plaats van met een
echte kaart (zie `stub/`). Na afloop kan je in `sdtest/vlucht.txt` kijken.

Je hebt een gewone gcc nodig — MSYS2/MinGW, WSL of Linux:

```sh
cd test
make run
```

Dat schrijft een plan weg, leest het terug, print het, en draait daarna de
controles: tekst- en binaire roundtrip, CRC op een beschadigd bestand, en of
foute plannen (throttle boven 100 %, duur van 0 ms, commando na `Land`) netjes
afgekeurd worden.

Wil je het binaire formaat testen in plaats van tekst:

```sh
make clean
make run CFLAGS="-std=gnu11 -Wall -Istub -I../Core/Inc -DFPIO_FORMAT=1"
```

Opruimen met `make clean`. STM32CubeIDE compileert deze map niet mee, dus je
firmware-build verandert hier niet door.
