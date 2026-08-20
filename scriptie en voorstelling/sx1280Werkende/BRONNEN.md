# Bronnen bij de SX1280 / ExpressLRS-ontvanger

Waar elke niet-vanzelfsprekende constante, formule en bewering uit het commentaar
vandaan komt. Alle ExpressLRS-links wijzen naar **tag 4.1.0**, dezelfde versie die
in `ELRS_OTA_VERSION 4` staat. Wil je een ander punt in de geschiedenis zien,
vervang dan `refs/tags/4.1.0` door `master` of een andere tag.

Onderaan staan drie punten waar het commentaar **niet** klopt met de bron. Die zijn
het belangrijkst om te lezen.

---

## 1. De twee seeds uit de UID

### CRC-seed

```c
#define ELRS_CRC_INIT ((uint16_t)((((uint16_t)ELRS_UID4 << 8) | ELRS_UID5) \
                                   ^ ((uint16_t)ELRS_OTA_VERSION << 8)))
```

Bron: `OtaUpdateCrcInitFromUid()` in
[`src/lib/OTA/OTA.cpp`](https://github.com/ExpressLRS/ExpressLRS/blob/4.1.0/src/lib/OTA/OTA.cpp).
Letterlijk in de broncode:

```cpp
OtaCrcInitializer  = (UID[4] << 8) | UID[5];
OtaCrcInitializer ^= (uint16_t)OTA_VERSION_ID << 8;
```

### FHSS-seed

```c
#define ELRS_FHSS_SEED (((uint32_t)ELRS_UID2 << 24) | ((uint32_t)ELRS_UID3 << 16) \
                       | ((uint32_t)ELRS_UID4 << 8) | (ELRS_UID5 ^ ELRS_OTA_VERSION))
```

Bron: `OtaGetUidSeed()` in hetzelfde bestand:

```cpp
uint32_t OtaGetUidSeed()
{
    return ((uint32_t)UID[2] << 24) + ((uint32_t)UID[3] << 16) +
           ((uint32_t)UID[4] << 8) + (UID[5]^OTA_VERSION_ID);
}
```

Let op wat hieruit volgt: **UID0 en UID1 komen in geen van beide seeds voor.** Ze
staan alleen ter documentatie in de header. UID2 en UID3 zitten wél in de
FHSS-seed maar niet in de CRC — precies daarom kan een pakket keurig valideren
terwijl je hoptabel volledig verkeerd is.

### OTA_VERSION_ID

Bron: [`src/include/targets.h`](https://github.com/ExpressLRS/ExpressLRS/blob/4.1.0/src/include/targets.h)

```cpp
// Used to XOR with OtaCrcInitializer and macSeed to reduce compatibility with
// previous versions. It should be incremented when the OTA packet structure is modified.
#define OTA_VERSION_ID      4
```

### Waar de UID zelf vandaan komt

De 6 bytes zijn de **eerste 6 bytes van de MD5-hash van je binding phrase**.
Bron: `process_build_flag()` in
[`src/python/build_flags.py`](https://github.com/ExpressLRS/ExpressLRS/blob/4.1.0/src/python/build_flags.py):

```python
bindingPhraseHash = hashlib.md5(define.encode()).digest()
UIDbytes = ",".join(list(map(str, bindingPhraseHash))[0:6])
```

En in [`src/lib/OPTIONS/options.h`](https://github.com/ExpressLRS/ExpressLRS/blob/4.1.0/src/lib/OPTIONS/options.h):
`uint8_t uid[6];  // MY_UID derived from MY_BINDING_PHRASE`

Wil je je eigen UID narekenen: hash de string `-DMY_BINDING_PHRASE="jouwzin"`
precies zoals PlatformIO hem doorgeeft en neem de eerste 6 bytes.

---

## 2. De CRC14

### Polynoom 0x2E57 en de pakketstructuur

Bron: [`src/lib/OTA/OTA.h`](https://github.com/ExpressLRS/ExpressLRS/blob/4.1.0/src/lib/OTA/OTA.h)

- `ELRS_CRC14_POLY 0x2E57` (14 bits, Koopman-notatie)
- `OTA4_PACKET_SIZE` = 8 bytes, `OTA8_PACKET_SIZE` = 13 bytes
- Pakkettypes: `PACKET_TYPE_RCDATA 0b00`, `PACKET_TYPE_DATA 0b01`,
  `PACKET_TYPE_SYNC 0b10` — **0b11 bestaat niet**
- `OTA_Packet4_s`: `type` (2 bits) + `crcHigh` (6 bits) in byte 0, `crcLow` in byte 7
- `OTA_Sync_s` draagt `fhssIndex`, `nonce`, `rfRateEnum`, **`UID4` en `UID5`** mee —
  dat is waarom de UID-controle op `payload[5]`/`payload[6]` werkt

### De tabelopbouw

Bron: `Crc2Byte::init()` en `::calc()` in
[`src/lib/CRC/crc.cpp`](https://github.com/ExpressLRS/ExpressLRS/blob/4.1.0/src/lib/CRC/crc.cpp).
Daar staat `crc = i << (bits - 8)`, `highbit = 1 << (_bits - 1)` en
`_bitmask = (1 << _bits) - 1`. Voor 14 bits wordt dat `i << 6`, `1 << 13` en
`0x3FFF` — exact wat `ELRS_InitCRC()` doet. `calc()` gebruikt
`crc = (crc << 8) ^ _crctab[((crc >> (_bits - 8)) ^ byte) & 0xFF]`, dus `crc >> 6`.

### De nonce in de seed

`nonceValidator = (type == PACKET_TYPE_SYNC) ? 0 : OtaNonce` komt uit
`ValidatePacketCrcStd()` / `GeneratePacketCrcStd()` in `OTA.cpp` (link hierboven).
Dat is de basis voor `ELRS_SEED_VOOR()`.

Achtergrond over waarom de nonce erin zit:
[PR #3294 "Add OtaNonce to OtaCrcInitializer"](https://github.com/ExpressLRS/ExpressLRS/pull/3294).

---

## 3. FHSS: tabel, band en hop-interval

### De PRNG

Bron: [`src/lib/FHSS/random.cpp`](https://github.com/ExpressLRS/ExpressLRS/blob/4.1.0/src/lib/FHSS/random.cpp).
Lineaire congruentiegenerator met `m = 2147483648`, `a = 214013`, `c = 2531011`,
`rng()` geeft `seed >> 16` (0..0x7FFF), `rngN(max)` geeft `rng() % max`. Exact wat
in `main.c` staat.

### De tabelopbouw en de shuffle

Bron: `FHSSrandomiseFHSSsequenceBuild()` in
[`src/lib/FHSS/FHSS.cpp`](https://github.com/ExpressLRS/ExpressLRS/blob/4.1.0/src/lib/FHSS/FHSS.cpp):

```cpp
if (i % freqCount == 0)              { inSequence[i] = syncChannel; }
else if (i % freqCount == syncChannel) { inSequence[i] = 0; }
else                                   { inSequence[i] = i % freqCount; }
...
if (i % freqCount != 0) {
    uint8_t offset = (i / freqCount) * freqCount;
    uint8_t rand   = rngN(freqCount - 1) + 1;
    // swap inSequence[i] <-> inSequence[offset+rand]
}
```

Regel voor regel hetzelfde als `FHSS_GenerateSequence()`.

### 240 in plaats van 256

`FHSS_SEQUENCE_LEN` is 256, gedefinieerd in
[`src/lib/FHSS/FHSS.h`](https://github.com/ExpressLRS/ExpressLRS/blob/4.1.0/src/lib/FHSS/FHSS.h).
Maar `FHSSrandomiseFHSSsequence()` in `FHSS.cpp` rekent:

```cpp
primaryBandCount = (FHSS_SEQUENCE_LEN / FHSSconfig->freq_count) * FHSSconfig->freq_count;
```

`(256 / 80) * 80 = 240`. Vandaar `ELRS_SEQUENCE_COUNT 240`, en vandaar ook dat de
shuffle-lus tot index 319 kan schrijven terwijl `FHSSsequence` maar 256 lang is —
reden voor `ELRS_SEQUENCE_BUFFER 320`.

### De band

Domein `ISM2G4` in `FHSS.cpp`:

```cpp
{"ISM2G4", FREQ_HZ_TO_REG_VAL(2400400000), FREQ_HZ_TO_REG_VAL(2479400000), 80, 2440000000}
```

Startfrequentie 2400,4 MHz, 80 kanalen, en `sync_channel = freq_count / 2` = 40.
Dekt `ELRS_FREQ_START`, `ELRS_FREQ_COUNT` en `ELRS_SYNC_CHANNEL`.

---

## 4. De rate-tabel

Bron: `ExpressLRS_AirRateConfig[]` in
[`src/src/common.cpp`](https://github.com/ExpressLRS/ExpressLRS/blob/4.1.0/src/src/common.cpp).
De SX1280-rijen, ter controle naast de `#if ELRS_RATE`-tabel in de header:

| Rate | SF | BW | CR | Interval | FHSShop | Preamble | Pakket |
|---|---|---|---|---|---|---|---|
| 500 Hz | SF5 | 800 | LI_4_6 | 2000 us | 4 | 12 | OTA4 (8 B) |
| 333 Hz | SF5 | 800 | LI_4_8 | 3003 us | 4 | 12 | OTA8 (13 B) |
| 250 Hz | SF6 | 800 | LI_4_8 | 4000 us | 4 | **14** | OTA4 (8 B) |
| 150 Hz | SF7 | 800 | LI_4_8 | 6666 us | 4 | 12 | OTA4 (8 B) |
| 100 Hz | SF7 | 800 | LI_4_8 | 10000 us | 4 | 12 | OTA8 (13 B) |
| 50 Hz | SF8 | 800 | LI_4_8 | 20000 us | **2** | 12 | OTA4 (8 B) |

333 Hz en 100 Hz staan bewust niet in de header: die gebruiken het 13-byte
OTA8-formaat met een andere CRC en passen niet in deze code.

---

## 5. Kanalen uitpakken

Bron: `UnpackChannels4x10ToUInt11()` in `OTA.cpp`. De lus is identiek aan
`ELRS_UnpackChannels()`, met twee verschillen die geen fout zijn:

- ELRS begint op `readByteIndex = 0` omdat het al een pointer naar de rc-payload
  krijgt; wij beginnen op 1 omdat we het hele 8-byte pakket doorgeven.
- ELRS doet `dest[n] = (readValue & mask) << precisionShift` en schaalt 10 bits
  naar 11 (0..2046). Wij laten de shift weg, dus onze kanalen lopen van 0..1023.
  Dat is een halvering; als je ooit ELRS-waarden naast de jouwe legt, is dat de
  verklaring.

---

## 6. De SX1280-datasheet

Gebruik **Rev 3.2, maart 2020**, want de sectie- en paginanummers in het
commentaar horen daarbij:
[SX1280/SX1281 Data Sheet Rev 3.2 (DigiKey-spiegel, 158 p.)](https://media.digikey.com/pdf/Data%20Sheets/Semtech%20PDFs/SX1280-81_Rev3.2_Mar2020.pdf)

Andere revisies hebben andere paginanummers:
[Rev 3.0 (143 p., TME)](https://www.tme.eu/Document/1042f35a88b6ee421559d19923804032/SX128x.pdf) ·
[Rev 2.2 (137 p., Mouser)](https://www.mouser.com/datasheet/2/761/DS_SX1280-1_V2.2-1511144.pdf)

| Wat het commentaar zegt | Waar het staat in Rev 3.2 |
|---|---|
| "datasheet 11.8.1", `GetRxBufferStatus` | § 11.8.1, p. 92 |
| "datasheet 11.8.2", `GetPacketStatus`, RSSI = −rssiSync/2, SNR = snr/4 | § 11.8.2 |
| `SetRx` periodBase / periodBaseCount | § 11.6.5, p. 80 |
| "tabel 14-47 t/m 14-54" (modulatie- en pakketparameters) | Hoofdstuk 14 "LoRa Operation", p. 130–133 |
| "p.133: 0x40 = standaard IQ, 0x00 = omgedraaid" | Tabel 14-54 "IQ Swapping in LoRa or Ranging Packet", p. 133 |
| "tabel 6-3 (p.37): ±80 ppm bij BW 800 kHz en SF5..SF11" | Tabel 6-3 "Total Permissible Reference Drift", p. 37 — bevestigd: ±80 ppm voor SF5–SF10, ±50 ppm voor SF12 |

Het commandoblok bovenin de header (opcodes 0xC0, 0x18, 0x19, 0x1A, 0x1B, 0x80,
0x82, 0x86, 0x8A…) staat in hoofdstuk 11, en is ook 1-op-1 terug te vinden in
[`src/lib/SX1280Driver/SX1280_Regs.h`](https://github.com/ExpressLRS/ExpressLRS/blob/4.1.0/src/lib/SX1280Driver/SX1280_Regs.h)
van ExpressLRS. Datzelfde bestand bevat de IRQ-bits die wij gebruiken
(RX_DONE 0x0002, CRC_ERROR 0x0040, RX_TX_TIMEOUT 0x4000, PREAMBLE_DETECTED 0x8000)
en de twee registers `SX1280_REG_SF_ADDITIONAL_CONFIG 0x925` en
`SX1280_REG_FREQ_ERR_CORRECTION 0x93C`.

De verwijderde `SX1280_GetFrequencyError()` was ook echt gefundeerd: ELRS kent
`SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB 0x0954`.

---

## 7. Drie plekken waar het commentaar niet klopt

### a. De kristaltrim op 0x0A0E / 0x0A0F bestaat niet in de datasheet

Het commentaar zei: *"Datasheet 15.5 Foot Capacitance Tuning: de SX1280 heeft
interne belastingcondensatoren, instelbaar via registers 0xA0E en 0xA0F, bereik
12 tot 27 pF."*

Dat heb ik nergens kunnen bevestigen:

- In Rev 3.2 staat over het kristal alleen § 3.7 (p. 26, tabel 3-9) met
  `CLOAD` typisch 10 pF. Geen trimregisters, geen "foot capacitance"-sectie.
- In `SX1280_Regs.h` van ExpressLRS is **geen** register 0x0A0E of 0x0A0F gedefinieerd.
- Foot-capacitance-trim via registers bestaat wél, maar bij de **SX126x**
  (XTA/XTB), niet bij de SX1280. Waarschijnlijk zijn die twee door elkaar gehaald.

Dit is niet theoretisch: `SX1280_Setup_ELRS()` roept `SX1280_SetXtalTrim(0)` aan
en schrijft dus bij elke start een 0 naar twee ongedocumenteerde adressen, na een
overgang naar STDBY_XOSC. Je link werkt, dus fataal is het niet — maar als je ooit
onverklaarbaar gedrag ziet bij het opstarten, zet dan `ELRS_XTAL_TRIM` op `-1`
(dan slaat de `#if` het over) en kijk of er iets verandert.

### b. ELRS schrijft juist géén 0x01 naar 0x093C

Ons commentaar zegt: *"verplicht direct na SetModulationParams: 0x01 naar 0x093C"*.
De datasheet zegt dat inderdaad, maar ExpressLRS doet het bewust níet. Uit
[`src/lib/SX1280Driver/SX1280.cpp`](https://github.com/ExpressLRS/ExpressLRS/blob/4.1.0/src/lib/SX1280Driver/SX1280.cpp):

> Datasheet in LoRa Operation says "After SetModulationParams command: In all
> cases 0x1 must be written to the Frequency Error Compensation mode register
> 0x093C" However, this causes CRC errors for SF9 when using a high deviation TX
> (145kHz) and not using Explicit Header mode. The default register value (0x1b)
> seems most compatible, so don't mess with it

De SF-afhankelijke waarden voor 0x0925 (0x1E voor SF5/SF6, 0x37 voor SF7/SF8,
0x32 daarboven) doet ELRS wél, precies zoals wij. Alleen die ene regel
`SX1280_WriteRegister(SX1280_REG_FREQ_ERR_CORRECTION, 0x01)` wijkt af van wat de
zenderkant doet. Je draait SF6, waar ELRS het probleem niet meldt, dus het kan
prima zo blijven — maar het is een bewuste afwijking, geen datasheet-plicht.

### c. De beschrijving van de ELRS-tickvolgorde is achterhaald

Het commentaar bij `ELRS_Tick()` beschreef ELRS als: eerst `HandleFHSS()` met een
test op `(OtaNonce + 1) % interval`, daarna pas `OtaNonce++`. In 4.1.0 staat het
andersom in
[`src/src/rx_main.cpp`](https://github.com/ExpressLRS/ExpressLRS/blob/4.1.0/src/src/rx_main.cpp):
`HWtimerCallbackTock()` doet `OtaNonce++;` en dán `HandleFHSS();`, en `HandleFHSS()`
test gewoon `OtaNonce % FHSShopInterval`.

**Onze code doet het goed** — ophogen, dan `elrs_nonce % ELRS_FHSS_HOP_INTERVAL`.
Alleen de uitleg erbij beschrijft een oudere ELRS-versie. Ik heb de tekst in de
opgeschoonde versie al aangepast, maar het is goed om te weten dat de
`(nonce + 1)`-formulering die je op internet tegenkomt bij 3.x hoort.

### Voetnoot: bindmodus

Het (inmiddels verwijderde) `ELRS_BIND_TEST` ging uit van een publieke UID
`{0,1,2,3,4,5}`. In 4.1.0 werkt dat anders: `EnterBindingMode()` in `rx_main.cpp`
zet `OtaCrcInitializer = OTA_VERSION_ID` en houdt de eigen UID aan. Wat wél klopt
is dat er in bindmodus niet gehopt wordt: `HandleFHSS()` slaat het hoppen over
zolang `InBindingMode` waar is, en de radio blijft op `FHSSgetInitialFreq()`.

---

## Snel zelf opzoeken

Elk ELRS-bestand is direct te lezen zonder de repo te klonen, door in de URL
`github.com/.../blob/` te vervangen door `raw.githubusercontent.com/.../`:

```
https://raw.githubusercontent.com/ExpressLRS/ExpressLRS/refs/tags/4.1.0/<pad>
```

De paden die je nodig hebt:

| Onderwerp | Pad |
|---|---|
| CRC-seed, FHSS-seed, pakketvalidatie, kanalen uitpakken | `src/lib/OTA/OTA.cpp` |
| Polynoom, pakketstructuur, pakkettypes | `src/lib/OTA/OTA.h` |
| CRC-tabelopbouw | `src/lib/CRC/crc.cpp` |
| Hoptabel, band, sync-kanaal, primaryBandCount | `src/lib/FHSS/FHSS.cpp` |
| `FHSS_SEQUENCE_LEN` | `src/lib/FHSS/FHSS.h` |
| PRNG | `src/lib/FHSS/random.cpp` |
| Rate-tabel | `src/src/common.cpp` |
| Nonce, hop-timing, bindmodus | `src/src/rx_main.cpp` |
| `OTA_VERSION_ID` | `src/include/targets.h` |
| Opcodes, registers, IRQ-bits | `src/lib/SX1280Driver/SX1280_Regs.h` |
| Radio-instellingen en de 0x0925/0x093C-kwestie | `src/lib/SX1280Driver/SX1280.cpp` |
| Binding phrase naar UID | `src/python/build_flags.py` |

Officiële ELRS-documentatie: [expresslrs.org](https://www.expresslrs.org/) —
onder andere [Binding](https://www.expresslrs.org/quick-start/binding/) en
[CRC Testing](https://www.expresslrs.org/software/testing/crc-testing/).
