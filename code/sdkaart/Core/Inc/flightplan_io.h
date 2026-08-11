/*
 * flightplan_io.h
 *
 *  Created on: 3 aug 2026
 *      Author: simon
 *
 * Opslaan, inlezen, valideren en tonen van een vluchtplan.
 *
 * flightplan.c doet het eigenlijke parsen van tekstregels. Deze module zit daar
 * bovenop en voegt toe wat er nog ontbrak:
 *
 *   - een vluchtplan in RAM opbouwen (FlightPlan_Clear / FlightPlan_Add)
 *   - datzelfde plan terugschrijven naar de SD-kaart (FlightPlanIO_Save)
 *   - het plan controleren voor je het wegschrijft (FlightPlan_Validate)
 *   - het plan leesbaar over UART tonen (FlightPlan_Print)
 *
 * OMWISSELBAAR FORMAAT
 * --------------------
 * Het bestandsformaat zit achter een enkele define. Nu staat hij op tekst,
 * zodat je vluchtplan.txt met een gewone editor kan lezen. Wil je later naar
 * binair (compacter, met CRC, en klaar om over een radiolink te sturen), dan
 * zet je in fc_config.h of via de compiler-defines:
 *
 *     #define FPIO_FORMAT  FPIO_FMT_BINARY
 *
 * De rest van de code (flightrun.c, flightcontrol.c) verandert niet mee: die
 * blijft gewoon FlightPlanIO_Load() aanroepen.
 *
 * BINAIR FORMAAT (FPL1)
 * ---------------------
 * Alles little-endian, zoals de Cortex-M7 zelf.
 *
 *   offset  grootte  veld
 *   0       4        magic "FPL1"
 *   4       2        versie          (= 1)
 *   6       2        aantal commando's
 *   8       2        recordgrootte   (= 14)
 *   10      2        CRC16-CCITT over alle recordbytes
 *   12      n*14     records
 *
 *   record: uint8 type, uint8 gereserveerd, float32 p0, float32 p1, float32 p2
 *
 * LET OP: alle Save/Load-functies gaan ervan uit dat de SD-kaart al gemount is
 * (SDCard_Mount). Zie flightplan_tool.c voor het volledige recept.
 */

#ifndef INC_FLIGHTPLAN_IO_H_
#define INC_FLIGHTPLAN_IO_H_

#include <stdint.h>
#include "flightplan.h"

/* ==========================================================================
 * Formaatkeuze
 * ========================================================================== */

#define FPIO_FMT_TEXT       0
#define FPIO_FMT_BINARY     1

#ifndef FPIO_FORMAT
#define FPIO_FORMAT         FPIO_FMT_TEXT
#endif

/* Grootste bestand dat we in één keer in RAM trekken. */
#define FPIO_MAX_FILE_SIZE  4096

/* Binair formaat */
#define FPIO_BIN_MAGIC0     'F'
#define FPIO_BIN_MAGIC1     'P'
#define FPIO_BIN_MAGIC2     'L'
#define FPIO_BIN_MAGIC3     '1'
#define FPIO_BIN_VERSION    1u
#define FPIO_BIN_HDR_SIZE   12u
#define FPIO_BIN_REC_SIZE   14u

/* ==========================================================================
 * Statuscodes
 * ========================================================================== */

typedef enum {
    FPIO_OK = 0,
    FPIO_ERR_SD,         /* mounten, openen, lezen of schrijven mislukt        */
    FPIO_ERR_MAGIC,      /* binair bestand begint niet met "FPL1"              */
    FPIO_ERR_VERSION,    /* binair bestand van een andere versie               */
    FPIO_ERR_CRC,        /* checksum klopt niet, bestand is beschadigd         */
    FPIO_ERR_TOO_BIG,    /* plan past niet in FLIGHTPLAN_MAX_CMDS of de buffer */
    FPIO_ERR_EMPTY,      /* geen enkel geldig commando gevonden                */
    FPIO_ERR_INVALID,    /* validatie afgekeurd, zie de foutstring             */
} FPIO_Status_t;

/* Leesbare tekst bij een statuscode, voor printf. */
const char *FPIO_StatusStr(FPIO_Status_t s);

/* Naam van een commando zoals hij in het tekstbestand staat ("Throttle", ...). */
const char *FlightPlan_CmdName(FlightCmdType_t t);

/* ==========================================================================
 * Een plan in RAM opbouwen
 * ========================================================================== */

/* Maakt het plan leeg. Roep dit altijd aan voor de eerste FlightPlan_Add(). */
void FlightPlan_Clear(FlightPlan_t *plan);

/* Voegt één commando achteraan toe. Ongebruikte parameters geef je als 0.
 * Retourneert FPIO_ERR_TOO_BIG als het plan vol zit. */
FPIO_Status_t FlightPlan_Add(FlightPlan_t *plan, FlightCmdType_t type,
                             float p0, float p1, float p2);

/* Kopieert een kant-en-klare array commando's in het plan (vervangt de inhoud). */
FPIO_Status_t FlightPlan_SetFromArray(FlightPlan_t *plan,
                                      const FlightCmd_t *cmds, uint16_t count);

/* ==========================================================================
 * Validatie
 * ========================================================================== */

/* Controleert het plan op fouten die de drone in de problemen brengen:
 * onbekende commando's, throttle buiten 0..100 %, duur van 0 of negatief,
 * commando's na Land. Twijfelgevallen (geen Land op het einde, totale duur
 * boven FAILSAFE_TIMEOUT) worden als waarschuwing geprint maar niet afgekeurd.
 *
 * Bij een fout komt er een uitleg in err (mag NULL zijn) en is het resultaat
 * FPIO_ERR_INVALID of FPIO_ERR_EMPTY. */
FPIO_Status_t FlightPlan_Validate(const FlightPlan_t *plan, char *err, uint16_t err_len);

/* Totale geplande vluchtduur in ms, voor zover die uit de commando's blijkt. */
uint32_t FlightPlan_TotalDurationMs(const FlightPlan_t *plan);

/* ==========================================================================
 * Tonen over UART (gaat via printf, dus via UART8)
 * ========================================================================== */

/* Zet één commando om naar een tekstregel zoals hij in vluchtplan.txt staat,
 * bv. "Throttle 30 2000". Retourneert het aantal geschreven tekens. */
uint16_t FlightPlan_FormatCmd(const FlightCmd_t *cmd, char *out, uint16_t out_len);

/* Print het volledige plan genummerd, met de totale duur eronder.
 * Dit is de controle die je draait na het laden, voor je laat vliegen. */
void FlightPlan_Print(const FlightPlan_t *plan, const char *titel);

/* ==========================================================================
 * Opslaan en laden - formaat volgens FPIO_FORMAT
 * ========================================================================== */

FPIO_Status_t FlightPlanIO_Save(const FlightPlan_t *plan, const char *filename);
FPIO_Status_t FlightPlanIO_Load(FlightPlan_t *plan, const char *filename);

/* Schrijft weg en leest meteen terug, en vergelijkt beide.
 * Zo weet je zeker dat wat er op de kaart staat ook echt leesbaar is. */
FPIO_Status_t FlightPlanIO_SaveVerified(const FlightPlan_t *plan, const char *filename);

/* Expliciet formaat, handig om beide naast elkaar te testen. */
FPIO_Status_t FlightPlanIO_SaveText(const FlightPlan_t *plan, const char *filename);
FPIO_Status_t FlightPlanIO_LoadText(FlightPlan_t *plan, const char *filename);
FPIO_Status_t FlightPlanIO_SaveBinary(const FlightPlan_t *plan, const char *filename);
FPIO_Status_t FlightPlanIO_LoadBinary(FlightPlan_t *plan, const char *filename);

/* ==========================================================================
 * Serialiseren naar een buffer in plaats van naar een bestand
 *
 * Dit is de haak voor later: wil je het plan over UART7 of een radiolink naar
 * een tweede bord sturen, dan bouw je met deze functies het pakket op en geef
 * je die bytes door aan je transportlaag. Geen SD-kaart nodig.
 * ========================================================================== */

/* Retourneert het aantal geschreven bytes, of 0 bij een fout. */
uint32_t FlightPlanIO_SerializeText(const FlightPlan_t *plan, uint8_t *buf, uint32_t buf_size);
uint32_t FlightPlanIO_SerializeBinary(const FlightPlan_t *plan, uint8_t *buf, uint32_t buf_size);

FPIO_Status_t FlightPlanIO_DeserializeText(FlightPlan_t *plan, char *buf);
FPIO_Status_t FlightPlanIO_DeserializeBinary(FlightPlan_t *plan, const uint8_t *buf, uint32_t size);

/* CRC16-CCITT (poly 0x1021, init 0xFFFF), zoals gebruikt in het binaire formaat. */
uint16_t FPIO_Crc16(const uint8_t *data, uint32_t len);

#endif /* INC_FLIGHTPLAN_IO_H_ */
