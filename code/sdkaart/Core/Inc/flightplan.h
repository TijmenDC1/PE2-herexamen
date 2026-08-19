/*
 * flightplan.h
 *
 *  Created on: 15 jul 2026
 *      Author: tijme
 *

 * flightplan.h
 vluchtplan: de types, en alles om een plan van de SD-kaart lezen.
 * Wegschrijven staat in flightplan_write.h.

 * Bestandsformaat op de kaart (gewone tekst, 1 commando per regel):
 *
 *   # regels met een # zijn commentaar
 *   Throttle 30 2000      procent 0-100, daarna de duur in ms
 *   Hover 1500            duur in ms, houdt de huidige throttle aan
 *   Left 1000             duur in ms, kantelt LEFT_RIGHT_TILT_DEG naar links
 *   Right 1000            idem naar rechts
 *   Land                  bouwt de throttle af en zet de motoren uit
 *   RelativeHeight 1.5    meter, wordt geparsed maar nog niet uitgevoerd
 *   AbsoluteHeight 10     meter, idem (barometer nodig)
 *   Move 1 2 3            x y z, idem (GPS nodig)
 */

#ifndef FLIGHTPLAN_H
#define FLIGHTPLAN_H

#include "fc_config.h"
#include <stdint.h>

#ifndef FLIGHTPLAN_MAX_CMDS
#define FLIGHTPLAN_MAX_CMDS     32          /* commando's die in één plan passen */
#endif
#ifndef FLIGHTPLAN_FILE_MAX
#define FLIGHTPLAN_FILE_MAX     4096        /* grootte van de gedeelde werkbuffer */
#endif
#ifndef FLIGHTPLAN_ACTIVE_FILE
#define FLIGHTPLAN_ACTIVE_FILE  "plan.txt"  /* het plan dat flightrun.c uitvoert */
#endif

#define FLIGHTPLAN_LINE_MAX     96          /* langste regel die we aankunnen */

#ifndef FLIGHTPLAN_MAX_DUR_MS
#define FLIGHTPLAN_MAX_DUR_MS   300000.0f   /* 5 min: alles daarboven is een typfout
                                             * of een kapot bestand, en wordt afgekeurd */
#endif

typedef enum {
    CMD_UNKNOWN = 0,
    CMD_RELATIVE_HEIGHT,
    CMD_ABSOLUTE_HEIGHT,
    CMD_HOVER,
    CMD_THROTTLE,
    CMD_MOVE,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_LAND
} FlightCmdType_t;

typedef struct {
    FlightCmdType_t type;
    float           param[3];   /* max 3, bij Move is dat x, y, z */
} FlightCmd_t;

typedef struct {
    FlightCmd_t cmds[FLIGHTPLAN_MAX_CMDS];
    uint16_t    count;          /* hoeveel commando's er in staan */
    uint16_t    current;        /* welk commando aan de beurt is */
} FlightPlan_t;

/* Handig om het aantal commando's van je eigen array in main.c mee te geven:
 *   FlightPlan_Save(mijn_plan, FLIGHTPLAN_AANTAL(mijn_plan), FLIGHTPLAN_ACTIVE_FILE); */
#define FLIGHTPLAN_AANTAL(a)    ((uint16_t)(sizeof(a) / sizeof((a)[0])))

typedef enum {
    FP_OK = 0,
    FP_ERR_SD,        /* mounten, openen, lezen of schrijven mislukt */
    FP_ERR_EMPTY,     /* geen enkel geldig commando in het bestand */
    FP_ERR_TOO_BIG,   /* meer commando's of bytes dan er passen */
    FP_ERR_INVALID    /* afgekeurd door de validatie, of anders teruggelezen */
} FP_Status_t;

/* Het plan dat nu in RAM staat. Eén exemplaar voor het hele project: er vliegt
 * er toch maar één tegelijk, en een FlightPlan_t is een halve kilobyte. */
extern FlightPlan_t g_flightplan;

/* Gedeelde werkbuffer voor bestandsinhoud, ook gebruikt door flightplan_write.c. */
extern uint8_t g_fp_buf[FLIGHTPLAN_FILE_MAX];

/* --- lezen (de kaart moet gemount zijn) --------------------------------- */

/* Zet één tekstregel om naar een FlightCmd_t. 1 = gelukt, 0 = overgeslagen. */
uint8_t FlightPlan_ParseLine(const char *line, FlightCmd_t *cmd);

/* Leest het bestand van de kaart en parset het naar g_flightplan. */
FP_Status_t FlightPlan_Load(const char *filename);

/* --- alles-in-één voor main.c (mount en unmount zitten erin) ------------- */

/* Mount, print eventueel de ruwe bytes, laadt, print genummerd en valideert. */
FP_Status_t FlightPlan_Show(const char *filename, uint8_t toon_ruw);

/* --- doorlopen tijdens de vlucht ---------------------------------------- */

void         FlightPlan_Reset  (FlightPlan_t *plan);
uint8_t      FlightPlan_HasNext(FlightPlan_t *plan);
FlightCmd_t *FlightPlan_Next   (FlightPlan_t *plan);

/* --- tonen en controleren ----------------------------------------------- */

const char *FP_StatusStr       (FP_Status_t s);
const char *FlightPlan_CmdName (FlightCmdType_t t);
uint8_t     FlightPlan_CmdParams(FlightCmdType_t t);

/* Zet een commando om naar exact de tekstregel die ook op de kaart komt. */
uint16_t FlightPlan_FormatCmd(const FlightCmd_t *cmd, char *out, uint16_t out_len);

void     FlightPlan_Print(const FlightCmd_t *cmds, uint16_t count, const char *titel);
uint32_t FlightPlan_TotalDurationMs(const FlightCmd_t *cmds, uint16_t count);

/* err krijgt de reden van afkeuring, mag NULL zijn. */
FP_Status_t FlightPlan_Validate(const FlightCmd_t *cmds, uint16_t count,
                                char *err, uint16_t err_len);

#endif /* FLIGHTPLAN_H */
