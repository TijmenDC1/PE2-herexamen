/*
 * flightplan_tool.h
 *
 *  Created on: 3 aug 2026
 *      Author: simon
 *
 * De twee programma's rond het vluchtplan, allebei op dit bord:
 *
 *   SCHRIJVEN  FlightPlanTool_WriteAll()  zet de hardgecodeerde plannen uit
 *              flightplan_tool.c op de SD-kaart. Nodig omdat dit bordje geen
 *              SD-lezer op de PC heeft: je flasht de firmware en het plan staat
 *              op de kaart. Elk plan wordt eerst gevalideerd en na het
 *              wegschrijven meteen teruggelezen ter controle.
 *
 *   UITLEZEN   FlightPlanTool_Dump()      leest een plan van de kaart en print
 *              het genummerd over UART8. Zo zie je in je terminal precies wat de
 *              drone straks gaat uitvoeren, voor er een motor draait.
 *
 * Een nieuw vluchtplan toevoegen: schrijf een FlightCmd_t-array in
 * flightplan_tool.c en zet hem in de tabel g_plannen. Verder hoef je niets
 * aan te passen.
 */

#ifndef INC_FLIGHTPLAN_TOOL_H_
#define INC_FLIGHTPLAN_TOOL_H_

#include <stdint.h>
#include "flightplan.h"
#include "flightplan_io.h"

/* Het plan dat flightrun.c uitvoert. */
#define FLIGHTPLAN_ACTIVE_FILE  "vluchtplan.txt"

/* Eén hardgecodeerd plan uit de tabel in flightplan_tool.c. */
typedef struct {
    const char        *naam;      /* voor de printf's                       */
    const char        *bestand;   /* bestandsnaam op de SD-kaart (8.3)      */
    const FlightCmd_t *cmds;
    uint16_t           count;
} FlightPlanDef_t;

/* ==========================================================================
 * Schrijven
 * ========================================================================== */

/* Zet alle plannen uit de tabel op de SD-kaart. Mount en unmount zelf.
 * Retourneert het aantal plannen dat geschreven én teruggelezen kon worden. */
int FlightPlanTool_WriteAll(void);

/* Zet één plan uit de tabel op de kaart, gezocht op naam. SD moet al gemount
 * zijn. Retourneert FPIO_OK bij succes. */
FPIO_Status_t FlightPlanTool_WriteByName(const char *naam);

/* Schrijft een plan dat je zelf in RAM hebt opgebouwd. Valideert eerst en
 * leest daarna terug ter controle. SD moet al gemount zijn. */
FPIO_Status_t FlightPlanTool_WritePlan(const FlightPlan_t *plan, const char *filename);

/* ==========================================================================
 * Uitlezen
 * ========================================================================== */

/* Leest een plan van de kaart, valideert het en print het over UART8.
 * Mount en unmount zelf. Dit is het "uitlees-programma".
 * plan_out mag NULL zijn als je het resultaat niet nodig hebt. */
FPIO_Status_t FlightPlanTool_Dump(const char *filename, FlightPlan_t *plan_out);

/* Print de ruwe inhoud van een tekstbestand op de kaart, zonder te parsen.
 * Handig als het parsen faalt en je wil zien wat er echt op de kaart staat. */
FPIO_Status_t FlightPlanTool_DumpRaw(const char *filename);

/* Lijst de plannen op die in de firmware zitten. */
void FlightPlanTool_ListBuiltin(void);

#endif /* INC_FLIGHTPLAN_TOOL_H_ */
