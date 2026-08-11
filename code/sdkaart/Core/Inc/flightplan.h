/*
 * flightplan.h
 *
 *  Created on: 15 jul 2026
 *      Author: tijme
 *
 * Inladen en uitvoeren van een vluchplan bestand van de SD kaart.
 *
 * Bestandsformaat (één commando per lijn, regels met # zijn commentaar):
 *   RelativeHeight <meter>
 *   AbsoluteHeight <meter>
 *   Hover          <ms>
 *   Throttle       <procent 0-100> <ms>
 *   Move           <x> <y> <z>
 *   Left           <ms>
 *   Right          <ms>
 *   Land
 *
 * Voorbeeld vluchtplan.txt:
 *   # voorzichtige testvlucht
 *   Throttle 30 2000
 *   Throttle 40 3000
 *   Left 1000
 *   Right 1000
 *   Land
 *
 * Let op: een eerdere versie van dit commentaar zette "end" achter elke regel.
 * De parser heeft dat nooit nodig gehad en negeert extra tokens; de schrijver
 * in flightplan_io.c zet het er dus ook niet bij.
 */

#ifndef INC_FLIGHTPLAN_H_
#define INC_FLIGHTPLAN_H_

#include <stdint.h>

#define FLIGHTPLAN_MAX_CMDS   64    // maximaal aantal commando's in één vluchtplan

typedef enum {
    CMD_UNKNOWN = 0,
    CMD_RELATIVE_HEIGHT,
    CMD_ABSOLUTE_HEIGHT,
    CMD_HOVER,
    CMD_THROTTLE,
    CMD_MOVE,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_LAND,
} FlightCmdType_t;
typedef struct {
    FlightCmdType_t type;
    float param[3];   // param[0..2] afhankelijk van commando
                      // RelativeHeight : param[0] = meter
                      // AbsoluteHeight : param[0] = meter
                      // Hover          : param[0] = ms
                      // Throttle       : param[0] = procent, param[1] = ms
                      // Move           : param[0]=x  param[1]=y  param[2]=z
} FlightCmd_t;

typedef struct {
    FlightCmd_t cmds[FLIGHTPLAN_MAX_CMDS];
    uint16_t    count;      // aantal geladen commando's
    uint16_t    current;    // huidige uitvoeringsindex
} FlightPlan_t;

/* Zet één tekstregel om naar een FlightCmd_t.
 * Retourneert 1 bij een geldig commando, 0 bij commentaar, een lege regel of
 * een onbekend commando (dat wordt dan overgeslagen).
 * Wordt ook gebruikt door flightplan_io.c, zodat schrijven en lezen precies
 * dezelfde commandonamen hanteren. */
uint8_t FlightPlan_ParseLine(const char *line, FlightCmd_t *cmd);

/* Laad het volledige vluchtplan van de SD kaart.
 * Retourneert het aantal geladen commando's, of -1 bij een fout.
 * Dit is de tekst-only variant. Voor het formaatonafhankelijke pad, zie
 * FlightPlanIO_Load() in flightplan_io.h. */
int FlightPlan_Load(FlightPlan_t *plan, const char *filename);

/* Geeft 1 als er nog commando's zijn om uit te voeren. */
uint8_t FlightPlan_HasNext(FlightPlan_t *plan);

/* Geeft een pointer naar het volgende commando en schuift de index op.
 * Retourneert NULL als het plan afgelopen is. */
FlightCmd_t *FlightPlan_Next(FlightPlan_t *plan);

/* Reset de uitvoeringsindex zodat het plan opnieuw begint. */
void FlightPlan_Reset(FlightPlan_t *plan);

#endif /* INC_FLIGHTPLAN_H_ */
