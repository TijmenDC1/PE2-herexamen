/*
 * flightplan.h
 *
 *  Created on: 15 jul 2026
 *      Author: tijme
 *
 * Inladen en uitvoeren van een vluchplan bestand van de SD kaart.
 *
 * Bestandsformaat (één commando per lijn):
 *   RelativeHeight <meter> end
 *   AbsoluteHeight <meter> end
 *   Hover          <ms>    end
 *   Throttle       <procent> <ms> end
 *   Move           <x> <y> <z> end
 *
 * Voorbeeld vluchtplan.txt:
 *   RelativeHeight 10 end
 *   Hover 3000 end
 *   Move 1.0 0.0 0.0 end
 *   AbsoluteHeight 0 end
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

/* Laad het volledige vluchtplan van de SD kaart.
 * Retourneert het aantal geladen commando's, of -1 bij een fout. */
int FlightPlan_Load(FlightPlan_t *plan, const char *filename);

/* Geeft 1 als er nog commando's zijn om uit te voeren. */
uint8_t FlightPlan_HasNext(FlightPlan_t *plan);

/* Geeft een pointer naar het volgende commando en schuift de index op.
 * Retourneert NULL als het plan afgelopen is. */
FlightCmd_t *FlightPlan_Next(FlightPlan_t *plan);

/* Reset de uitvoeringsindex zodat het plan opnieuw begint. */
void FlightPlan_Reset(FlightPlan_t *plan);

#endif /* INC_FLIGHTPLAN_H_ */
