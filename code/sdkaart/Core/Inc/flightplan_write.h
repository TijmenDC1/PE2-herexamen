/*
 * flightplan_write.h
 *
 *  Author: tijmen
 *
 * Een vluchtplan aanmaken en op de SD-kaart zetten.
 *
 * De commando's zelf staan in main.c
 */

#ifndef FLIGHTPLAN_WRITE_H
#define FLIGHTPLAN_WRITE_H

#include "flightplan.h"

/* Schrijft de commando's naar filename. Het bestand wordt aangemaakt als het nog
 * niet bestaat en anders overschreven. De kaart moet gemount zijn.
 *
 *   FP_OK           gelukt en teruggelezen
 *   FP_ERR_INVALID  afgekeurd door de validatie, of anders teruggelezen
 *   FP_ERR_TOO_BIG  past niet in de werkbuffer
 *   FP_ERR_SD       schrijven of teruglezen mislukt
 *
 * Er gaat niets naar de kaart als de validatie het plan afkeurt.
 */
FP_Status_t FlightPlan_Save(const FlightCmd_t *cmds, uint16_t count, const char *filename);

#endif /* FLIGHTPLAN_WRITE_H */
