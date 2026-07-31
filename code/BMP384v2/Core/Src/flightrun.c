/*
 * flightrun.c
 *
 *  Created on: 30 jul 2026
 *      Author: tijme
 *
 */

#include "flightrun.h"
#include "main.h"
#include "sdcard.h"
#include "flightplan.h"
#include "flightcontrol.h"
#include <stdio.h>

#define FLIGHTPLAN_FILENAME  "vluchtplan.txt"
#define CONTROL_LOOP_DT_MS   10   // moet overeenkomen met de dt in FlightControl_Update

static FlightPlan_t plan;

void FlightRun_Execute(void)
{
    // Stap 1: SD kaart mounten
    if (SDCard_Mount() != SDCARD_OK) {
        printf("kon SD kaart niet mounten, vlucht geannuleerd\n");
        return;
    }

    // Stap 2: vluchtplan inladen
    int n = FlightPlan_Load(&plan, FLIGHTPLAN_FILENAME);
    if (n < 0) {
        printf("vluchtplan bestand niet gevonden, vlucht geannuleerd\n");
        SDCard_Unmount();
        return;
    }
    if (n == 0) {
        printf("vluchtplan bevat geen geldige commando's, vlucht geannuleerd\n");
        SDCard_Unmount();
        return;
    }

    // Stap 3: PID('s) initialiseren
    if (FlightControl_Init() != 0) {
        printf("kon vluchtregeling niet initialiseren, vlucht geannuleerd\n");
        SDCard_Unmount();
        return;
    }

    // Stap 4: vluchtplan uitvoeren (blokkerend tot het plan afgelopen is)
    printf("start uitvoering van %d commando's\n", n);
    FlightControl_Run(&plan, HAL_GetTick, HAL_Delay, CONTROL_LOOP_DT_MS);
    printf("vluchtplan volledig uitgevoerd\n");

    SDCard_Unmount();
}
