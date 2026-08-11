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
#include "flightplan_io.h"
#include "flightplan_tool.h"
#include "flightcontrol.h"
#include <stdio.h>

#define FLIGHTPLAN_FILENAME  FLIGHTPLAN_ACTIVE_FILE
#define CONTROL_LOOP_DT_MS   10   // moet overeenkomen met de dt in FlightControl_Update

static FlightPlan_t plan;

void FlightRun_Execute(void)
{
    //1 SD kaart mounten
    if (SDCard_Mount() != SDCARD_OK) {
        printf("kon SD kaart niet mounten, vlucht geannuleerd\n");
        return;
    }

    //vluchtplan inladen (formaat volgt FPIO_FORMAT: tekst of binair)
    FPIO_Status_t res = FlightPlanIO_Load(&plan, FLIGHTPLAN_FILENAME);
    if (res != FPIO_OK) {
        printf("vluchtplan inlezen mislukt: %s, vlucht geannuleerd\n", FPIO_StatusStr(res));
        SDCard_Unmount();
        return;
    }

    //tonen wat er geladen is, zodat je het over UART8 kan nakijken
    FlightPlan_Print(&plan, FLIGHTPLAN_FILENAME);

    //en controleren voor er een motor draait
    char err[96];
    res = FlightPlan_Validate(&plan, err, sizeof(err));
    if (res != FPIO_OK) {
        printf("vluchtplan afgekeurd: %s, vlucht geannuleerd\n", err);
        SDCard_Unmount();
        return;
    }
    int n = (int)plan.count;

    //PID('s) initialiseren
    if (FlightControl_Init() != 0) {
        printf("kon vluchtregeling niet initialiseren, vlucht geannuleerd\n");
        SDCard_Unmount();
        return;
    }

    //vluchtplan uitvoeren (blokkerend tot het plan afgelopen is)
    printf("start uitvoering van %d commando's\n", n);
    FlightControl_Run(&plan, HAL_GetTick, HAL_Delay, CONTROL_LOOP_DT_MS);
    printf("vluchtplan volledig uitgevoerd\n");

    SDCard_Unmount();
}
