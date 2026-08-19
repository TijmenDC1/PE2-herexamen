/*
 * flightrun.c
 *
 *  Created on: 30 jul 2026
 *      Author: tijme
 */

#include "flightrun.h"
#include "main.h"
#include "sdcard.h"
#include "flightplan.h"
#include "flightcontrol.h"
#include <stdio.h>

#define CONTROL_LOOP_DT_MS   10   //moet overeenkomen met de dt in FlightControl_Update

void FlightRun_Execute(void)
{
    //1 SD kaart mounten
    if (SDCard_Mount() != SDCARD_OK) {
        printf("kon SD kaart niet mounten, vlucht geannuleerd\n");
        return;
    }

    //vluchtplan inladen naar g_flightplan
    FP_Status_t res = FlightPlan_Load(FLIGHTPLAN_ACTIVE_FILE);
    if (res != FP_OK) {
        printf("vluchtplan inlezen mislukt: %s, vlucht geannuleerd\n", FP_StatusStr(res));
        SDCard_Unmount();
        return;
    }

    //tonen wat er geladen is, zodat je het over UART8 kan nakijken
    FlightPlan_Print(g_flightplan.cmds, g_flightplan.count, FLIGHTPLAN_ACTIVE_FILE);

    //en controleren voor er een motor draait
    char err[80];
    res = FlightPlan_Validate(g_flightplan.cmds, g_flightplan.count, err, sizeof(err));
    if (res != FP_OK) {
        printf("vluchtplan afgekeurd: %s, vlucht geannuleerd\n", err);
        SDCard_Unmount();
        return;
    }

    //PID('s) initialiseren
    if (FlightControl_Init() != 0) {
        printf("kon vluchtregeling niet initialiseren, vlucht geannuleerd\n");
        SDCard_Unmount();
        return;
    }

    //vluchtplan uitvoeren (blokkerend tot het plan afgelopen is)
    printf("start uitvoering van %u commando's\n", (unsigned)g_flightplan.count);
    FlightControl_Run(&g_flightplan, HAL_GetTick, HAL_Delay, CONTROL_LOOP_DT_MS);
    printf("vluchtplan volledig uitgevoerd\n");

    SDCard_Unmount();
}
