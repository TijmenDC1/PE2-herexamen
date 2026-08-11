/*
 * flightplan_tool.c
 *
 *  Created on: 3 aug 2026
 *      Author: simon
 *
 * Zie flightplan_tool.h. Vervangt de losse tekststring die eerst in main.c
 * stond: de plannen staan nu als FlightCmd_t-arrays in de firmware, worden
 * gevalideerd voor ze naar de kaart gaan, en na het schrijven teruggelezen.
 */

#include "flightplan_tool.h"
#include "flightplan_io.h"
#include "sdcard.h"

#include <string.h>
#include <stdio.h>

/* ==========================================================================
 * De hardgecodeerde vluchtplannen
 *
 * Throttle <procent 0-100> <ms>   0 % = THROTTLE_MIN, 100 % = THROTTLE_MAX
 * Hover    <ms>                   houdt de huidige throttle aan
 * Left     <ms>                   kantelt LEFT_RIGHT_TILT_DEG naar links
 * Right    <ms>                   idem naar rechts
 * Land                            bouwt throttle af en zet de motoren uit
 *
 * RelativeHeight / AbsoluteHeight / Move worden wel geparsed maar door
 * flightcontrol.c nog overgeslagen: die hebben de barometer en GPS nodig.
 * ========================================================================== */

/* Eerste voorzichtige testvlucht.
 * TEST DIT EERST ZONDER PROPELLERS, of met het frame vastgezet op een teststaaf.
 * FlightControl_Init() kalibreert de gyro en het level: het frame moet dan stil
 * en waterpas liggen (volg de printf's over UART8). */
static const FlightCmd_t plan_test1[] = {
    { CMD_THROTTLE, { 30.0f, 2000.0f, 0.0f } },   /* rustig aanlopen           */
    { CMD_THROTTLE, { 40.0f, 3000.0f, 0.0f } },   /* ~THROTTLE_BASE uit fc_config.h */
    { CMD_LEFT,     { 1000.0f, 0.0f, 0.0f } },    /* kantelrichting controleren */
    { CMD_RIGHT,    { 1000.0f, 0.0f, 0.0f } },
    { CMD_THROTTLE, { 35.0f, 1500.0f, 0.0f } },   /* even rustig voor de landing */
    { CMD_LAND,     { 0.0f, 0.0f, 0.0f } },
};

/* Nog voorzichtiger: alleen aanlopen en weer landen, geen kantelbewegingen.
 * Goed om de hele keten (SD -> parser -> ESC's) één keer door te lopen. */
static const FlightCmd_t plan_min[] = {
    { CMD_THROTTLE, { 25.0f, 2000.0f, 0.0f } },
    { CMD_LAND,     { 0.0f, 0.0f, 0.0f } },
};

/* Kantelrichting apart controleren, op een lage throttle. */
static const FlightCmd_t plan_tilt[] = {
    { CMD_THROTTLE, { 30.0f, 1500.0f, 0.0f } },
    { CMD_LEFT,     { 1500.0f, 0.0f, 0.0f } },
    { CMD_THROTTLE, { 30.0f, 1000.0f, 0.0f } },
    { CMD_RIGHT,    { 1500.0f, 0.0f, 0.0f } },
    { CMD_LAND,     { 0.0f, 0.0f, 0.0f } },
};

#define ARRAY_COUNT(a)  ((uint16_t)(sizeof(a) / sizeof((a)[0])))

/* De tabel. Het eerste plan gaat naar FLIGHTPLAN_ACTIVE_FILE en is dus het plan
 * dat flightrun.c straks uitvoert. De andere staan er als reserve op de kaart;
 * wil je er een gebruiken, wissel dan de bestandsnamen hieronder om. */
static const FlightPlanDef_t g_plannen[] = {
    { "testvlucht 1",   FLIGHTPLAN_ACTIVE_FILE, plan_test1, ARRAY_COUNT(plan_test1) },
    { "minimaal",       "plan_min.txt",         plan_min,   ARRAY_COUNT(plan_min)   },
    { "kanteltest",     "plan_tilt.txt",        plan_tilt,  ARRAY_COUNT(plan_tilt)  },
};

#define PLANNEN_COUNT  ARRAY_COUNT(g_plannen)

/* Werkgeheugen. Static, want een FlightPlan_t is ruim 1 kB. */
static FlightPlan_t g_work;

/* ==========================================================================
 * Schrijven
 * ========================================================================== */

FPIO_Status_t FlightPlanTool_WritePlan(const FlightPlan_t *plan, const char *filename)
{
    char err[96];

    FPIO_Status_t res = FlightPlan_Validate(plan, err, sizeof(err));
    if (res != FPIO_OK) {
        printf("  AFGEKEURD: %s (%s)\n", err, FPIO_StatusStr(res));
        return res;
    }

    res = FlightPlanIO_SaveVerified(plan, filename);
    if (res != FPIO_OK) {
        printf("  schrijven van %s mislukt: %s\n", filename, FPIO_StatusStr(res));
        return res;
    }

    printf("  %s geschreven en teruggelezen (%u commando's)\n",
           filename, (unsigned)plan->count);
    return FPIO_OK;
}

FPIO_Status_t FlightPlanTool_WriteByName(const char *naam)
{
    for (uint16_t i = 0; i < PLANNEN_COUNT; i++) {
        if (strcmp(g_plannen[i].naam, naam) == 0) {
            FPIO_Status_t res = FlightPlan_SetFromArray(&g_work,
                                                        g_plannen[i].cmds,
                                                        g_plannen[i].count);
            if (res != FPIO_OK) return res;
            return FlightPlanTool_WritePlan(&g_work, g_plannen[i].bestand);
        }
    }
    printf("flightplan_tool: geen plan met de naam '%s'\n", naam);
    return FPIO_ERR_INVALID;
}

int FlightPlanTool_WriteAll(void)
{
    if (SDCard_Mount() != SDCARD_OK) {
        printf("flightplan_tool: kon de SD-kaart niet mounten\n");
        return -1;
    }

    printf("flightplan_tool: %u plannen wegschrijven...\n", (unsigned)PLANNEN_COUNT);

    int ok = 0;
    for (uint16_t i = 0; i < PLANNEN_COUNT; i++) {
        const FlightPlanDef_t *def = &g_plannen[i];
        printf("- %s -> %s\n", def->naam, def->bestand);

        if (FlightPlan_SetFromArray(&g_work, def->cmds, def->count) != FPIO_OK) {
            printf("  plan te groot voor FLIGHTPLAN_MAX_CMDS (%d)\n", FLIGHTPLAN_MAX_CMDS);
            continue;
        }
        if (FlightPlanTool_WritePlan(&g_work, def->bestand) == FPIO_OK) {
            ok++;
        }
    }

    printf("flightplan_tool: %d van de %u plannen staan op de kaart\n",
           ok, (unsigned)PLANNEN_COUNT);

    SDCard_Unmount();
    return ok;
}

/* ==========================================================================
 * Uitlezen
 * ========================================================================== */

FPIO_Status_t FlightPlanTool_Dump(const char *filename, FlightPlan_t *plan_out)
{
    if (SDCard_Mount() != SDCARD_OK) {
        printf("flightplan_tool: kon de SD-kaart niet mounten\n");
        return FPIO_ERR_SD;
    }

    FPIO_Status_t res = FlightPlanIO_Load(&g_work, filename);
    if (res != FPIO_OK) {
        printf("flightplan_tool: %s inlezen mislukt: %s\n", filename, FPIO_StatusStr(res));
        SDCard_Unmount();
        return res;
    }

    FlightPlan_Print(&g_work, filename);

    char err[96];
    FPIO_Status_t val = FlightPlan_Validate(&g_work, err, sizeof(err));
    if (val != FPIO_OK) {
        printf("flightplan_tool: LET OP, dit plan is niet geldig: %s\n", err);
    } else {
        printf("flightplan_tool: plan is geldig\n");
    }

    if (plan_out != NULL) {
        *plan_out = g_work;
    }

    SDCard_Unmount();
    return val;
}

FPIO_Status_t FlightPlanTool_DumpRaw(const char *filename)
{
    static uint8_t raw[FPIO_MAX_FILE_SIZE];
    uint32_t bytes_read = 0;

    if (SDCard_Mount() != SDCARD_OK) {
        printf("flightplan_tool: kon de SD-kaart niet mounten\n");
        return FPIO_ERR_SD;
    }

    if (SDCard_ReadFile(filename, raw, sizeof(raw) - 1, &bytes_read) != SDCARD_OK) {
        printf("flightplan_tool: %s niet gevonden of onleesbaar\n", filename);
        SDCard_Unmount();
        return FPIO_ERR_SD;
    }
    raw[bytes_read] = '\0';

    printf("---- ruwe inhoud van %s (%lu bytes) ----\n",
           filename, (unsigned long)bytes_read);
    printf("%s", (char *)raw);
    printf("\n---------------------------------------\n");

    SDCard_Unmount();
    return FPIO_OK;
}

void FlightPlanTool_ListBuiltin(void)
{
    printf("flightplan_tool: plannen in de firmware:\n");
    for (uint16_t i = 0; i < PLANNEN_COUNT; i++) {
        printf("  %u: %-14s -> %-16s (%u commando's)\n",
               (unsigned)(i + 1), g_plannen[i].naam, g_plannen[i].bestand,
               (unsigned)g_plannen[i].count);
    }
}
