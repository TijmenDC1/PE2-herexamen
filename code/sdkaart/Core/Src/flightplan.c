/*
 * flightplan.c
 *
 *  Created on: 15 jul 2026
 *      Author: tijme
 *
 *Het vluchtplan van de SD-kaart lezen, parsen, tonen en controleren.
 *
 * De kern is de tabel CMDS hieronder: daar staat één keer welke commando's er
 * bestaan, hoe ze heten en hoeveel parameters ze hebben. Parsen, printen én
 * wegschrijven
 */


#include "flightplan.h"
#include "sdcard.h"

#include <stdio.h>
#include <string.h>

/* De twee grote geheugenblokken van de hele SD-keten, allebei één keer. */
FlightPlan_t g_flightplan;
uint8_t      g_fp_buf[FLIGHTPLAN_FILE_MAX];

/* ==========================================================================
 * De commandotabel
 * ========================================================================== */

static const struct {
    const char     *naam;
    FlightCmdType_t type;
    uint8_t         params;
} CMDS[] = {
    { "RelativeHeight", CMD_RELATIVE_HEIGHT, 1 }, //gaat niet zonder bmp
    { "AbsoluteHeight", CMD_ABSOLUTE_HEIGHT, 1 }, //ook niet
    { "Hover",          CMD_HOVER,           1 },  //ook niet
    { "Throttle",       CMD_THROTTLE,        2 },
    { "Move",           CMD_MOVE,            3 },
    { "Left",           CMD_LEFT,            1 },
    { "Right",          CMD_RIGHT,           1 },
    { "Land",           CMD_LAND,            0 }, //ook niet
};
#define CMDS_N  ((uint8_t)(sizeof(CMDS) / sizeof(CMDS[0])))

const char *FlightPlan_CmdName(FlightCmdType_t t)
{
    for (uint8_t i = 0; i < CMDS_N; i++) {
        if (CMDS[i].type == t) return CMDS[i].naam;
    }
    return "Unknown";
}

uint8_t FlightPlan_CmdParams(FlightCmdType_t t)
{
    for (uint8_t i = 0; i < CMDS_N; i++) {
        if (CMDS[i].type == t) return CMDS[i].params;
    }
    return 0;
}

const char *FP_StatusStr(FP_Status_t s)
{
    switch (s) {
    case FP_OK:          return "ok";
    case FP_ERR_SD:      return "SD-kaartfout (mounten/openen/lezen/schrijven)";
    case FP_ERR_EMPTY:   return "geen geldige commando's in het bestand";
    case FP_ERR_TOO_BIG: return "plan te groot voor de buffer";
    case FP_ERR_INVALID: return "plan afgekeurd of anders teruggelezen";
    default:             return "onbekende fout";
    }
}

/* ==========================================================================
 * Parsen
 * ========================================================================== */

uint8_t FlightPlan_ParseLine(const char *line, FlightCmd_t *cmd)
{
    cmd->type     = CMD_UNKNOWN;
    cmd->param[0] = 0.0f;
    cmd->param[1] = 0.0f;
    cmd->param[2] = 0.0f;

    if (line == NULL) return 0;
    while (*line == ' ' || *line == '\t') line++;        /* inspringen toegelaten */
    if (*line == '\0' || *line == '#') return 0;         /* lege regel of commentaar */

    char token[24];
    if (sscanf(line, "%23s", token) != 1) return 0;   /* lege of witte regel */

    for (uint8_t i = 0; i < CMDS_N; i++) {
        if (strcmp(token, CMDS[i].naam) != 0) continue;

        /* Alles achter het commando in één keer inlezen. sscanf geeft terug
         * hoeveel getallen er echt stonden; te weinig = fout formaat. */
        int n = sscanf(line, "%*s %f %f %f",
                       &cmd->param[0], &cmd->param[1], &cmd->param[2]);
        if (n < 0) n = 0;                              /* geen getallen (bv. Land) */

        if ((uint8_t)n < CMDS[i].params) {
            printf("flightplan: %s heeft %u getal(len) nodig, overgeslagen: %s\n",
                   CMDS[i].naam, (unsigned)CMDS[i].params, line);
            return 0;
        }
        cmd->type = CMDS[i].type;
        return 1;
    }

    printf("flightplan: onbekend commando overgeslagen: %s\n", line);
    return 0;
}

/* ==========================================================================
 * Inlezen van de kaart
 * ========================================================================== */

FP_Status_t FlightPlan_Load(const char *filename)
{
    FlightPlan_t *plan = &g_flightplan;
    plan->count   = 0;
    plan->current = 0;

    uint32_t bytes = 0;
    if (SDCard_ReadFile(filename, g_fp_buf, sizeof(g_fp_buf) - 1, &bytes) != SDCARD_OK) {
        printf("flightplan: %s niet gevonden of onleesbaar\n", filename);
        return FP_ERR_SD;
    }
    g_fp_buf[bytes] = '\0';                 /* nulterminatie, we lezen 'm als tekst */

    char  line[FLIGHTPLAN_LINE_MAX];
    char *cursor = (char *)g_fp_buf;

    while ((cursor = SDCard_ReadLine(cursor, line, sizeof(line))) != NULL) {
        if (plan->count >= FLIGHTPLAN_MAX_CMDS) {
            printf("flightplan: meer dan %d commando's, de rest is overgeslagen\n",
                   FLIGHTPLAN_MAX_CMDS);
            break;
        }
        FlightCmd_t cmd;
        if (FlightPlan_ParseLine(line, &cmd)) {
            plan->cmds[plan->count++] = cmd;
        }
    }

    printf("flightplan: %u commando's geladen uit %s (%lu bytes)\n",
           (unsigned)plan->count, filename, (unsigned long)bytes);

    return (plan->count > 0) ? FP_OK : FP_ERR_EMPTY;
}

FP_Status_t FlightPlan_Show(const char *filename, uint8_t toon_ruw)
{
    if (SDCard_Mount() != SDCARD_OK) {
        printf("flightplan: kon de SD-kaart niet mounten\n");
        return FP_ERR_SD;
    }

    /* Eerst de ruwe bytes, zodat je ziet dat er echt iets op de kaart staat
     * en niet alleen in RAM. Daarna leest Load dezelfde buffer opnieuw in. */
    if (toon_ruw) {
        uint32_t bytes = 0;
        if (SDCard_ReadFile(filename, g_fp_buf, sizeof(g_fp_buf) - 1, &bytes) == SDCARD_OK) {
            g_fp_buf[bytes] = '\0';
            printf("---- ruwe inhoud van %s (%lu bytes) ----\n",
                   filename, (unsigned long)bytes);
            printf("%s\n", (char *)g_fp_buf);
        }
    }

    FP_Status_t res = FlightPlan_Load(filename);
    if (res != FP_OK) {
        printf("flightplan: %s inlezen mislukt: %s\n", filename, FP_StatusStr(res));
        SDCard_Unmount();
        return res;
    }

    FlightPlan_Print(g_flightplan.cmds, g_flightplan.count, filename);

    char err[80];
    res = FlightPlan_Validate(g_flightplan.cmds, g_flightplan.count, err, sizeof(err));
    if (res != FP_OK) printf("flightplan: LET OP, dit plan is niet geldig: %s\n", err);
    else              printf("flightplan: plan is geldig\n");

    SDCard_Unmount();
    return res;
}

/* ==========================================================================
 * Doorlopen tijdens de vlucht
 * ========================================================================== */

void FlightPlan_Reset(FlightPlan_t *plan)
{
    plan->current = 0;                      /* om hetzelfde plan opnieuw te draaien */
}

uint8_t FlightPlan_HasNext(FlightPlan_t *plan)
{
    return (plan->current < plan->count) ? 1 : 0;
}

FlightCmd_t *FlightPlan_Next(FlightPlan_t *plan)
{
    if (!FlightPlan_HasNext(plan)) return NULL;
    return &plan->cmds[plan->current++];
}

/* ==========================================================================
 * Tonen
 * ========================================================================== */

uint16_t FlightPlan_FormatCmd(const FlightCmd_t *cmd, char *out, uint16_t out_len)
{
    if (out == NULL || out_len == 0) return 0;
    out[0] = '\0';

    int len = snprintf(out, out_len, "%s", FlightPlan_CmdName(cmd->type));
    if (len < 0 || (uint16_t)len >= out_len) return 0;

    uint8_t n = FlightPlan_CmdParams(cmd->type);
    for (uint8_t i = 0; i < n; i++) {
        /* %g laat onnodige nullen weg: 30.0 wordt "30", 1.5 blijft "1.5".
         * Past het getal er niet meer bij, dan kappen we netjes af. */
        uint16_t ruimte = (uint16_t)(out_len - len);
        int m = snprintf(&out[len], ruimte, " %g", (double)cmd->param[i]);
        if (m < 0 || (uint16_t)m >= ruimte) {
            out[len] = '\0';
            break;
        }
        len += m;
    }
    return (uint16_t)len;
}

uint32_t FlightPlan_TotalDurationMs(const FlightCmd_t *cmds, uint16_t count)
{
    uint32_t total = 0;
    for (uint16_t i = 0; i < count; i++) {
        /* bij Throttle staat de duur in param[1], bij de rest in param[0] */
        float ms = (cmds[i].type == CMD_THROTTLE) ? cmds[i].param[1] : cmds[i].param[0];
        switch (cmds[i].type) {
        case CMD_THROTTLE:
        case CMD_HOVER:
        case CMD_LEFT:
        case CMD_RIGHT:
            if (ms > 0.0f) total += (uint32_t)ms;
            break;
        default:
            break;
        }
    }
    return total;
}

void FlightPlan_Print(const FlightCmd_t *cmds, uint16_t count, const char *titel)
{
    char line[FLIGHTPLAN_LINE_MAX];

    printf("---- %s ----\n", (titel != NULL) ? titel : "vluchtplan");
    printf("  %u commando's\n", (unsigned)count);

    for (uint16_t i = 0; i < count; i++) {
        FlightPlan_FormatCmd(&cmds[i], line, sizeof(line));
        printf("  %2u: %s\n", (unsigned)(i + 1), line);
    }

    printf("  geplande duur: %lu ms\n",
           (unsigned long)FlightPlan_TotalDurationMs(cmds, count));
    printf("------------------------------\n");
}

/* ==========================================================================
 * Controleren, vóór er een motor draait
 * ========================================================================== */

#define FOUT(...)  do { if (err != NULL && err_len > 0) snprintf(err, err_len, __VA_ARGS__); } while (0)

FP_Status_t FlightPlan_Validate(const FlightCmd_t *cmds, uint16_t count,
                                char *err, uint16_t err_len)
{
    if (err != NULL && err_len > 0) err[0] = '\0';

    if (count == 0) {
        FOUT("het plan is leeg");
        return FP_ERR_EMPTY;
    }
    if (count > FLIGHTPLAN_MAX_CMDS) {
        FOUT("%u commando's, maximum is %d", (unsigned)count, FLIGHTPLAN_MAX_CMDS);
        return FP_ERR_TOO_BIG;
    }

    uint8_t land_gezien = 0;

    for (uint16_t i = 0; i < count; i++) {
        const FlightCmd_t *c = &cmds[i];
        unsigned           r = (unsigned)(i + 1);       /* regelnummer voor de mens */

        if (land_gezien) {
            FOUT("regel %u: commando na Land, dat wordt nooit uitgevoerd", r);
            return FP_ERR_INVALID;
        }

        switch (c->type) {

        /* De tests staan met opzet in de positieve vorm (moet TUSSEN x en y
         * liggen) i.p.v. "als het te groot is". Een kapot getal uit een
         * beschadigd bestand komt er als inf of nan uit, en die vallen dan
         * vanzelf buiten het bereik i.p.v. door de mazen te glippen. */
        case CMD_THROTTLE:
            if (!(c->param[0] >= 0.0f && c->param[0] <= 100.0f)) {
                FOUT("regel %u: Throttle %.1f%% ligt buiten 0..100", r, (double)c->param[0]);
                return FP_ERR_INVALID;
            }
            if (!(c->param[1] > 0.0f && c->param[1] <= FLIGHTPLAN_MAX_DUR_MS)) {
                FOUT("regel %u: Throttle heeft een duur tussen 1 en %.0f ms nodig",
                     r, (double)FLIGHTPLAN_MAX_DUR_MS);
                return FP_ERR_INVALID;
            }
            break;

        case CMD_HOVER:
        case CMD_LEFT:
        case CMD_RIGHT:
            if (!(c->param[0] > 0.0f && c->param[0] <= FLIGHTPLAN_MAX_DUR_MS)) {
                FOUT("regel %u: %s heeft een duur tussen 1 en %.0f ms nodig",
                     r, FlightPlan_CmdName(c->type), (double)FLIGHTPLAN_MAX_DUR_MS);
                return FP_ERR_INVALID;
            }
            break;

        case CMD_LAND:
            land_gezien = 1;
            break;

        case CMD_RELATIVE_HEIGHT:
        case CMD_ABSOLUTE_HEIGHT:
        case CMD_MOVE:
            /* Geldig formaat, maar flightcontrol.c slaat deze over zolang de
             * barometer en de GPS er niet zijn. Alleen melden dus. */
            printf("  let op: regel %u (%s) wordt overgeslagen, barometer/GPS nog niet actief\n",
                   r, FlightPlan_CmdName(c->type));
            break;

        case CMD_UNKNOWN:
        default:
            FOUT("regel %u: onbekend commando (type %d)", r, (int)c->type);
            return FP_ERR_INVALID;
        }
    }

    /* Waarschuwingen: geen reden om af te keuren, wel om te melden. */
    if (!land_gezien) {
        printf("  let op: geen Land op het einde. flightcontrol.c landt dan zelf,\n"
               "          maar zet hem er beter expliciet bij.\n");
    }

    uint32_t total_ms = FlightPlan_TotalDurationMs(cmds, count);
    if (total_ms > (uint32_t)(FAILSAFE_TIMEOUT * 1000.0f)) {
        printf("  let op: geplande duur is %lu ms, de failsafe kapt af na %lu ms\n",
               (unsigned long)total_ms, (unsigned long)(FAILSAFE_TIMEOUT * 1000.0f));
    }

    return FP_OK;
}
