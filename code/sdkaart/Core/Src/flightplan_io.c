/*
 * flightplan_io.c
 *
 *  Created on: 3 aug 2026
 *      Author: simon
 *
 * Zie flightplan_io.h voor de beschrijving van de module en van het binaire
 * formaat. Het parsen van een tekstregel gebeurt in flightplan.c
 * (FlightPlan_ParseLine), zodat schrijven en lezen gegarandeerd dezelfde
 * commandonamen gebruiken.
 */

#include "flightplan_io.h"
#include "sdcard.h"
#include "fc_config.h"

#include <string.h>
#include <stdio.h>

#define LINE_MAX_LEN  128

/* Eén gedeelde werkbuffer voor serialiseren en deserialiseren. Static, want
 * 4 kB op de stack van een taak is te veel. Niet herintredend, maar er is maar
 * één plek die vluchtplannen laadt. */
static uint8_t io_buf[FPIO_MAX_FILE_SIZE];

/* ==========================================================================
 * Kleine hulpfuncties
 * ========================================================================== */

const char *FPIO_StatusStr(FPIO_Status_t s)
{
    switch (s) {
    case FPIO_OK:          return "ok";
    case FPIO_ERR_SD:      return "SD-kaartfout (mounten/openen/lezen/schrijven)";
    case FPIO_ERR_MAGIC:   return "geen geldig FPL1-bestand";
    case FPIO_ERR_VERSION: return "onbekende formaatversie";
    case FPIO_ERR_CRC:     return "checksum klopt niet, bestand beschadigd";
    case FPIO_ERR_TOO_BIG: return "plan te groot voor de buffer";
    case FPIO_ERR_EMPTY:   return "plan bevat geen geldige commando's";
    case FPIO_ERR_INVALID: return "plan afgekeurd door de validatie";
    default:               return "onbekende fout";
    }
}

const char *FlightPlan_CmdName(FlightCmdType_t t)
{
    switch (t) {
    case CMD_RELATIVE_HEIGHT: return "RelativeHeight";
    case CMD_ABSOLUTE_HEIGHT: return "AbsoluteHeight";
    case CMD_HOVER:           return "Hover";
    case CMD_THROTTLE:        return "Throttle";
    case CMD_MOVE:            return "Move";
    case CMD_LEFT:            return "Left";
    case CMD_RIGHT:           return "Right";
    case CMD_LAND:            return "Land";
    default:                  return "Unknown";
    }
}

/* Aantal parameters dat bij een commando hoort. */
static uint8_t cmd_arity(FlightCmdType_t t)
{
    switch (t) {
    case CMD_RELATIVE_HEIGHT:
    case CMD_ABSOLUTE_HEIGHT:
    case CMD_HOVER:
    case CMD_LEFT:
    case CMD_RIGHT:           return 1;
    case CMD_THROTTLE:        return 2;
    case CMD_MOVE:            return 3;
    case CMD_LAND:            return 0;
    default:                  return 0;
    }
}

/* Print een float zonder onnodige nullen: 30.0 wordt "30", 1.5 wordt "1.5". */
static int fmt_num(char *dst, uint16_t n, float v)
{
    int len = snprintf(dst, n, "%.3f", (double)v);
    if (len < 0 || (uint16_t)len >= n) return (len < 0) ? 0 : (int)n - 1;

    /* trailing nullen en een kale punt weghalen */
    while (len > 0 && dst[len - 1] == '0') dst[--len] = '\0';
    if (len > 0 && dst[len - 1] == '.')    dst[--len] = '\0';
    return len;
}

uint16_t FPIO_Crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ==========================================================================
 * Een plan in RAM opbouwen
 * ========================================================================== */

void FlightPlan_Clear(FlightPlan_t *plan)
{
    plan->count   = 0;
    plan->current = 0;
}

FPIO_Status_t FlightPlan_Add(FlightPlan_t *plan, FlightCmdType_t type,
                             float p0, float p1, float p2)
{
    if (plan->count >= FLIGHTPLAN_MAX_CMDS) return FPIO_ERR_TOO_BIG;

    FlightCmd_t *c = &plan->cmds[plan->count];
    c->type     = type;
    c->param[0] = p0;
    c->param[1] = p1;
    c->param[2] = p2;
    plan->count++;
    return FPIO_OK;
}

FPIO_Status_t FlightPlan_SetFromArray(FlightPlan_t *plan,
                                      const FlightCmd_t *cmds, uint16_t count)
{
    if (count > FLIGHTPLAN_MAX_CMDS) return FPIO_ERR_TOO_BIG;

    FlightPlan_Clear(plan);
    for (uint16_t i = 0; i < count; i++) {
        plan->cmds[i] = cmds[i];
    }
    plan->count = count;
    return FPIO_OK;
}

/* ==========================================================================
 * Validatie
 * ========================================================================== */

uint32_t FlightPlan_TotalDurationMs(const FlightPlan_t *plan)
{
    uint32_t total = 0;
    for (uint16_t i = 0; i < plan->count; i++) {
        const FlightCmd_t *c = &plan->cmds[i];
        switch (c->type) {
        case CMD_THROTTLE: if (c->param[1] > 0.0f) total += (uint32_t)c->param[1]; break;
        case CMD_HOVER:
        case CMD_LEFT:
        case CMD_RIGHT:    if (c->param[0] > 0.0f) total += (uint32_t)c->param[0]; break;
        default: break;
        }
    }
    return total;
}

FPIO_Status_t FlightPlan_Validate(const FlightPlan_t *plan, char *err, uint16_t err_len)
{
    if (err != NULL && err_len > 0) err[0] = '\0';

    if (plan->count == 0) {
        if (err) snprintf(err, err_len, "het plan is leeg");
        return FPIO_ERR_EMPTY;
    }
    if (plan->count > FLIGHTPLAN_MAX_CMDS) {
        if (err) snprintf(err, err_len, "%u commando's, maximum is %d",
                          (unsigned)plan->count, FLIGHTPLAN_MAX_CMDS);
        return FPIO_ERR_TOO_BIG;
    }

    uint8_t land_gezien = 0;

    for (uint16_t i = 0; i < plan->count; i++) {
        const FlightCmd_t *c = &plan->cmds[i];

        if (land_gezien) {
            if (err) snprintf(err, err_len,
                              "regel %u: commando na Land, dat wordt nooit uitgevoerd",
                              (unsigned)(i + 1));
            return FPIO_ERR_INVALID;
        }

        switch (c->type) {

        case CMD_UNKNOWN:
            if (err) snprintf(err, err_len, "regel %u: onbekend commando", (unsigned)(i + 1));
            return FPIO_ERR_INVALID;

        case CMD_THROTTLE:
            if (c->param[0] < 0.0f || c->param[0] > 100.0f) {
                if (err) snprintf(err, err_len,
                                  "regel %u: Throttle %.1f%% ligt buiten 0..100",
                                  (unsigned)(i + 1), (double)c->param[0]);
                return FPIO_ERR_INVALID;
            }
            if (c->param[1] <= 0.0f) {
                if (err) snprintf(err, err_len,
                                  "regel %u: Throttle heeft een duur > 0 ms nodig",
                                  (unsigned)(i + 1));
                return FPIO_ERR_INVALID;
            }
            break;

        case CMD_HOVER:
        case CMD_LEFT:
        case CMD_RIGHT:
            if (c->param[0] <= 0.0f) {
                if (err) snprintf(err, err_len,
                                  "regel %u: %s heeft een duur > 0 ms nodig",
                                  (unsigned)(i + 1), FlightPlan_CmdName(c->type));
                return FPIO_ERR_INVALID;
            }
            break;

        case CMD_LAND:
            land_gezien = 1;
            break;

        case CMD_RELATIVE_HEIGHT:
        case CMD_ABSOLUTE_HEIGHT:
        case CMD_MOVE:
            /* Geldig formaat, maar flightcontrol.c slaat deze nog over zolang
             * barometer en GPS er niet zijn. Alleen melden. */
            printf("  let op: regel %u (%s) wordt overgeslagen, barometer/GPS nog niet actief\n",
                   (unsigned)(i + 1), FlightPlan_CmdName(c->type));
            break;

        default:
            if (err) snprintf(err, err_len, "regel %u: onbekend commandotype %d",
                              (unsigned)(i + 1), (int)c->type);
            return FPIO_ERR_INVALID;
        }
    }

    /* Waarschuwingen: geen reden om af te keuren, wel om te melden. */
    if (!land_gezien) {
        printf("  let op: geen Land op het einde. flightcontrol.c landt dan zelf,\n"
               "          maar zet hem er beter expliciet bij.\n");
    }

    uint32_t total_ms = FlightPlan_TotalDurationMs(plan);
    if (total_ms > (uint32_t)(FAILSAFE_TIMEOUT * 1000.0f)) {
        printf("  let op: geplande duur is %lu ms, de failsafe kapt af na %lu ms\n",
               (unsigned long)total_ms, (unsigned long)(FAILSAFE_TIMEOUT * 1000.0f));
    }

    return FPIO_OK;
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

    uint8_t n = cmd_arity(cmd->type);
    for (uint8_t i = 0; i < n; i++) {
        if ((uint16_t)len + 2 >= out_len) break;
        out[len++] = ' ';
        out[len]   = '\0';
        len += fmt_num(&out[len], (uint16_t)(out_len - len), cmd->param[i]);
    }
    return (uint16_t)len;
}

void FlightPlan_Print(const FlightPlan_t *plan, const char *titel)
{
    char line[LINE_MAX_LEN];

    printf("---- %s ----\n", (titel != NULL) ? titel : "vluchtplan");
    printf("  %u commando's, formaat: %s\n", (unsigned)plan->count,
           (FPIO_FORMAT == FPIO_FMT_BINARY) ? "binair (FPL1)" : "tekst");

    for (uint16_t i = 0; i < plan->count; i++) {
        FlightPlan_FormatCmd(&plan->cmds[i], line, sizeof(line));
        printf("  %2u: %s\n", (unsigned)(i + 1), line);
    }

    printf("  geplande duur: %lu ms\n", (unsigned long)FlightPlan_TotalDurationMs(plan));
    printf("------------------------------\n");
}

/* ==========================================================================
 * Serialiseren naar een buffer
 * ========================================================================== */

uint32_t FlightPlanIO_SerializeText(const FlightPlan_t *plan, uint8_t *buf, uint32_t buf_size)
{
    if (buf == NULL || buf_size == 0) return 0;

    char *dst = (char *)buf;
    uint32_t pos = 0;
    int n;

    n = snprintf(&dst[pos], buf_size - pos,
                 "# vluchtplan - automatisch weggeschreven door flightplan_io.c\n"
                 "# 1 commando per lijn, regels met # zijn commentaar\n"
                 "# formaat: zie flightplan.h\n"
                 "#\n");
    if (n < 0 || (uint32_t)n >= buf_size - pos) return 0;
    pos += (uint32_t)n;

    for (uint16_t i = 0; i < plan->count; i++) {
        char line[LINE_MAX_LEN];
        FlightPlan_FormatCmd(&plan->cmds[i], line, sizeof(line));

        n = snprintf(&dst[pos], buf_size - pos, "%s\n", line);
        if (n < 0 || (uint32_t)n >= buf_size - pos) return 0;   /* buffer vol */
        pos += (uint32_t)n;
    }

    return pos;
}

uint32_t FlightPlanIO_SerializeBinary(const FlightPlan_t *plan, uint8_t *buf, uint32_t buf_size)
{
    if (buf == NULL) return 0;

    uint32_t need = FPIO_BIN_HDR_SIZE + (uint32_t)plan->count * FPIO_BIN_REC_SIZE;
    if (need > buf_size) return 0;

    /* records eerst, zodat we de CRC kunnen berekenen voor we de header vullen */
    uint8_t *rec = &buf[FPIO_BIN_HDR_SIZE];
    for (uint16_t i = 0; i < plan->count; i++) {
        const FlightCmd_t *c = &plan->cmds[i];
        uint8_t *r = &rec[(uint32_t)i * FPIO_BIN_REC_SIZE];

        r[0] = (uint8_t)c->type;
        r[1] = 0;                                   /* gereserveerd */
        memcpy(&r[2],  &c->param[0], 4);
        memcpy(&r[6],  &c->param[1], 4);
        memcpy(&r[10], &c->param[2], 4);
    }

    uint16_t crc = FPIO_Crc16(rec, (uint32_t)plan->count * FPIO_BIN_REC_SIZE);

    buf[0] = FPIO_BIN_MAGIC0;
    buf[1] = FPIO_BIN_MAGIC1;
    buf[2] = FPIO_BIN_MAGIC2;
    buf[3] = FPIO_BIN_MAGIC3;
    buf[4] = (uint8_t)(FPIO_BIN_VERSION & 0xFF);
    buf[5] = (uint8_t)(FPIO_BIN_VERSION >> 8);
    buf[6] = (uint8_t)(plan->count & 0xFF);
    buf[7] = (uint8_t)(plan->count >> 8);
    buf[8] = (uint8_t)(FPIO_BIN_REC_SIZE & 0xFF);
    buf[9] = (uint8_t)(FPIO_BIN_REC_SIZE >> 8);
    buf[10] = (uint8_t)(crc & 0xFF);
    buf[11] = (uint8_t)(crc >> 8);

    return need;
}

FPIO_Status_t FlightPlanIO_DeserializeText(FlightPlan_t *plan, char *buf)
{
    FlightPlan_Clear(plan);

    char line[LINE_MAX_LEN];
    char *cursor = buf;

    while ((cursor = SDCard_ReadLine(cursor, line, sizeof(line))) != NULL) {
        if (plan->count >= FLIGHTPLAN_MAX_CMDS) {
            printf("flightplan_io: max %d commando's bereikt, rest overgeslagen\n",
                   FLIGHTPLAN_MAX_CMDS);
            break;
        }
        FlightCmd_t cmd;
        if (FlightPlan_ParseLine(line, &cmd)) {
            plan->cmds[plan->count++] = cmd;
        }
    }

    return (plan->count > 0) ? FPIO_OK : FPIO_ERR_EMPTY;
}

FPIO_Status_t FlightPlanIO_DeserializeBinary(FlightPlan_t *plan, const uint8_t *buf, uint32_t size)
{
    FlightPlan_Clear(plan);

    if (size < FPIO_BIN_HDR_SIZE) return FPIO_ERR_MAGIC;

    if (buf[0] != FPIO_BIN_MAGIC0 || buf[1] != FPIO_BIN_MAGIC1 ||
        buf[2] != FPIO_BIN_MAGIC2 || buf[3] != FPIO_BIN_MAGIC3) {
        return FPIO_ERR_MAGIC;
    }

    uint16_t version = (uint16_t)(buf[4] | ((uint16_t)buf[5] << 8));
    uint16_t count   = (uint16_t)(buf[6] | ((uint16_t)buf[7] << 8));
    uint16_t reclen  = (uint16_t)(buf[8] | ((uint16_t)buf[9] << 8));
    uint16_t crc_exp = (uint16_t)(buf[10] | ((uint16_t)buf[11] << 8));

    if (version != FPIO_BIN_VERSION || reclen != FPIO_BIN_REC_SIZE) return FPIO_ERR_VERSION;
    if (count > FLIGHTPLAN_MAX_CMDS) return FPIO_ERR_TOO_BIG;
    if (size < FPIO_BIN_HDR_SIZE + (uint32_t)count * reclen) return FPIO_ERR_CRC;

    const uint8_t *rec = &buf[FPIO_BIN_HDR_SIZE];
    if (FPIO_Crc16(rec, (uint32_t)count * reclen) != crc_exp) return FPIO_ERR_CRC;

    for (uint16_t i = 0; i < count; i++) {
        const uint8_t *r = &rec[(uint32_t)i * reclen];
        FlightCmd_t *c = &plan->cmds[i];

        c->type = (FlightCmdType_t)r[0];
        memcpy(&c->param[0], &r[2],  4);
        memcpy(&c->param[1], &r[6],  4);
        memcpy(&c->param[2], &r[10], 4);
    }
    plan->count = count;

    return (count > 0) ? FPIO_OK : FPIO_ERR_EMPTY;
}

/* ==========================================================================
 * Opslaan en laden op de SD-kaart
 * ========================================================================== */

FPIO_Status_t FlightPlanIO_SaveText(const FlightPlan_t *plan, const char *filename)
{
    uint32_t size = FlightPlanIO_SerializeText(plan, io_buf, sizeof(io_buf));
    if (size == 0) return FPIO_ERR_TOO_BIG;

    if (SDCard_WriteFile(filename, io_buf, size) != SDCARD_OK) return FPIO_ERR_SD;
    return FPIO_OK;
}

FPIO_Status_t FlightPlanIO_LoadText(FlightPlan_t *plan, const char *filename)
{
    uint32_t bytes_read = 0;

    if (SDCard_ReadFile(filename, io_buf, sizeof(io_buf) - 1, &bytes_read) != SDCARD_OK) {
        return FPIO_ERR_SD;
    }
    io_buf[bytes_read] = '\0';

    return FlightPlanIO_DeserializeText(plan, (char *)io_buf);
}

FPIO_Status_t FlightPlanIO_SaveBinary(const FlightPlan_t *plan, const char *filename)
{
    uint32_t size = FlightPlanIO_SerializeBinary(plan, io_buf, sizeof(io_buf));
    if (size == 0) return FPIO_ERR_TOO_BIG;

    if (SDCard_WriteFile(filename, io_buf, size) != SDCARD_OK) return FPIO_ERR_SD;
    return FPIO_OK;
}

FPIO_Status_t FlightPlanIO_LoadBinary(FlightPlan_t *plan, const char *filename)
{
    uint32_t bytes_read = 0;

    if (SDCard_ReadFile(filename, io_buf, sizeof(io_buf), &bytes_read) != SDCARD_OK) {
        return FPIO_ERR_SD;
    }
    return FlightPlanIO_DeserializeBinary(plan, io_buf, bytes_read);
}

FPIO_Status_t FlightPlanIO_Save(const FlightPlan_t *plan, const char *filename)
{
#if FPIO_FORMAT == FPIO_FMT_BINARY
    return FlightPlanIO_SaveBinary(plan, filename);
#else
    return FlightPlanIO_SaveText(plan, filename);
#endif
}

FPIO_Status_t FlightPlanIO_Load(FlightPlan_t *plan, const char *filename)
{
#if FPIO_FORMAT == FPIO_FMT_BINARY
    return FlightPlanIO_LoadBinary(plan, filename);
#else
    return FlightPlanIO_LoadText(plan, filename);
#endif
}

FPIO_Status_t FlightPlanIO_SaveVerified(const FlightPlan_t *plan, const char *filename)
{
    FPIO_Status_t res = FlightPlanIO_Save(plan, filename);
    if (res != FPIO_OK) return res;

    /* meteen teruglezen en vergelijken: staat er echt op de kaart wat we bedoelden? */
    static FlightPlan_t check;
    res = FlightPlanIO_Load(&check, filename);
    if (res != FPIO_OK) return res;

    if (check.count != plan->count) {
        printf("flightplan_io: teruggelezen %u van de %u commando's\n",
               (unsigned)check.count, (unsigned)plan->count);
        return FPIO_ERR_INVALID;
    }

    for (uint16_t i = 0; i < plan->count; i++) {
        if (check.cmds[i].type != plan->cmds[i].type) {
            printf("flightplan_io: commando %u verschilt na het teruglezen\n", (unsigned)(i + 1));
            return FPIO_ERR_INVALID;
        }
    }

    return FPIO_OK;
}
