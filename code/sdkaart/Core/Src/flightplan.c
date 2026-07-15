/*
 * flightplan.c
 *
 *  Created on: 15 jul 2026
 *      Author: tijme
 */

#include "flightplan.h"
#include "sdcard.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define FLIGHTPLAN_MAX_FILE_SIZE  4096
#define LINE_MAX_LEN              128

// Zet een tekstregel om naar een FlightCmd_t
// Retourneert 0 als onbekend commando (wordt overgeslagen)
static uint8_t parse_line(const char *line, FlightCmd_t *cmd)
{
    cmd->type = CMD_UNKNOWN;
    cmd->param[0] = 0;
    cmd->param[1] = 0;
    cmd->param[2] = 0;

    if (line == NULL || line[0] == '\0' || line[0] == '#') return 0;

    char token[32];
    if (sscanf(line, "%31s", token) != 1) return 0;

    if (strcmp(token, "RelativeHeight") == 0) {
        if (sscanf(line, "%*s %f", &cmd->param[0]) == 1) {
            cmd->type = CMD_RELATIVE_HEIGHT;
            return 1;
        }

    } else if (strcmp(token, "AbsoluteHeight") == 0) {
        if (sscanf(line, "%*s %f", &cmd->param[0]) == 1) {
            cmd->type = CMD_ABSOLUTE_HEIGHT;
            return 1;
        }

    } else if (strcmp(token, "Hover") == 0) {
        if (sscanf(line, "%*s %f", &cmd->param[0]) == 1) {
            cmd->type = CMD_HOVER;
            return 1;
        }

    } else if (strcmp(token, "Throttle") == 0) {
        if (sscanf(line, "%*s %f %f", &cmd->param[0], &cmd->param[1]) == 2) {
            cmd->type = CMD_THROTTLE;
            return 1;
        }

    } else if (strcmp(token, "Move") == 0) {
        if (sscanf(line, "%*s %f %f %f", &cmd->param[0], &cmd->param[1], &cmd->param[2]) == 3) {
            cmd->type = CMD_MOVE;
            return 1;
        }
    }

    //Onbekend of fout formaat → overslaan
    printf("flightplan: onbekend commando overgeslagen: %s\n", line);
    return 0;
}

int FlightPlan_Load(FlightPlan_t *plan, const char *filename)
{
    plan->count   = 0;
    plan->current = 0;

    //Lees het volledige bestand in één keer
    static uint8_t file_buf[FLIGHTPLAN_MAX_FILE_SIZE];
    uint32_t bytes_read = 0;

    SDCard_Status_t res = SDCard_ReadFile(filename, file_buf, sizeof(file_buf) - 1, &bytes_read);
    if (res != SDCARD_OK) {
        printf("flightplan: bestand niet gevonden: %s\n", filename);
        return -1;
    }
    file_buf[bytes_read] = '\0';

    // Parseer regel voor regel
    char line[LINE_MAX_LEN];
    char *cursor = (char *)file_buf;

    while ((cursor = SDCard_ReadLine(cursor, line, sizeof(line))) != NULL) {
        if (plan->count >= FLIGHTPLAN_MAX_CMDS) {
            printf("flightplan: max commando's bereikt (%d), rest overgeslagen\n", FLIGHTPLAN_MAX_CMDS);
            break;
        }
        FlightCmd_t cmd;
        if (parse_line(line, &cmd)) {
            plan->cmds[plan->count++] = cmd;
        }
    }

    printf("flightplan: %d commando's geladen uit %s\n", plan->count, filename);
    return plan->count;
}

uint8_t FlightPlan_HasNext(FlightPlan_t *plan)
{
    return plan->current < plan->count;
}

FlightCmd_t *FlightPlan_Next(FlightPlan_t *plan)
{
    if (!FlightPlan_HasNext(plan)) return NULL;
    return &plan->cmds[plan->current++];
}

void FlightPlan_Reset(FlightPlan_t *plan)
{
    plan->current = 0;
}
