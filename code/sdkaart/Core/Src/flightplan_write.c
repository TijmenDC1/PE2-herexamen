/*
 * flightplan_write.c
 *
 *  Created on: 3 aug 2026
 *      Author: simon
 *
 * Een vluchtplan op de SD-kaart zetten. Zie flightplan_write.h.
 *
 * De commando's komen uit main.c, dus hier staat geen enkel plan hardgecodeerd
 * en is er ook geen FlightPlan_t-werkgeheugen nodig: valideren en serialiseren
 * werken rechtstreeks op de array die je meegeeft.
 */

#include "flightplan_write.h"
#include "sdcard.h"

#include <stdio.h>
#include <string.h>

/* Zet de commando's om naar de tekst die op de kaart komt. Geeft het aantal
 * bytes terug, of 0 als het niet in de buffer past. */
static uint32_t serialiseer(const FlightCmd_t *cmds, uint16_t count, char *dst, uint32_t max)
{
    int n = snprintf(dst, max,
                     "# vluchtplan, automatisch weggeschreven door flightplan_write.c\n"
                     "# 1 commando per regel, regels met # zijn commentaar\n");
    if (n < 0 || (uint32_t)n >= max) return 0;

    uint32_t pos = (uint32_t)n;

    for (uint16_t i = 0; i < count; i++) {
        char regel[FLIGHTPLAN_LINE_MAX];
        FlightPlan_FormatCmd(&cmds[i], regel, sizeof(regel));

        n = snprintf(&dst[pos], max - pos, "%s\n", regel);
        if (n < 0 || (uint32_t)n >= max - pos) return 0;    /* buffer vol */
        pos += (uint32_t)n;
    }
    return pos;
}

FP_Status_t FlightPlan_Save(const FlightCmd_t *cmds, uint16_t count, const char *filename)
{
    char err[80];

    /* Eerst controleren. Een afgekeurd plan komt niet op de kaart, zodat er
     * nooit iets blijft staan dat de drone later zou proberen te vliegen. */
    FP_Status_t res = FlightPlan_Validate(cmds, count, err, sizeof(err));
    if (res != FP_OK) {
        printf("flightplan_write: AFGEKEURD: %s (%s)\n", err, FP_StatusStr(res));
        return res;
    }

    /* We gebruiken de gedeelde buffer in twee helften: vooraan wat we willen
     * schrijven, achteraan wat we daarna van de kaart terugkrijgen. Zo kunnen
     * we byte voor byte vergelijken zonder extra geheugen te reserveren. */
    const uint32_t helft = sizeof(g_fp_buf) / 2;

    uint32_t size = serialiseer(cmds, count, (char *)g_fp_buf, helft);
    if (size == 0) {
        printf("flightplan_write: plan past niet in de buffer (max %lu bytes)\n",
               (unsigned long)helft);
        return FP_ERR_TOO_BIG;
    }

    if (SDCard_WriteFile(filename, g_fp_buf, size) != SDCARD_OK) {
        printf("flightplan_write: schrijven van %s mislukt\n", filename);
        return FP_ERR_SD;
    }

    uint32_t terug = 0;
    if (SDCard_ReadFile(filename, &g_fp_buf[helft], helft, &terug) != SDCARD_OK) {
        printf("flightplan_write: %s teruglezen mislukt\n", filename);
        return FP_ERR_SD;
    }

    if (terug != size || memcmp(g_fp_buf, &g_fp_buf[helft], size) != 0) {
        printf("flightplan_write: %s staat er anders op dan bedoeld (%lu i.p.v. %lu bytes)\n",
               filename, (unsigned long)terug, (unsigned long)size);
        return FP_ERR_INVALID;
    }

    printf("flightplan_write: %s geschreven en byte voor byte teruggelezen "
           "(%u commando's, %lu bytes)\n",
           filename, (unsigned)count, (unsigned long)size);
    return FP_OK;
}

