/*
 * test_flightplan.c
 *
 * PC-test van de vluchtplan-keten, zonder STM32 en zonder SD-kaart.
 * De "kaart" is de map ./sdtest/ (zie stub/host_fatfs.c).
 *
 * Deel 1 is de demo: een plan wordt weggeschreven, teruggelezen en geprint.
 * Deel 2 zijn de controles: roundtrip, CRC, en of foute plannen afgekeurd worden.
 *
 * Bouwen en draaien: zie README.md in deze map, of gewoon `make run`.
 */

#include "flightplan.h"
#include "flightplan_io.h"
#include "sdcard.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

static int fails = 0;

#define CHECK(cond, msg)                                  \
    do {                                                  \
        if (!(cond)) { printf("  FAIL : %s\n", msg); fails++; } \
        else         { printf("  ok   : %s\n", msg); }     \
    } while (0)

/* Hetzelfde plan als plan_test1 in flightplan_tool.c, met een Move erbij om
 * ook kommagetallen door de molen te halen. */
static const FlightCmd_t demo[] = {
    { CMD_THROTTLE, { 30.0f, 2000.0f, 0.0f } },
    { CMD_THROTTLE, { 40.0f, 3000.0f, 0.0f } },
    { CMD_LEFT,     { 1000.0f, 0.0f, 0.0f } },
    { CMD_RIGHT,    { 1000.0f, 0.0f, 0.0f } },
    { CMD_MOVE,     { 1.5f, -0.25f, 0.125f } },
    { CMD_THROTTLE, { 35.0f, 1500.0f, 0.0f } },
    { CMD_LAND,     { 0.0f, 0.0f, 0.0f } },
};
#define DEMO_COUNT ((uint16_t)(sizeof(demo) / sizeof(demo[0])))

static int same(const FlightPlan_t *a, const FlightPlan_t *b)
{
    if (a->count != b->count) return 0;
    for (uint16_t i = 0; i < a->count; i++) {
        if (a->cmds[i].type != b->cmds[i].type) return 0;
        for (int p = 0; p < 3; p++) {
            if (fabsf(a->cmds[i].param[p] - b->cmds[i].param[p]) > 1e-4f) return 0;
        }
    }
    return 1;
}

static void toon_bestand(const char *naam)
{
    char pad[256];
    snprintf(pad, sizeof(pad), "sdtest/%s", naam);

    FILE *f = fopen(pad, "rb");
    if (f == NULL) { printf("  (%s bestaat niet)\n", naam); return; }

    unsigned char buf[4096];
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    buf[n] = '\0';
    fclose(f);

    printf("  ---- %s, %lu bytes ----\n", naam, (unsigned long)n);

    if (n >= 4 && memcmp(buf, "FPL1", 4) == 0) {
        /* binair: hexdump, want tekst uitprinten heeft hier geen zin */
        for (size_t i = 0; i < n; i += 16) {
            printf("  %04lx  ", (unsigned long)i);
            for (size_t j = 0; j < 16; j++) {
                if (i + j < n) printf("%02x ", buf[i + j]);
                else           printf("   ");
            }
            printf(" |");
            for (size_t j = 0; j < 16 && i + j < n; j++) {
                unsigned char c = buf[i + j];
                printf("%c", (c >= 32 && c < 127) ? c : '.');
            }
            printf("|\n");
        }
    } else {
        printf("%s", (char *)buf);
    }
    printf("  ------------------------\n");
}

int main(void)
{
    FlightPlan_t plan, terug;
    char err[96];

    printf("\n########  DEMO: schrijven -> uitlezen -> printen  ########\n");

    printf("\n[1] plan opbouwen in RAM\n");
    CHECK(FlightPlan_SetFromArray(&plan, demo, DEMO_COUNT) == FPIO_OK, "plan opgebouwd");
    CHECK(FlightPlan_Validate(&plan, err, sizeof(err)) == FPIO_OK, "validatie keurt het goed");

    printf("\n[2] wegschrijven naar de kaart\n");
    CHECK(SDCard_Mount() == SDCARD_OK, "kaart gemount");
    CHECK(FlightPlanIO_SaveVerified(&plan, "vlucht.txt") == FPIO_OK,
          "weggeschreven en meteen teruggelezen");

    printf("\n[3] wat er nu echt op de kaart staat\n");
    toon_bestand("vlucht.txt");

    printf("\n[4] terug inlezen en printen\n");
    CHECK(FlightPlanIO_Load(&terug, "vlucht.txt") == FPIO_OK, "ingelezen");
    FlightPlan_Print(&terug, "vlucht.txt");
    CHECK(same(&plan, &terug), "identiek aan wat we wegschreven");

    printf("\n\n########  CONTROLES  ########\n");

    printf("\n[5] duur en binair formaat\n");
    CHECK(FlightPlan_TotalDurationMs(&plan) == 8500, "geplande duur = 8500 ms");
    CHECK(FlightPlanIO_SaveBinary(&plan, "vlucht.bin") == FPIO_OK, "binair weggeschreven");
    CHECK(FlightPlanIO_LoadBinary(&terug, "vlucht.bin") == FPIO_OK, "binair ingelezen");
    CHECK(same(&plan, &terug), "binair: identiek heen en terug");
    {
        FILE *f = fopen("sdtest/vlucht.bin", "rb");
        fseek(f, 0, SEEK_END);
        long sz = ftell(f);
        fclose(f);
        printf("         binair = %ld bytes, tekst was een stuk groter\n", sz);
        CHECK(sz == (long)(FPIO_BIN_HDR_SIZE + DEMO_COUNT * FPIO_BIN_REC_SIZE),
              "bestandsgrootte klopt met het FPL1-formaat");
    }

    printf("\n[6] CRC vangt een beschadigd bestand op\n");
    {
        FILE *f = fopen("sdtest/vlucht.bin", "r+b");
        unsigned char c;
        fseek(f, 20, SEEK_SET); fread(&c, 1, 1, f);
        c ^= 0x40;                                  /* één bit omklappen */
        fseek(f, 20, SEEK_SET); fwrite(&c, 1, 1, f);
        fclose(f);
    }
    CHECK(FlightPlanIO_LoadBinary(&terug, "vlucht.bin") == FPIO_ERR_CRC,
          "omgeklapte bit -> FPIO_ERR_CRC");

    /* expliciet een tekstbestand maken: FPIO_FORMAT kan hier op binair staan,
     * dus vlucht.txt is niet gegarandeerd tekst. */
    FlightPlan_SetFromArray(&terug, demo, DEMO_COUNT);
    CHECK(FlightPlanIO_SaveText(&terug, "alleen.txt") == FPIO_OK, "los tekstbestand gemaakt");
    CHECK(FlightPlanIO_LoadBinary(&terug, "alleen.txt") == FPIO_ERR_MAGIC,
          "tekstbestand als binair lezen -> FPIO_ERR_MAGIC");

    printf("\n[7] foute plannen worden afgekeurd\n");
    FlightPlan_Clear(&plan);
    FlightPlan_Add(&plan, CMD_THROTTLE, 130.0f, 1000.0f, 0.0f);
    CHECK(FlightPlan_Validate(&plan, err, sizeof(err)) == FPIO_ERR_INVALID, "throttle 130 %");
    printf("         -> %s\n", err);

    FlightPlan_Clear(&plan);
    FlightPlan_Add(&plan, CMD_HOVER, 0.0f, 0.0f, 0.0f);
    CHECK(FlightPlan_Validate(&plan, err, sizeof(err)) == FPIO_ERR_INVALID, "Hover van 0 ms");
    printf("         -> %s\n", err);

    FlightPlan_Clear(&plan);
    FlightPlan_Add(&plan, CMD_LAND, 0, 0, 0);
    FlightPlan_Add(&plan, CMD_THROTTLE, 30.0f, 1000.0f, 0.0f);
    CHECK(FlightPlan_Validate(&plan, err, sizeof(err)) == FPIO_ERR_INVALID, "commando na Land");
    printf("         -> %s\n", err);

    FlightPlan_Clear(&plan);
    CHECK(FlightPlan_Validate(&plan, err, sizeof(err)) == FPIO_ERR_EMPTY, "leeg plan");

    printf("\n[8] plan vol laten lopen\n");
    FlightPlan_Clear(&plan);
    FPIO_Status_t r = FPIO_OK;
    for (int i = 0; i < FLIGHTPLAN_MAX_CMDS + 5 && r == FPIO_OK; i++) {
        r = FlightPlan_Add(&plan, CMD_HOVER, 100.0f, 0.0f, 0.0f);
    }
    CHECK(r == FPIO_ERR_TOO_BIG && plan.count == FLIGHTPLAN_MAX_CMDS,
          "stopt netjes op FLIGHTPLAN_MAX_CMDS");

    printf("\n[9] handgeschreven bestand met commentaar en rommel\n");
    {
        FILE *f = fopen("sdtest/hand.txt", "wb");
        fputs("# met de hand getypt\n"
              "\n"
              "Throttle 25 1200\n"
              "BlablaCommando 5\n"
              "   \n"
              "Land\n", f);
        fclose(f);
    }
    CHECK(FlightPlanIO_LoadText(&plan, "hand.txt") == FPIO_OK, "ingelezen");
    CHECK(plan.count == 2 &&
          plan.cmds[0].type == CMD_THROTTLE &&
          plan.cmds[1].type == CMD_LAND,
          "commentaar, lege regels en onzin overgeslagen");

    SDCard_Unmount();

    printf("\n=========================================\n");
    printf("%s (%d fouten)\n", fails ? "TESTS GEFAALD" : "ALLE TESTS GESLAAGD", fails);
    printf("De 'SD-kaart' staat in ./sdtest/, kijk gerust in de bestanden.\n\n");
    return fails ? 1 : 0;
}
