/*
 * BMM350v2.c
 *
 * From-scratch driver voor de BMM350 (Bosch magnetometer) over I2C1.
 * Zie BMM350v2.h voor het verschil met de bestaande, niet-gebruikte BMM350.c.
 */

#include "BMM350v2.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

#ifndef PI
#define PI 3.14159265358979323846
#endif

#define I2C_TIMEOUT_MS   100

/* De BMM350 kan op 4 adressen zitten, afhankelijk van hoe SDO/CSB op het board
 * bedraad zijn: 0x14, 0x15, 0x16 of 0x17. We proberen ze alle vier bij init
 * i.p.v. 0x14 hard te veronderstellen. HAL verwacht het 8-bit adres (7-bit << 1). */
static const uint8_t bmm350_addr_candidates[] = { 0x14, 0x15, 0x16, 0x17 };

/* Het 7-bit adres waarop de sensor daadwerkelijk geantwoord heeft; wordt door
 * BMM350v2_Init() ingevuld en daarna gebruikt door alle lees-/schrijfacties. */
static uint8_t bmm350_addr7 = 0x14;
#define BMM350_I2C_ADDR       ((uint16_t)(bmm350_addr7 << 1))

/* --- registeradressen --- */
#define REG_CHIP_ID           0x00
#define REG_PMU_CMD_AGGR_SET  0x04
#define REG_PMU_CMD_AXIS_EN   0x05
#define REG_PMU_CMD           0x06
#define REG_I2C_WDT_SET       0x0A
#define REG_INT_CTRL          0x2E
#define REG_INT_STATUS        0x30
#define REG_MAG_X_XLSB        0x31 /* X/Y/Z: 3 bytes elk, vanaf hier aaneengesloten */
#define REG_CMD               0x7E

#define CMD_SOFTRESET         0xB6

#define CHIP_ID_EXPECTED      0x33

#define INT_STATUS_DRDY_MASK  0x01

/* Standaardwaarde voor België; pas aan naar je eigen locatie
 * (https://www.magnetic-declination.com/) voor een correcte kompashoek t.o.v.
 * het geografische i.p.v. het magnetische noorden. */
#define MAGNETIC_DECLINATION_DEG 3.0f

/* --- lage-niveau I2C-toegang, elke stap checkt zijn eigen HAL-status --- */

static HAL_StatusTypeDef BMM350v2_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t frame[2] = { reg, value };
    return HAL_I2C_Master_Transmit(&hi2c1, BMM350_I2C_ADDR, frame, sizeof(frame), I2C_TIMEOUT_MS);
}

/* De BMM350 stuurt bij elke I2C-leesactie 2 dummy-bytes vóór de echte data
 * (gedocumenteerd gedrag van deze chip, geen protocolfout hier).
 *
 * We gebruiken HAL_I2C_Mem_Read i.p.v. een losse Transmit + Receive: dat doet
 * een repeated start i.p.v. een STOP tussen het registeradres en de data, wat
 * is wat de BMM350 verwacht. Met een STOP ertussen vergeet de chip welk
 * register je wou lezen. */
static HAL_StatusTypeDef BMM350v2_ReadReg(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t buf[2 + 32]; /* ruim genoeg voor de 9 bytes (X/Y/Z) die we hier max. lezen */
    HAL_StatusTypeDef status;

    if (len > sizeof(buf) - 2) {
        return HAL_ERROR;
    }

    status = HAL_I2C_Mem_Read(&hi2c1, BMM350_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                              buf, (uint16_t)(len + 2), I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }

    for (uint16_t i = 0; i < len; i++) {
        data[i] = buf[i + 2];
    }
    return HAL_OK;
}

/* Vertaalt de HAL-status + ErrorCode naar iets leesbaars, zodat je in de
 * terminal ziet of het een NACK (niemand antwoordt), een bus-fout (SDA/SCL
 * blijven laag, meestal ontbrekende pull-ups) of een timeout is. */
static void BMM350v2_PrintI2CError(const char *what, HAL_StatusTypeDef status)
{
    uint32_t err = HAL_I2C_GetError(&hi2c1);
    printf("BMM350v2: %s -> HAL status %d, ErrorCode 0x%02lX", what, (int)status, (unsigned long)err);
    if (err & HAL_I2C_ERROR_AF)      printf(" [AF: geen ACK, niets op dit adres]");
    if (err & HAL_I2C_ERROR_BERR)    printf(" [BERR: busfout]");
    if (err & HAL_I2C_ERROR_ARLO)    printf(" [ARLO: arbitration lost]");
    if (err & HAL_I2C_ERROR_TIMEOUT) printf(" [TIMEOUT: bus hangt, check pull-ups/voeding]");
    printf("\n");
}

void BMM350v2_ScanBus(void)
{
    printf("BMM350v2: I2C1-scan van 0x08 t/m 0x77...\n");
    uint8_t found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 2, 5) == HAL_OK) {
            printf("BMM350v2:   gevonden op 0x%02X\n", addr);
            found++;
        }
    }
    if (found == 0) {
        printf("BMM350v2:   niets gevonden. Check voeding (VDD/VDDIO), GND, "
               "pull-ups op PB8/PB9 en of SDA/SCL niet verwisseld zijn.\n");
    }
}

uint8_t BMM350v2_IsDataReady(void)
{
    uint8_t status_reg = 0;
    if (BMM350v2_ReadReg(REG_INT_STATUS, &status_reg, 1) != HAL_OK) {
        return 0;
    }
    return (status_reg & INT_STATUS_DRDY_MASK) ? 1 : 0;
}

BMM350v2_Status BMM350v2_Init(void)
{
    uint8_t chip_id = 0;
    HAL_StatusTypeDef last_status = HAL_ERROR;
    uint8_t addr_found = 0;

    /* De BMM350 heeft na power-on ~3ms nodig voor hij op I2C antwoordt; ruim
     * nemen, want de voedingsrail van het board komt mogelijk later op gang. */
    HAL_Delay(20);

    /* Soft reset via het CMD-register, zodat we niet afhangen van in welke
     * toestand de chip stond na een warme reset van de MCU. */
    (void)BMM350v2_WriteReg(REG_CMD, CMD_SOFTRESET);
    HAL_Delay(24); /* datasheet: soft reset duurt max. 24ms */

    /* Probeer alle 4 mogelijke adressen; welke het is hangt af van de SDO/CSB-
     * bedrading op dit board. */
    for (uint8_t i = 0; i < sizeof(bmm350_addr_candidates); i++) {
        bmm350_addr7 = bmm350_addr_candidates[i];
        last_status = BMM350v2_ReadReg(REG_CHIP_ID, &chip_id, 1);
        if (last_status == HAL_OK) {
            printf("BMM350v2: antwoord op adres 0x%02X, chip-ID 0x%02X\n", bmm350_addr7, chip_id);
            addr_found = 1;
            break;
        }
    }

    if (!addr_found) {
        /* Scan + checklist maar 1x printen, anders spamt de retry-loop de terminal vol. */
        static uint8_t diag_printed = 0;
        if (!diag_printed) {
            diag_printed = 1;
            BMM350v2_PrintI2CError("geen antwoord op 0x14/0x15/0x16/0x17", last_status);
            BMM350v2_ScanBus();
            printf("BMM350v2: de BMM350 kan ALLEEN I2C/I3C (geen SPI), dus als hij\n"
                   "          hier niet antwoordt is het voeding of bedrading of de chip zelf:\n"
                   "          1) meet VDD en VDDIO RECHTSTREEKS op de BMM350 (niet op de regulator)\n"
                   "          2) meet doorverbinding PB8->SCL-pad en PB9->SDA-pad van de chip\n"
                   "          3) meet GND-pad van de chip naar massa\n"
                   "          Antwoordt er wel iets anders in de scan hierboven, dan is de bus zelf OK.\n");
        }
        return BMM350V2_ERR_I2C;
    }

    if (chip_id != CHIP_ID_EXPECTED) {
        printf("BMM350v2: verkeerde chip-ID (0x%02X, verwacht 0x%02X)\n", chip_id, CHIP_ID_EXPECTED);
        BMM350v2_ScanBus();
        return BMM350V2_ERR_ID;
    }
    printf("BMM350v2: chip-ID OK (0x%02X) op adres 0x%02X\n", chip_id, bmm350_addr7);

    uint8_t config_ok = 1;
    /* ODR 50Hz, gemiddelde van 4 metingen (zelfde instelling als datasheet-
     * voorbeeld voor een drone-toepassing). */
    config_ok &= (BMM350v2_WriteReg(REG_PMU_CMD_AGGR_SET, 0x25) == HAL_OK);
    /* alle 3 assen aan */
    config_ok &= (BMM350v2_WriteReg(REG_PMU_CMD_AXIS_EN, 0x07) == HAL_OK);
    /* i2c-watchdog op de lange timeout (40.96ms) */
    config_ok &= (BMM350v2_WriteReg(REG_I2C_WDT_SET, 0x03) == HAL_OK);
    /* data-ready interrupt aan, actief-hoog, naar de pin */
    config_ok &= (BMM350v2_WriteReg(REG_INT_CTRL, 0x8E) == HAL_OK);
    /* van suspend naar normal mode */
    config_ok &= (BMM350v2_WriteReg(REG_PMU_CMD, 0x01) == HAL_OK);
    if (!config_ok) {
        BMM350v2_PrintI2CError("I2C-fout bij configureren", HAL_ERROR);
        return BMM350V2_ERR_I2C;
    }
    HAL_Delay(5); /* PMU-commando's hebben even tijd nodig om te verwerken */

    printf("BMM350v2: init geslaagd\n");
    return BMM350V2_OK;
}

BMM350v2_Status BMM350v2_ReadRaw(int32_t *x, int32_t *y, int32_t *z)
{
    uint8_t data[9]; /* 3 bytes per as, X/Y/Z aaneengesloten vanaf REG_MAG_X_XLSB */

    if (BMM350v2_ReadReg(REG_MAG_X_XLSB, data, sizeof(data)) != HAL_OK) {
        return BMM350V2_ERR_I2C;
    }

    /* 24-bit signed waarden, LSB eerst; bit 20 is het tekenbit dat we uitbreiden
     * naar de volle 32-bit int (2-complement sign extend). */
    int32_t raw_x = (int32_t)((uint32_t)data[2] << 16 | (uint32_t)data[1] << 8 | data[0]);
    if (raw_x & 0x00100000) raw_x |= 0xFFF00000;

    int32_t raw_y = (int32_t)((uint32_t)data[5] << 16 | (uint32_t)data[4] << 8 | data[3]);
    if (raw_y & 0x00100000) raw_y |= 0xFFF00000;

    int32_t raw_z = (int32_t)((uint32_t)data[8] << 16 | (uint32_t)data[7] << 8 | data[6]);
    if (raw_z & 0x00100000) raw_z |= 0xFFF00000;

    *x = raw_x;
    *y = raw_y;
    *z = raw_z;
    return BMM350V2_OK;
}

void BMM350v2_CalibrateReset(BMM350v2_Calib *calib)
{
    calib->min_x = calib->min_y = calib->min_z = 1e9f;
    calib->max_x = calib->max_y = calib->max_z = -1e9f;
    calib->offset_x = calib->offset_y = calib->offset_z = 0.0f;
}

void BMM350v2_CalibrateSample(BMM350v2_Calib *calib, int32_t raw_x, int32_t raw_y, int32_t raw_z)
{
    if ((float)raw_x < calib->min_x) calib->min_x = (float)raw_x;
    if ((float)raw_x > calib->max_x) calib->max_x = (float)raw_x;
    if ((float)raw_y < calib->min_y) calib->min_y = (float)raw_y;
    if ((float)raw_y > calib->max_y) calib->max_y = (float)raw_y;
    if ((float)raw_z < calib->min_z) calib->min_z = (float)raw_z;
    if ((float)raw_z > calib->max_z) calib->max_z = (float)raw_z;
}

void BMM350v2_CalibrateFinish(BMM350v2_Calib *calib)
{
    calib->offset_x = (calib->max_x + calib->min_x) / 2.0f;
    calib->offset_y = (calib->max_y + calib->min_y) / 2.0f;
    calib->offset_z = (calib->max_z + calib->min_z) / 2.0f;
}

float BMM350v2_Heading(float cal_x, float cal_y)
{
    float heading = atan2f(cal_y, cal_x) * (180.0f / (float)PI);
    if (heading < 0.0f) {
        heading += 360.0f;
    }
    heading += MAGNETIC_DECLINATION_DEG;
    if (heading >= 360.0f) heading -= 360.0f;
    if (heading < 0.0f)    heading += 360.0f;
    return heading;
}

float BMM350v2_TiltCompensatedHeading(int32_t raw_x, int32_t raw_y, int32_t raw_z,
                                       const BMM350v2_Calib *calib,
                                       float roll_deg, float pitch_deg)
{
    float x = (float)raw_x - calib->offset_x;
    float y = (float)raw_y - calib->offset_y;
    float z = (float)raw_z - calib->offset_z;

    /* roll/pitch komen in graden binnen (zelfde conventie als attitude.c) -
     * de trig-functies hieronder verwachten radialen. */
    float roll  = roll_deg  * ((float)PI / 180.0f);
    float pitch = pitch_deg * ((float)PI / 180.0f);

    float x_comp = x * cosf(pitch) + y * sinf(roll) * sinf(pitch) + z * cosf(roll) * sinf(pitch);
    float y_comp = y * cosf(roll) - z * sinf(roll);

    return BMM350v2_Heading(x_comp, y_comp);
}
