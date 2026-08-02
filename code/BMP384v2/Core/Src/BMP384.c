/*
* BMP384.c
*
* Created on: 31 jun 2026
* Author: tijme
*/

#include "BMP384.h"
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

extern SPI_HandleTypeDef hspi4;

#define SPI_TIMEOUT_MS  100

//registeradressen
#define REG_CHIP_ID         0x00
#define REG_ERR             0x02
#define REG_STATUS          0x03
#define REG_DATA_PRESS_0    0x04 //druk: 3 bytes vanaf hier 24-bit
#define REG_DATA_TEMP_0     0x07 //temp: 3 bytes vanaf hier 24-bit
#define REG_SENSORTIME      0x0C //24-bit interne teller, loopt in normal mode
#define REG_EVENT           0x10
#define REG_INT_STATUS      0x11
#define REG_INT_CTRL        0x19
#define REG_IF_CONF         0x1A
#define REG_PWR_CTRL        0x1B
#define REG_OSR             0x1C
#define REG_ODR             0x1D
#define REG_CONFIG          0x1F
#define REG_CALIB_START     0x31 //21 bytes NVM-kalibratiedata vanaf hier
#define REG_CMD             0x7E

#define CHIP_ID_EXPECTED    0x50
#define CMD_SOFTRESET       0xB6

#define STATUS_CMD_RDY_MASK 0x10
#define STATUS_DRDY_MASK    0x60

/* --- lage-niveau SPI-toegang ---
 *
 * BELANGRIJK: een BMP3xx-leescyclus is 1 adresbyte + 1 dummybyte + N databytes,
 * en dat moet EEN ononderbroken SPI-transactie zijn. De oude versie deed dit met
 * 3 losse HAL-calls (Transmit + Receive + Receive). Op de STM32F7 laat
 * HAL_SPI_Transmit ontvangen bytes achter in de RX-FIFO en zet het de OVR-vlag;
 * de daaropvolgende Receive levert dan verschoven of nul-data. Daarom nu alles
 * in 1 HAL_SPI_TransmitReceive met een gedeelde buffer. */

#define BMP_MAX_XFER    24  /* adres + dummy + max 21 kalibratiebytes + marge */

static HAL_StatusTypeDef BMP384_ReadReg(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx[BMP_MAX_XFER];
    uint8_t rx[BMP_MAX_XFER];
    HAL_StatusTypeDef status;

    if (len + 2u > BMP_MAX_XFER) {
        return HAL_ERROR;
    }

    memset(tx, 0xFF, sizeof(tx));
    tx[0] = (uint8_t)(reg | 0x80);   /* bit7 = 1 -> lezen */

    HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(&hspi4, tx, rx, (uint16_t)(len + 2u), SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);

    if (status == HAL_OK) {
        /* rx[0] = rommel tijdens adresbyte, rx[1] = dummybyte, rx[2..] = data */
        memcpy(data, &rx[2], len);
    }
    return status;
}

static HAL_StatusTypeDef BMP384_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), value };
    uint8_t rx[2];
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(&hspi4, tx, rx, 2, SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);

    return status;
}

/* Schrijft een register en leest het terug om te controleren dat de sensor het
 * echt overgenomen heeft. Zo zie je meteen of een instelling niet aankomt. */
static uint8_t BMP384_WriteVerify(uint8_t reg, uint8_t value)
{
    uint8_t readback = 0;

    if (BMP384_WriteReg(reg, value) != HAL_OK) {
        printf("BMP384: SPI-fout bij schrijven reg 0x%02X\n", reg);
        return 0;
    }
    HAL_Delay(2);
    if (BMP384_ReadReg(reg, &readback, 1) != HAL_OK) {
        printf("BMP384: SPI-fout bij teruglezen reg 0x%02X\n", reg);
        return 0;
    }
    if (readback != value) {
        printf("BMP384: reg 0x%02X = 0x%02X, verwacht 0x%02X\n", reg, readback, value);
        return 0;
    }
    return 1;
}

//datasheet 9.1

static HAL_StatusTypeDef BMP384_ReadCalibration(BMP384_Calib *calib)
{
    uint8_t raw[21];
    HAL_StatusTypeDef status = BMP384_ReadReg(REG_CALIB_START, raw, sizeof(raw));
    if (status != HAL_OK) {
        return status;
    }

    calib->t1 = (float)((uint16_t)(raw[1] << 8 | raw[0])) / powf(2.0f, -8.0f);
    calib->t2 = (float)((uint16_t)(raw[3] << 8 | raw[2])) / powf(2.0f, 30.0f);
    calib->t3 = (float)((int8_t)raw[4]) / powf(2.0f, 48.0f);

    calib->p1  = ((float)((int16_t)(raw[6]  << 8 | raw[5]))  - powf(2.0f, 14.0f)) / powf(2.0f, 20.0f);
    calib->p2  = ((float)((int16_t)(raw[8]  << 8 | raw[7]))  - powf(2.0f, 14.0f)) / powf(2.0f, 29.0f);
    calib->p3  = (float)((int8_t)raw[9])  / powf(2.0f, 32.0f);
    calib->p4  = (float)((int8_t)raw[10]) / powf(2.0f, 37.0f);
    calib->p5  = (float)((uint16_t)(raw[12] << 8 | raw[11])) / powf(2.0f, -3.0f);
    calib->p6  = (float)((uint16_t)(raw[14] << 8 | raw[13])) / powf(2.0f, 6.0f);
    calib->p7  = (float)((int8_t)raw[15]) / powf(2.0f, 8.0f);
    calib->p8  = (float)((int8_t)raw[16]) / powf(2.0f, 15.0f);
    calib->p9  = (float)((int16_t)(raw[18] << 8 | raw[17])) / powf(2.0f, 48.0f);
    calib->p10 = (float)((int8_t)raw[19]) / powf(2.0f, 48.0f);
    calib->p11 = (float)((int8_t)raw[20]) / powf(2.0f, 65.0f);

    /* sanity check: alles 0x00 of 0xFF betekent dat de NVM-read mislukt is */
    uint8_t all_zero = 1, all_ff = 1;
    for (int i = 0; i < 21; i++) {
        if (raw[i] != 0x00) all_zero = 0;
        if (raw[i] != 0xFF) all_ff = 0;
    }
    if (all_zero || all_ff) {
        printf("BMP384: kalibratiedata ongeldig (alles 0x%02X)\n", raw[0]);
        return HAL_ERROR;
    }

    printf("BMP384: calib t1=%.1f t2=%.9f p5=%.1f p6=%.4f\n",
           calib->t1, calib->t2, calib->p5, calib->p6);

    return HAL_OK;
}

//compensatie ruwe 24-bit ADC-waarden -> temperatuur (°C) / druk (Pa)
//formules volgens Bosch datasheet 9.2/9.3 Bosch-eigen kwantisatie

static float BMP384_CompensateTemperature(BMP384_Calib *calib, uint32_t raw_temp)
{
    float d1 = (float)raw_temp - calib->t1;
    float d2 = d1 * calib->t2;
    return d2 + (d1 * d1) * calib->t3;
}

static float BMP384_CompensatePressure(BMP384_Calib *calib, uint32_t raw_press)
{
    float p = (float)raw_press;
    float t = calib->t_lin;

    float out1 = calib->p5
               + calib->p6 * t
               + calib->p7 * (t * t)
               + calib->p8 * (t * t * t);

    float out2 = p * (calib->p1
                     + calib->p2 * t
                     + calib->p3 * (t * t)
                     + calib->p4 * (t * t * t));

    float out3 = (p * p) * (calib->p9 + calib->p10 * t)
               + (p * p * p) * calib->p11;

    return out1 + out2 + out3;
}

uint8_t BMP384_IsDataReady(void)
{
    uint8_t status_reg = 0;
    if (BMP384_ReadReg(REG_STATUS, &status_reg, 1) != HAL_OK) {
        return 0;
    }
    return ((status_reg & STATUS_DRDY_MASK) == STATUS_DRDY_MASK) ? 1 : 0;
}

BMP384_Status BMP384_Init(BMP384_Calib *calib)
{
    uint8_t chip_id = 0;
    uint8_t reg = 0;

    /* CS hoog houden en even wachten: de eerste dalende CS-flank zet de sensor
     * in SPI-mode (anders blijft hij op I2C staan). */
    HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(10); /* datasheet start-up time na power-on */

    /* dummy read om de interface-detectie af te ronden */
    (void)BMP384_ReadReg(REG_CHIP_ID, &chip_id, 1);

    if (BMP384_ReadReg(REG_CHIP_ID, &chip_id, 1) != HAL_OK) {
        printf("BMP384: SPI-fout bij lezen chip-ID\n");
        return BMP384_ERR_SPI;
    }
    if (chip_id != CHIP_ID_EXPECTED) {
        printf("BMP384: verkeerde chip-ID (0x%02X, verwacht 0x%02X)\n", chip_id, CHIP_ID_EXPECTED);
        return BMP384_ERR_ID;
    }
    printf("BMP384: chip-ID OK (0x%02X)\n", chip_id);

    if (BMP384_WriteReg(REG_CMD, CMD_SOFTRESET) != HAL_OK) {
        printf("BMP384: SPI-fout bij softreset\n");
        return BMP384_ERR_SPI;
    }
    HAL_Delay(10);

    /* na een softreset staat de interface-detectie opnieuw open -> dummy read */
    (void)BMP384_ReadReg(REG_CHIP_ID, &reg, 1);

    /* wachten tot de sensor het reset-commando verwerkt heeft (cmd_rdy) */
    for (int i = 0; i < 50; i++) {
        if (BMP384_ReadReg(REG_STATUS, &reg, 1) == HAL_OK &&
            (reg & STATUS_CMD_RDY_MASK)) {
            break;
        }
        HAL_Delay(2);
    }

    if (BMP384_ReadCalibration(calib) != HAL_OK) {
        printf("BMP384: fout bij lezen kalibratiedata\n");
        return BMP384_ERR_SPI;
    }
    calib->t_lin = 0.0f;

    /* VOLGORDE IS BELANGRIJK: eerst OSR / ODR / IIR instellen terwijl de sensor
     * nog in sleep staat, pas daarna normal mode aanzetten. Andersom draait de
     * sensor even op de default 200 Hz met x8 oversampling, wat niet haalbaar is
     * en conf_err in ERR_REG zet. */
    uint8_t config_ok = 1;
    config_ok &= BMP384_WriteVerify(REG_OSR,      0x03); /* druk x8, temp x1 */
    config_ok &= BMP384_WriteVerify(REG_ODR,      0x02); /* 50 Hz */
    config_ok &= BMP384_WriteVerify(REG_CONFIG,   0x04); /* IIR coef 3 */
    config_ok &= BMP384_WriteVerify(REG_INT_CTRL, 0x42); /* drdy-int, actief hoog */
    config_ok &= BMP384_WriteVerify(REG_PWR_CTRL, 0x33); /* press+temp aan, normal mode */
    if (!config_ok) {
        printf("BMP384: configureren mislukt\n");
        return BMP384_ERR_SPI;
    }

    /* ERR_REG: bit0 fatal_err, bit1 cmd_err, bit2 conf_err */
    HAL_Delay(50);
    if (BMP384_ReadReg(REG_ERR, &reg, 1) == HAL_OK && reg != 0x00) {
        printf("BMP384: ERR_REG = 0x%02X (fatal=%d cmd=%d conf=%d)\n",
               reg, reg & 1, (reg >> 1) & 1, (reg >> 2) & 1);
    }

    printf("BMP384: init geslaagd\n");
    return BMP384_OK;
}

BMP384_Status BMP384_ReadPressureTemperature(BMP384_Calib *calib,
                                              float *pressure_pa,
                                              float *temperature_c)
{
    uint8_t raw[6];

    /* druk (0x04..0x06) en temperatuur (0x07..0x09) in EEN burst, zodat beide
     * gegarandeerd uit dezelfde meting komen */
    if (BMP384_ReadReg(REG_DATA_PRESS_0, raw, sizeof(raw)) != HAL_OK) {
        return BMP384_ERR_SPI;
    }

    uint32_t uncomp_press = ((uint32_t)raw[2] << 16) | ((uint32_t)raw[1] << 8) | raw[0];
    uint32_t uncomp_temp  = ((uint32_t)raw[5] << 16) | ((uint32_t)raw[4] << 8) | raw[3];

    /* ruwe waardes van 0 of 0xFFFFFF = sensor meet niet / SPI-read stuk */
    if ((uncomp_press == 0 && uncomp_temp == 0) ||
        (uncomp_press == 0xFFFFFF && uncomp_temp == 0xFFFFFF)) {
        printf("BMP384: ruwe data ongeldig (p=%lu t=%lu)\n",
               (unsigned long)uncomp_press, (unsigned long)uncomp_temp);
        return BMP384_ERR_DATA;
    }

    calib->t_lin = BMP384_CompensateTemperature(calib, uncomp_temp);
    float pressure = BMP384_CompensatePressure(calib, uncomp_press);

    if (pressure_pa   != NULL) *pressure_pa   = pressure;
    if (temperature_c != NULL) *temperature_c = calib->t_lin;

    return BMP384_OK;
}

/* --- diagnostiek ---
 *
 * Situatie: alle registers lezen/schrijven correct, kalibratie is geldig,
 * ERR_REG is schoon, maar 0x04..0x09 blijven 0x000000. De vraag is dus of de
 * meetmotor van de sensor uberhaupt draait. SENSORTIME (0x0C) is daarvoor de
 * beste test: die teller loopt in normal mode continu door (25.6 kHz). Staat
 * hij stil, dan draait de sensor niet ondanks PWR_CTRL = 0x33. */

void BMP384_Diagnose(void)
{
    uint8_t reg = 0;
    uint8_t data[6] = {0};
    uint8_t st1[3] = {0}, st2[3] = {0};

    printf("--- BMP384 diagnose ---\n");

    struct { uint8_t addr; const char *naam; } lijst[] = {
        { REG_CHIP_ID,    "CHIP_ID " },
        { REG_ERR,        "ERR_REG " },
        { REG_STATUS,     "STATUS  " },
        { REG_EVENT,      "EVENT   " },
        { REG_INT_STATUS, "INT_STAT" },
        { REG_IF_CONF,    "IF_CONF " },
        { REG_PWR_CTRL,   "PWR_CTRL" },
        { REG_OSR,        "OSR     " },
        { REG_ODR,        "ODR     " },
        { REG_CONFIG,     "CONFIG  " },
    };

    for (unsigned i = 0; i < sizeof(lijst) / sizeof(lijst[0]); i++) {
        if (BMP384_ReadReg(lijst[i].addr, &reg, 1) == HAL_OK) {
            printf("  %s (0x%02X) = 0x%02X\n", lijst[i].naam, lijst[i].addr, reg);
        } else {
            printf("  %s (0x%02X) = SPI-FOUT\n", lijst[i].naam, lijst[i].addr);
        }
    }

    if (BMP384_ReadReg(REG_DATA_PRESS_0, data, 6) == HAL_OK) {
        printf("  DATA 0x04..0x09 = %02X %02X %02X %02X %02X %02X\n",
               data[0], data[1], data[2], data[3], data[4], data[5]);
    }

    /* loopt de interne tijdteller? */
    BMP384_ReadReg(REG_SENSORTIME, st1, 3);
    HAL_Delay(100);
    BMP384_ReadReg(REG_SENSORTIME, st2, 3);

    uint32_t t1 = ((uint32_t)st1[2] << 16) | ((uint32_t)st1[1] << 8) | st1[0];
    uint32_t t2 = ((uint32_t)st2[2] << 16) | ((uint32_t)st2[1] << 8) | st2[0];

    printf("  SENSORTIME %lu -> %lu (%s)\n",
           (unsigned long)t1, (unsigned long)t2,
           (t2 != t1) ? "LOOPT - sensor draait" : "STAAT STIL - sensor meet niet");

    printf("-----------------------\n");
}

/* Losse meting in forced mode: zet de sensor voor 1 conversie aan, met de
 * eenvoudigst mogelijke instellingen (x1 oversampling, geen IIR-filter).
 * Werkt dit wel terwijl normal mode nul geeft, dan zit het probleem in de
 * ODR/OSR-combinatie. Geeft dit ook nul, dan meet de sensor echt niet. */
BMP384_Status BMP384_ForcedTest(uint32_t *raw_press, uint32_t *raw_temp)
{
    uint8_t data[6] = {0};
    uint8_t reg = 0;

    if (BMP384_WriteReg(REG_PWR_CTRL, 0x00) != HAL_OK) return BMP384_ERR_SPI; /* sleep */
    HAL_Delay(5);
    if (BMP384_WriteReg(REG_OSR,    0x00) != HAL_OK) return BMP384_ERR_SPI;   /* x1 / x1 */
    if (BMP384_WriteReg(REG_CONFIG, 0x00) != HAL_OK) return BMP384_ERR_SPI;   /* IIR uit */
    if (BMP384_WriteReg(REG_PWR_CTRL, 0x13) != HAL_OK) return BMP384_ERR_SPI; /* forced */

    /* wachten tot drdy, ruim binnen 100 ms bij x1 oversampling */
    for (int i = 0; i < 50; i++) {
        HAL_Delay(2);
        if (BMP384_ReadReg(REG_STATUS, &reg, 1) == HAL_OK &&
            (reg & STATUS_DRDY_MASK) == STATUS_DRDY_MASK) {
            break;
        }
    }
    printf("BMP384: forced-test STATUS = 0x%02X\n", reg);

    if (BMP384_ReadReg(REG_DATA_PRESS_0, data, 6) != HAL_OK) {
        return BMP384_ERR_SPI;
    }
    printf("BMP384: forced-test bytes = %02X %02X %02X %02X %02X %02X\n",
           data[0], data[1], data[2], data[3], data[4], data[5]);

    if (raw_press != NULL) {
        *raw_press = ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | data[0];
    }
    if (raw_temp != NULL) {
        *raw_temp = ((uint32_t)data[5] << 16) | ((uint32_t)data[4] << 8) | data[3];
    }
    return BMP384_OK;
}

BMP384_Status BMP384_ReadRaw(uint32_t *raw_press, uint32_t *raw_temp)
{
    uint8_t raw[6];

    if (BMP384_ReadReg(REG_DATA_PRESS_0, raw, sizeof(raw)) != HAL_OK) {
        return BMP384_ERR_SPI;
    }
    if (raw_press != NULL) {
        *raw_press = ((uint32_t)raw[2] << 16) | ((uint32_t)raw[1] << 8) | raw[0];
    }
    if (raw_temp != NULL) {
        *raw_temp = ((uint32_t)raw[5] << 16) | ((uint32_t)raw[4] << 8) | raw[3];
    }
    return BMP384_OK;
}

float BMP384_PressureToAltitude(float pressure_pa, float ground_pressure_pa)
{
    return 44330.0f * (1.0f - powf(pressure_pa / ground_pressure_pa, 1.0f / 5.255f));
}
