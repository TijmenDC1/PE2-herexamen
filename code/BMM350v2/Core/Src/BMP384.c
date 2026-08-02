/*
* dshotsturen.c
*
* Created on: 31 jun 2026
* Author: tijme
*/

#include "BMP384.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi4;

#define SPI_TIMEOUT_MS  100

//registeradressen
#define REG_CHIP_ID         0x00
#define REG_STATUS          0x03
#define REG_DATA_PRESS_0    0x04 //druk: 3 bytes vanaf hier 24-bit
#define REG_DATA_TEMP_0     0x07 //temp: 3 bytes vanaf hier 24-bit
#define REG_INT_CTRL        0x19
#define REG_PWR_CTRL        0x1B
#define REG_OSR             0x1C
#define REG_ODR             0x1D
#define REG_CONFIG          0x1F
#define REG_CALIB_START     0x31 //21 bytes NVM-kalibratiedata vanaf hier
#define REG_CMD             0x7E

#define CHIP_ID_EXPECTED    0x50
#define CMD_SOFTRESET       0xB6

#define STATUS_DRDY_MASK    0x60

/* --- lage-niveau SPI-toegang, elke stap checkt zijn eigen HAL-status --- */

static HAL_StatusTypeDef BMP384_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t frame[2] = { (uint8_t)(reg & 0x7F), value };
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(&hspi4, frame, sizeof(frame), SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);

    return status;
}

static HAL_StatusTypeDef BMP384_ReadReg(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t addr = (uint8_t)(reg | 0x80);
    uint8_t dummy = 0xFF;
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET);

    status = HAL_SPI_Transmit(&hspi4, &addr, 1, SPI_TIMEOUT_MS);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(&hspi4, &dummy, 1, SPI_TIMEOUT_MS);
    }
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(&hspi4, data, len, SPI_TIMEOUT_MS);
    }

    HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);
    return status;
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


    HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(2); /* datasheet start-up time na power-on */

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

    if (BMP384_ReadCalibration(calib) != HAL_OK) {
        printf("BMP384: SPI-fout bij lezen kalibratiedata\n");
        return BMP384_ERR_SPI;
    }
    calib->t_lin = 0.0f;

    /* normal mode, druk+temp aan, x8 oversampling druk, IIR-filter aan,
     * ~50Hz output-rate, data-ready interrupt actief-hoog */
    uint8_t config_ok = 1;
    config_ok &= (BMP384_WriteReg(REG_PWR_CTRL, 0x33) == HAL_OK);
    config_ok &= (BMP384_WriteReg(REG_OSR,      0x03) == HAL_OK);
    config_ok &= (BMP384_WriteReg(REG_CONFIG,   0x04) == HAL_OK);
    config_ok &= (BMP384_WriteReg(REG_ODR,      0x02) == HAL_OK);
    config_ok &= (BMP384_WriteReg(REG_INT_CTRL, 0x42) == HAL_OK);
    if (!config_ok) {
        printf("BMP384: SPI-fout bij configureren\n");
        return BMP384_ERR_SPI;
    }

    printf("BMP384: init geslaagd\n");
    return BMP384_OK;
}

BMP384_Status BMP384_ReadPressureTemperature(BMP384_Calib *calib,
                                              float *pressure_pa,
                                              float *temperature_c)
{
    uint8_t raw_p[3];
    uint8_t raw_t[3];

    if (BMP384_ReadReg(REG_DATA_PRESS_0, raw_p, 3) != HAL_OK) {
        return BMP384_ERR_SPI;
    }
    if (BMP384_ReadReg(REG_DATA_TEMP_0, raw_t, 3) != HAL_OK) {
        return BMP384_ERR_SPI;
    }

    uint32_t uncomp_press = (uint32_t)(raw_p[2] << 16 | raw_p[1] << 8 | raw_p[0]);
    uint32_t uncomp_temp  = (uint32_t)(raw_t[2] << 16 | raw_t[1] << 8 | raw_t[0]);

    calib->t_lin = BMP384_CompensateTemperature(calib, uncomp_temp);
    float pressure = BMP384_CompensatePressure(calib, uncomp_press);

    if (pressure_pa   != NULL) *pressure_pa   = pressure;
    if (temperature_c != NULL) *temperature_c = calib->t_lin;

    return BMP384_OK;
}

float BMP384_PressureToAltitude(float pressure_pa, float ground_pressure_pa)
{
    return 44330.0f * (1.0f - powf(pressure_pa / ground_pressure_pa, 1.0f / 5.255f));
}
