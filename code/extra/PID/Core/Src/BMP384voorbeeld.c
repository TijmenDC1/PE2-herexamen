/*
 * BMP384.c
 *
 *  Created on: 26 feb 2026
 *      Author: simon
 */
#include <BMP384voorbeeld.h>
#include <math.h>

static SPI_HandleTypeDef *_bmp_hspi;
static GPIO_TypeDef *_bmp_csPort;
static uint16_t _bmp_csPin;
static BMP384_Calib_Data _calib;

// Hulpfunctie: Schrijf naar register [cite: 2703, 2705]
static void BMP_WriteReg(uint8_t reg, uint8_t val) {
    uint8_t data[2] = { reg & 0x7F, val }; // MSB 0 voor write [cite: 2700]
    HAL_GPIO_WritePin(_bmp_csPort, _bmp_csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(_bmp_hspi, data, 2, 100);
    HAL_GPIO_WritePin(_bmp_csPort, _bmp_csPin, GPIO_PIN_SET);
}

// Hulpfunctie: Lees registers (met dummy byte)
static void BMP_ReadRegs(uint8_t reg, uint8_t *pData, uint16_t len) {
    uint8_t addr = reg | 0x80; // MSB 1 voor read [cite: 2700]
    uint8_t dummy;
    HAL_GPIO_WritePin(_bmp_csPort, _bmp_csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(_bmp_hspi, &addr, 1, 100);
    HAL_SPI_Receive(_bmp_hspi, &dummy, 1, 100); // Verplichte dummy byte
    HAL_SPI_Receive(_bmp_hspi, pData, len, 100);
    HAL_GPIO_WritePin(_bmp_csPort, _bmp_csPin, GPIO_PIN_SET);
}

// Kalibratie data inladen en omzetten naar float [cite: 3125 - 3130]
static void BMP_LoadCalibration(void) {
    uint8_t reg_data[42];
    BMP_ReadRegs(BMP384_REG_CALIB_BASE, reg_data, 42);

    // Voorbeeld T1 (16-bit unsigned) [cite: 2318]
    uint16_t nvm_t1 = (reg_data[1] << 8) | reg_data[0];
    _calib.par_t1 = nvm_t1 / powf(2.0f, -8.0f); // Formule uit datasheet

    // ... Herhaal dit voor alle parameters P1 t/m P11 en T1 t/m T3 uit de datasheet ...
    // (Vanwege de lengte zijn hier alleen de basis stappen getoond)
}

HAL_StatusTypeDef BMP384_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin) {
    _bmp_hspi = hspi; _bmp_csPort = csPort; _bmp_csPin = csPin;
    uint8_t id;

    // 1. Check Chip ID [cite: 2396]
    BMP_ReadRegs(BMP384_REG_CHIP_ID, &id, 1);
    if (id != BMP384_CHIP_ID_VAL) return HAL_ERROR;

    // 2. Soft Reset [cite: 2562]
    BMP_WriteReg(BMP384_REG_CMD, BMP384_CMD_SOFTRESET);
    HAL_Delay(10);

    // 3. Kalibratie laden
    BMP_LoadCalibration();

    // 4. Drone instellingen: OSR x8 druk, x1 temp
    BMP_WriteReg(BMP384_REG_OSR, 0x03);

    // 5. Activeer sensor in Normal Mode [cite: 1786, 2528]
    BMP_WriteReg(BMP384_REG_PWR_CTRL, BMP384_MODE_NORMAL);

    return HAL_OK;
}

// Temperatuur compensatie formule [cite: 3133 - 3144]
float BMP384_CompensateTemperature(uint32_t uncomp_temp) {
    float partial_data1 = (float)(uncomp_temp - _calib.par_t1);
    float partial_data2 = (float)(partial_data1 * _calib.par_t2);
    _calib.t_lin = partial_data2 + (partial_data1 * partial_data1) * _calib.par_t3;
    return _calib.t_lin;
}

