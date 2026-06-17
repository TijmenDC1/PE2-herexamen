/*
 * BMP3844.h
 *
 *  Created on: 26 feb 2026
 *      Author: simon
 */

#ifndef BMP384_H
#define BMP384_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h" // Pas aan naar jouw type (f1, f4, h7, etc.)

// Belangrijke Registers [cite: 2335]
#define BMP384_REG_CHIP_ID      0x00
#define BMP384_REG_ERR_REG      0x02
#define BMP384_REG_STATUS       0x03
#define BMP384_REG_DATA_0       0x04 // Druk XLSB [cite: 2415]
#define BMP384_REG_PWR_CTRL     0x1B
#define BMP384_REG_OSR          0x1C
#define BMP384_REG_ODR          0x1D
#define BMP384_REG_CONFIG       0x1F
#define BMP384_REG_CALIB_BASE   0x31 // Start van kalibratie data [cite: 2318]
#define BMP384_REG_CMD          0x7E

// Instellingen [cite: 1786, 2535]
#define BMP384_CHIP_ID_VAL      0x50 //[cite: 2391]
#define BMP384_MODE_NORMAL      0x33 // Druk aan, Temp aan, Normal mode [cite: 2528]
#define BMP384_CMD_SOFTRESET    0xB6 //[cite: 2562]

// Struct voor kalibratie-coëfficiënten [cite: 3119]
typedef struct {
    float par_t1, par_t2, par_t3;
    float par_p1, par_p2, par_p3, par_p4, par_p5, par_p6, par_p7, par_p8, par_p9, par_p10, par_p11;
    float t_lin; // Voor interne berekening [cite: 3120]
} BMP384_Calib_Data;

// Functie prototypes
HAL_StatusTypeDef BMP384_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin);
float BMP384_ReadTemperature(void);
float BMP384_ReadPressure(void);

#endif
