/*
 * BMP384.h
 *
 *  Created on: 26 feb 2026
 *      Author: simon
 */

#ifndef INC_BMP384_H_
#define INC_BMP384_H_

#include "stm32f7xx_hal.h"

#define BMP384_ID				0x50
#define BMP384_MODE_NORMAL      0x33	// press aan, temp aan, mode normal
#define BMP384_CMD_SOFTRESET    0xB6

#define	BMP384_CHIP_ID			0x00
#define	BMP384_ERR_REG			0x02
#define	BMP384_STATUS			0x03
#define	BMP384_DATA0			0x04
#define	BMP384_DATA1			0x05
#define	BMP384_DATA2			0x06
#define	BMP384_DATA3			0x07
#define	BMP384_DATA4			0x08
#define	BMP384_DATA5			0x09
#define	BMP384_SENSORTIME_0		0x0C
#define	BMP384_SENSORTIME_1		0x0D
#define	BMP384_SENSORTIME_2		0x0F
#define	BMP384_EVENT			0x10
#define	BMP384_INT_STATUS		0x11
#define	BMP384_FIFO_LENGTH_0	0x12
#define	BMP384_FIFO_LENGTH_1	0x13
#define	BMP384_FIFO_DATA		0x14
#define	BMP384_FIFO_WTM_0		0x15
#define	BMP384_FIFO_WTM_1		0x16
#define	BMP384_FIFO_CONFIG_1	0x17
#define	BMP384_FIFO_CONFIG_2	0x18
#define	BMP384_INT_CTRL			0x19
#define	BMP384_IF_CONF			0x1A
#define	BMP384_PWR_CTRL			0x1B	// Bit 1 = pressure on/off, Bit 2  = Temp on/off, Bit 4,5 = Sleep, forced, normal mode
#define	BMP384_OSR				0x1C	// [5:3]  == oversampling temp, [2:0] == oversampling pressure
#define	BMP384_ODR				0x1D
#define	BMP384_CONFIG			0x1F
#define	BMP384_NVM_PAR_T1_LSB	0x31
#define	BMP384_NVM_PAR_T1_MSB	0x32
#define	BMP384_NVM_PAR_T2_LSB	0x33
#define	BMP384_NVM_PAR_T2_LSB	0x34
#define	BMP384_NVM_PAR_T3_MSB	0x35
#define	BMP384_NVM_PAR_P1_LSB	0x36
#define	BMP384_NVM_PAR_P1_MSB	0x37
#define	BMP384_NVM_PAR_P2_LSB	0x38
#define	BMP384_NVM_PAR_P2_MSB	0x39
#define	BMP384_NVM_PAR_P3		0x3A
#define	BMP384_NVM_PAR_P4		0x3B
#define	BMP384_NVM_PAR_P5_LSB	0x3C
#define	BMP384_NVM_PAR_P5_MSB	0x3D
#define	BMP384_NVM_PAR_P6_LSB	0x3E
#define	BMP384_NVM_PAR_P6_MSB	0x3F
#define	BMP384_NVM_PAR_P7		0x40
#define	BMP384_NVM_PAR_P8		0x41
#define	BMP384_NVM_PAR_P9_LSB	0x42
#define	BMP384_NVM_PAR_P9_MSB	0x43
#define	BMP384_NVM_PAR_P10		0x44
#define	BMP384_NVM_PAR_P11		0x45
#define	BMP384_CMD				0x7E

typedef struct {
    float t1, t2, t3;
    float p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
    float t_lin;
} BMP384_Calib;

HAL_StatusTypeDef BMP384_Init(void);
static void BMP384_WriteReg(uint8_t reg, uint8_t val);
static void BMP384_ReadReg(uint8_t reg, uint8_t *pData, uint16_t len);
float BMP384_ReadData(float *pressure_pa);
static void BMP384_ReadCalibration(void);
static float BMP384_Compensate_Pressure(uint32_t uncomp_p, float t_lin);
uint8_t BMP384_IsDataReady();
float BMP384_CalculateAltitude(float pressure_pa);

#endif
