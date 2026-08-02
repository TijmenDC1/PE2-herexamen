/*
 * BMM350.h
 *
 *  Created on: 28 feb 2026
 *      Author: simon
 */

#ifndef INC_BMM350_H_
#define INC_BMM350_H_

#include "main.h"
#include "stdio.h"

#define BMM350_I2C_ADDRESS			(0x14 << 1)	//Bitshift van 1 moet volgens HAL_I2C library

#define BMM350_CHIP_ID				0X00	//default 0x33
#define BMM350_ERR_REG				0x02
#define BMM350_PAD_CTRL				0x03	//default 0x07
#define BMM350_PMU_CMD_AGGR_SET		0x04	//default 0x14		Zet odr en avg
#define BMM350_PMU_CMD_AXIS_EN		0x05	//default 0x70
#define BMM350_PMU_CMD				0x06
#define BMM350_PMU_CMD_STATUS_0		0x07
#define BMM350_PMU_CMD_STATUS_1		0x08
#define BMM350_I3C_ERR				0x09
#define BMM350_I2C_WDT_SET			0x0A
#define BMM350_TRANSDUCER_REV_ID	0x0D	//default 0x33
#define BMM350_INT_CTRL				0x2E
#define BMM350_INT_CTRL_IBI			0x2F
#define BMM350_INT_STATUS			0x30
#define BMM350_MAG_X_XLSB			0x31	//default 0x7F
#define BMM350_MAG_X_LSB			0x32	//default 0x7F
#define BMM350_MAG_X_MSB			0x33	//default 0x7F
#define BMM350_MAG_Y_XLSB			0x34	//default 0x7F
#define BMM350_MAG_Y_LSB			0x35	//default 0x7F
#define BMM350_MAG_Y_MSB			0x36	//default 0x7F
#define BMM350_MAG_Z_XLSB			0x37	//default 0x7F
#define BMM350_MAG_Z_LSB			0x38	//default 0x7F
#define BMM350_MAG_Z_MSB			0x39	//default 0x7F
#define BMM350_TEMP_XLSB			0x3A	//default 0x7F
#define BMM350_TEMP_LSB				0x3B	//default 0x7F
#define BMM350_TEMP_MSB				0x3C	//default 0x7F
#define BMM350_SENSORTIME_XLSB		0x3D	//default 0x7F
#define BMM350_SENSORTIME_LSB		0x3E	//default 0x7F
#define BMM350_SENSORTIME_MSB		0x3F	//default 0x7F
#define BMM350_OTP_CMD_REG			0x50
#define BMM350_OTP_DATA_MSB_REG		0x52
#define BMM350_OTP_DATA_LSB_REG		0x53
#define BMM350_OTP_STATUS_REG		0x55	//default 0x10
#define BMM350_TMR_SELFTEST_USER	0x60
#define BMM350_CTRL_USER			0x61
#define BMM350_CMD					0x7E

typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
    float richting;
    // offsets moeten nog bepaald worden maar zijn hardcoded waardes
    uint8_t x_offset;
    uint8_t y_offset;
    uint8_t z_offset;
} BMM350_MagData;

float BMM350_Angle(float x, float y);
void BMM350_Calibrate(float raw_x, float raw_y, float raw_z, float *offset_x, float *offset_y, float *offset_z);
float BMM350_TiltCompensation(float roll, float pitch);
void BMM350_WriteReg(uint8_t reg, uint8_t val);
void BMM350_ReadReg(uint8_t reg, uint8_t *pData, uint16_t len);

#endif /* INC_BMM350_H_ */
