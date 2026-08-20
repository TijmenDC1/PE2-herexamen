/*
 * BMI330.h
 *
 *  Created on: 28 feb 2026
 *      Author: simon
 */

#ifndef INC_BMI330_H_
#define INC_BMI330_H_

#include "stm32f7xx_hal.h"

#define BMI330_ID					0x48

// NORMAL MEMORY MAP

#define BMI330_CHIP_ID				0x00
#define BMI330_ERR_REG				0x01
#define BMI330_STATUS				0x02
#define BMI330_ACC_DATA_X			0x03
#define BMI330_ACC_DATA_Y			0x04
#define BMI330_ACC_DATA_Z			0x05
#define BMI330_GYR_DATA_X			0x06
#define BMI330_GYR_DATA_Y			0x07
#define BMI330_GYR_DATA_Z			0x08
#define BMI330_TEMP_DATA			0x09
#define BMI330_SENSOR_TIME_0		0x0A
#define BMI330_SENSOR_TIME_1		0x0B
#define BMI330_SET_FLAGS			0x0C
#define BMI330_INT_STATUS_INT1		0x0D
#define BMI330_INT_STATUS_INT2		0x0E
#define BMI330_INT_STATUS_IBI		0x0F
#define BMI330_FEATURE_IO0			0x10
#define BMI330_FEATURE_IO1			0x11
#define BMI330_FEATURE_IO2			0x12
#define BMI330_FEATURE_IO3			0x13
#define BMI330_FEATURE_IO_STATUS	0x14
#define BMI330_FIFO_FILL_LEVEL		0x15
#define BMI330_FIFO_DATA			0x16
#define BMI330_ACC_CONF				0x20
#define BMI330_GYR_CONF				0x21
#define BMI330_ALT_ACC_CONF			0x28
#define BMI330_ALT_GYR_CONF			0x29
#define BMI330_ALT_CONF				0x2A
#define BMI330_ALT_STATUS			0x2B
#define BMI330_FIFO_WATERMARK		0x35
#define BMI330_FIFO_CONF			0x36
#define BMI330_FIFO_CTRL			0x37
#define BMI330_IO_INT_CTRL			0x38
#define BMI330_INT_CONF				0x39
#define BMI330_INT_MAP1				0x3A
#define BMI330_INT_MAP2				0x3B
#define BMI330_FEATURE_CTRL			0x40
#define BMI330_FEATURE_DATA_ADDR	0x41
#define BMI330_FEATURE_DATA_TX		0x42
#define BMI330_FEATURE_DATA_STATUS	0x43
#define BMI330_FEATURE_ENGINE_STATUS	0x45
#define BMI330_FEATURE_EVENT_EXT	0x47
#define BMI330_IO_PDN_CTRL			0x4F
#define BMI330_IO_SPI_IF			0x50
#define BMI330_IO_PAD_STRENGTH		0x51
#define BMI330_IO_I2C_IF			0x52
#define BMI330_IO_ODR_DEVIATION		0x53
#define BMI330_ACC_DP_OFF_X			0x60
#define BMI330_ACC_DP_DGAIN_X		0x61
#define BMI330_ACC_DP_OFF_Y			0x62
#define BMI330_ACC_DP_DGAIN_Y		0x63
#define BMI330_ACC_DP_OFF_Z			0x64
#define BMI330_ACC_DP_DGAIN_Z		0x65
#define BMI330_GYR_DP_OFF_X			0x66
#define BMI330_GYR_DP_DGAIN_X		0x67
#define BMI330_GYR_DP_OFF_Y			0x68
#define BMI330_GYR_DP_DGAIN_Y		0x69
#define BMI330_GYR_DP_OFF_Z			0x6A
#define BMI330_GYR_DP_DGAIN_Z		0x6B
#define BMI330_I3C_TC_SYNC_TPH		0x70
#define BMI330_I3C_TC_SYNC_TU		0x71
#define BMI330_I3C_TC_SYNC_ODR		0x72
#define BMI330_CMD					0x7E
#define BMI330_CFG_RES				0x7F

// EXTENDED MEMORY MAP

//
//	@note om waardes naar ext. mem. map te schrijven schrijf je eerst het address naar BMI330_FEATURE_DATA_ADDR en dan de waarde naar BMI330_FEATURE_DATA_TX
//

#define BMI330_GEN_SET_1			0x02
#define BMI330_AXIS_MAP_1			0x03
#define BMI330_ANYMO_1				0x05
#define BMI330_ANYMO_2				0x06
#define BMI330_ANYMO_3				0x07
#define BMI330_NOMO_1				0x08
#define BMI330_NOMO_2				0x09
#define BMI330_NOMO_3				0x0A
#define BMI330_FLAT_1				0x0B
#define BMI330_FLAT_2				0x0C
#define BMI330_SIGMO_1				0x0D
#define BMI330_SIGMO_2				0x0E
#define BMI330_SIGMO_3				0x0F
#define BMI330_SC_1					0x10
#define BMI330_ORIENT_1				0x1C
#define BMI330_ORIENT_2				0x1D
#define BMI330_TAP_1				0x1E
#define BMI330_TAP_2				0x1F
#define BMI330_TAP_3				0x20
#define BMI330_TILT_1				0x21
#define BMI330_TILT_2				0x22
#define BMI330_ALT_CONFIG_CHG		0x23
#define BMI330_ST_RESULT			0x24
#define BMI330_ST_SELECT			0x25
#define BMI330_GYR_SC_SELECT		0x26
#define BMI330_GYR_SC_ST_CONF		0x27
#define BMI330_SC_ST_VALUE0			0x28
#define BMI330_SC_ST_VALUE1			0x29
#define BMI330_SC_ST_VALUE2			0x2A
#define BMI330_SC_ST_VALUE3			0x2B
#define BMI330_SC_ST_VALUE4			0x2C
#define BMI330_SC_ST_VALUE5			0x2D
#define BMI330_SC_ST_VALUE6			0x2E
#define BMI330_SC_ST_VALUE7			0x2F
#define BMI330_SC_ST_VALUE8			0x30
#define BMI330_SC_ST_VALUE9			0x31
#define BMI330_SC_ST_VALUE10		0x32
#define BMI330_SC_ST_VALUE11		0x33
#define BMI330_SC_ST_VALUE12		0x34
#define BMI330_GYR_MOT_DET			0x35
#define BMI330_I3C_TC				0x36
#define BMI330_SYNC_ACC_X			0x37
#define BMI330_SYNC_ACC_Y			0x38
#define BMI330_SYNC_ACC_Z			0x39
#define BMI330_SYNC_GYR_X			0x3A
#define BMI330_SYNC_GYR_Y			0x3B
#define BMI330_SYNC_GYR_Z			0x3C
#define BMI330_SYNC_TEMP			0x3D
#define BMI330_SYNC_TIME			0x3E

typedef struct {
    int16_t acc_x, acc_y, acc_z;
    int16_t gyr_x, gyr_y, gyr_z;
    uint16_t sensor_temp; // 24-bit
    uint8_t offset;
} BMI330_Frame;

#endif
