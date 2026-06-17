/*
 * MPU6050.c
 *
 *  Created on: 2 mei 2026
 *      Author: tijme
 */


#include "MPU6050.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1; // eff tijdelijk en zien of we het op de i2c pin krijgen toch

void MPU6050_Init(void) {
    uint8_t data;

    //verbinding checken met who am i register
    uint8_t check;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR << 1, MPU6050_WHO_AM_I, 1, &check, 1, 100);

    if (check == 0x68) {
        //PWR_MGMT_1: uit sleepmode halen (bit naar 0) en kies klokbron
        //we zette klokbron naar gyroreferentie pll met gyro word aangeraden in datasheet ipv interne oscilator gebruiken
        data = 0x01;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR << 1, MPU6050_PWR_MGMT_1, 1, &data, 1, 100);

        //de sample rate en filter instellen
        data = MPU6050_DLPF_42HZ;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR << 1, MPU6050_CONFIG, 1, &data, 1, 100);

        //SMPLRT_DIV: Sample Rate Divider op 0 zetten
        // Formule: Sample Rate = 1kHz / (1 + 0) = 1kHz (elke 1ms nieuwe data)
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR << 1, MPU6050_SMPLRT_DIV, 1, &data, 1, 100);

        // 5. GYRO_CONFIG op 500
        data = MPU6050_GYRO_FS_500;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR << 1, MPU6050_GYRO_CONFIG, 1, &data, 1, 100);

        // accel config gevoeligste zetten
        data = MPU6050_ACCEL_FS_8;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR << 1, MPU6050_ACCEL_CONFIG, 1, &data, 1, 100);
    }
}

void MPU6050_Read_Gyro(float *gx, float *gy, float *gz) {
    uint8_t buffer[6];
    int16_t raw_x, raw_y, raw_z;

    //Lees 6 bytes vanaf GYRO_XOUT_H
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR << 1, MPU6050_GYRO_XOUT_H, 1, buffer, 6, 100);

    raw_x = (int16_t)(buffer[0] << 8 | buffer[1]);
    raw_y = (int16_t)(buffer[2] << 8 | buffer[3]);
    raw_z = (int16_t)(buffer[4] << 8 | buffer[5]);

    // Omrekenen naar dps: Gevoeligheid bij +/- 500 dps is 65.5 LSB/dps
    *gx = (float)raw_x / 65.5f;
    *gy = (float)raw_y / 65.5f;
    *gz = (float)raw_z / 65.5f;
}


void MPU6050_Read_Accel(float *ax, float *ay, float *az) {
    uint8_t buffer[6];
    int16_t raw_x, raw_y, raw_z;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR << 1, MPU6050_ACCEL_XOUT_H, 1, buffer, 6, 100);

    raw_x = (int16_t)(buffer[0] << 8 | buffer[1]);
    raw_y = (int16_t)(buffer[2] << 8 | buffer[3]);
    raw_z = (int16_t)(buffer[4] << 8 | buffer[5]);

    // Gevoeligheid bij +/- 8g is 4096 LSB/g
    *ax = (float)raw_x / 4096.0f;
    *ay = (float)raw_y / 4096.0f;
    *az = (float)raw_z / 4096.0f;
}
