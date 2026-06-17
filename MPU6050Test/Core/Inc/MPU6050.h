/*
 * MPU6050.h
 *
 *  Created on: 1 mei 2026
 *      Author: tijme
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_


/*
 * MPU6050_registers.h
 * Gebaseerd op Document Number: RM-MPU-6000A-00 (Revision 4.0)
 */

#ifndef INC_MPU6050_REGISTERS_H_
#define INC_MPU6050_REGISTERS_H_

//I2C Adress
#define MPU6050_I2C_ADDR            0x68

//configureer registers
#define MPU6050_SMPLRT_DIV          0x19 	//Sample Rate Divider
#define MPU6050_CONFIG               0x1A 	//Algemene configuratie
#define MPU6050_GYRO_CONFIG        0x1B     //Gyroscoop bereik
#define MPU6050_ACCEL_CONFIG        0x1C   //Accelerometer bereik
#define MPU6050_FIFO_EN           0x23     //data in fifo
#define MPU6050_INT_PIN_CFG         0x37 //Interrupt pin gedrag[cite: 1]
#define MPU6050_INT_ENABLE         0x38   //event triggeren input
#define MPU6050_INT_STATUS         0x3A   //Status interrupts


#define MPU6050_ACCEL_XOUT_H        0x3B   //Accelerometer X-as HSB
#define MPU6050_ACCEL_XOUT_L        0x3C    //Accelerometer X-as LSB
#define MPU6050_ACCEL_YOUT_H        0x3D
#define MPU6050_ACCEL_YOUT_L        0x3E
#define MPU6050_ACCEL_ZOUT_H        0x3F
#define MPU6050_ACCEL_ZOUT_L        0x40

#define MPU6050_TEMP_OUT_H          0x41   //Temperatuur Hoog
#define MPU6050_TEMP_OUT_L          0x42

#define MPU6050_GYRO_XOUT_H         0x43   //Gyroscoop X-as HSV
#define MPU6050_GYRO_XOUT_L         0x44   //Gyroscoop X-as LSB
#define MPU6050_GYRO_YOUT_H         0x45
#define MPU6050_GYRO_YOUT_L         0x46
#define MPU6050_GYRO_ZOUT_H         0x47
#define MPU6050_GYRO_ZOUT_L         0x48

// --- Power & System Registers ---

#define MPU6050_USER_CTRL           0x6A //FIFO en I2C Master enable/reset[cite: 1]
#define MPU6050_PWR_MGMT_1          0x6B //Belangrijkste: Sleep/Clock/Reset[cite: 1]
#define MPU6050_PWR_MGMT_2          0x6C   //standby zetten
#define MPU6050_WHO_AM_I            0x75

// GYRO_CONFIG full scale range gevoegligheid
#define MPU6050_GYRO_FS_250         0x00 //250dps
#define MPU6050_GYRO_FS_500         0x08  //500dps
#define MPU6050_GYRO_FS_1000        0x10  //1000dps
#define MPU6050_GYRO_FS_2000        0x18  //2000dps

// ACCEL_CONFIG ook fsr en dus gevoeligheid
#define MPU6050_ACCEL_FS_2          0x00 //2g
#define MPU6050_ACCEL_FS_4          0x08 //4g
#define MPU6050_ACCEL_FS_8          0x10 //8g
#define MPU6050_ACCEL_FS_16         0x18 //16g

// PWR_MGMT_1
#define MPU6050_DEVICE_RESET        0x80   //default reset
#define MPU6050_SLEEP_MODE          0x40


//filtering en sample rate
#define MPU6050_DLPF_256HZ          0x00
#define MPU6050_DLPF_188HZ          0x01
#define MPU6050_DLPF_98HZ           0x02
#define MPU6050_DLPF_42HZ           0x03
#define MPU6050_DLPF_20HZ           0x04

void MPU6050_Init(void);
void MPU6050_Read_Gyro(float *gx, float *gy, float *gz);
void MPU6050_Read_Accel(float *ax, float *ay, float *az);

#endif /* INC_MPU6050_REGISTERS_H_ */

#endif /* INC_MPU6050_H_ */
