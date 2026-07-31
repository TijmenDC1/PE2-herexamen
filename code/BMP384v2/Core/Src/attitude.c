/*
 * attitude.c
 *
 *  Created on: 24 jul 2026
 *      Author: simon
 */

// attitude.c
#include "attitude.h"
#include "main.h"
#include "fc_config.h"
#include <math.h>

#define ALPHA ATTITUDE_ALPHA

// uit DshotFCMetMPU6050/Core/Src/attitude.c (auteur: simon)
void Attitude_AccelAngles(float ax, float ay, float az, float *roll, float *pitch)
{
    // hoek die de accelerometer alleen zou meten (zwaartekracht als referentie)
    *roll  = atan2f(ay, az) * (180.0f / PI);
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / PI);
}

// uit DshotFCMetMPU6050/Core/Src/attitude.c (auteur: simon)
void Attitude_Update(float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float dt, float *roll, float *pitch)
{
    float accel_roll, accel_pitch;
    Attitude_AccelAngles(ax, ay, az, &accel_roll, &accel_pitch);

    // gyro (deg/s) integreren en blenden met de accel-referentie
    *roll  = ALPHA * (*roll  + gx * dt) + (1.0f - ALPHA) * accel_roll;
    *pitch = ALPHA * (*pitch + gy * dt) + (1.0f - ALPHA) * accel_pitch;
}
