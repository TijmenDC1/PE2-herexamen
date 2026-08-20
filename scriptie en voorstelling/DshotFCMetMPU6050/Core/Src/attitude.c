/*
 * attitude.c
 *
 *  Created on: 24 jul 2026
 *      Author: simon
 */

#include "attitude.h"
#include "main.h"
#include "fc_config.h"
#include <math.h>

#define ALPHA ATTITUDE_ALPHA

void Attitude_AccelAngles(float ax, float ay, float az, float *roll, float *pitch)
{
    // hoek uit de zwaartekracht
    *roll  = atan2f(ay, az) * (180.0f / PI);
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / PI);
}

void Attitude_Update(float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float dt, float *roll, float *pitch)
{
    float a_roll, a_pitch;
    Attitude_AccelAngles(ax, ay, az, &a_roll, &a_pitch);

    // gyro erbij optellen en mengen met de accel-hoek
    *roll  = ALPHA * (*roll  + gx * dt) + (1.0f - ALPHA) * a_roll;
    *pitch = ALPHA * (*pitch + gy * dt) + (1.0f - ALPHA) * a_pitch;
}
