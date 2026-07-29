/*
 * attitude.h
 *
 *  Created on: 24 jul 2026
 *      Author: simon
 */

#ifndef INC_ATTITUDE_H_
#define INC_ATTITUDE_H_

// Hoeken puur uit de accelerometer (zwaartekracht-referentie), in graden.
// Handig om de vaste montage-offset van de IMU op te meten.
void Attitude_AccelAngles(float ax, float ay, float az, float *roll, float *pitch);

void Attitude_Update(float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float dt, float *roll, float *pitch);

#endif /* INC_ATTITUDE_H_ */

