/*
 * attitude.h
 *
 *  Created on: 24 jul 2026
 *      Author: simon
 */

#ifndef INC_ATTITUDE_H_
#define INC_ATTITUDE_H_

// hoeken uit alleen de accelerometer, in graden
void Attitude_AccelAngles(float ax, float ay, float az, float *roll, float *pitch);

void Attitude_Update(float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float dt, float *roll, float *pitch);

#endif /* INC_ATTITUDE_H_ */
