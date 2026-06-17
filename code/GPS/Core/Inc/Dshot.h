/*
* dshotsturen.h
*
* Created on: 27 mrt 2026
* Author: tijme
*/

#ifndef INC_DSHOTSTUREN_H_
#define INC_DSHOTSTUREN_H_

#define DSHOT_BUF_SIZE 17

void update_motor_buffer(uint8_t motor_id, uint16_t throttle, uint8_t telemetry);
void send_dshot(void);

#endif /* INC_DSHOTSTUREN_H_ */
