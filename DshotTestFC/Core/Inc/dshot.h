/*
 * dshot.h
 *
 *  Created on: 13 mei 2026
 *      Author: tijme
 */

#ifndef INC_DSHOT_H_
#define INC_DSHOT_H_

#include "main.h"

#define DSHOT_FRAME_SIZE 16
#define DSHOT_THRESHOLD 45  // Bij 48MHz clock
#define DSHOT_BUF_SIZE   17

typedef struct {
    uint16_t gas;
    uint8_t  telemetry;
    uint8_t  geldig;
    uint8_t  crc;
} DShot_Data_t;

void update_motor_buffer(uint8_t motor_id, uint16_t throttle, uint8_t telemetry);
void send_dshot(void);

#endif /* INC_DSHOT_H_ */
