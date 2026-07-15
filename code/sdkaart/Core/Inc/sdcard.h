/*
 * sdcard.h
 *
 *  Created on: 28 jun 2026
 *      Author: tijme
 *
 * SD kaart driver bovenop FATFS van CubeMX
 */

#ifndef INC_SDCARD_H_
#define INC_SDCARD_H_

#include "fatfs.h"
#include <stdint.h>

typedef enum {
    SDCARD_OK              = 0,
    SDCARD_ERR_MOUNT       = 1,
    SDCARD_ERR_NOT_MOUNTED = 2,
    SDCARD_ERR_OPEN        = 3,
    SDCARD_ERR_WRITE       = 4,
    SDCARD_ERR_READ        = 5,
} SDCard_Status_t;

SDCard_Status_t SDCard_Mount(void);
void            SDCard_Unmount(void);
uint8_t         SDCard_IsMounted(void);

SDCard_Status_t SDCard_WriteFile(const char *filename, const uint8_t *data, uint32_t size);
SDCard_Status_t SDCard_AppendFile(const char *filename, const uint8_t *data, uint32_t size);
SDCard_Status_t SDCard_ReadFile(const char *filename, uint8_t *buf, uint32_t buf_size, uint32_t *bytes_read);
SDCard_Status_t SDCard_DeleteFile(const char *filename);
uint8_t         SDCard_FileExists(const char *filename);

char           *SDCard_ReadLine(char *src, char *line_out, uint16_t max_len);

#endif /* INC_SDCARD_H_ */
