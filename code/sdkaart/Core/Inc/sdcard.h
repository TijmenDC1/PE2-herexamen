/*
 * sdcard.h
 *
 *  Created on: 28 jun 2026
 *      Author: tijme
 *
 * SD kaart driver bovenop FATFS van CubeMX
 */

/*
 * DRIVER VIA FATFS LIBRARY VAN CUBEMX
 *
 *SDCard_Mount          kaart mounten
 *SDCard_Unmount         kaart unmounten
 *SDCard_IsMounted          1 als de kaart gemount is, anders 0
 *SDCard_WriteFile    		nieuw bestand aanmaken en schrijven (bestaande wordt overschreven)
 *SDCard_AppendFile   		achteraan een bestaand bestand toevoegen
 *SDCard_ReadFile     	bestand in een buffer lezen
 *SDCard_ReadLine    	 een regel uit een tekstbuffer halen (geen SD nodig)
 *SDCard_DeleteFile  	 bestand verwijderen
 *SDCard_FileExists   checken of een bestand bestaat
 */

#ifndef SDCARD_H
#define SDCARD_H

#include "fatfs.h"
#include <stdint.h>

typedef enum {
    SDCARD_OK = 0,
    SDCARD_ERR_MOUNT,
    SDCARD_ERR_NOT_MOUNTED,
    SDCARD_ERR_OPEN,
    SDCARD_ERR_READ,
    SDCARD_ERR_WRITE
} SDCard_Status_t;

SDCard_Status_t SDCard_Mount(void);
void            SDCard_Unmount(void);
uint8_t         SDCard_IsMounted(void);

SDCard_Status_t SDCard_WriteFile (const char *filename, const uint8_t *data, uint32_t size);
SDCard_Status_t SDCard_AppendFile(const char *filename, const uint8_t *data, uint32_t size);
SDCard_Status_t SDCard_ReadFile  (const char *filename, uint8_t *buf, uint32_t buf_size,
                                  uint32_t *bytes_read);

SDCard_Status_t SDCard_DeleteFile(const char *filename);
uint8_t         SDCard_FileExists(const char *filename);

/* Leest een regel uit een tekstbuffer die al in RAM staat.
 * Geeft een pointer terug naar het begin van de volgende regel, of NULL op het einde. */
char *SDCard_ReadLine(char *src, char *line_out, uint16_t max_len);

#endif /* SDCARD_H */

