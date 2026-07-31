/*
 * sdcard.c
 *
 *  Created on: 28 jun 2026
 *      Author: tijme
 *
 * SD kaart driver bovenop FATFS van CubeMX
 * SD kaart zit op SDMMC1.

 */


/*
 * SDCard_Mount(void) 		sd kaart mounten
 * SDCard_Unmount(void) 	sd kaart unmounten
 * SDCard_IsMounted 		checken of sd kaart gemount is als ze er is 1 anders 0
 * SDCard_WriteFile			nieuw bestand toevoegen
 * SDCard_AppendFile		toevoegen aan bestaand bestand
 * SDCard_ReadFile			bestand lezen
 * SDCard_ReadLine			één regel lezen uit een tekstbuffer
 * SDCard_DeleteFile		bestand verwijderen
 * SDCard_FileExists		checken of bestand bestaat
 * */


#include "sdcard.h"
#include <string.h>

static FATFS  sd_fatfs;
static uint8_t sd_mounted = 0;

/* -------------------------------------------------------------------------
Mount en unmount van de sd kaart
------------------------------------------------------------------------- */

SDCard_Status_t SDCard_Mount(void)
{
    FRESULT res = f_mount(&sd_fatfs, SDPath, 1);
    if (res != FR_OK) {
        sd_mounted = 0;
        return SDCARD_ERR_MOUNT;
    }
    sd_mounted = 1;
    return SDCARD_OK;
}

void SDCard_Unmount(void)
{
    f_mount(NULL, SDPath, 0);
    sd_mounted = 0;
}

uint8_t SDCard_IsMounted(void)
{
    return sd_mounted;			// als 1 = gemount anders unmounted
}

/*
 * Schrijven nieuw bestand of toevoegen aan bestaand bestand
 * Eventuele vlucht informatie of defecten terugschrijven om daarna te kunnen uitlezen
 *  */

SDCard_Status_t SDCard_WriteFile(const char *filename, const uint8_t *data, uint32_t size)
{
    if (sd_mounted == 0)
    	return SDCARD_ERR_NOT_MOUNTED;

    FIL file;
    FRESULT res = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK)
    	return SDCARD_ERR_OPEN;

    UINT written;
    res = f_write(&file, data, size, &written);
    f_close(&file);

    if (res == FR_OK)
    	return SDCARD_OK;
    else
    	return SDCARD_ERR_WRITE;
}


SDCard_Status_t SDCard_AppendFile(const char *filename, const uint8_t *data, uint32_t size)
{
    if (sd_mounted == 0)
    	return SDCARD_ERR_NOT_MOUNTED;

    FIL file;
    FRESULT res = f_open(&file, filename, FA_OPEN_APPEND | FA_WRITE);
    if (res != FR_OK)
        	return SDCARD_ERR_OPEN;

    UINT written;
    res = f_write(&file, data, size, &written);
    f_close(&file);

    if (res == FR_OK)
        	return SDCARD_OK;
        else
        	return SDCARD_ERR_WRITE;
}

/*
 Lezen van bestanden voor vlucht informatie
 *  */

SDCard_Status_t SDCard_ReadFile(const char *filename, uint8_t *buf, uint32_t buf_size, uint32_t *bytes_read)
{
	if (sd_mounted == 0)
	    	return SDCARD_ERR_NOT_MOUNTED;

    FIL file;
    FRESULT res = f_open(&file, filename, FA_READ);
    if (res != FR_OK)
            	return SDCARD_ERR_OPEN;

    UINT read;
    res = f_read(&file, buf, buf_size, &read);
    f_close(&file);

    if (bytes_read != NULL) *bytes_read = read;

    if (res == FR_OK)
            	return SDCARD_OK;
            else
            	return SDCARD_ERR_READ;
}

/*
 * Lees één regel uit een tekstbuffer
 * Geeft een pointer terug naar het begin van de volgende regel
 * of NULL als het einde van de buffer bereikt is
 *  */

char *SDCard_ReadLine(char *src, char *line_out, uint16_t max_len)
{
    if (src == NULL || *src == '\0') return NULL;

    uint16_t i = 0;
    while (*src != '\0' && *src != '\n' && i < max_len - 1) {
        if (*src != '\r') line_out[i++] = *src;
        src++;
    }
    line_out[i] = '\0';
    if (*src == '\n') src++;
    return src;
}

/*
 *
 Bestand verwijderen
 *
 *  */

SDCard_Status_t SDCard_DeleteFile(const char *filename)
{
	if (sd_mounted == 0)
		    	return SDCARD_ERR_NOT_MOUNTED;

    FRESULT res = f_unlink(filename);
    if (res == FR_OK)
            	return SDCARD_OK;
            else
            	return SDCARD_ERR_OPEN;
}

/*
 *
 Checken of bestand bestaat
 *
 *  */

uint8_t SDCard_FileExists(const char *filename)
{
	if (sd_mounted == 0)
			    	return 0;

    FILINFO fno;

    if (f_stat(filename, &fno) == FR_OK)
    	return 1;
    else
    	return 0;
}
