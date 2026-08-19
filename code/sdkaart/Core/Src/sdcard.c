/*
 * sdcard.c
 *
 *  Created on: 28 jun 2026
 *      Author: tijme
 *
 *
 */

#include "sdcard.h"
#include <stddef.h>     /* NULL */

static FATFS   sd_fatfs;
static uint8_t sd_mounted = 0;

/* -------------------------------------------------------------------------
 *Mounten en unmounten
 * ------------------------------------------------------------------------- */

SDCard_Status_t SDCard_Mount(void)
{
    if (sd_mounted) return SDCARD_OK;          /* al gemount, niets te doen */

    if (f_mount(&sd_fatfs, SDPath, 1) != FR_OK) {
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
    return sd_mounted;                          /* 1 = gemount, 0 = niet */
}

/* -------------------------------------------------------------------------
 *Schrijven
 *
 * ------------------------------------------------------------------------- */

static SDCard_Status_t write_common(const char *filename, const uint8_t *data,
                                    uint32_t size, BYTE mode)
{
    if (!sd_mounted) return SDCARD_ERR_NOT_MOUNTED;

    FIL file;
    if (f_open(&file, filename, mode) != FR_OK) return SDCARD_ERR_OPEN;

    UINT written = 0;
    FRESULT res = f_write(&file, data, size, &written);
    f_close(&file);

    /*f_write kan minder wegschrijven dan gevraagd als de kaart vol zit */
    if (res != FR_OK || written != size) return SDCARD_ERR_WRITE;
    return SDCARD_OK;
}

SDCard_Status_t SDCard_WriteFile(const char *filename, const uint8_t *data, uint32_t size)
{
    return write_common(filename, data, size, FA_CREATE_ALWAYS | FA_WRITE);
}

SDCard_Status_t SDCard_AppendFile(const char *filename, const uint8_t *data, uint32_t size)
{
    return write_common(filename, data, size, FA_OPEN_APPEND | FA_WRITE);
}

/* -------------------------------------------------------------------------
 *Leze
 * ------------------------------------------------------------------------- */

SDCard_Status_t SDCard_ReadFile(const char *filename, uint8_t *buf, uint32_t buf_size,
                                uint32_t *bytes_read)
{
    if (bytes_read != NULL) *bytes_read = 0;
    if (!sd_mounted) return SDCARD_ERR_NOT_MOUNTED;

    FIL file;
    if (f_open(&file, filename, FA_READ) != FR_OK) return SDCARD_ERR_OPEN;

    UINT read = 0;
    FRESULT res = f_read(&file, buf, buf_size, &read);
    f_close(&file);

    if (bytes_read != NULL) *bytes_read = (uint32_t)read;
    return (res == FR_OK) ? SDCARD_OK : SDCARD_ERR_READ;
}

/* Eén regel uit een tekstbuffer halen \r wordt weggelaten zodat een bestand
 * dat op Windows is gemaakt ook gewoon werktg eeft het begin van de volgende
 * regel terug, of NULL als de buffer op is */
char *SDCard_ReadLine(char *src, char *line_out, uint16_t max_len)
{
    if (src == NULL || *src == '\0' || max_len == 0) return NULL;

    uint16_t i = 0;
    while (*src != '\0' && *src != '\n' && i < (uint16_t)(max_len - 1)) {
        if (*src != '\r') line_out[i++] = *src;
        src++;
    }
    line_out[i] = '\0';

    while (*src != '\0' && *src != '\n') src++;   //rest van een te lange regel weg
    if (*src == '\n') src++;
    return src;
}

/* -------------------------------------------------------------------------
 *Beheren
 * ------------------------------------------------------------------------- */

SDCard_Status_t SDCard_DeleteFile(const char *filename)
{
    if (!sd_mounted) return SDCARD_ERR_NOT_MOUNTED;
    return (f_unlink(filename) == FR_OK) ? SDCARD_OK : SDCARD_ERR_OPEN;
}

uint8_t SDCard_FileExists(const char *filename)
{
    if (!sd_mounted) return 0;

    FILINFO fno;
    return (f_stat(filename, &fno) == FR_OK) ? 1 : 0;
}
