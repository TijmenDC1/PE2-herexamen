/*
 * fatfs.h - namaak-FATFS voor de PC-test
 *
 * sdcard.c praat met FATFS. Op de PC bestaat FATFS niet, dus vervangen we het
 * door een handvol functies die gewoon in een map ./sdtest/ schrijven. Zo kan
 * sdcard.c en flightplan.c ongewijzigd meecompileren en test je echt de code
 * die straks op de STM32 draait.
 *
 * Alleen voor de test in deze map. STM32CubeIDE compileert deze map niet mee
 * (test/ staat niet bij de source folders van het project).
 */

#ifndef HOST_FATFS_H
#define HOST_FATFS_H

#include <stdint.h>
#include <stdio.h>

typedef unsigned int UINT;
typedef struct { int dummy; }        FATFS;
typedef struct { FILE *fp; }         FIL;
typedef struct { unsigned long fsize; } FILINFO;
typedef enum { FR_OK = 0, FR_ERR = 1 } FRESULT;

#define FA_READ           0x01
#define FA_WRITE          0x02
#define FA_CREATE_ALWAYS  0x08
#define FA_OPEN_APPEND    0x30

extern char SDPath[4];

FRESULT f_mount (FATFS *fs, const char *path, uint8_t opt);
FRESULT f_open  (FIL *fp, const char *path, uint8_t mode);
FRESULT f_write (FIL *fp, const void *buf, UINT n, UINT *written);
FRESULT f_read  (FIL *fp, void *buf, UINT n, UINT *read);
FRESULT f_close (FIL *fp);
FRESULT f_unlink(const char *path);
FRESULT f_stat  (const char *path, FILINFO *fno);

#endif /* HOST_FATFS_H */
