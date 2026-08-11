/*
 * host_fatfs.c - namaak-FATFS voor de PC-test, zie fatfs.h
 *
 * De "SD-kaart" is de map ./sdtest/ naast het testprogramma. Na het draaien
 * kan je daar met een gewone editor in kijken.
 */

#include "fatfs.h"

#include <string.h>
#include <stdlib.h>

#if defined(_WIN32)
  #include <direct.h>
  #define MAKE_DIR(p)  _mkdir(p)
#else
  #include <sys/stat.h>
  #define MAKE_DIR(p)  mkdir((p), 0777)
#endif

char SDPath[4] = "";

static const char *SD_DIR = "sdtest";
static char pathbuf[512];

static const char *full(const char *p)
{
    snprintf(pathbuf, sizeof(pathbuf), "%s/%s", SD_DIR, p);
    return pathbuf;
}

FRESULT f_mount(FATFS *fs, const char *path, uint8_t opt)
{
    (void)path; (void)opt;
    if (fs == NULL) return FR_OK;      /* unmount */
    MAKE_DIR(SD_DIR);
    return FR_OK;
}

FRESULT f_open(FIL *fp, const char *path, uint8_t mode)
{
    const char *m = "rb";
    if (mode & FA_CREATE_ALWAYS)                 m = "wb";
    else if ((mode & FA_OPEN_APPEND) == FA_OPEN_APPEND) m = "ab";

    fp->fp = fopen(full(path), m);
    return (fp->fp != NULL) ? FR_OK : FR_ERR;
}

FRESULT f_write(FIL *fp, const void *buf, UINT n, UINT *written)
{
    *written = (UINT)fwrite(buf, 1, n, fp->fp);
    return (*written == n) ? FR_OK : FR_ERR;
}

FRESULT f_read(FIL *fp, void *buf, UINT n, UINT *read)
{
    *read = (UINT)fread(buf, 1, n, fp->fp);
    return FR_OK;
}

FRESULT f_close(FIL *fp)
{
    if (fp->fp) fclose(fp->fp);
    fp->fp = NULL;
    return FR_OK;
}

FRESULT f_unlink(const char *path)
{
    return (remove(full(path)) == 0) ? FR_OK : FR_ERR;
}

FRESULT f_stat(const char *path, FILINFO *fno)
{
    FILE *f = fopen(full(path), "rb");
    if (f == NULL) return FR_ERR;
    fclose(f);
    fno->fsize = 0;
    return FR_OK;
}
