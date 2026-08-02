/*
 * BMM350v2.h
 *
 * From-scratch driver voor de BMM350-magnetometer (Bosch, I2C1). Losstaand
 * van de bestaande BMM350.c/.h (die blijft ongewijzigd, maar staat uitgesloten
 * van de build in dit project).
 *
 * Verschil met de oude driver:
 *  - elke I2C-transactie checkt zijn eigen HAL-status
 *  - BMM350v2_Init() geeft een duidelijke foutcode terug i.p.v. enkel een printf
 *  - BMM350v2_TiltCompensatedHeading() gebruikt geen impliciete globale
 *    BMM350_Data meer (die in de oude driver nergens gedeclareerd stond,
 *    dus niet compileerde) en verwacht roll/pitch in GRADEN, net als de rest
 *    van deze codebase (attitude.c) - de oude driver stak die rechtstreeks in
 *    cosf/sinf, die radialen verwachten.
 */

#ifndef INC_BMM350V2_H_
#define INC_BMM350V2_H_

#include "stm32f7xx_hal.h"
#include <stdint.h>

typedef enum {
    BMM350V2_OK = 0,
    BMM350V2_ERR_I2C, /* een I2C-transactie is mislukt/timeout */
    BMM350V2_ERR_ID,  /* chip-ID komt niet overeen (verkeerde sensor/bekabeling) */
} BMM350v2_Status;

/* Hard-iron kalibratie: houdt de min/max van elke as bij tijdens het rondjes
 * draaien met de sensor, en levert daaruit de offset per as. */
typedef struct {
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    float offset_x, offset_y, offset_z;
} BMM350v2_Calib;

/* Verifieert de chip-ID en zet de sensor in normal mode (ODR + averaging,
 * interrupt op data-ready). Blokkerend. */
BMM350v2_Status BMM350v2_Init(void);

/* 1 = nieuwe magnetometerdata beschikbaar, 0 = nog niet. */
uint8_t BMM350v2_IsDataReady(void);

/* Leest de ruwe (niet-gekalibreerde) X/Y/Z-tellingen. */
BMM350v2_Status BMM350v2_ReadRaw(int32_t *x, int32_t *y, int32_t *z);

/* Kalibratie: reset de min/max-tracking (aanroepen voor je begint rond te draaien). */
void BMM350v2_CalibrateReset(BMM350v2_Calib *calib);

/* Kalibratie: 1 sample verwerken terwijl je de sensor ronddraait (alle assen). */
void BMM350v2_CalibrateSample(BMM350v2_Calib *calib, int32_t raw_x, int32_t raw_y, int32_t raw_z);

/* Kalibratie: berekent offset_x/y/z uit de tot nu toe gemeten min/max. */
void BMM350v2_CalibrateFinish(BMM350v2_Calib *calib);

/* Kompashoek (0-360 graden, 0 = magnetisch noorden) uit gekalibreerde X/Y,
 * zonder rekening te houden met roll/pitch (enkel geldig als de sensor vlak ligt). */
float BMM350v2_Heading(float cal_x, float cal_y);

/* Zelfde, maar met tilt-compensatie: roll/pitch in GRADEN (bv. van attitude.c),
 * zodat de hoek ook klopt als de drone niet perfect vlak hangt. */
float BMM350v2_TiltCompensatedHeading(int32_t raw_x, int32_t raw_y, int32_t raw_z,
                                       const BMM350v2_Calib *calib,
                                       float roll_deg, float pitch_deg);

#endif /* INC_BMM350V2_H_ */
