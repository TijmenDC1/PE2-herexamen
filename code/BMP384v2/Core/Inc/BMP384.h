/*
* dshotsturen.h
*
* Created on: 31 jun 2026
* Author: tijme
*/

#ifndef INC_BMP384_H_
#define INC_BMP384_H_

#include "stm32f7xx_hal.h"
#include <stdint.h>

typedef struct {
    float t1, t2, t3;
    float p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
    float t_lin; //compensatie-tussenwaarde, gezet door BMP384_ReadPressureTemperature()
} BMP384_Calib;

typedef enum {
    BMP384_OK = 0,
    BMP384_ERR_SPI, /*een SPI-transactie is mislukt/timeout */
    BMP384_ERR_ID,  /*chip-ID komt niet overeen (verkeerde sensor/bekabeling) */
    BMP384_ERR_DATA,/*SPI werkt, maar de ruwe ADC-data is 0x000000 of 0xFFFFFF */
} BMP384_Status;

/*Verifieert de chip-ID, doet een soft-reset, leest de fabriekskalibratie uit
 *het NVM en zet de sensor in normal mode (druk + temp aan, x8 oversampling
 *op druk, IIR-filter aan). Blokkerend. */
BMP384_Status BMP384_Init(BMP384_Calib *calib);

/*1 = nieuwe druk- en temperatuurdata beschikbaar, 0 = nog niet. */
uint8_t BMP384_IsDataReady(void);

/*Leest en compenseert 1 druk+temperatuur-meting.
 *pressure_pa/temperature_c mogen NULL zijn als je die waarde niet nodig hebt. */
BMP384_Status BMP384_ReadPressureTemperature(BMP384_Calib *calib,
                                              float *pressure_pa,
                                              float *temperature_c);

/*Leest de ongecompenseerde 24-bit ADC-waarden uit. Handig om te debuggen:
 *als deze 0 of 0xFFFFFF zijn, ligt het probleem bij SPI/config, niet bij de
 *compensatieformules. */
BMP384_Status BMP384_ReadRaw(uint32_t *raw_press, uint32_t *raw_temp);

/*Dumpt alle relevante registers en controleert of SENSORTIME doorloopt. */
void BMP384_Diagnose(void);

/*Doet 1 meting in forced mode met de simpelst mogelijke instellingen. */
BMP384_Status BMP384_ForcedTest(uint32_t *raw_press, uint32_t *raw_temp);

/*Rekent een pascalwaarde om naar hoogte (m) t.o.v. ground_pressure_pa. */
float BMP384_PressureToAltitude(float pressure_pa, float ground_pressure_pa);

#endif /* INC_BMP384_H_ */
