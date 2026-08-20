/*
 * BMM350_API.c
 *
 *  Created on: 2 apr 2026
 *      Author: simon
 */


#include "bmm350.h"
#include "stm32f7xx_hal.h"

// De I2C handle uit je STM32Cube configuratie
extern I2C_HandleTypeDef hi2c1;

/* * 1. Wrapper voor de I2C Read
 * De Bosch API roept deze functie aan wanneer het data nodig heeft.
 */
BMM350_INTF_RET_TYPE stm32_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    // De Bosch API vangt de BMM350 specifieke "dummy bytes" intern af,
    // we kunnen hier dus de standaard HAL I2C read gebruiken.
    if (HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, 100) == HAL_OK) {
        return BMM350_OK;
    }
    return BMM350_E_COM_FAIL;
}

/* * 2. Wrapper voor de I2C Write
 */
BMM350_INTF_RET_TYPE stm32_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    if (HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)reg_data, len, 100) == HAL_OK) {
        return BMM350_OK;
    }
    return BMM350_E_COM_FAIL;
}

/* * 3. Wrapper voor de Delay
 * (Bosch API werkt met microseconden, STM32 HAL_Delay werkt met milliseconden)
 */
void stm32_delay_us(uint32_t period, void *intf_ptr) {
    // Zet microseconden om naar milliseconden en voeg 1ms toe om te vroeg ontwaken te voorkomen
    uint32_t ms = (period / 1000) + 1;
    HAL_Delay(ms);
}

/* * 4. De Hoofd Initialisatie
 */
void Setup_BMM350_Bosch_API(void) {
    struct bmm350_dev dev;
    int8_t rslt;

    // Het 7-bit adres (0x14) moet voor de STM32 HAL 1 bit naar links worden geschoven
    uint8_t i2c_addr = (0x14 << 1);

    // Verbind de functies die we hierboven hebben geschreven met de API
    dev.intf = BMM350_I2C_INTF;
    dev.intf_ptr = &i2c_addr;
    dev.read = stm32_i2c_read;
    dev.write = stm32_i2c_write;
    dev.delay_us = stm32_delay_us;

    // Voer de initialisatie uit via de Bosch API bestanden
    rslt = bmm350_init(&dev);

    if (rslt == BMM350_OK) {
        // Succes! De API is gestart en de compensatiedata uit de OTP is ingeladen
    } else {
        // Error handling (bv. check of de bekabeling goed is)
    }
}
