/*
 * BMI330.c
 *
 *  Created on: 28 feb 2026
 *      Author: simon
 */

#include "BMI330.h"
#include "math.h"

extern SPI_HandleTypeDef hspi1;
BMI330_Frame frame;

void BMI330_Init(void){
	uint16_t id;
	uint16_t error;
	uint16_t status;
	uint16_t feature;
	uint16_t feature_status = 0;
	uint32_t timeout = 100;
	// De volgende 3 regs en de while loop is een soort protocol om de feature engine op te starten
	BMI330_WriteReg(BMI330_FEATURE_IO2, 0x012C);
	BMI330_WriteReg(BMI330_FEATURE_IO_STATUS, 0x01);
	BMI330_WriteReg(BMI330_FEATURE_CTRL, 0x01);

	do {
	    BMI330_ReadReg(BMI330_FEATURE_IO1, &feature_status, 2);
	    HAL_Delay(1);
	    timeout--;
	} while ((feature_status != 0x0001) && (timeout > 0));

	if (timeout == 0){
		printf("Fout bij initialisatie BMI330");
	}

	BMI330_ReadReg(BMI330_CHIP_ID, &id, 2);
	BMI330_ReadReg(BMI330_ERR_REG, &error, 2);
	BMI330_ReadReg(BMI330_STATUS, &status, 2);
	BMI330_ReadReg(BMI330_FEATURE_IO1, &feature, 2);
	if(id != BMI330_ID || error != 0x00 || status != 0x01 || feature != 0x05){	// Als hier fout is kijk bij power up status en feature reg andere waardes?
		printf("Fout bij initialisatie BMI330");
	}

	// In de volgende 4 registers gaan we waardes schrijven naar de extended memory map
	BMI330_WriteReg(BMI330_FEATURE_DATA_ADDR, BMI330_ANYMO_1);
	BMI330_WriteReg(BMI330_FEATURE_DATA_TX, 0x66);	// Waarde = G-drempel * 512			66 is maar een testwaarde voor 0.2g verander indien nodig
	BMI330_WriteReg(BMI330_FEATURE_DATA_ADDR, BMI330_ANYMO_3);
	BMI330_WriteReg(BMI330_FEATURE_DATA_ADDR, 0x6030);		// 60 is de default wait time en 30 is het aantal samples dat het boven de drempel moet zijn, verander indien nodig
	BMI330_WriteReg(BMI330_FEATURE_IO0, 0x38);				// Stel in feature anymotion in voor de x,y en z
	BMI330_WriteReg(BMI330_FEATURE_IO_STATUS, 0x01);		// We doen dit om de veranderingen in IO0 door te voeren

	BMI330_WriteReg(BMI330_ACC_CONF	, 0x70AB);		// Stel de accelormeter in
	BMI330_WriteReg(BMI330_GYR_CONF	, 0x70CB);		// Stel de gyroscoop in
	// Als de drone niet beweegt gaan we de sensor heel traag/niet laten werken en dan nemen we de alt registers
	BMI330_WriteReg(BMI330_ALT_ACC_CONF	, 0x11);	// Stel de alt accelormeter instellingen in
	BMI330_WriteReg(BMI330_ALT_GYR_CONF	, 0x00);	// Stel de alt gyroscoop instellingen in
	BMI330_WriteReg(BMI330_ALT_CONF, 0x00);			// 0 = Normale regs, 1 = ALT regs
	BMI330_WriteReg(BMI330_FIFO_CONF, 0x0700);		// Stel in welke waardes er in de fifo komen, timestamps, wat bij overflow
	BMI330_WriteReg(BMI330_IO_INT_CTRL, 0x05);		// INT als output, push pull en actief hoog
	BMI330_WriteReg(BMI330_INT_CONF, 0x00);			// we zeggen dat het interuptsignaal niet latched moet blijven maar gwn een puls mag geven
	BMI330_WriteReg(BMI330_INT_MAP2, 0x1000); 		// we zeggen dat er een interupt op int1 moet komen vanaf de watermark actief is
	BMI330_WriteReg(BMI330_FIFO_WATERMARK, 15);		// we gaan een interupt sturen als er 15 bytes in de fifo zitten gyro X,Y,Z = 6 en ook voor acc dus 12 bytes + 3 bytes voor tijd
	BMI330_WriteReg(BMI330_INT_MAP1, 0x08);			// we zetten de any motion interupt op int 2

	return;
}

/*
 * @brief schrijf een waarde naar een register
 * @param reg addres van het reg waar je data naar wilt sturen
 * @param val is de value die je in het register wilt steken
 */
void BMI330_WriteReg(uint8_t reg, uint16_t val){
	//MSB van Data is om te zeggen voor Read of Write(1 = Read, 0 = Write)
	//write commando = 16 bits = (R/W + Address + Data(16bit) = 1 + 7 + 8 + 8)
	//einde van sturen = CS hoog
	uint8_t data[3] = {reg & 0x7F, val & 0xFF, val >> 8 & 0xFF};
	HAL_GPIO_WritePin(BMI_CS_GPIO_Port, BMI_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, data, 3, 100);
	HAL_GPIO_WritePin(BMI_CS_GPIO_Port, BMI_CS_Pin, GPIO_PIN_SET);
}

/*
 * @brief meeste register uitlezen uit de BMI330
 * @param reg addres van het reg dat je wilt uitlezen
 * @param pData pionter naar het addres waar je de data gaat opslagen
 * @param len Het aantal bytes van data dat je wilt inlezen
 */
void BMI330_ReadReg(uint8_t reg, uint16_t *pData, uint8_t len){
	//Begin is zelfde als schrijven, 1 RW bit dan 7 adres bits
	//hierna 1 dummy byte en daarna data.
	//Bij meerdere blokken data => adres auto increment

	uint8_t addres = reg | 0x80;
	uint8_t dummy;
	HAL_GPIO_WritePin(BMI_CS_GPIO_Port, BMI_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &addres, 1, 100);
	HAL_SPI_Receive(&hspi1, &dummy, 1, 100);
	HAL_SPI_Receive(&hspi1, (uint8_t*)pData, len*2, 100);	//len moet x2 want we hebben 16 bit registers ipv 8 bit
	HAL_GPIO_WritePin(BMI_CS_GPIO_Port, BMI_CS_Pin, GPIO_PIN_SET);
}


float BMI330_GetTemp(void){
	uint16_t temperature;
	BMI330_ReadReg(BMI330_TEMP_DATA, &temperature, (uint8_t)2);
	return (float)temperature / 256.0 + 23.0;
}

/**
 * @brief Leest een specifiek aantal bytes uit de FIFO buffer
 * @param pData Pointer naar de array waar de data in moet (meestal 15 bytes voor 1 frame)
 * @param len Aantal bytes om te lezen (voor jouw setup is dit 15)
 */
void BMI330_ReadFIFO(uint8_t *pData, uint16_t len) {
    uint8_t address = BMI330_FIFO_DATA;
    uint8_t dummy;

    HAL_GPIO_WritePin(BMI_CS_GPIO_Port, BMI_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &address, 1, 100);
    HAL_SPI_Receive(&hspi1, &dummy, 1, 100);
    HAL_SPI_Receive(&hspi1, pData, len, 100);
    HAL_GPIO_WritePin(BMI_CS_GPIO_Port, BMI_CS_Pin, GPIO_PIN_SET);
}

/*
 * @brief We nemen ons frame van 15 bytes en splitsen dit op
 * @param de ruwe data van de sensor
 * @param het frame waar we de goede data insteken
 * @note we delen de waardes door 4096 en 16384 om naar de juiste eenheden te gaan
 */
void BMI330_ParseFIFOFrame(uint8_t *raw_data, BMI330_Frame *frame) {
	// voor ACC:
	// 32768(16 bit getal) / 8(sensor = 8g) = 4096
	// 32768 / 2000(dps) = 16,384
    // Accelerometer (Byte 0 tot 5)
    frame->acc_x = ((float)((raw_data[1] << 8) | raw_data[0])) / 4096;
    frame->acc_y = ((float)((raw_data[3] << 8) | raw_data[2])) / 4096;
    frame->acc_z = ((float)((raw_data[5] << 8) | raw_data[4])) / 4096;

    // Gyroscoop (Byte 6 tot 11)
    frame->gyr_x = ((float)((raw_data[7] << 8) | raw_data[6])) / 16.384;
    frame->gyr_y = ((float)((raw_data[9] << 8) | raw_data[8])) / 16.384;
    frame->gyr_z = ((float)((raw_data[11] << 8) | raw_data[10])) / 16.384;

    // Sensortime (Byte 12 t/m 14)
    frame->sensor_temp = (uint16_t)((raw_data[13] << 8) | raw_data[12]);
}

/**
 * @brief Berekent de Roll en Pitch hoeken in graden
 * @param frame De struct met geschaalde (float) acc waarden
 */
void BMI330_GetAngles(BMI330_Frame *frame, float *roll, float *pitch) {
    // Roll berekenen = rond X as
    *roll = atan2f(frame->acc_y, frame->acc_z) * (180 / PI);

    // Pitch berekenen = rond y as
    float acc_z_y_combined = sqrtf(frame->acc_y * frame->acc_y + frame->acc_z * frame->acc_z);
    *pitch = atan2f(-frame->acc_x, acc_z_y_combined) * (180 / PI);
}
