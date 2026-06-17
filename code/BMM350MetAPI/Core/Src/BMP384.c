/*
 * BMP384.c
 *
 *  Created on: 26 feb 2026
 *      Author: simon
 */

#include <BMP384.h>
#include <math.h>

extern SPI_HandleTypeDef hspi4;

/*
 * Wij gaan normal mode gebruiken dus 33 naar pwr_ctrl register
 * Druk moet accuraat zijn maar niet mega dus x8 oversampling = 011 naar osr_p register
 * De oversampling en normal mode worden in de datasheet zo aangeraden voor een drone applicatie
 */


float g_ground_pa = 0;

/*
 * @brief We gaan de calibratiedata in de struct steken om hiermee straks de data te kunnen calibreren en juiste data te krijgen
 * @use deze functie wordt opgeroepen bij de init van de sensor en later gebruikt bij het lezen van data
 * @note Deze data komt uit de DATASHEET p.55 Hoofdstuk 9.1
 */
static void BMP384_ReadCalibration(BMP384_Calib *calibData) {
    uint8_t cal[21];
    BMP384_ReadReg(BMP384_NVM_PAR_T1_LSB, cal, 21);	// per read gaat het reg address automatisch 1 omhoog, hierdoor kunnen we al de 21 kalib. registers in 1x uitlezen

    // Temperatuur
    calibData->t1  =  (float)((uint16_t)((cal[1] << 8) | cal[0])) / powf(2, -8);	// we maken van 2 8 bits 1 16 bits getal
    calibData->t2  =  (float)((uint16_t)((cal[3] << 8) | cal[2])) / powf(2, 30);
    calibData->t3  =  (float)((int8_t)cal[4]) / powf(2, 48);

    // Druk
    calibData->p1  =  ((float)((int16_t)((cal[6] << 8) | cal[5])) - powf(2, 14)) / powf(2, 20);
    calibData->p2  =  ((float)((int16_t)((cal[8] << 8) | cal[7])) - powf(2, 14)) / powf(2, 29);
    calibData->p3  =  (float)((int8_t)cal[9])   / powf(2, 32);
    calibData->p4  =  (float)((int8_t)cal[10])  / powf(2, 37);
    calibData->p5  =  (float)((uint16_t)((cal[12] << 8) | cal[11])) / powf(2, -3);
    calibData->p6  =  (float)((uint16_t)((cal[14] << 8) | cal[13])) / powf(2, 6);
    calibData->p7  =  (float)((int8_t)cal[15])  / powf(2, 8);
    calibData->p8  =  (float)((int8_t)cal[16])  / powf(2, 15);
    calibData->p9  =  (float)((int16_t)((cal[18] << 8) | cal[17]))  / powf(2, 48);
    calibData->p10 = (float)((int8_t)cal[19])  / powf(2, 48);
    calibData->p11 = (float)((int8_t)cal[20])  / powf(2, 65);
}


/*
 * @brief Hier gaan we de gelezen data gaan compenseren met de calibratiedata die we in BMP384_ReadCalibration hebben verkregen
 * @param uncomp_p dit is de ongecompenseerde druk data
 * @param t_lin dit is de compensatiewaarde van de temperatuur die we nodig hebben voor de compensatie op de druk
 * @note Deze functie komt uit de DATASHEET p.56 Hoofdstuk 9.3
 */
static float BMP384_Compensate_Pressure(uint32_t uncomp_p, BMP384_Calib *calibData) {
	float t_lin = BMP384_GetTemp(calibData);
	// deze functie komt bijna rechtstreeks uit datasheet hoofdstuk 9
    float partial_data1 = calibData->p6 * t_lin;
    float partial_data2 = calibData->p7 * (t_lin * t_lin);
    float partial_data3 = calibData->p8 * (t_lin * t_lin * t_lin);
    float partial_out1 = calibData->p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = calibData->p2 * t_lin;
    partial_data2 = calibData->p3 * (t_lin * t_lin);
    partial_data3 = calibData->p4 * (t_lin * t_lin * t_lin);
    float partial_out2 = (float)uncomp_p * (calibData->p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float)uncomp_p * (float)uncomp_p;
    partial_data2 = calibData->p9 + calibData->p10 * t_lin;
    partial_data3 = partial_data1 * partial_data2;
    float partial_data4 = partial_data3 + ((float)uncomp_p * (float)uncomp_p * (float)uncomp_p) * calibData->p11;

    return partial_out1 + partial_out2 + partial_data4;
}



HAL_StatusTypeDef BMP384_Init(BMP384_Calib *calibData){
	uint8_t id;
	BMP384_ReadReg(BMP384_CHIP_ID, &id, 1);
	if(id != BMP384_ID	){
		printf("ID komt niet overeen\nID is %d\n\r", id);
		HAL_Delay(3000);
		return HAL_ERROR;
	}else{
		printf("succesvol geconnect met id %d\n\r", id);
		HAL_Delay(3000);
	}
	printf("ID check gedaan\n");
	BMP384_WriteReg(BMP384_CMD, BMP384_CMD_SOFTRESET);
	HAL_Delay(10);
	BMP384_ReadCalibration(calibData);
	BMP384_WriteReg(BMP384_PWR_CTRL, BMP384_MODE_NORMAL );	// Zet de mode op normal en temp aan en press on
	BMP384_WriteReg(BMP384_OSR, 3);							// Oversampling Pressure, zet op x8 voor hoge resolutie
	BMP384_WriteReg(BMP384_CONFIG, 4);						// Zet de iir filter cooficient op 3
	BMP384_WriteReg(BMP384_ODR, 2);
	BMP384_WriteReg(BMP384_INT_CTRL	, 0x42);				// we stellen in hoe de INT pin moet werken en dat het een INT stuurd wnr de data klaar is
	BMP384_ground(calibData);										// bereken de druk op de grond.

	return HAL_OK;
}

/*
 * @brief in bijna elk register een bepaalde waarde steken
 * @param reg addres van het reg waar je data naar wilt sturen
 * @param val is de value die je in het register wilt steken
 */
void BMP384_WriteReg(uint8_t reg, uint8_t val){
	//MSB van Data is om te zeggen voor Read of Write(1 = REad, 0 = write)
	//write commando = 16 bits = (R/W + Address + Data = 1 + 7 + 8)
	//einde van sturen = CS hoog
	uint8_t data[2] = {reg & 0x7F, val};
	HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET);
	// HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
	HAL_SPI_Transmit(&hspi4, data, 2, 1000);
	HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);
}

/*
 * @brief elk register uitlezen uit de BMP384
 * @param reg addres van het reg dat je wilt uitlezen
 * @param pData pionter naar het addres waar je de data gaat lopslagen
 * @param len Het aantal bytes van data dat je wilt inlezen
 */
void BMP384_ReadReg(uint8_t reg, uint8_t *pData, uint16_t len){
	//Begin is zelfde al schrijven, 1 RW bit dan 7 adres bits
	//hierna 1 dummy byte en daarna data.
	//Bij meerdere blokken data => adres auto increment

	uint8_t data = reg | 0x80;
	uint8_t dummy;
	HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4, &data, 1, 100);
	HAL_SPI_Receive(&hspi4, &dummy, 1, 100);
	HAL_SPI_Receive(&hspi4, pData, len, 100);
	HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);
}


/*
 * @brief Deze functie gaat de druk lezen
 * @return pressure_pa pointer is de variabele via waar we de druk gaan teruggeven in pa
 * @note Deze functie komt uit de DATASHEET p.55 Hoofdstuk 9.2
 */
float BMP384_ReadData(BMP384_Calib *calibData) {
    uint8_t raw[6]; // 3 voor druk, 3 voor temp
    BMP384_ReadReg(BMP384_DATA0, raw, 6); // Data begint bij 0x04

    // 24-bit conversie
    uint32_t uncomp_p = (uint32_t)((raw[2] << 16) | (raw[1] << 8) | raw[0]);	// maak van 3 8 bits 1 24 bit getal
    uint32_t uncomp_t = (uint32_t)((raw[5] << 16) | (raw[4] << 8) | raw[3]);

    float partial_t1 = (float)uncomp_t - calibData->t1;
    float partial_t2 = (float)partial_t1 * calibData->t2;
    calibData->t_lin = partial_t2 + (partial_t1 * partial_t1) * calibData->t3;

    // Gebruik t_lin om de druk te compenseren
    return(BMP384_Compensate_Pressure(uncomp_p, calibData));

}

/*
 * @brief Geeft een 1 terug als sensor data en temp data klaar is, anders 0
 * @note dit is een alternatief voor als we niet via interupt basis willen werken
 */
uint8_t BMP384_IsDataReady(void){
	uint8_t status;
	BMP384_ReadReg(BMP384_STATUS, &status, 1);	// We gaan lezen of de data ready bit van de sensor en de temp hoog staan = data klaar
	status = status & 0x60;						// we zijn enkel geintereseerd in bit 5 en 6
	if(status == 0x60){
		return 1;
	}else{
		return 0;
	}
}

/*
 * @brief Zet de eerder berekende pascalwaarde(van BMP384_ReadData) om in een hoogte in meters
 * @formule h = 44330 * [1 - (p/p0)^1/5.255]
 * @waarde P0 = 101325 pascal
 * @param De eerder berekende actuele druk
 */
float BMP384_CalculateAltitude(float pressure_pa){
	float altitude = 44330.0 * (1.0 - powf(pressure_pa / g_ground_pa, 1.0 / 5.255));
	return altitude;
}

/*
 * @brief neem de druk op het punt war de drone momenteel staat(nrml de grond)
 */
void BMP384_ground(BMP384_Calib *calibData) {
	g_ground_pa = 0;
    for (int i = 0; i < 50; i++) {
        while(BMP384_IsDataReady() == 0);
        g_ground_pa += BMP384_ReadData(calibData);
    }
    g_ground_pa /= 50.0f;
}

/*
 * @brief deze functie geeft de temperatuur terug
 * @note deze functie vereist dat BMP384_ReadData() al minstens 1x is opgeroepen
 */
float BMP384_GetTemp(BMP384_Calib *calibData) {
    return calibData->t_lin;
}
