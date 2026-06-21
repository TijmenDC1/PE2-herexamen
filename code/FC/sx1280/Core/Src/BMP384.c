// hier ben ik
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

BMP384_Calib calibData;
float g_ground_pa = 0;

/*
 * @brief in bijna elk register een bepaalde waarde steken
 * @param reg addres van het reg waar je data naar wilt sturen
 * @param val is de value die je in het register wilt steken
 */
static void BMP384_WriteReg(uint8_t reg, uint8_t val){
	// write commando: R/W bit (0=write) + 7-bit adres + 8-bit data
	uint8_t data[2] = {reg & 0x7F, val};
	HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4, data, 2, 1000);
	HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);
}

/*
 * @brief elk register uitlezen uit de BMP384
 * @param reg addres van het reg dat je wilt uitlezen
 * @param pData pointer naar het adres waar je de data gaat opslaan
 * @param len Het aantal bytes van data dat je wilt inlezen
 */
static void BMP384_ReadReg(uint8_t reg, uint8_t *pData, uint16_t len){
	// read commando: R/W bit (1=read) + 7-bit adres, daarna 1 dummy byte, dan data
	uint8_t data = reg | 0x80;
	uint8_t dummy;
	HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4, &data, 1, 100);
	HAL_SPI_Receive(&hspi4, &dummy, 1, 100);
	HAL_SPI_Receive(&hspi4, pData, len, 100);
	HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);
}

/*
 * @brief deze functie geeft de temperatuur terug
 * @note deze functie vereist dat BMP384_ReadData() al minstens 1x is opgeroepen
 */
float BMP384_GetTemp(void) {
    return calibData.t_lin;
}

/*
 * @brief Hier gaan we de gelezen data compenseren met de calibratiedata
 * @param uncomp_p dit is de ongecompenseerde druk data
 * @note Deze functie komt uit de DATASHEET p.56 Hoofdstuk 9.3
 */
static float BMP384_Compensate_Pressure(uint32_t uncomp_p) {
	float t_lin = calibData.t_lin;
    float partial_data1 = calibData.p6 * t_lin;
    float partial_data2 = calibData.p7 * (t_lin * t_lin);
    float partial_data3 = calibData.p8 * (t_lin * t_lin * t_lin);
    float partial_out1 = calibData.p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = calibData.p2 * t_lin;
    partial_data2 = calibData.p3 * (t_lin * t_lin);
    partial_data3 = calibData.p4 * (t_lin * t_lin * t_lin);
    float partial_out2 = (float)uncomp_p * (calibData.p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float)uncomp_p * (float)uncomp_p;
    partial_data2 = calibData.p9 + calibData.p10 * t_lin;
    partial_data3 = partial_data1 * partial_data2;
    float partial_data4 = partial_data3 + ((float)uncomp_p * (float)uncomp_p * (float)uncomp_p) * calibData.p11;

    return partial_out1 + partial_out2 + partial_data4;
}

/*
 * @brief We gaan de calibratiedata in de struct steken
 * @note Deze data komt uit de DATASHEET p.55 Hoofdstuk 9.1
 */
void BMP384_ReadCalibration(void) {
    uint8_t cal[21];
    BMP384_ReadReg(BMP384_NVM_PAR_T1_LSB, cal, 21);

    // Temperatuur
    calibData.t1 =  (float)((uint16_t)((cal[1] << 8) | cal[0])) / powf(2, -8);
    calibData.t2 =  (float)((uint16_t)((cal[3] << 8) | cal[2])) / powf(2, 30);
    calibData.t3 =  (float)((int8_t)cal[4]) / powf(2, 48);

    // Druk
    calibData.p1 =  ((float)((int16_t)((cal[6] << 8) | cal[5])) - powf(2, 14)) / powf(2, 20);
    calibData.p2 =  ((float)((int16_t)((cal[8] << 8) | cal[7])) - powf(2, 14)) / powf(2, 29);
    calibData.p3 =  (float)((int8_t)cal[9])   / powf(2, 32);
    calibData.p4 =  (float)((int8_t)cal[10])  / powf(2, 37);
    calibData.p5 =  (float)((uint16_t)((cal[12] << 8) | cal[11])) / powf(2, -3);
    calibData.p6 =  (float)((uint16_t)((cal[14] << 8) | cal[13])) / powf(2, 6);
    calibData.p7 =  (float)((int8_t)cal[15])  / powf(2, 8);
    calibData.p8 =  (float)((int8_t)cal[16])  / powf(2, 15);
    calibData.p9 =  (float)((int16_t)((cal[18] << 8) | cal[17]))  / powf(2, 48);
    calibData.p10 = (float)((int8_t)cal[19])  / powf(2, 48);
    calibData.p11 = (float)((int8_t)cal[20])  / powf(2, 65);
}

/*
 * @brief Geeft een 1 terug als sensor data en temp data klaar is, anders 0
 */
uint8_t BMP384_IsDataReady(void){
	uint8_t status;
	BMP384_ReadReg(BMP384_STATUS, &status, 1);
	status = status & 0x60;
	if(status == 0x60){
		return 1;
	}else{
		return 0;
	}
}

/*
 * @brief Deze functie gaat de druk lezen en teruggeven in Pascal
 * @note Deze functie komt uit de DATASHEET p.55 Hoofdstuk 9.2
 */
float BMP384_ReadData(void) {
    uint8_t raw[6]; // 3 voor druk, 3 voor temp
    BMP384_ReadReg(BMP384_DATA0, raw, 6);

    // 24-bit conversie
    uint32_t uncomp_p = (uint32_t)((raw[2] << 16) | (raw[1] << 8) | raw[0]);
    uint32_t uncomp_t = (uint32_t)((raw[5] << 16) | (raw[4] << 8) | raw[3]);

    float partial_t1 = (float)uncomp_t - calibData.t1;
    float partial_t2 = partial_t1 * calibData.t2;
    calibData.t_lin = partial_t2 + (partial_t1 * partial_t1) * calibData.t3;

    return BMP384_Compensate_Pressure(uncomp_p);
}

/*
 * @brief neem de druk op het punt waar de drone momenteel staat (normaal de grond)
 */
void BMP384_ground(void) {
    g_ground_pa = 0;
    for (int i = 0; i < 50; i++) {
        while(BMP384_IsDataReady() == 0);
        g_ground_pa += BMP384_ReadData();
    }
    g_ground_pa /= 50.0f;
}

/*
 * @brief Zet de pascalwaarde om in een hoogte in meters
 * @formule h = 44330 * [1 - (p/p0)^(1/5.255)]
 */
float BMP384_CalculateAltitude(float pressure_pa){
	float altitude = 44330.0f * (1.0f - powf(pressure_pa / g_ground_pa, 1.0f / 5.255f));
	return altitude;
}

HAL_StatusTypeDef BMP384_Init(void){
	uint8_t id;
	BMP384_ReadReg(BMP384_CHIP_ID, &id, 1);
	if(id != BMP384_ID){
		return HAL_ERROR;
	}
	BMP384_WriteReg(BMP384_CMD, BMP384_CMD_SOFTRESET);
	HAL_Delay(10);
	BMP384_ReadCalibration();
	BMP384_WriteReg(BMP384_PWR_CTRL, BMP384_MODE_NORMAL);	// press aan, temp aan, normal mode
	BMP384_WriteReg(BMP384_OSR, 3);						// oversampling druk x8
	BMP384_WriteReg(BMP384_CONFIG, 4);					// IIR filter coëfficiënt 3
	BMP384_WriteReg(BMP384_ODR, 2);
	BMP384_WriteReg(BMP384_INT_CTRL, 0x46);				// INT pin configuratie + data ready interrupt
	BMP384_ground();									// referentiedruk op de grond meten

	return HAL_OK;
}
