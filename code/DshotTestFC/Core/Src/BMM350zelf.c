/*
 * BMM350.c
 *
 *  Created on: 28 feb 2026
 *      Author: simon
 */
/*
 * I2C functies:
 * HAL_I2C_Master_Transmit()
 * HAL_I2C_Master_Receive()
 * HAL_I2C_Slave_Transmit()
 * HAL_I2C_Slave_Receive()
 * HAL_I2C_Mem_Write()
 * HAL_I2C_Mem_Read()
 */


// TODO: bij BMM350_TiltCompensation nog roll en pitch toevoegen, deze komt later van de pid

#include <BMM350.h>
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

/*
void BMM350_Init(void){
	uint8_t id;
	HAL_Delay(3000);
	BMM350_ReadReg(BMM350_CHIP_ID, &id, 1);
	while(0x0E != id && 4 != id){ //0x33
		BMM350_ReadReg(BMM350_CHIP_ID, &id, 1);
		printf("Fout bij initialisatie BMM350, %d\n", id);
		HAL_Delay(3000);
	}
		printf("Succesvol geconnecteerd\n");
		printf("geconnecteerd met BMM350, ID: %d\n", id);
		HAL_Delay(3000);

	BMM350_WriteReg(BMM350_OTP_CMD_REG, 0x80);
	HAL_Delay(50);
	BMM350_WriteReg(BMM350_PMU_CMD_AGGR_SET, 0x25);		// We zetten odr naar 50hz en we nemen het gemiddelde van 4 metingen, we krijgen dus een meting elke 20ms
	BMM350_WriteReg(BMM350_PMU_CMD, 0x02);
	HAL_Delay(50);
	BMM350_WriteReg(BMM350_I2C_WDT_SET, 0x03);			// We zetten de watchdog timer aan op lang(40.96ms)
	HAL_Delay(50);
	BMM350_WriteReg(BMM350_INT_CTRL, 0x8E);				// We zetten de interupts aan voor als er data is en zetten het op de pin naar buiten
	HAL_Delay(50);
	BMM350_WriteReg(BMM350_PMU_CMD, 0x01);
	HAL_Delay(50);
	BMM350_WriteReg(BMM350_PMU_CMD_AXIS_EN, 0x07);
	HAL_Delay(50);

	return;
}*/
void BMM350_Init(void) {
    uint8_t chip_id = 0;

    // 1. Controleer of de sensor reageert en de juiste ID heeft
    BMM350_ReadReg(BMM350_CHIP_ID, &chip_id, 1);
    if (chip_id != BMM350_EXPRECTED_ID) {
        return; // Verkeerde CHIP ID ontdekt
    }

    // 2. Beëindig de boot-fase (verplicht)
    BMM350_WriteReg(BMM350_OTP_CMD_REG, 0x80);
    HAL_Delay(2); // Geef de sensor even de tijd na OTP commando

    // 3. Configureer Output Data Rate (ODR) en Averaging (Ruis/Stroomverbruik)
    // 0x14 = 100Hz ODR met Regular Power instelling (gemiddelde van 2 samples)
    BMM350_WriteReg(BMM350_PMU_CMD_AGGR_SET, 0x14);

    // 4. Update ODR/AVG in de Power Management Unit (UPD_OAE commando: 0x02)
    BMM350_WriteReg(BMM350_PMU_CMD, 0x02);
    HAL_Delay(2);

    // 5. Activeer alle assen (X, Y, Z)
    // 0x07 = b00000111 (en_z, en_y, en_x)
    BMM350_WriteReg(BMM350_PMU_CMD_AXIS_EN, 0x07);

    // 6. Zet de sensor in Normal Mode om continu te meten (NM commando: 0x01)
    BMM350_WriteReg(BMM350_PMU_CMD, 0x01);
    HAL_Delay(5);

    return; // Initialisatie succesvol
}
/*
/*
 * @brief in bijna elk register een bepaalde waarde steken
 * @param reg addres van het reg waar je data naar wilt sturen
 * @param val is de value die je in het register wilt steken
 */
/*
void BMM350_WriteReg(uint8_t reg, uint8_t val){
	//HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
	uint8_t data[2] = {reg, val};	// we zetten beide 8 bits in 1 16bit var zodat we het in 1x kunnen versturen
	HAL_I2C_Master_Transmit(&hi2c1, BMM350_I2C_ADDRESS, data, 2, 100);
}
*/

/*
 * @brief elk register uitlezen uit de BMM350
 * @param reg addres van het reg dat je wilt uitlezen
 * @param pData pionter naar het addres waar je de data gaat lopslagen
 * @param len Het aantal bytes van data dat je wilt inlezen
 */

//void BMM350_ReadReg(uint8_t reg, uint8_t *pData, uint16_t len){
	/*uint8_t data[len+2];
	HAL_I2C_Master_Transmit(&hi2c1, BMM350_I2C_ADDRESS, &reg, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, BMM350_I2C_ADDRESS, data, len + 2, 100);	// we lezen de 2 dummy bytes in + de goeie byte

	for(int i = 0; i < len; i++) {
	    pData[i] = data[i + 2];	// we zetten hier de goeie byte in een nieuwe var
	}
	uint8_t buffer[len + 2];

	    // HAL_I2C_Mem_Read haalt alles op. De eerste 2 bytes in 'buffer' zullen de dummy bytes zijn
	    HAL_I2C_Mem_Read(&hi2c1, BMM350_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buffer, len + 2, 100);

	    // We kopiëren nu alleen de ECHTE data naar jouw pData, we slaan buffer[0] en buffer[1] over
	    for(int i = 0; i < len; i++) {
	        pData[i] = buffer[i + 2];
	    }
}
*/
void BMM350_WriteReg(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(&hi2c1, BMM350_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

/**
 * @brief Lees data uit een BMM350 register (inclusief correctie voor dummy bytes)
 */
void BMM350_ReadReg(uint8_t reg, uint8_t *data, uint16_t len) {
    /* De BMM350 stuurt altijd 2 dummy bytes bij elke I2C lees-actie, deze moeten we negeren */
    uint8_t buffer[len + 2];

    HAL_I2C_Mem_Read(&hi2c1, BMM350_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buffer, len + 2, 100);

        for (uint16_t i = 0; i < len; i++) {
            data[i] = buffer[i + 2]; // Verwijder de eerste 2 bytes
        }
    return;
}

/*
 * @brief lees de data in(x, y en z) en zet om naar signed 32 int getal en slaag het op in een struct
 * @param De struct waarin we de x, y en z waardes gaan opslagen
 */
void BMM350_GetData(BMM350_MagData *BMM350_Data, float roll, float pitch){
	uint8_t data[9]; // 3*3 want 3 regs per richting(x,y,z)
	float x_offset = 0;
	float y_offset = 0;
	float z_offset = 0;
	//BMM350_Data->x_offset = 4510779;
	//BMM350_Data->y_offset = 262180.5;
	//BMM350_Data->z_offset = 1312;
	BMM350_ReadReg(BMM350_MAG_X_XLSB, data, 9);
	printf("Raw bytes: ");
	    for(int i = 0; i < 9; i++) {
	        printf("%02X ", data[i]);
	    }
	    printf("\n");

	// X-as (0x31-0x33)
	int32_t val_x = (int32_t)((uint32_t)data[2] << 16 | (uint32_t)data[1] << 8 | data[0]);
	if (val_x & 0x00100000) val_x |= 0xFFF00000; // we kijken of bit 20 hoog 1, zo ja dan zetten we bit 21-31 hoog zodat het getal negatief blijft.
	BMM350_Data->x = val_x - x_offset;

	// Y-as (0x34-0x36)
	int32_t val_y = (int32_t)((uint32_t)data[5] << 16 | (uint32_t)data[4] << 8 | data[3]);
	if (val_y & 0x00100000) val_y |= 0xFFF00000;
	BMM350_Data->y = val_y - y_offset;

	// Z-as (0x37-0x39)
	int32_t val_z = (int32_t)((uint32_t)data[8] << 16 | (uint32_t)data[7] << 8 | data[6]);
	if (val_z & 0x00100000) val_z |= 0xFFF00000;
	BMM350_Data->z = val_z - z_offset;

	//BMM350_Data->richting = BMM350_TiltCompensation(roll, pitch, BMM350_Data);
	// calibrate is iets wat we eenmalig moeten doen, hieruit halen we de vaste ofset waardes die we daarna kunnen gebruiken
	BMM350_Calibrate(val_x, val_y, val_z, &BMM350_Data->x_offset, &BMM350_Data->y_offset, &BMM350_Data->z_offset);
}

/*
 * @brief Deze functie gaat de hoek berekenen tussen de neus van de drone en het noorden
 * @note We gebruiken de Z as voor de correctie te doen als de drone niet perfect vlak hangt
 */
float BMM350_Angle(float x, float y){
	//formule richting = bgtan2f(y,x) * (180 / PI)
	float richting = atan2f(y, x);
	richting = richting * (180.0 / PI);
	if (richting < 0) {
	    richting += 360.0;
	}
	return richting + 3.0;	// +3 graden is om het geografische noorden te verkrijgen, dit veranderd van plaats tot plaats maar in belgie is dit 3
}

/*
 * @brief er gaat altijd een soort afwijking zitten op de standaardmeting daarom eerst kalibreren
 * @param raw_x de ongecompenseerde x waarde
 * @param raw_y de ongecompenseerde y waarde
 * @param raw_z de ongecompenseerde z waarde
 * @param offset_x de berekende offset die op raw_x moet worden toegepast om de juiste x waarde te verkrijgen
 * @param offset_y de berekende offset die op raw_y moet worden toegepast om de juiste y waarde te verkrijgen
 * @param offset_z de berekende offset die op raw_z moet worden toegepast om de juiste z waarde te verkrijgen
 * @note meet 1 ass en draai de sensor 360 graden en kijk de min en max waarde
 * @note dit is een temp functie, eenmaal de offset geweten is hebben we deze functie niet meer nodig en kunnen we de offset hardcoden
 */
void BMM350_Calibrate(float raw_x, float raw_y, float raw_z, float *offset_x, float *offset_y, float *offset_z) {
    static float min_x = 10000, max_x = -10000, min_y = 10000, max_y = -10000, min_z = -10000, max_z = -10000;

    if(raw_x < min_x) min_x = raw_x;
    if(raw_x > max_x) max_x = raw_x;
    if(raw_y < min_y) min_y = raw_y;
    if(raw_y > max_y) max_y = raw_y;
    if(raw_z < min_z) min_z = raw_z;
    if(raw_z > max_z) max_z = raw_z;

    *offset_x = (max_x + min_x) / 2.0;
    *offset_y = (max_y + min_y) / 2.0;
    *offset_z = (max_z + min_z) / 2.0;
}

/*
 * @brief we compenseren voor de roll en de pitch van de drone zodat we altijd de richting juist hebben
 * @param x_offset dit is de offset die normaal komt van de BMM350_Calibrate functie.
 * @param y_offset zelfde als x maar voor y
 * @param z_offset zelfde als x maar voor z
 * @param roll de momentele roll dat de drone heeft
 * @param pitch de momentele pitch die de drone heeft
 */
float BMM350_TiltCompensation(float roll, float pitch, BMM350_MagData *BMM350_Data){
	// Euler-hoeken
	float x = BMM350_Data->x;
	float y = BMM350_Data->y;
	float z = BMM350_Data->z;
	float roll_rad = roll * (PI / 180);	//cosf en sinf verwachten radialen
	float pitch_rad = pitch * (PI / 180);

	float x_comp = x * cosf(pitch_rad) + y * sinf(roll_rad) * sinf(pitch_rad) + z * cosf(roll_rad) * sinf(pitch_rad);
	float y_comp = y * cosf(roll_rad) - z * sinf(roll_rad);
	float richting = BMM350_Angle(x_comp, y_comp);

	return richting;
}
