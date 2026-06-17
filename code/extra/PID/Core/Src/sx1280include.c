#include "sx1280include.h"
#include "main.h"



extern SPI_HandleTypeDef hspi1;

void SX1280_BUSY(void)
{
    while(HAL_GPIO_ReadPin(SX1280_BUSY_GPIO_Port, SX1280_BUSY_Pin) == 1);
}



void SX1280_Select(void)
{
    HAL_GPIO_WritePin(SX1280_CS_GPIO_Port, SX1280_CS_Pin, 0);
}

void SX1280_Deselect(void)
{
    HAL_GPIO_WritePin(SX1280_CS_GPIO_Port, SX1280_CS_Pin, GPIO_PIN_SET);
}

void SX1280_Reset(void)
{
    HAL_GPIO_WritePin(SX1280_RESET_GPIO_Port, SX1280_RESET_Pin, 0);
    HAL_Delay(20);
    HAL_GPIO_WritePin(SX1280_RESET_GPIO_Port, SX1280_RESET_Pin, 1);
    SX1280_BUSY();
}


void SX1280_WriteRegister(uint16_t address, uint8_t value)
{
    uint8_t buf[4];
    buf[0] = SX1280_WRITEREGISTER;             	 //header file
    buf[1] = (uint8_t)(address >> 8);  		// MSb
    buf[2] = (uint8_t)(address & 0xFF);		// LSB
    buf[3] = value;                    			//data

    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_Transmit(&hspi1, buf, 4, HAL_MAX_DELAY);  ///versture buffer
    SX1280_Deselect();
}


uint8_t SX1280_ReadRegister(uint16_t address)  //instellen sx1280
{
    uint8_t status;
    uint8_t addr_buf[3];
    uint8_t NOP = 0x00;              // NOP byte
    uint8_t result = 0;

    addr_buf[0] = SX1280_READREGISTER;
    addr_buf[1] = (uint8_t)(address >> 8);
    addr_buf[2] = (uint8_t)(address & 0xFF);

    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_Transmit(&hspi1, addr_buf, 3, HAL_MAX_DELAY);

    //nop zegt datasheet om te doen
    HAL_SPI_Transmit(&hspi1, &NOP, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &result, 1, HAL_MAX_DELAY);  //geeft register
    SX1280_Deselect();
    return result;
}

void SX1280_ReadBuffer(uint8_t offset, uint8_t *data, uint8_t size)
{
	uint8_t NOP = 0x00;
	uint8_t READBUFFER = SX1280_READBUFFER;
	SX1280_BUSY();
    SX1280_Select();

    HAL_SPI_Transmit(&hspi1, &READBUFFER, 1, HAL_MAX_DELAY);//1byte sturen
    HAL_SPI_Transmit(&hspi1, &offset, 1, HAL_MAX_DELAY);//zelf offset kiezen tussen 0 en 256
    HAL_SPI_Transmit(&hspi1, &NOP, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, data, size, HAL_MAX_DELAY);
    SX1280_Deselect();
}


//frequentie Hz 2400000000 2.4GHz

void SX1280_SetRfFrequency(uint32_t frequency)
{
    // We gebruiken 52000000HZ kristal: (freq << 18) / 52000000
    uint32_t rfwaarde = (uint32_t)(((uint64_t)frequency << 18) / 52000000);

    uint8_t buf[4];
    buf[0] = SX1280_SETRFFREQUENCY;
    buf[1] = (uint8_t)( rfwaarde >> 16); // MSB
    buf[2] = (uint8_t)( rfwaarde >> 8);  // Middelste byte
    buf[3] = (uint8_t)( rfwaarde & 0xFF); // LSB

    SX1280_BUSY();
    SX1280_Select();

    HAL_SPI_Transmit(&hspi1, buf, 4, HAL_MAX_DELAY);

    SX1280_Deselect();
}



void SX1280_SetRx(void)
{
    uint8_t buf[4];
    buf[0] = SX1280_SETRX;
    buf[1] = 0x01; // 1ms
    buf[2] = 0xFF;
    buf[3] = 0xFF; // continu blijven sampelen

    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_Transmit(&hspi1, buf, 4, HAL_MAX_DELAY);
    SX1280_Deselect();
}
