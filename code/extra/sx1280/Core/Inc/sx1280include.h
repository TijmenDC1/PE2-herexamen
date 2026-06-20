/*
 * sx1280include.h
 *
 *  Created on: 1 apr 2026
 *      Author: tijme
 */
#include "stdint.h"
#include "main.h"

#ifndef INC_SX1280INCLUDE_H_
#define INC_SX1280INCLUDE_H_

/*
#define SX1280_CS_GPIO_Port    					GPIOB
#define SX1280_CS_Pin          					GPIO_PIN_12
#define SX1280_RESET_GPIO_Port 					GPIOD
#define SX1280_RESET_Pin       					GPIO_PIN_8
#define SX1280_BUSY_GPIO_Port  					GPIOD
#define SX1280_BUSY_Pin                         GPIO_PIN_9
*/

#define SX1280_GETSTATUS                        0xC0
#define SX1280_WRITEREGISTER                    0x18
#define SX1280_READREGISTER                     0x19
#define SX1280_WRITEBUFFER                      0x1A
#define SX1280_READBUFFER                       0x1B
#define SX1280_SETSLEEP                         0x84
#define SX1280_SET_STANDBY                      0x80
#define SX1280_SETFS                            0xC1
#define SX1280_SETTX                            0x83
#define SX1280_SETRX                            0x82
#define SX1280_SETRXDUTYCYCLE                   0x94
#define SX1280_SETCAD                           0xC5
#define SX1280_SETTXCONTINUOUSWAVE              0xD1
#define SX1280_SETTXCONTINUOUSPREAMBLE          0xD2
#define SX1280_SETPACKET_TYPE                   0x8A
#define SX1280_GETPACKET_TYPE                   0x03
#define SX1280_SETRFFREQUENCY                   0x86
#define SX1280_SETTXPARAMS                      0x8E
#define SX1280_SETCADPARAMS                     0x88
#define SX1280_SETBUFFER_BASEADDRESS            0x8F
#define SX1280_SETMODULATIONPARAMS              0x8B
#define SX1280_SET_PACKET_PARAMS                0x8C
#define SX1280_GETRXBUFFERSTATUS                0x17
#define SX1280_GETPACKETSTATUS                  0x1D
#define SX1280_GETRSSIINST                      0x1F
#define SX1280_GETIRQSTATUS                     0x15
#define SX1280_CLRIRQSTATUS                     0x97
#define SX1280_SETREGULATORMODE                 0x96
#define SX1280_SET_DIO_IRQ_PARAMS               0x8D
#define SX1280_SET_SAVE_CONTEXT                 0xD5
#define SX1280_SET_AUTO_FS                      0x9E
#define SX1280_SET_AUTO_TX                      0x98
#define SX1280_SET_LONG_PREAMBLE                0x9B
#define SX1280_SET_UART_SPEED                   0x9D
#define SX1280_SET_RANGING_ROLE                 0xA3
#define SX1280_SET_ADVANCED_RANGING             0x9A

// --- Functie Prototypes ---
uint8_t SX1280_BUSY(void);
void SX1280_Select(void);
void SX1280_Deselect(void);
void SX1280_getstatus(void);
void SX1280_Reset(void);
int8_t SX1280_GetRssiInst(void);

// Register & Buffer communicatie
void SX1280_WriteRegister(uint16_t address, uint8_t value);
uint8_t SX1280_ReadRegister(uint16_t address);
void SX1280_WriteBufferByte(uint8_t offset, uint8_t data);
uint8_t SX1280_ReadBufferByte(uint8_t offset);
void SX1280_ReadBuffer(uint8_t offset, uint8_t *data, uint8_t size);

// Radio instellingen
void SX1280_SetRfFrequency(uint32_t frequency);
void SX1280_SetRx(void);
void SX1280_SetDioIrqParams(void);

// Hoofdfuncties voor je main.c
void SX1280_Setup_Sniper(void);
void SX1280_OnPacketReceived(void);
uint8_t SX1280_TestConnection(void);
void SX1280_PrintStatus(const char* label);

// Hulpfuncties
void zeroingAnArray(uint8_t arrayToZero[], uint16_t arrayLength);
void SX1280_WriteBufferArray(uint8_t offset, uint8_t *data, uint8_t size);
void SX1280_SetTxParams(int8_t power, uint8_t rampTime);
void SX1280_SetTxContinuousWave(void);
uint8_t SX1280_CheckRxDonePolling(void);

#endif /* INC_SX1280INCLUDE_H_ */
