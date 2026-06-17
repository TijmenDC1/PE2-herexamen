/*
 * sx1280include.h
 *
 *  Created on: 1 apr 2026
 *      Author: tijme
 */

#ifndef INC_SX1280INCLUDE_H_
#define INC_SX1280INCLUDE_H_


#define SX1280_CS_GPIO_Port    					GPIOB
#define SX1280_CS_Pin          					GPIO_PIN_12
#define SX1280_RESET_GPIO_Port 					GPIOD
#define SX1280_RESET_Pin       					GPIO_PIN_8
#define SX1280_BUSY_GPIO_Port  					GPIOD
#define SX1280_BUSY_Pin                         GPIO_PIN_9


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



#endif /* INC_SX1280INCLUDE_H_ */
