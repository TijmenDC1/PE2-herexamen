/*
 * sx1280include.h
 *
 * SX1280 driver + ELRS 2.4GHz ontvanger.
 *
 *  Created on: 1 apr 2026
 *      Author: tijme
 */
#ifndef INC_SX1280INCLUDE_H_
#define INC_SX1280INCLUDE_H_

#include "stdint.h"
#include "main.h"

/* --- opcodes --- */
#define SX1280_GETSTATUS                        0xC0
#define SX1280_WRITEREGISTER                    0x18
#define SX1280_READREGISTER                     0x19
#define SX1280_WRITEBUFFER                      0x1A
#define SX1280_READBUFFER                       0x1B
#define SX1280_SET_STANDBY                      0x80
#define SX1280_SETRX                            0x82
#define SX1280_SETPACKET_TYPE                   0x8A
#define SX1280_SETRFFREQUENCY                   0x86
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

/* --- modulatie / packet params --- */
#define SX1280_PACKET_TYPE_LORA                 0x01

#define SX1280_LORA_SF5                         0x50
#define SX1280_LORA_SF6                         0x60
#define SX1280_LORA_SF7                         0x70
#define SX1280_LORA_SF8                         0x80

#define SX1280_LORA_BW_0800                     0x18

#define SX1280_LORA_CR_LI_4_6                   0x06
#define SX1280_LORA_CR_LI_4_8                   0x07

#define SX1280_LORA_PACKET_IMPLICIT             0x80
#define SX1280_LORA_CRC_OFF                     0x00

#define SX1280_LORA_IQ_NORMAL                   0x40
#define SX1280_LORA_IQ_INVERTED                 0x00

/* --- registers --- */
#define SX1280_REG_SF_CONFIG                    0x0925
#define SX1280_REG_FREQ_ERR                     0x093C
#define SX1280_REG_SYNCWORD_MSB                 0x0944
#define SX1280_REG_SYNCWORD_LSB                 0x0945
#define SX1280_REG_XTAL_A                       0x0A0E
#define SX1280_REG_XTAL_B                       0x0A0F

/* --- irq bits --- */
#define SX1280_IRQ_RX_DONE                      0x0002
#define SX1280_IRQ_CRC_ERROR                    0x0040
#define SX1280_IRQ_RX_TX_TIMEOUT                0x4000

/* --- ELRS identiteit --- */
/* UID van de binding phrase, zelfde nummer als in de zender */
#define ELRS_OTA_VERSION        4

#define ELRS_UID0               163
#define ELRS_UID1               59
#define ELRS_UID2               219
#define ELRS_UID3               118
#define ELRS_UID4               199
#define ELRS_UID5               162

#define ELRS_CRC_INIT   ((uint16_t)((((uint16_t)ELRS_UID4 << 8) | ELRS_UID5) \
                                    ^ ((uint16_t)ELRS_OTA_VERSION << 8)))

#define ELRS_FHSS_SEED  ((uint32_t)(((uint32_t)ELRS_UID2 << 24) | ((uint32_t)ELRS_UID3 << 16) \
                                   | ((uint32_t)ELRS_UID4 << 8) | (ELRS_UID5 ^ ELRS_OTA_VERSION)))

#define ELRS_IQ_MODE ((ELRS_UID5 & 0x01) ? SX1280_LORA_IQ_INVERTED : SX1280_LORA_IQ_NORMAL)

/* --- packet rate, moet gelijk staan met de zender --- */
#define ELRS_RATE_500HZ         0
#define ELRS_RATE_250HZ         1
#define ELRS_RATE_150HZ         2
#define ELRS_RATE_50HZ          3

#define ELRS_RATE               ELRS_RATE_250HZ

#if   ELRS_RATE == ELRS_RATE_500HZ
  #define ELRS_SF                 SX1280_LORA_SF5
  #define ELRS_CR                 SX1280_LORA_CR_LI_4_6
  #define ELRS_PREAMBLE_LEN       12
  #define ELRS_HOP_INTERVAL       4
  #define ELRS_INTERVAL_US        2000
  #define ELRS_SF_REG             0x1E
  #define ELRS_RATE_NAAM          "500Hz SF5"
#elif ELRS_RATE == ELRS_RATE_250HZ
  #define ELRS_SF                 SX1280_LORA_SF6
  #define ELRS_CR                 SX1280_LORA_CR_LI_4_8
  #define ELRS_PREAMBLE_LEN       14
  #define ELRS_HOP_INTERVAL       4
  #define ELRS_INTERVAL_US        4000
  #define ELRS_SF_REG             0x1E
  #define ELRS_RATE_NAAM          "250Hz SF6"
#elif ELRS_RATE == ELRS_RATE_150HZ
  #define ELRS_SF                 SX1280_LORA_SF7
  #define ELRS_CR                 SX1280_LORA_CR_LI_4_8
  #define ELRS_PREAMBLE_LEN       12
  #define ELRS_HOP_INTERVAL       4
  #define ELRS_INTERVAL_US        6666
  #define ELRS_SF_REG             0x37
  #define ELRS_RATE_NAAM          "150Hz SF7"
#else
  #define ELRS_SF                 SX1280_LORA_SF8
  #define ELRS_CR                 SX1280_LORA_CR_LI_4_8
  #define ELRS_PREAMBLE_LEN       12
  #define ELRS_HOP_INTERVAL       2
  #define ELRS_INTERVAL_US        20000
  #define ELRS_SF_REG             0x37
  #define ELRS_RATE_NAAM          "50Hz SF8"
#endif

#define ELRS_BW                 SX1280_LORA_BW_0800

/* printen kost tijd, dus max 1 regel per seconde */
#define PRINT_ELKE              (1000000 / ELRS_INTERVAL_US)

#define ELRS_FREQ_CORRECTIE_HZ  0
#define ELRS_XTAL_TRIM          0

/* --- failsafe (AETR, kanaal 2 = gas) --- */
#define FS_GAS_KANAAL           2
#define FS_GAS_WAARDE           0
#define FS_MIDDEN               512

/* --- band en FHSS --- */
#define FREQ_START              2400400000UL
#define FREQ_STEP               1000000UL
#define FREQ_COUNT              80
#define SYNC_CHANNEL            (FREQ_COUNT / 2)
#define SYNC_FREQ               (FREQ_START + (uint32_t)SYNC_CHANNEL * FREQ_STEP)

/* de index wrapt op 240, de tabel is ruimer omdat de shuffle tot 319 schrijft */
#define HOP_COUNT               240
#define HOP_TABLE_SIZE          320

/* --- zoeken naar de zender --- */
#define ACQ_TIJD_MS             600
#define ACQ_BEVESTIG            20
#define ACQ_MIN_SNR             5
#define ACQ_POGINGEN            ((HOP_COUNT + FREQ_COUNT - 1) / FREQ_COUNT)
#define LINK_TIMEOUT_MS         1000

/* tellerstand waarop een pakket hoort binnen te komen, hop valt net erna */
#define FASE_OFFSET_US          3700
#define INDEX_OFFSET            0

/* --- pakket --- */
#define PKT_RCDATA              0x00
#define PKT_DATA                0x01
#define PKT_SYNC                0x02

#define PKT_SIZE                8
#define CRC_LEN                 7
#define CRC14_POLY              0x2E57

#define NONCE_UIT_SEED(s)       ((uint8_t)(((s) ^ ELRS_CRC_INIT) & 0xFF))
#define SEED_VOOR(type, nonce)  ((uint16_t)(ELRS_CRC_INIT ^ \
                                (((type) == PKT_SYNC) ? 0u : (uint16_t)(nonce))))

/* --- gedeelde variabelen (staan in sx1280include.c) --- */
#define STATE_ZOEKEN            0
#define STATE_LOCKED            1

extern volatile uint8_t  link_state;
extern volatile uint8_t  nonce;
extern volatile uint8_t  do_hop;
extern volatile uint32_t pkt_count;
extern volatile uint16_t rc_channels[4];
extern volatile uint8_t  failsafe;
extern volatile uint8_t  lq;

extern volatile uint8_t  try_nr;
extern volatile uint8_t  lock_ok;
extern volatile uint8_t  ok_count;
extern volatile uint8_t  spi_busy;
extern volatile uint8_t  anchor_nonce;
extern volatile uint16_t anchor_index;
extern volatile uint32_t ticks;
extern volatile uint32_t anchor_ticks;

/* --- functies --- */
uint8_t ELRS_LinkOK(void);

uint8_t SX1280_BUSY(void);
void    SX1280_Select(void);
void    SX1280_Deselect(void);
void    SX1280_getstatus(void);
uint8_t SX1280_GetStatusByte(void);
void    SX1280_PrintStatus(const char *label);
void    SX1280_Reset(void);
int8_t  SX1280_GetRssiInst(void);
void    SX1280_GetPacketStatus(int8_t *rssi, int8_t *snr);

void    SX1280_WriteRegister(uint16_t address, uint8_t value);
uint8_t SX1280_ReadRegister(uint16_t address);
void    SX1280_ReadBuffer(uint8_t offset, uint8_t *data, uint8_t size);

void    SX1280_SetRfFrequency(uint32_t frequency);
void    SX1280_SetRx(void);
void    SX1280_SetDioIrqParams(void);
void    SX1280_SetXtalTrim(uint8_t trim);

void    SX1280_Setup_ELRS(void);
uint8_t SX1280_TestConnection(void);
uint8_t SX1280_CheckRxDonePolling(void);
void    SX1280_OnPacketReceived(void);

void     ELRS_InitCRC(void);
uint16_t ELRS_CalculateCRC(uint8_t *data, uint8_t len, uint16_t seed);
void     ELRS_UnpackChannels(uint8_t *payload, uint16_t *ch);
uint8_t  ELRS_ValidatePacket(uint8_t *buf, uint16_t crc_in, uint16_t *seed_uit);

void    ELRS_Tick(void);
void    ELRS_HandleHop(void);
void    ELRS_PhaseReset(void);

#endif /* INC_SX1280INCLUDE_H_ */
