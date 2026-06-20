#include "sx1280include.h"
#include "main.h"
#include "stdio.h"


extern SPI_HandleTypeDef hspi2;

uint8_t SX1280_BUSY(void)
{
    uint32_t start_time = HAL_GetTick(); // Gebruik uint32_t voor HAL_GetTick!

    if(HAL_GPIO_ReadPin(SX1280_Bussy_GPIO_Port, SX1280_Bussy_Pin) == GPIO_PIN_SET){
    	printf("Busy after reset: %lu\n", start_time);
    }

    while(HAL_GPIO_ReadPin(SX1280_Bussy_GPIO_Port, SX1280_Bussy_Pin) == GPIO_PIN_SET){
        // Als we langer dan 10ms wachten, is er iets mis
        if((HAL_GetTick() - start_time) > 10000){
        	printf("TIMEOUT!! %lu\n", (HAL_GetTick() - start_time));
            return 1; // 1 betekent: Timeout / Fout!
        }
    }
    //printf("Busy for: %lu\n", (HAL_GetTick() - start_time));
    return 0; // 0 betekent: Alles is oké, chip is klaar (BUSY is laag).
}



void SX1280_Select(void)
{
	//3x NOP gevonden online: https://github.com/cschorn01/2.4GHz_Lora_for_Arduino/blob/main/src/sx1280OverSpi.cpp
	__asm__ volatile ("NOP \n NOP \n NOP");
    HAL_GPIO_WritePin(SX1280_CS_GPIO_Port, SX1280_CS_Pin, 0);
    __asm__ volatile ("NOP \n NOP \n NOP");
}

void SX1280_Deselect(void)
{
	//3x NOP gevonden online: https://github.com/cschorn01/2.4GHz_Lora_for_Arduino/blob/main/src/sx1280OverSpi.cpp
	__asm__ volatile ("NOP \n NOP \n NOP");
    HAL_GPIO_WritePin(SX1280_CS_GPIO_Port, SX1280_CS_Pin, GPIO_PIN_SET);
    __asm__ volatile ("NOP \n NOP \n NOP");
}

int8_t SX1280_GetRssiInst(void)
{
    uint8_t tx_buf[3] = {SX1280_GETRSSIINST, 0x00, 0x00};
    uint8_t rx_buf[3] = {0};

    SX1280_BUSY();

    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, 3, HAL_MAX_DELAY);
    SX1280_Deselect();

    printf("Status Byte: 0x%02X | Raw Data: 0x%02X\n", rx_buf[0], rx_buf[2]);

    int8_t rssi_dbm = -(rx_buf[2] / 2);

    return rssi_dbm;
}

void SX1280_getstatus(void)
{


    // Zet chip in STDBY_RC
    uint8_t cmd[2] = {0xC0,01}; //00= intern RC 01 = xosc
    uint8_t receive[2];
    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, cmd, receive, 2, HAL_MAX_DELAY);//(&hspi2, cmd, 2, HAL_MAX_DELAY);
    printf("Status = %02x\n", receive[0]);
    SX1280_Deselect();
}


void SX1280_Reset(void)
{
    HAL_GPIO_WritePin(SX1280_nRST_GPIO_Port, SX1280_nRST_Pin, 0);
    HAL_Delay(20);
    __asm__ volatile ("NOP \n NOP \n NOP");
    HAL_GPIO_WritePin(SX1280_nRST_GPIO_Port, SX1280_nRST_Pin, 1);
    HAL_Delay(20);          // iets langer wachten
    SX1280_BUSY();

    SX1280_getstatus();
    SX1280_getstatus();

    // Zet chip in STDBY_RC
    /*
    uint8_t cmd[2] = {0x80, 0x00}; //00= intern RC 01 = xosc
    uint8_t receive[2];
    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, cmd, receive, 2, HAL_MAX_DELAY);//(&hspi2, cmd, 2, HAL_MAX_DELAY);
    printf("%02x, %02x\n", receive[0], receive[1]);
    SX1280_Deselect();
    HAL_Delay(5);
    SX1280_BUSY();
    */
}

void SX1280_WriteRegister(uint16_t address, uint8_t value)
{
    uint8_t tx_buf[4] = {SX1280_WRITEREGISTER, (uint8_t)(address >> 8), (uint8_t)(address & 0xFF), value};
    uint8_t rx_buf[4] = {0};

    if(SX1280_BUSY() == 1){
        printf("Fout: SX1280 blijft BUSY voor Write!\n");
        return;
    }

    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, 4, HAL_MAX_DELAY);
    SX1280_Deselect();
}
/*
void SX1280_WriteRegister(uint16_t address, uint8_t value)
{
    uint8_t buf[4];
    buf[0] = SX1280_WRITEREGISTER;             	 //header file
    buf[1] = (uint8_t)(address >> 8);  		// MSb
    buf[2] = (uint8_t)(address & 0xFF);		// LSB
    buf[3] = value;                    			//data
    uint8_t is_busy = SX1280_BUSY();
	if(is_busy == 1){
		printf("Fout: SX1280 blijft BUSY!\n");
		return;
	}
    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, buf, 4, HAL_MAX_DELAY);  ///versture buffer
    SX1280_Deselect();
}
*/
/*
uint8_t SX1280_ReadRegister(uint16_t address)
{
    // MOSI: Opcode -> Addr MSB -> Addr LSB -> NOP -> NOP
    uint8_t tx_buf[5] = {SX1280_READREGISTER, (uint8_t)(address >> 8), (uint8_t)(address & 0xFF), 0x00, 0x00};
    uint8_t rx_buf[5] = {0}; // Hier komt het resultaat in

    SX1280_BUSY();
    SX1280_Select();

    // Stuur 5 bytes en ontvang tegelijk 5 bytes
    HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, 5, HAL_MAX_DELAY);

    SX1280_Deselect();

    // De data komt pas binnen op de allerlaatste (5e) byte
    return rx_buf[4];
}
*/
uint8_t SX1280_ReadRegister(uint16_t address)
{
    uint8_t tx_buf[5] = {SX1280_READREGISTER, (uint8_t)(address >> 8), (uint8_t)(address & 0xFF), 0x00, 0x00};
    uint8_t rx_buf[5] = {0};

    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, 5, HAL_MAX_DELAY);
    SX1280_Deselect();

    printf("RX Data: %02X %02X %02X %02X %02X\n", rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]);

    return rx_buf[4];
}
void SX1280_ReadBuffer(uint8_t offset, uint8_t *data, uint8_t size)
{
	uint8_t NOP = 0x00;
	uint8_t READBUFFER = SX1280_READBUFFER;
	SX1280_BUSY();
    SX1280_Select();

    HAL_SPI_Transmit(&hspi2, &READBUFFER, 1, HAL_MAX_DELAY);//1byte sturen
    HAL_SPI_Transmit(&hspi2, &offset, 1, HAL_MAX_DELAY);//zelf offset kiezen tussen 0 en 256
    HAL_SPI_Transmit(&hspi2, &NOP, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, data, size, HAL_MAX_DELAY);
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

    HAL_SPI_Transmit(&hspi2, buf, 4, HAL_MAX_DELAY);

    SX1280_Deselect();
}



void SX1280_SetRx(void)
{
    uint8_t buf[4];
    buf[0] = SX1280_SETRX;
    buf[1] = 0x02; // TickBase 1ms
    buf[2] = 0xFF; // Continu
    buf[3] = 0xFF; // Continu

    SX1280_BUSY(); // Wachten voor commando
    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, buf, 4, HAL_MAX_DELAY);
    SX1280_Deselect();

    SX1280_BUSY(); // CRUCIAAL: Wachten tot de chip in RX mode is beland!
}

void SX1280_WriteBufferByte(uint8_t offset, uint8_t data)
{
    uint8_t tx_buf[3] = {SX1280_WRITEBUFFER, offset, data};
    uint8_t rx_buf[3] = {0};

    if(SX1280_BUSY() == 1) return;

    SX1280_Deselect();      // zeker zijn dat CS hoog is
    HAL_Delay(1);
    SX1280_Select();

    HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, 3, HAL_MAX_DELAY);

    SX1280_Deselect();
    printf("Write RX: %02X %02X %02X\n", rx_buf[0], rx_buf[1], rx_buf[2]);
}

uint8_t SX1280_ReadBufferByte(uint8_t offset)
{
    uint8_t tx_buf[4] = {SX1280_READBUFFER, offset, 0x00, 0x00};
    uint8_t rx_buf[4] = {0};

    if(SX1280_BUSY() == 1) return 0;

    HAL_Delay(1);
    SX1280_Select();
    HAL_Delay(1);

    HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, 4, HAL_MAX_DELAY);

    HAL_Delay(1);
    SX1280_Deselect();
    HAL_Delay(1);

    printf("Read RX: %02X %02X %02X %02X\n", rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

    return rx_buf[3];
}

void SX1280_SetDioIrqParams()
{
    // RxDone interrupt is bit 1 (0x0002) in het register
    uint16_t irqMask = 0x0002;  // Welke interrupts we in de chip willen activeren
    uint16_t dio1Mask = 0x0000; // Welke interrupts DIO1 hoog mogen maken
    uint16_t dio2Mask = 0x0002; // Gebruiken we niet
    uint16_t dio3Mask = 0x0000; // Gebruiken we niet

    uint8_t buf[9];
    buf[0] = SX1280_SET_DIO_IRQ_PARAMS;

    buf[1] = (uint8_t)(irqMask >> 8);
    buf[2] = (uint8_t)(irqMask & 0xFF);

    buf[3] = (uint8_t)(dio1Mask >> 8);
    buf[4] = (uint8_t)(dio1Mask & 0xFF);

    buf[5] = (uint8_t)(dio2Mask >> 8);
    buf[6] = (uint8_t)(dio2Mask & 0xFF);

    buf[7] = (uint8_t)(dio3Mask >> 8);
    buf[8] = (uint8_t)(dio3Mask & 0xFF);

    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, buf, 9, HAL_MAX_DELAY);
    SX1280_Deselect();

    printf("DIO2 IRQ ingesteld op RxDone.\n");
}
void SX1280_Setup_Sniper(void)
{
    printf("\n--- Start ELRS 50Hz Sync-Sniper Setup ---\n");

    uint8_t buf[10];

    SX1280_Reset();
    HAL_Delay(10);

    // zet op standby
    buf[0] = SX1280_SET_STANDBY;
    buf[1] = 0x00; // STDBY_RC
    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, buf, 2, HAL_MAX_DELAY);
    SX1280_Deselect();
    SX1280_BUSY();

    // zet regulator op LDO
    buf[0] = SX1280_SETREGULATORMODE;
    buf[1] = 0x00;
    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, buf, 2, HAL_MAX_DELAY);
    SX1280_Deselect();
    SX1280_BUSY();

    // zet packet type op LORA
    buf[0] = SX1280_SETPACKET_TYPE;
    buf[1] = 0x01;
    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, buf, 2, HAL_MAX_DELAY);
    SX1280_Deselect();
    SX1280_BUSY();

    // *** DIT IS DE BELANGRIJKSTE AANPASSING ***
    // Frequentie: 2.4404 GHz (Kanaal 40 in de ELRS ISM2400 band)
    SX1280_SetRfFrequency(2440400000);

    // standaard startadres 0x00 maar voor zekerheid nog eens zetten
    buf[0] = SX1280_SETBUFFER_BASEADDRESS;
    buf[1] = 0x00; // TX base address
    buf[2] = 0x00; // RX base address
    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, buf, 3, HAL_MAX_DELAY);
    SX1280_Deselect();
    SX1280_BUSY();

    // Modulatie (LoRa, SF7, BW 800kHz, CR 4/5) - Dit komt overeen met ELRS 50Hz
    buf[0] = SX1280_SETMODULATIONPARAMS;
    buf[1] = 0x80;
    buf[2] = 0x18;
    buf[3] = 0x07;
    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, buf, 4, HAL_MAX_DELAY);
    SX1280_Deselect();
    SX1280_BUSY();

    // Optioneel: status checken
    SX1280_PrintStatus("Na ModParams");

    // Fixes voor de chip (verplicht na setmodulation)
    SX1280_WriteRegister(0x925, 0x1E);
    SX1280_WriteRegister(0x93C, 0x01);

    // Packet Params (Preamble 12, CRC OFF, Standaard IQ)
    buf[0] = SX1280_SET_PACKET_PARAMS;
    buf[1] = 12;
    buf[2] = 0x80;  // Fixed/Implicit header
    buf[3] = 8;     // payload length, moet matchen met je payload[8] buffer
    buf[4] = 0x00;  // CRC OFF
    buf[5] = 0x00;
    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, buf, 6, HAL_MAX_DELAY);
    SX1280_Deselect();
    SX1280_BUSY();
    SX1280_PrintStatus("Na PacketParams");

    // Interrupts instellen op RxDone en RX starten
    SX1280_SetDioIrqParams();
    SX1280_SetRx();
    SX1280_PrintStatus("Na SetRx\n");

    printf("Volledige Setup klaar. Luisteren gestart op 2.4404 GHz (Sync Kanaal)...\n\n");
}

void SX1280_OnPacketReceived(void)
{
    // 1. Vraag buffer status op (4 bytes: Opcode -> NOP -> NOP -> NOP)
    uint8_t buf[4] = {SX1280_GETRXBUFFERSTATUS, 0x00, 0x00, 0x00};
    uint8_t status[4] = {0};

    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, buf, status, 4, HAL_MAX_DELAY);
    SX1280_Deselect();

    // Bij een 4-byte transfer zit de data hier:
    uint8_t length = status[2]; // rxPayloadLength
    uint8_t offset = status[3]; // rxStartBufferPointer

    printf("RX Status - Lengte gerapporteerd: %d, Offset: %d\n", length, offset);

    // 2. Veilig de data lezen
    uint8_t read_len = (length > 0) ? length : 8;

    // Voorkom crashes! Maximaal 255 bytes (SX1280 buffer max)
    uint8_t payload[255] = {0};

    SX1280_ReadBuffer(offset, payload, read_len);

    printf("PAKKET GESNIPED! Data: ");
    for(int i = 0; i < read_len; i++) {
        printf("%02X ", payload[i]);
    }
    printf("\n");

    // 3. Wis de RxDone Interrupt Flag
    uint8_t clr_buf[3] = {SX1280_CLRIRQSTATUS, 0xFF, 0xFF};
    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, clr_buf, 3, HAL_MAX_DELAY);
    SX1280_Deselect();

    // 4. Zet radio weer terug op ontvangen
    SX1280_SetRx();
}


void zeroingAnArray( uint8_t arrayToZero[], uint16_t arrayLength ){

    for( uint16_t i = 0; i < arrayLength; i ++ ){
        arrayToZero[ i ] = 0;
    }
}
uint8_t SX1280_TestConnection(void)
{
    printf("\n--- TEST ---\n");
    SX1280_Reset();
    HAL_Delay(50);
    uint8_t test_val = 0x45;
    uint8_t test_offset = 0x00;
    // schrijven naar buffer: Opcode (0x1A) -> Offset -> Data
    uint8_t tx_write[3] = {SX1280_WRITEBUFFER, test_offset, test_val};
    uint8_t rx_write[3] = {0};
    if (SX1280_BUSY() == 1) {
        printf("Fout: chip blijft bussy\n");
        return 0;
    }

    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, tx_write, rx_write, 3, HAL_MAX_DELAY);
    SX1280_Deselect();

    //Lezen buffer: Opcode (0x1B) -> Offset -> NOP -> DATA
    uint8_t tx_read[4] = {SX1280_READBUFFER, test_offset, 0x00, 0x00};
    uint8_t rx_read[4] = {0};

    if (SX1280_BUSY() == 1) {
    	printf("Fout: chip blijft bussy\n");
        return 0;
    }

    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, tx_read, rx_read, 4, HAL_MAX_DELAY);
    SX1280_Deselect();

    printf("Verwacht: 0x%02X\n", test_val);
    printf("Gelezen:  0x%02X\n", rx_read[3]);

    if(rx_read[3] == test_val) {
        printf("Status: SUCCES!\n\n");
        return 1;
    } else {
        printf("Status: FOUT!\n\n");
        return 0;
    }
}


void SX1280_PrintStatus(const char* label)
{
    uint8_t cmd[2] = {0xC0, 0x00};
    uint8_t rx[2]  = {0};
    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, cmd, rx, 2, HAL_MAX_DELAY);
    SX1280_Deselect();

    uint8_t mode   = (rx[0] >> 5) & 0x07;
    uint8_t cmdst  = (rx[0] >> 2) & 0x07;

    const char* modes[] = {"?","?","STDBY_RC","STDBY_XOSC","FS","RX","TX","?"};
    const char* cmds[]  = {"?","?","DataAvail","Timeout","ProcError","FAIL","TxDone","?"};

    printf("[%s] Mode: %s | CmdStatus: %s (0x%02X)\n",
           label, modes[mode], cmds[cmdst], rx[0]);
}







// array van data naar buffer schrijven
void SX1280_WriteBufferArray(uint8_t offset, uint8_t *data, uint8_t size)
{
    uint8_t cmd = SX1280_WRITEBUFFER;

    if(SX1280_BUSY() == 1) return;

    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi2, &offset, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi2, data, size, HAL_MAX_DELAY);
    SX1280_Deselect();
}

// Packet versturen
void SX1280_Transmit(uint8_t *payload, uint8_t size)
{
    // data naar TX buffer
    SX1280_WriteBufferArray(0x00, payload, size);

    // oude INT's weg
    uint8_t clrIrq[3] = {SX1280_CLRIRQSTATUS, 0xFF, 0xFF};
    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, clrIrq, 3, HAL_MAX_DELAY);
    SX1280_Deselect();

    // zet chip in TX mode
    uint8_t buf[4];
    buf[0] = SX1280_SETTX;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = 0x00; // 0 = Verzend 1x daarna stoppen

    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, buf, 4, HAL_MAX_DELAY);
    SX1280_Deselect();

    printf("Pakket verzonden!\n");
}




// Functie om het zendvermogen in te stellen (als je die nog niet had)
void SX1280_SetTxParams(int8_t power, uint8_t rampTime)
{
    uint8_t buf[3];
    buf[0] = SX1280_SETTXPARAMS;
    buf[1] = power;    // 0x1F = 13 dBm (maximaal), 0x12 = 0 dBm
    buf[2] = rampTime; // 0xE0 voor 20us

    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, buf, 3, HAL_MAX_DELAY);
    SX1280_Deselect();
}

// Functie om de testmodus voor antennes/filters te starten (CW)
void SX1280_SetTxContinuousWave(void)
{
    uint8_t cmd = SX1280_SETTXCONTINUOUSWAVE;

    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);
    SX1280_Deselect();

    printf("Waarschuwing: SX1280 zendt nu een continue draaggolf (CW) uit!\n");
}
uint8_t SX1280_CheckRxDonePolling(void)
{
    // 4 bytes: Opcode -> NOP -> NOP -> NOP
    uint8_t tx_buf[4] = {SX1280_GETIRQSTATUS, 0x00, 0x00, 0x00};
    uint8_t rx_buf[4] = {0};

    if (SX1280_BUSY() == 1) return 0;

    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, 4, HAL_MAX_DELAY);
    SX1280_Deselect();

    // rx_buf[0] = Status Byte
    // rx_buf[1] = Status Byte (herhaling)
    // rx_buf[2] = IRQ bits 15-8 (MSB)
    // rx_buf[3] = IRQ bits 7-0 (LSB)

    uint16_t irq_status = (rx_buf[2] << 8) | rx_buf[3];

    if (irq_status != 0) {
        printf("IRQ STATUS GETRIGGERD: 0x%04X\n", irq_status);

        // Wis de flags direct
        uint8_t clrIrq[3] = {SX1280_CLRIRQSTATUS, 0xFF, 0xFF};
        SX1280_Select();
        HAL_SPI_Transmit(&hspi2, clrIrq, 3, HAL_MAX_DELAY);
        SX1280_Deselect();
    }

    if ((irq_status & 0x0002) != 0) return 1; // RxDone is bit 1
    return 0;
}
