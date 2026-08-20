#include "sx1280include.h"
#include "main.h"
#include "stdio.h"

extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim3;
extern volatile uint8_t hop_index;
extern volatile uint8_t synced;
extern uint8_t hop_table[];

volatile uint8_t  link_state = STATE_ZOEKEN;
volatile uint8_t  nonce      = 0;
volatile uint8_t  do_hop     = 0;
volatile uint32_t pkt_count  = 0;
volatile uint16_t rc_channels[4] = {0};

volatile uint8_t  failsafe = 1;
volatile uint8_t  lq       = 0;

volatile uint8_t try_nr   = 0;
volatile uint8_t lock_ok  = 0;
volatile uint8_t ok_count = 0;

volatile uint8_t  anchor_nonce = 0;
volatile uint16_t anchor_index = 0;
volatile uint32_t ticks        = 0;
volatile uint32_t anchor_ticks = 0;

volatile uint8_t spi_busy = 0;

static uint32_t last_pkt_time = 0;
static uint32_t print_teller = 0;

static void ZetFailsafe(void)
{
    for (int i = 0; i < 4; i++) rc_channels[i] = FS_MIDDEN;
    rc_channels[FS_GAS_KANAAL] = FS_GAS_WAARDE;
    failsafe = 1;
}

uint8_t ELRS_LinkOK(void)
{
    return (link_state == STATE_LOCKED) && !failsafe;
}

/* ------------------ SPI ------------------ */

uint8_t SX1280_BUSY(void)
{
    /* lusteller ipv HAL_GetTick, want dit draait ook in de timer interrupt */
    uint32_t wacht = 2000000u;

    while (HAL_GPIO_ReadPin(SX1280_Bussy_GPIO_Port, SX1280_Bussy_Pin) == GPIO_PIN_SET) {
        if (--wacht == 0) return 1;
    }
    return 0;
}

void SX1280_Select(void)
{
    spi_busy = 1;
    __asm__ volatile ("NOP \n NOP \n NOP");
    HAL_GPIO_WritePin(SX1280_CS_GPIO_Port, SX1280_CS_Pin, 0);
    __asm__ volatile ("NOP \n NOP \n NOP");
}

void SX1280_Deselect(void)
{
    __asm__ volatile ("NOP \n NOP \n NOP");
    HAL_GPIO_WritePin(SX1280_CS_GPIO_Port, SX1280_CS_Pin, GPIO_PIN_SET);
    __asm__ volatile ("NOP \n NOP \n NOP");
    spi_busy = 0;
}

static void SX1280_Cmd(uint8_t *buf, uint8_t len)
{
    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, buf, len, HAL_MAX_DELAY);
    SX1280_Deselect();
    SX1280_BUSY();
}

int8_t SX1280_GetRssiInst(void)
{
    uint8_t tx[3] = {SX1280_GETRSSIINST, 0x00, 0x00};
    uint8_t rx[3] = {0};

    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 3, HAL_MAX_DELAY);
    SX1280_Deselect();

    return -(rx[2] / 2);
}

/* rssi = -rx[2]/2 dBm, snr = rx[3]/4 dB */
void SX1280_GetPacketStatus(int8_t *rssi, int8_t *snr)
{
    uint8_t tx[7] = {SX1280_GETPACKETSTATUS, 0, 0, 0, 0, 0, 0};
    uint8_t rx[7] = {0};

    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 7, HAL_MAX_DELAY);
    SX1280_Deselect();

    if (rssi) *rssi = -(int8_t)(rx[2] / 2);
    if (snr)  *snr  = ((int8_t)rx[3]) / 4;
}

void SX1280_getstatus(void)
{
    uint8_t tx[2] = {SX1280_GETSTATUS, 0x00};
    uint8_t rx[2];
    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, HAL_MAX_DELAY);
    SX1280_Deselect();
}

uint8_t SX1280_GetStatusByte(void)
{
    uint8_t tx[2] = {SX1280_GETSTATUS, 0x00};
    uint8_t rx[2] = {0};
    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, HAL_MAX_DELAY);
    SX1280_Deselect();
    return rx[0] ? rx[0] : rx[1];
}

/* bits 7:5 = mode, bits 4:2 = commando status */
void SX1280_PrintStatus(const char *label)
{
    uint8_t st = SX1280_GetStatusByte();
    uint8_t mode = (st >> 5) & 0x07;
    uint8_t cmd  = (st >> 2) & 0x07;

    printf("status %s: 0x%02X mode=%u cmd=%u%s\n", label, st, mode, cmd,
           (cmd == 4 || cmd == 5) ? " FOUT" : "");
}

void SX1280_Reset(void)
{
    HAL_GPIO_WritePin(SX1280_nRST_GPIO_Port, SX1280_nRST_Pin, 0);
    HAL_Delay(20);
    __asm__ volatile ("NOP \n NOP \n NOP");
    HAL_GPIO_WritePin(SX1280_nRST_GPIO_Port, SX1280_nRST_Pin, 1);
    HAL_Delay(20);
    SX1280_BUSY();

    SX1280_getstatus();
    SX1280_getstatus();
}

void SX1280_WriteRegister(uint16_t address, uint8_t value)
{
    uint8_t tx[4] = {SX1280_WRITEREGISTER, (uint8_t)(address >> 8), (uint8_t)address, value};
    uint8_t rx[4] = {0};

    if (SX1280_BUSY() == 1) return;

    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 4, HAL_MAX_DELAY);
    SX1280_Deselect();
}

uint8_t SX1280_ReadRegister(uint16_t address)
{
    uint8_t tx[5] = {SX1280_READREGISTER, (uint8_t)(address >> 8), (uint8_t)address, 0x00, 0x00};
    uint8_t rx[5] = {0};

    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 5, HAL_MAX_DELAY);
    SX1280_Deselect();
    return rx[4];
}

void SX1280_ReadBuffer(uint8_t offset, uint8_t *data, uint8_t size)
{
    uint8_t nop = 0x00;
    uint8_t opcode = SX1280_READBUFFER;

    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, &opcode, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi2, &offset, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi2, &nop, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, data, size, HAL_MAX_DELAY);
    SX1280_Deselect();
}

void SX1280_SetRfFrequency(uint32_t frequency)
{
    frequency = frequency + ELRS_FREQ_CORRECTIE_HZ;

    uint32_t rf = (uint32_t)(((uint64_t)frequency << 18) / 52000000);
    uint8_t buf[4] = {SX1280_SETRFFREQUENCY, (uint8_t)(rf >> 16), (uint8_t)(rf >> 8), (uint8_t)rf};

    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, buf, 4, HAL_MAX_DELAY);
    SX1280_Deselect();
}

void SX1280_SetRx(void)
{
    /* 0x02 = 1ms tijdbasis, 0xFFFF = continu */
    uint8_t buf[4] = {SX1280_SETRX, 0x02, 0xFF, 0xFF};

    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, buf, 4, HAL_MAX_DELAY);
    SX1280_Deselect();
    SX1280_BUSY();
}

void SX1280_SetDioIrqParams(void)
{
    uint16_t irq  = SX1280_IRQ_RX_DONE | SX1280_IRQ_RX_TX_TIMEOUT | SX1280_IRQ_CRC_ERROR;
    uint16_t dio1 = SX1280_IRQ_RX_DONE;

    uint8_t buf[9] = {SX1280_SET_DIO_IRQ_PARAMS,
                      (uint8_t)(irq >> 8),  (uint8_t)irq,
                      (uint8_t)(dio1 >> 8), (uint8_t)dio1,
                      0x00, 0x00,
                      0x00, 0x00};

    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_Transmit(&hspi2, buf, 9, HAL_MAX_DELAY);
    SX1280_Deselect();
}

void SX1280_SetXtalTrim(uint8_t trim)
{
    uint8_t buf[2] = {SX1280_SET_STANDBY, 0x01};
    SX1280_Cmd(buf, 2);

    trim &= 0x1F;
    SX1280_WriteRegister(SX1280_REG_XTAL_A, trim);
    SX1280_WriteRegister(SX1280_REG_XTAL_B, trim);
}

/* ------------------ setup ------------------ */

void SX1280_Setup_ELRS(void)
{
    uint8_t buf[10];

    printf("\nSetup %s\n", ELRS_RATE_NAAM);
    printf("crc seed 0x%04X, fhss seed 0x%08lX, iq 0x%02X\n",
           (unsigned)ELRS_CRC_INIT, (unsigned long)ELRS_FHSS_SEED, ELRS_IQ_MODE);

    SX1280_Reset();
    HAL_Delay(10);

    buf[0] = SX1280_SET_STANDBY;      buf[1] = 0x00;
    SX1280_Cmd(buf, 2);

    buf[0] = SX1280_SETREGULATORMODE; buf[1] = 0x00;
    SX1280_Cmd(buf, 2);

    /* packet type moet voor de modulatie- en packetparameters */
    buf[0] = SX1280_SETPACKET_TYPE;   buf[1] = SX1280_PACKET_TYPE_LORA;
    SX1280_Cmd(buf, 2);

    SX1280_SetRfFrequency(SYNC_FREQ);

    buf[0] = SX1280_SETBUFFER_BASEADDRESS; buf[1] = 0x00; buf[2] = 0x00;
    SX1280_Cmd(buf, 3);

    buf[0] = SX1280_SETMODULATIONPARAMS;
    buf[1] = ELRS_SF;
    buf[2] = ELRS_BW;
    buf[3] = ELRS_CR;
    SX1280_Cmd(buf, 4);

    /* waarde hangt af van de SF */
    SX1280_WriteRegister(SX1280_REG_SF_CONFIG, ELRS_SF_REG);
    SX1280_WriteRegister(SX1280_REG_FREQ_ERR, 0x01);

    /* implicit header, 8 bytes, hardware crc uit (ELRS doet eigen crc14) */
    buf[0] = SX1280_SET_PACKET_PARAMS;
    buf[1] = ELRS_PREAMBLE_LEN;
    buf[2] = SX1280_LORA_PACKET_IMPLICIT;
    buf[3] = PKT_SIZE;
    buf[4] = SX1280_LORA_CRC_OFF;
    buf[5] = ELRS_IQ_MODE;
    SX1280_Cmd(buf, 6);

#if ELRS_XTAL_TRIM >= 0
    SX1280_SetXtalTrim(ELRS_XTAL_TRIM);
#endif

    SX1280_SetDioIrqParams();

    /* teruglezen om te zien of de schrijfacties aankomen */
    uint8_t r925 = SX1280_ReadRegister(SX1280_REG_SF_CONFIG);
    uint8_t r93C = SX1280_ReadRegister(SX1280_REG_FREQ_ERR);

    printf("reg 0x0925=0x%02X 0x093C=0x%02X\n", r925, r93C);
    if (r925 != ELRS_SF_REG || r93C != 0x01) {
        printf("registers kloppen niet, check spi\n");
    }

    SX1280_PrintStatus("setup");

    /* timer telt 1us per tick */
    __HAL_TIM_SET_AUTORELOAD(&htim3, ELRS_INTERVAL_US - 1);

    SX1280_SetRx();
    SX1280_PrintStatus("rx");

    link_state = STATE_ZOEKEN;
    nonce      = 0;
    do_hop     = 0;
    synced     = 0;
    hop_index  = 0;
    last_pkt_time = HAL_GetTick();

    printf("zoeken op kanaal %d\n", SYNC_CHANNEL);
}

uint8_t SX1280_TestConnection(void)
{
    SX1280_Reset();
    HAL_Delay(50);

    uint8_t testwaarde = 0x45;
    uint8_t tx1[3] = {SX1280_WRITEBUFFER, 0x00, testwaarde};
    uint8_t rx1[3] = {0};

    if (SX1280_BUSY() == 1) return 0;
    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, tx1, rx1, 3, HAL_MAX_DELAY);
    SX1280_Deselect();

    uint8_t tx2[4] = {SX1280_READBUFFER, 0x00, 0x00, 0x00};
    uint8_t rx2[4] = {0};

    if (SX1280_BUSY() == 1) return 0;
    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, tx2, rx2, 4, HAL_MAX_DELAY);
    SX1280_Deselect();

    if (rx2[3] == testwaarde) return 1;
    return 0;
}

/* ------------------ polling ------------------ */

uint8_t SX1280_CheckRxDonePolling(void)
{
    static uint32_t lq_tijd = 0;
    static uint32_t lq_vorig = 0;

    /* link quality = percentage van wat er in 3 seconden had kunnen binnenkomen */
    if ((HAL_GetTick() - lq_tijd) > 3000) {
        lq_tijd = HAL_GetTick();

        uint32_t nieuw    = pkt_count - lq_vorig;
        uint32_t verwacht = 3000000UL / ELRS_INTERVAL_US;
        uint32_t proc     = (nieuw * 100UL) / verwacht;
        if (proc > 100) proc = 100;
        lq = (uint8_t)proc;

        printf("%s LQ %lu%% rssi %d%s\n",
               (link_state == STATE_LOCKED) ? "lock" : "zoek",
               (unsigned long)proc, SX1280_GetRssiInst(),
               failsafe ? " FS" : "");
        lq_vorig = pkt_count;
    }

    /* te lang niks ontvangen: link kwijt. onbevestigde poging krijgt minder tijd */
    uint32_t grens = lock_ok ? LINK_TIMEOUT_MS : ACQ_TIJD_MS;

    if (link_state == STATE_LOCKED && (HAL_GetTick() - last_pkt_time) > grens) {
        link_state = STATE_ZOEKEN;
        synced = 0;
        ZetFailsafe();
        ELRS_PhaseReset();
        HAL_TIM_Base_Stop_IT(&htim3);

        if (lock_ok) {
            printf("link kwijt\n");
            try_nr = 0;
        } else {
            printf("poging %u mislukt (%u ok)\n", (unsigned)(try_nr + 1), ok_count);
            try_nr = (try_nr + 1) % ACQ_POGINGEN;
        }
        lock_ok = 0;

        SX1280_SetRfFrequency(SYNC_FREQ);
        SX1280_SetRx();
        last_pkt_time = HAL_GetTick();
    }

    uint8_t tx[4] = {SX1280_GETIRQSTATUS, 0x00, 0x00, 0x00};
    uint8_t rx[4] = {0};

    if (SX1280_BUSY() == 1) return 0;

    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 4, HAL_MAX_DELAY);
    SX1280_Deselect();

    uint16_t irq = (rx[2] << 8) | rx[3];

    if (irq != 0) {
        uint8_t clr[3] = {SX1280_CLRIRQSTATUS, 0xFF, 0xFF};
        SX1280_Select();
        HAL_SPI_Transmit(&hspi2, clr, 3, HAL_MAX_DELAY);
        SX1280_Deselect();
    }

    if (irq & SX1280_IRQ_RX_DONE) return 1;
    return 0;
}

/* ------------------ crc14 ------------------ */

static uint16_t crc_tabel[256];

void ELRS_InitCRC(void)
{
    uint16_t poly = CRC14_POLY;

    for (uint16_t i = 0; i < 256; i++) {
        uint16_t crc = i << 6;
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc << 1) ^ ((crc & (1 << 13)) ? poly : 0);
        }
        crc_tabel[i] = crc & 0x3FFF;
    }
}

uint16_t ELRS_CalculateCRC(uint8_t *data, uint8_t len, uint16_t seed)
{
    uint16_t crc = seed;
    for (uint8_t i = 0; i < len; i++) {
        crc = (crc << 8) ^ crc_tabel[((crc >> 6) ^ data[i]) & 0xFF];
    }
    return crc & 0x3FFF;
}

/* sync heeft altijd nonce 0, bij rc-data proberen we alle 256 nonces */
uint8_t ELRS_ValidatePacket(uint8_t *buf, uint16_t crc_in, uint16_t *seed_uit)
{
    if ((buf[0] & 0x03) == PKT_SYNC) {
        if (ELRS_CalculateCRC(buf, CRC_LEN, ELRS_CRC_INIT) == crc_in) {
            if (seed_uit) *seed_uit = ELRS_CRC_INIT;
            return 1;
        }
        return 0;
    }

    for (uint16_t n = 0; n < 256; n++) {
        uint16_t seed = ELRS_CRC_INIT ^ n;
        if (ELRS_CalculateCRC(buf, CRC_LEN, seed) == crc_in) {
            if (seed_uit) *seed_uit = seed;
            return 1;
        }
    }
    return 0;
}

/* 4 kanalen van 10 bit in payload[1..5] */
void ELRS_UnpackChannels(uint8_t *payload, uint16_t *ch)
{
    uint32_t waarde = 0;
    uint8_t  bits = 0;
    uint8_t  index = 1;

    for (uint8_t n = 0; n < 4; n++) {
        while (bits < 10) {
            waarde |= ((uint32_t)payload[index++]) << bits;
            bits += 8;
        }
        ch[n] = waarde & 0x03FF;
        waarde >>= 10;
        bits -= 10;
    }
}

/* ------------------ timing ------------------ */

/* nonce eerst ophogen, dan pas testen (zoals ELRS 4.x) */
void ELRS_Tick(void)
{
    nonce++;
    ticks++;

    if ((nonce % ELRS_HOP_INTERVAL) != 0) return;

    /* loopt er spi, dan doet de main loop de hop */
    if (spi_busy) {
        do_hop = 1;
    } else {
        ELRS_HandleHop();
    }
}

static uint8_t fase_ok = 0;
static int32_t fase_doel = 0;

void ELRS_PhaseReset(void)
{
    fase_ok = 0;
}

/* timer gelijk zetten met de zender, zacht bijsturen */
static void ELRS_PhaseLock(void)
{
    int32_t nu = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);

    if (!fase_ok) {
        fase_doel = nu + FASE_OFFSET_US;
        while (fase_doel >= ELRS_INTERVAL_US) fase_doel -= ELRS_INTERVAL_US;
        fase_ok = 1;
        return;
    }

    int32_t fout = nu - fase_doel;
    if (fout >  ELRS_INTERVAL_US / 2) fout -= ELRS_INTERVAL_US;
    if (fout < -ELRS_INTERVAL_US / 2) fout += ELRS_INTERVAL_US;

    int32_t nieuw = nu - (fout / 8);
    while (nieuw < 0)                  nieuw += ELRS_INTERVAL_US;
    while (nieuw >= ELRS_INTERVAL_US)  nieuw -= ELRS_INTERVAL_US;

    __HAL_TIM_SET_COUNTER(&htim3, (uint32_t)nieuw);
}

/* index wordt uitgerekend uit de nonce, niet opgehoogd, dan herstelt een
 * gemiste hop zichzelf */
void ELRS_HandleHop(void)
{
    do_hop = 0;

    uint32_t verstreken = ticks - anchor_ticks;
    uint32_t nonce_abs  = anchor_nonce + verstreken;

    int32_t idx = (int32_t)anchor_index
                + (int32_t)(nonce_abs / ELRS_HOP_INTERVAL)
                - (int32_t)(anchor_nonce / ELRS_HOP_INTERVAL)
                + INDEX_OFFSET;

    idx %= HOP_COUNT;
    if (idx < 0) idx += HOP_COUNT;

    hop_index = (uint8_t)idx;

    SX1280_SetRfFrequency(FREQ_START + (uint32_t)hop_table[hop_index] * FREQ_STEP);
    SX1280_SetRx();
}

/* ------------------ pakket verwerken ------------------ */

void SX1280_OnPacketReceived(void)
{
    uint8_t tx[4] = {SX1280_GETRXBUFFERSTATUS, 0x00, 0x00, 0x00};
    uint8_t rx[4] = {0};

    SX1280_BUSY();
    SX1280_Select();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 4, HAL_MAX_DELAY);
    SX1280_Deselect();

    /* bij implicit header is de lengte altijd 0, dus we lezen gewoon PKT_SIZE */
    uint8_t offset = rx[3];
    uint8_t payload[PKT_SIZE] = {0};

    SX1280_ReadBuffer(offset, payload, PKT_SIZE);
    pkt_count++;

    /* byte 0 = type + crc high, byte 7 = crc low */
    uint8_t  type   = payload[0] & 0x03;
    uint16_t crc_in = ((uint16_t)(payload[0] >> 2) << 8) | payload[7];

    uint8_t buf[CRC_LEN];
    for (int i = 0; i < CRC_LEN; i++) buf[i] = payload[i];
    buf[0] &= 0x03;

    if (link_state == STATE_ZOEKEN)
    {
        static uint32_t zk_totaal = 0, zk_geldig = 0, zk_print = 0;

        zk_totaal++;
        uint16_t seed = 0;
        uint8_t  geldig = ELRS_ValidatePacket(buf, crc_in, &seed);

        /* zwakke pakketten komen van een buurkanaal, daar mogen we niet op locken */
        int8_t rssi = 0, snr = -128;
        if (geldig) {
            zk_geldig++;
            SX1280_GetPacketStatus(&rssi, &snr);
        }

        if ((HAL_GetTick() - zk_print) > 3000) {
            zk_print = HAL_GetTick();
            printf("zoeken %lu / %lu ok\n",
                   (unsigned long)zk_geldig, (unsigned long)zk_totaal);
        }

        /* een sync pakket bevat de index en de nonce, en draagt uid4/uid5 mee */
        uint8_t sync_ok = geldig && (type == PKT_SYNC);
        if (sync_ok) {
            sync_ok = (payload[5] == ELRS_UID4)
                   && (payload[6] == ELRS_UID5)
                   && (payload[1] < HOP_COUNT);
        }

        if (sync_ok)
        {
            hop_index  = payload[1] % HOP_COUNT;
            nonce      = payload[2];
            link_state = STATE_LOCKED;
            synced     = 1;
            lock_ok    = 1;
            last_pkt_time = HAL_GetTick();

            anchor_nonce = nonce;
            anchor_index = hop_index;
            anchor_ticks = ticks;

            printf("sync, index %u nonce %u\n", hop_index, nonce);

            ELRS_PhaseReset();
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            HAL_TIM_Base_Start_IT(&htim3);
        }
        /* zonder sync: we staan op kanaal 40, dus de zender staat op index
         * 0, 80 of 160. die proberen we een voor een */
        else if (geldig && type == PKT_RCDATA && snr >= ACQ_MIN_SNR)
        {
            uint8_t k = try_nr % ACQ_POGINGEN;

            hop_index = k * FREQ_COUNT;
            nonce     = NONCE_UIT_SEED(seed);

            link_state = STATE_LOCKED;
            synced     = 1;
            lock_ok    = 0;
            ok_count   = 0;
            last_pkt_time = HAL_GetTick();

            anchor_nonce = nonce;
            anchor_index = hop_index;
            anchor_ticks = ticks;

            printf("poging %u: index %u nonce %u\n", (unsigned)(k + 1), hop_index, nonce);

            ELRS_PhaseReset();
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            HAL_TIM_Base_Start_IT(&htim3);
        }
    }
    else
    {
        int16_t nonce_ok = -1;
        uint8_t verwacht = nonce;

        if (type == PKT_SYNC) {
            if (ELRS_ValidatePacket(buf, crc_in, NULL)) {
                nonce_ok  = nonce;
                hop_index = payload[1] % HOP_COUNT;
                nonce     = payload[2];

                anchor_nonce = nonce;
                anchor_index = hop_index;
                anchor_ticks = ticks;
                lock_ok      = 1;
            }
        } else {
            /* nonce zit in de seed, klein venster rond de verwachte waarde */
            for (int8_t d = -2; d <= 2; d++) {
                uint8_t test = verwacht + d;
                if (ELRS_CalculateCRC(buf, CRC_LEN, SEED_VOOR(type, test)) == crc_in) {
                    nonce_ok = test;
                    nonce    = test;
                    break;
                }
            }
        }

        if (nonce_ok >= 0) {
            last_pkt_time = HAL_GetTick();

            if (!lock_ok && ++ok_count >= ACQ_BEVESTIG) {
                lock_ok = 1;
                printf("link bevestigd\n");
            }

            ELRS_PhaseLock();

            if (type == PKT_RCDATA) {
                uint16_t ch[4];
                ELRS_UnpackChannels(payload, ch);
                for (int i = 0; i < 4; i++) rc_channels[i] = ch[i];
                failsafe = 0;

                if ((++print_teller % PRINT_ELKE) == 0) {
                    printf("CH %4u %4u %4u %4u  nonce %u  idx %u\n",
                           ch[0], ch[1], ch[2], ch[3], (uint8_t)nonce_ok, hop_index);
                }
            }
        }
    }

    /* geen SetRx hier: we staan in rx continu, opnieuw starten gooit het
     * volgende pakket weg. na een hop gebeurt het wel */
}
