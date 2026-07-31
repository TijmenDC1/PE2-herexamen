/*
* dshotsturen.c
*
* Created on: 27 mrt 2026
* Author: tijme
*/

/*
* Deze functie gaat dshot uitsturen
* Het zal naar al de

Specs
DShot300 sturen (als dit werkt kan dit altijd worden aangepast
Frame: 16Bit
0-10 throthle bits
- 0 tot 46 speciale instructies
- 47 tot 2047 puur throthle dus 47 is af 2000 is vol gas
11 telemetry bits
- kan gebruikt worden om via fc data terug te vragen bij ons is dit momenteel de bedoeling de overcurrent terug te vragen
12-15 crc check * */
// m0=M1(PA0), m1=M2(PA2), m2=M3(PD13), m3=M4(PD12)

#include "Dshot.h"
#include <stdint.h>
#include "main.h"

/*
*als de timers niet appart staan ingesteld op 32 en 16
* uint32_t motor1_2_buffer[17]; // TIM2 32bit dus 16 dshot en 1 voor pauze
* uint16_t motor3_4_buffer[17]; // TIM4 16 bit
*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
//beide ingesteld op 16 en dus het eerste is motor id
//uint32_t motor_dmabuf[4][17];
// 17 words zijn er nodig (16 bits + stopbit), maar we reserveren er 24 per motor.
// Reden: 4*24*4 = 384 bytes = exact 12 cache-lijnen van 32 byte. Met de oude
// 4*17*4 = 272 bytes viel het einde MIDDEN in een cache-lijn, waardoor
// SCB_CleanDCache_by_Addr over de buffer heen ging (en dus lijnen van andere
// variabelen meepakte). Nu valt begin en einde precies op een lijngrens.
__attribute__((aligned(32))) uint32_t motor_dmabuf[4][24];

//we hebben dshot300 en de timer van ioc was op 108MHz gezet
#define DS_0 135 // 37.5% van 360		360 = 108MHz / 300000
#define DS_1 270 // 75% van 360
#define DSHOT_BUF_SIZE 17

// uit DshotFCMetMPU6050/Core/Src/dshot.c (auteur: tijme)
void update_motor_buffer(uint8_t motor_id, uint16_t throttle, uint8_t telemetry)
{
    uint16_t telemetry_bit;
    uint16_t gastelemetry;
    uint16_t dshotpakket;
    //0-48 geeen gas
    //if (throttle > 0 && throttle < 48)
    //{
    //    throttle = 48;
    //}
    //2047 is volle gas
    if (throttle > 2047)
    {
        throttle = 2047;
    }

    //telemtry
    if (telemetry ==1){
        telemetry_bit = 1;
    } else {
        telemetry_bit = 0;
    }

    // gas + telemetru
    gastelemetry = (throttle << 1) | telemetry_bit;

    //crc berekening
    uint16_t value = gastelemetry;
    //uitleg crc berekening zie obsidian
    uint16_t crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;

    //alles samen
    dshotpakket = (gastelemetry << 4 ) | crc;

    //buffer vullen
    for (int i = 0; i < 16; i++)
    {
        //0x8000 = 1000 0000 0000 0000 dus 16 bit waarbij ik de masker altijd naar juiste schuif
        uint16_t masker = (0x8000 >> i);
        if ((dshotpakket & masker) != 0)
        {
            motor_dmabuf[motor_id][i] = DS_1;
        } else {
            motor_dmabuf[motor_id][i] = DS_0;
        }

    }
    motor_dmabuf[motor_id][16] = 0; 	//stop bit

}


/* Eenmalige init, aanroepen na MX_TIM2_Init/MX_TIM4_Init en voor het armen.
 *
 * Waarom dit nodig is: na een reset staan de compare-registers (CCR) nog op een
 * willekeurige/oude waarde. De ALLEREERSTE PWM-periode na Start_DMA gebruikt die
 * waarde, waardoor de eerste puls van het eerste frame verkeerd lang is. De ESC
 * doet juist op die eerste frames zijn protocol-detectie (DShot300 herkennen).
 * Mislukt dat, dan negeert hij daarna alles - tot je opnieuw flasht en het per
 * toeval wel goed gaat. Dat verklaart het "om de keer werkt het"-gedrag.
 */
// uit DshotFCMetMPU6050/Core/Src/dshot.c (auteur: tijme)
void dshot_init(void) {
    // compare-registers expliciet op 0 -> lijn begint netjes laag
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);

    // tellers op 0 zodat beide timers vanaf hetzelfde punt starten
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    // hele buffer vullen met een geldig throttle-0 frame (incl. de padding)
    for (uint8_t m = 0; m < 4; m++) {
        update_motor_buffer(m, 0, 0);
    }
}

/* 4. Het verzenden
 * Stuurt de 4 DShot-frames tegelijk uit, elk op zijn eigen timerkanaal + DMA-stream:
 *   motor_dmabuf[0] -> TIM2_CH1 (PA0,  M1)
 *   motor_dmabuf[1] -> TIM2_CH3 (PA2,  M2)
 *   motor_dmabuf[2] -> TIM4_CH2 (PD13, M3)
 *   motor_dmabuf[3] -> TIM4_CH1 (PD12, M4)
 * De frame-klaar-melding (dshot_dma_complete) komt van TIM2_CH1; alle 4 de
 * frames zijn even lang en starten samen, dus dat kanaal is de referentie.
 */
// uit DshotFCMetMPU6050/Core/Src/dshot.c (auteur: tijme)
void send_dshot() {
    //cache verversen zodat DMA de actuele buffer-inhoud leest
    SCB_CleanDCache_by_Addr((uint32_t*)&motor_dmabuf, sizeof(motor_dmabuf));

    //huidige DMA-transfers stoppen
    HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_1);

    //nieuwe verzending starten
    HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*)motor_dmabuf[0], DSHOT_BUF_SIZE);
    HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, (uint32_t*)motor_dmabuf[1], DSHOT_BUF_SIZE);
    HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_2, (uint32_t*)motor_dmabuf[2], DSHOT_BUF_SIZE);
    HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_1, (uint32_t*)motor_dmabuf[3], DSHOT_BUF_SIZE);
}
