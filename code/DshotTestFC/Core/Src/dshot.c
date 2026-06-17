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
//extern TIM_HandleTypeDef htim4;
//beide ingesteld op 16 en dus het eerste is motor id
//uint32_t motor_dmabuf[4][17];
__attribute__((aligned(32))) uint32_t motor_dmabuf[4][17];	// alignen op 32 bit anders kan dcache clearen andere vars overschrijven

//we hebben dshot300 en de timer van ioc was op 108MHz gezet
#define DS_0 135 // 37.5% van 360		360 = 108MHz / 300000
#define DS_1 270 // 75% van 360
#define DSHOT_BUF_SIZE 17

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


/* 4. Het verzenden */
void send_dshot() {
//verschonen van de cache
   SCB_CleanDCache_by_Addr((uint32_t*)&motor_dmabuf, sizeof(motor_dmabuf));
    //Stop huidige DMA transfers
    HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
    //HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_3);
    //HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_1);
    //HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_2);

    //nieuwe verzending
    HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*)motor_dmabuf[0], DSHOT_BUF_SIZE);
    //HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, (uint32_t*)motor_dmabuf[1], DSHOT_BUF_SIZE);
    //HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_4, (uint32_t*)motor_dmabuf[2], DSHOT_BUF_SIZE);
    //HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_2, (uint32_t*)motor_dmabuf[3], DSHOT_BUF_SIZE);
}
