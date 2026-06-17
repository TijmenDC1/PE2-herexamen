/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BMP_SCK_Pin GPIO_PIN_2
#define BMP_SCK_GPIO_Port GPIOE
#define Autonoom_Controller_Pin GPIO_PIN_3
#define Autonoom_Controller_GPIO_Port GPIOE
#define BMP_CS_Pin GPIO_PIN_4
#define BMP_CS_GPIO_Port GPIOE
#define BMP_MISO_Pin GPIO_PIN_5
#define BMP_MISO_GPIO_Port GPIOE
#define BMP_MOSI_Pin GPIO_PIN_6
#define BMP_MOSI_GPIO_Port GPIOE
#define BMP_INT_Pin GPIO_PIN_13
#define BMP_INT_GPIO_Port GPIOC
#define BMP_INT_EXTI_IRQn EXTI15_10_IRQn
#define Dshot_M1_Pin GPIO_PIN_0
#define Dshot_M1_GPIO_Port GPIOA
#define Overcurrent_M1_Pin GPIO_PIN_1
#define Overcurrent_M1_GPIO_Port GPIOA
#define Dshot_M2_Pin GPIO_PIN_2
#define Dshot_M2_GPIO_Port GPIOA
#define Overcurrent_M2_Pin GPIO_PIN_3
#define Overcurrent_M2_GPIO_Port GPIOA
#define BMI_CS_Pin GPIO_PIN_4
#define BMI_CS_GPIO_Port GPIOA
#define BMI_SCK_Pin GPIO_PIN_5
#define BMI_SCK_GPIO_Port GPIOA
#define BMI_MISO_Pin GPIO_PIN_6
#define BMI_MISO_GPIO_Port GPIOA
#define BMI_MOSI_Pin GPIO_PIN_7
#define BMI_MOSI_GPIO_Port GPIOA
#define BMI_INT1_Pin GPIO_PIN_4
#define BMI_INT1_GPIO_Port GPIOC
#define BMI_INT1_EXTI_IRQn EXTI4_IRQn
#define BMI_INT2_Pin GPIO_PIN_5
#define BMI_INT2_GPIO_Port GPIOC
#define BMI_INT2_EXTI_IRQn EXTI9_5_IRQn
#define GPS_Timer_Pin GPIO_PIN_2
#define GPS_Timer_GPIO_Port GPIOB
#define GPS_RX_Pin GPIO_PIN_7
#define GPS_RX_GPIO_Port GPIOE
#define GPS_TX_Pin GPIO_PIN_8
#define GPS_TX_GPIO_Port GPIOE
#define SX1280_CS_Pin GPIO_PIN_12
#define SX1280_CS_GPIO_Port GPIOB
#define SX1280_SCK_Pin GPIO_PIN_13
#define SX1280_SCK_GPIO_Port GPIOB
#define SX1280_MISO_Pin GPIO_PIN_14
#define SX1280_MISO_GPIO_Port GPIOB
#define SX1280_MOSI_Pin GPIO_PIN_15
#define SX1280_MOSI_GPIO_Port GPIOB
#define SX1280_nRST_Pin GPIO_PIN_8
#define SX1280_nRST_GPIO_Port GPIOD
#define SX1280_Bussy_Pin GPIO_PIN_9
#define SX1280_Bussy_GPIO_Port GPIOD
#define Overcurrent_M4_Pin GPIO_PIN_10
#define Overcurrent_M4_GPIO_Port GPIOD
#define Overcurrent_M3_Pin GPIO_PIN_11
#define Overcurrent_M3_GPIO_Port GPIOD
#define Dshot_M4_Pin GPIO_PIN_12
#define Dshot_M4_GPIO_Port GPIOD
#define Dshot_M3_Pin GPIO_PIN_13
#define Dshot_M3_GPIO_Port GPIOD
#define SD_D0_Pin GPIO_PIN_8
#define SD_D0_GPIO_Port GPIOC
#define SD_D1_Pin GPIO_PIN_9
#define SD_D1_GPIO_Port GPIOC
#define STLink_DIO_Pin GPIO_PIN_13
#define STLink_DIO_GPIO_Port GPIOA
#define STLink_CLK_Pin GPIO_PIN_14
#define STLink_CLK_GPIO_Port GPIOA
#define SD_D2_Pin GPIO_PIN_10
#define SD_D2_GPIO_Port GPIOC
#define SD_D3_Pin GPIO_PIN_11
#define SD_D3_GPIO_Port GPIOC
#define SD_CK_Pin GPIO_PIN_12
#define SD_CK_GPIO_Port GPIOC
#define SD_CMD_Pin GPIO_PIN_2
#define SD_CMD_GPIO_Port GPIOD
#define SD_SWA_Pin GPIO_PIN_3
#define SD_SWA_GPIO_Port GPIOB
#define BMM_INT_Pin GPIO_PIN_7
#define BMM_INT_GPIO_Port GPIOB
#define BMM_INT_EXTI_IRQn EXTI9_5_IRQn
#define BMM_SCL_Pin GPIO_PIN_8
#define BMM_SCL_GPIO_Port GPIOB
#define BMM_SDA_Pin GPIO_PIN_9
#define BMM_SDA_GPIO_Port GPIOB
#define STLink_TX_Pin GPIO_PIN_0
#define STLink_TX_GPIO_Port GPIOE
#define STLink_RX_Pin GPIO_PIN_1
#define STLink_RX_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define PI 3.14159265358979323846
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
