
/**
  ******************************************************************************
  * @file    stm32g0xx_mc_it.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt
  *          service routine related to Motor Control
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup STM32G0xx_IRQ_Handlers
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_config.h"
#include "mc_type.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"
#include "stm32g0xx_hal.h"
#include "stm32g0xx.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32G0xx_IRQ_Handlers STM32G0xx IRQ Handlers
  * @{
  */

/* USER CODE BEGIN PRIVATE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */

void BEMF_READING_IRQHandler(void);
void PERIOD_COMM_IRQHandler(void);
void TIMx_BRK_M1_IRQHandler(void);

/**
  * @brief  This function handles TIMx Break-in interrupt request.
  * @param  None
  */
void TIMx_BRK_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_UP_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_UP_BRK_M1_IRQn 0 */

  if(LL_TIM_IsActiveFlag_BRK(TIM1) && LL_TIM_IsEnabledIT_BRK(TIM1))
  {
    LL_TIM_ClearFlag_BRK(TIM1);
    PWMC_BRK_IRQHandler(&PWM_Handle_M1);

    /* USER CODE BEGIN Break */

    /* USER CODE END Break */
  }
  else
  {
    /* Nothing to do */
  }

  if (LL_TIM_IsActiveFlag_BRK2(TIM1) && LL_TIM_IsEnabledIT_BRK(TIM1))
  {
    LL_TIM_ClearFlag_BRK2(TIM1);
    PWMC_BRK_IRQHandler(&PWM_Handle_M1);

    /* USER CODE BEGIN Break */

    /* USER CODE END Break */
  }
  else
  {
     /* No other interrupts are routed to this handler */
  }

  /* USER CODE BEGIN TIMx_UP_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_UP_BRK_M1_IRQn 1 */
}

/**
  * @brief  This function handles BEMF sensing interrupt request.
  * @param[in] None
  */
void BEMF_READING_IRQHandler(void)
{
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 0 */

  /* USER CODE END CURRENT_REGULATION_IRQn 0 */

  if(LL_ADC_IsActiveFlag_AWD1(ADC1) && LL_ADC_IsEnabledIT_AWD1(ADC1))
  {
    /* Clear Flags */
    LL_ADC_ClearFlag_AWD1(ADC1);
    BADC_IsZcDetected(&Bemf_ADC_M1, PWM_Handle_M1.Step);
  }
  else
  {
    /* Nothing to do */
  }

  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 2 */

  /* USER CODE END CURRENT_REGULATION_IRQn 2 */
}

/**
  * @brief     LFtimer interrupt handler
  * @param[in] None
  */
void PERIOD_COMM_IRQHandler(void)
{
  /* TIM Capture compare 1 event */

  if(LL_TIM_IsActiveFlag_CC1(TIM2) && LL_TIM_IsEnabledIT_CC1(TIM2))
  {
    LL_TIM_ClearFlag_CC1(TIM2);
    TSK_SpeedTIM_task();
  }
  else
  {
    /* Nothing to do */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
