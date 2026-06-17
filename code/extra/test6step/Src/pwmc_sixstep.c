
/**
  ******************************************************************************
  * @file    pwmc_sixstep.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides the firmware functions that implement the
  *          pwmc_sixstep component of the Motor Control SDK.
  *
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
  * @ingroup PWMC_SixStep
  */

/* Includes ------------------------------------------------------------------*/
#include "pwmc_sixstep.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SixStep
  * @{
  */

/**
 * @defgroup PWMC_SixStep PWM management
 *
 * @brief PWM generation implementation for Six-Step drive
 *
 * This implementation drives the bridges with 3 signals and
 * their complementary ones or 3 enables.
 *
 * @todo: Complete documentation.
 * @{
 */

/* Private defines -----------------------------------------------------------*/
#define TIMxCCER_MASK_CH123        ((uint16_t)(LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|\
                                               LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|\
                                               LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N))

/**
  * @brief  It initializes TIMx and NVIC.
  * @param  pHdl: handler of the current instance of the PWM component.
  * @retval none
  */
__weak void PWMC_Init(PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    /* Clear TIMx break flag. */
    LL_TIM_ClearFlag_BRK(pHandle->pParams_str->TIMx);
    LL_TIM_EnableIT_BRK(pHandle->pParams_str->TIMx);

    /* Disable ADC source trigger. */
    LL_TIM_SetTriggerOutput(pHandle->pParams_str->TIMx, LL_TIM_TRGO_RESET);

    /* Enable PWM channel. */
    LL_TIM_CC_EnableChannel(pHandle->pParams_str->TIMx, TIMxCCER_MASK_CH123);

    LL_TIM_EnableCounter(pHandle->pParams_str->TIMx);
    pHandle->pCCMR1_cfg = &(pHandle->TimerCfg->CCMR1_Standard_cfg[0]);
    pHandle->pCCMR2_cfg = &(pHandle->TimerCfg->CCMR2_Standard_cfg[0]);
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  }
#endif
}

/**
* @brief  It updates the stored duty cycle variable.
* @param  pHandle Pointer on the target component instance.
* @param  new duty cycle value.
* @retval none
*/
__weak void PWMC_SetPhaseVoltage(PWMC_Handle_t *pHandle, uint16_t DutyCycle)
{
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    pHandle->CntPh = DutyCycle;
    LL_TIM_WriteReg(pHandle->pParams_str->TIMx, CCR1, (uint32_t)DutyCycle);
    LL_TIM_WriteReg(pHandle->pParams_str->TIMx, CCR2, (uint32_t)DutyCycle);
    LL_TIM_WriteReg(pHandle->pParams_str->TIMx, CCR3, (uint32_t)DutyCycle);
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  }
#endif
}

/**
* @brief  It writes the duty cycle into timer shadow registers.
* @param  pHandle Pointer on the target component instance.
* @retval none
*/
__weak void PWMC_LoadNextStep(PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
    if (pHandle->AlignFlag != 0)
    {
      pHandle->pCCER_cfg = &(pHandle->TimerCfg->CCER_Align_cfg);
      if (pHandle->AlignFlag == -1)
      {
        pHandle->pCCMR1_cfg = &(pHandle->TimerCfg->CCMR1_CCW_Align_cfg[0]);
        pHandle->pCCMR2_cfg = &(pHandle->TimerCfg->CCMR2_CCW_Align_cfg[0]);
      }
      else
      {
        pHandle->pCCMR1_cfg = &(pHandle->TimerCfg->CCMR1_CW_Align_cfg[0]);
        pHandle->pCCMR2_cfg = &(pHandle->TimerCfg->CCMR2_CW_Align_cfg[0]);
      }

      LL_TIM_WriteReg(TIMx, CCER, *(pHandle->pCCER_cfg));

    }
    else
    {
      if (pHandle->QuasiSynchDecay)
      {
        pHandle->pCCER_cfg = &(pHandle->TimerCfg->CCER_QuasiSynch_cfg[0]);
        LL_TIM_WriteReg(TIMx, CCER, *(pHandle->pCCER_cfg + pHandle->Step));
      }
      else
      {
        pHandle->pCCER_cfg = &(pHandle->TimerCfg->CCER_cfg[0]);
        if (1U == pHandle->LSModArray[pHandle->Step])
        {
          pHandle->pCCMR1_cfg = &(pHandle->TimerCfg->CCMR1_LSMod_cfg[0]);
          pHandle->pCCMR2_cfg = &(pHandle->TimerCfg->CCMR2_LSMod_cfg[0]);
        }
        else
        {
          pHandle->pCCMR1_cfg = &(pHandle->TimerCfg->CCMR1_Standard_cfg[0]);
          pHandle->pCCMR2_cfg = &(pHandle->TimerCfg->CCMR2_Standard_cfg[0]);
        }
      }
      LL_TIM_WriteReg(TIMx, CCER, *(pHandle->pCCER_cfg + pHandle->Step));
    }
    LL_TIM_WriteReg(TIMx, CCMR1, *(pHandle->pCCMR1_cfg + pHandle->Step));
    LL_TIM_WriteReg(TIMx, CCMR2, *(pHandle->pCCMR2_cfg + pHandle->Step));
    LL_TIM_GenerateEvent_COM(TIMx);
    LL_TIM_GenerateEvent_UPDATE(TIMx);
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  }
#endif
}

/**
  * @brief  It turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers.
  * @param  pHdl: handler of the current instance of the PWM component.
  * @retval none
  */
__weak void PWMC_TurnOnLowSides(PWMC_Handle_t * pHandle, uint32_t ticks)
{
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

    pHandle->TurnOnLowSidesAction = true;  /* Set the Flag. */
    /* Turn on the three low side switches. */
    LL_TIM_OC_SetCompareCH1(TIMx, ticks);
    LL_TIM_OC_SetCompareCH2(TIMx, ticks);
    LL_TIM_OC_SetCompareCH3(TIMx, ticks);
    LL_TIM_WriteReg(TIMx, CCMR1, pHandle->TimerCfg->CCMR1_BootCharge);
    LL_TIM_WriteReg(TIMx, CCMR2, pHandle->TimerCfg->CCMR2_BootCharge);
    LL_TIM_WriteReg(TIMx, CCER, pHandle->TimerCfg->CCER_Align_cfg);
    LL_TIM_GenerateEvent_UPDATE(TIMx);
    LL_TIM_GenerateEvent_COM(TIMx);

    /* Main PWM Output Enable. */
    LL_TIM_EnableAllOutputs(TIMx);

#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  }
#endif
}

/**
  * @brief  This function enables the PWM outputs.
  * @param  pHandle: handler of the current instance of the PWM component.
  * @retval none
  */
__weak void PWMC_SwitchOnPWM(PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;

    pHandle->Step = pHandle->AlignStep;

    /* Select the Capture Compare preload feature. */
    LL_TIM_CC_EnablePreload(TIMx);

    LL_TIM_OC_SetCompareCH1(TIMx, 0u);
    LL_TIM_OC_SetCompareCH2(TIMx, 0u);
    LL_TIM_OC_SetCompareCH3(TIMx, 0u);

    LL_TIM_GenerateEvent_COM(TIMx);

    /* Main PWM Output Enable. */
    TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
    LL_TIM_EnableAllOutputs(TIMx);

    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_OC4REF);
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  }
#endif
}

/**
  * @brief  This function sets the capcture compare of the timer channel
  * used for ADC triggering.
  * @param  pHandle: handler of the current instance of the PWM component.
  * @param  SamplingPoint: trigger point.
  * @retval none
  */
__weak void PWMC_SetADCTriggerChannel(PWMC_Handle_t *pHandle, uint16_t SamplingPoint)
{
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;

    pHandle->ADCTriggerCnt = SamplingPoint;
    LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);
    LL_TIM_OC_SetCompareCH4(TIMx, pHandle->ADCTriggerCnt);
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  }
#endif
}

/**
  * @brief  It disables PWM generation on the proper Timer peripheral acting on
  *         MOE bit and reset the TIM status.
  * @param  pHandle: handler of the current instance of the PWM component.
  * @retval none
  */
__weak void PWMC_SwitchOffPWM(PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;

    /* Main PWM Output Disable. */
    LL_TIM_DisableAllOutputs(TIMx);
    LL_TIM_OC_SetCompareCH1(TIMx, 0u);
    LL_TIM_OC_SetCompareCH2(TIMx, 0u);
    LL_TIM_OC_SetCompareCH3(TIMx, 0u);
    pHandle->CntPh = 0;

    LL_TIM_SetTriggerOutput(pHandle->pParams_str->TIMx, LL_TIM_TRGO_RESET);
    pHandle->TurnOnLowSidesAction = false;  /* ResSet the Flag. */
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  }
#endif
}

/**
* @brief Forces the next step in closed loop operation.
* @param  pHandle Pointer on the target component instance.
* @param  Direction motor spinning direction.
* @param  tStep Step forced when direction is zero.
* @retval none
 */
void PWMC_ForceNextStep(PWMC_Handle_t *pHandle, int16_t Direction, uint8_t tStep)
{
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    if (Direction > 0)
    {
      if (pHandle->Step == 5U)
      {
        pHandle->Step = 0U;
      }
      else
      {
        pHandle->Step++;
      }
    }
    else if (Direction < 0)
    {
      if (pHandle->Step == 0U)
      {
        pHandle->Step = 5U;
      }
      else
      {
        pHandle->Step--;
      }
    }
    else
    {
      pHandle->Step = tStep;
    }
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  }
#endif
}

/*
  * @brief  Checks if an overcurrent occurred since last call.
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @retval uint16_t Returns #MC_OVER_CURR if an overcurrent has been
  *                  detected since last method call, #MC_NO_FAULTS otherwise.
  */
uint16_t PWMC_IsFaultOccurred(PWMC_Handle_t *pHandle)
{
  uint16_t retVal = MC_NO_FAULTS;
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  if (MC_NULL == pHandle)
  {
    retVal = MC_SW_ERROR;
  }
  else
  {
#endif
    if (true == pHandle->OverVoltageFlag)
    {
      retVal = MC_OVER_VOLT;
      pHandle->OverVoltageFlag = false;
    }
    else
    {
      /* Nothing to do. */
    }

    if (true == pHandle->OverCurrentFlag)
    {
      retVal |= MC_OVER_CURR;
      pHandle->OverCurrentFlag = false;
    }
    else
    {
      /* Nothing to do. */
    }

    if (true == pHandle->driverProtectionFlag)
    {
      retVal |= MC_DP_FAULT;
      pHandle->driverProtectionFlag = false;
    }
    else
    {
      /* Nothing to do. */
    }
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  }
#endif
  return (retVal);
}

uint8_t PWMC_GetLSModConfig(const PWMC_Handle_t *pHandle)
{
  uint8_t retVal = 0U;
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    for (uint8_t i = 0U; i < 6U; i++)
    {
      retVal |= pHandle->LSModArray[i] << i;
    }
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  }
#endif
  return (retVal);
}

void PWMC_SetLSModConfig(PWMC_Handle_t *pHandle, uint8_t newConfig)
{
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    uint8_t temp;
    for (int8_t i = 5; i >= 0; i--)
    {
      temp = ((newConfig >> (uint8_t)i) & 0x01U);
      if (1U == temp)
      {
        pHandle->LSModArray[(uint8_t)i] = 1U;
      }
      else
      {
        pHandle->LSModArray[(uint8_t)i] = 0U;
      }
    }
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  }
#endif
}

/**
  * @brief  It enables/disables the Qusi Synch feature at next step change.
  * @param  pHandle: handler of the current instance of the PWM component.
  * @param  uint8_t: 0=disable, 1=enable
  * @retval none
  */
void PWMC_SetQuasiSynchState(PWMC_Handle_t *pHandle, uint8_t State)
{
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    if (1U == State)
    {
      pHandle->QuasiSynchDecay = true;
    }
    else
    {
      pHandle->QuasiSynchDecay = false;
    }
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  }
#endif
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
