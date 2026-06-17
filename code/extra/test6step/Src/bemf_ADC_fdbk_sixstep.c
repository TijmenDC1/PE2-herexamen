
/**
  ******************************************************************************
  * @file    bemf_ADC_fdbk_sixstep.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement Bemf sensing
  *          class to be stantiated when the six-step sensorless driving mode
  *          topology is used.
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
  */

/* Includes ------------------------------------------------------------------*/
#include "bemf_ADC_fdbk_sixstep.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @addtogroup SpeednPosFdbk_Bemf
  * @{
  */

/* Private defines -----------------------------------------------------------*/
#define MAX_PSEUDO_SPEED  ((int16_t)0x7FFF)

#define ADC_CHANNEL_U      MC_ADC_CHANNEL_5
#define ADC_CHANNEL_V      MC_ADC_CHANNEL_6
#define ADC_CHANNEL_W      MC_ADC_CHANNEL_7

/* Private function prototypes -----------------------------------------------*/
void BADC_CalcAvrgElSpeedDpp(Bemf_ADC_Handle_t *pHandle);
void BADC_SelectAdcChannel(uint8_t step);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes ADC and NVIC for three bemf voltages reading.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component.
  */
__weak void BADC_Init(Bemf_ADC_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_BADC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
  if ((uint32_t)pHandle == (uint32_t)&pHandle->_Super) //cstat !MISRAC2012-Rule-11.4
  {

    LL_ADC_DisableIT_AWD1(ADC1);
    LL_ADC_ClearFlag_AWD1(ADC1);

    /* ADC Calibration. */
    LL_ADC_StartCalibration(ADC1);
    while ((SET == LL_ADC_IsCalibrationOnGoing(ADC1)) ||
           (SET == LL_ADC_REG_IsConversionOngoing(ADC1)) ||
           (SET == LL_ADC_REG_IsStopConversionOngoing(ADC1)) ||
           (SET == LL_ADC_IsDisableOngoing(ADC1)))
    {
      /* Wait ADC Calibration. */
    }

    /* Enables the ADC peripheral. */
    LL_ADC_Enable(ADC1);
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, M1_BEMF_SAMPLING_TIME);

    /* Wait ADC Ready. */
    while (RESET == LL_ADC_IsActiveFlag_ADRDY(ADC1))
    {
      /* Wait ADC Ready. */
    }

    uint16_t hMinReliableMecSpeedUnit = pHandle->_Super.hMinReliableMecSpeedUnit;
    uint16_t hMaxReliableMecSpeedUnit = pHandle->_Super.hMaxReliableMecSpeedUnit;

    /* Adjustment factor: minimum measurable speed is x time less than the minimum
    reliable speed. */
    hMinReliableMecSpeedUnit /= 4U;

    /* Adjustment factor: maximum measurable speed is x time greater than the
    maximum reliable speed. */
    hMaxReliableMecSpeedUnit *= 2U;

    /* SW Init. */
    if (0U == hMinReliableMecSpeedUnit)
    {
      pHandle->MaxPeriod = LL_TIM_GetAutoReload(ADC_TIMER_TRIGGER) - 1U;
    }
    else
    {
      pHandle->MaxPeriod = (uint32_t)(pHandle->_Super.speedConvFactor / ((uint32_t)hMinReliableMecSpeedUnit));
    }

    if (0U == hMaxReliableMecSpeedUnit)
    {
      pHandle->MinPeriod = LL_TIM_GetAutoReload(ADC_TIMER_TRIGGER) - 1U;
    }
    else
    {
      pHandle->MinPeriod = (uint32_t)(pHandle->_Super.speedConvFactor / ((uint32_t)hMaxReliableMecSpeedUnit));
    }

    pHandle->SatSpeed = hMaxReliableMecSpeedUnit;
    pHandle->pSensing_Point = &(pHandle->Pwm_H_L.SamplingPointOff);
    pHandle->IsOnSensingEnabled = false;
    pHandle->ZcEvents = 0U;

    LL_TIM_EnableCounter(ADC_TIMER_TRIGGER);
  }
#ifdef NULL_PTR_CHECK_BADC
  }
#endif
}

/**
  * @brief  Resets the parameter values of the component.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component.
  */
__weak void BADC_Clear(Bemf_ADC_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_BADC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
  uint8_t bSpeedBufferSize;
  uint8_t bIndex;
  pHandle->ZcEvents = 0U;

  pHandle->BufferFilled = 0U;

  /* Initialize speed buffer index. */
  pHandle->SpeedFIFOIdx = 0U;

  /* Clear speed error counter. */
  pHandle->_Super.bSpeedErrorNumber = 0U;
  pHandle->IsLoopClosed = false;
  pHandle->RequestLoopClosed=false;
  pHandle->SpeedTimerState = LFTIM_IDLE;
  pHandle->StepTime_Last = 0U;
  pHandle->ZCDetectionErrors = 0U;

  /* Erase speed buffer */
  bSpeedBufferSize = pHandle->SpeedBufferSize;
  for (bIndex = 0u; bIndex < bSpeedBufferSize; bIndex++)
  {
    pHandle->SpeedBufferDpp[bIndex]  = (int32_t)pHandle->MaxPeriod * pHandle->Direction;
  }

  pHandle->ElPeriodSum = (int32_t)pHandle->MaxPeriod * (int32_t)pHandle->SpeedBufferSize * pHandle->Direction;
  LL_TIM_EnableIT_CC1(ADC_TIMER_TRIGGER);
#ifdef NULL_PTR_CHECK_BADC
  }
#endif
}

/**
 * @brief  Starts bemf ADC conversion of the phase depending on current step.
 * @param  pHandle: handler of the current instance of the Bemf_ADC component.
 * @param  step: current step of the six-step sequence.
 * @param  LSModArray : Low Side modulation status for each step
 *         0 : no modulation on Low Side
 *         1 : modulation on Low Side
 */
__weak void BADC_Start(const Bemf_ADC_Handle_t *pHandle, uint8_t step, const uint8_t LSModArray[6])
{
#ifdef NULL_PTR_CHECK_BADC
  if ((MC_NULL == pHandle) || (MC_NULL == LSModArray))
  {
    /* Nothing to do. */
  }
  else
  {
#endif
  uint16_t Bemf_Threshold;

  while (LL_ADC_REG_IsConversionOngoing(ADC1))
  {
    LL_ADC_REG_StopConversion(ADC1);
    while (LL_ADC_REG_IsStopConversionOngoing(ADC1));
  }

/* Enable ADC source trigge.r */
  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM1_TRGO2);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, M1_BEMF_SAMPLING_TIME);
  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_FALLING);

  if (1U == LSModArray[step])
  {
    Bemf_Threshold = pHandle->pSensing_Threshold_LSMod;
  }
  else
  {
    Bemf_Threshold = pHandle->pSensing_Threshold_HSMod;
  }

  if (0U == (step & 0x1U))
  {
    /* case STEP_1:
       case STEP_3:
       case STEP_5: */
    if (1 == pHandle->Direction)
    {
      LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD1, LL_ADC_AWD_THRESHOLD_HIGH, 0xFFFU);
      LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD1, LL_ADC_AWD_THRESHOLD_LOW, ((uint32_t)Bemf_Threshold >> 4U));
    }
    else
    {
      LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD1, LL_ADC_AWD_THRESHOLD_LOW, 0U);
      LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD1, LL_ADC_AWD_THRESHOLD_HIGH, ((uint32_t)Bemf_Threshold >> 4U));
    }
  }
  else
  {
    /* case STEP_2:
       case STEP_4:
       case STEP_6: */
    if (1 == pHandle->Direction)
    {
      LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD1, LL_ADC_AWD_THRESHOLD_LOW, 0U);
      LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD1, LL_ADC_AWD_THRESHOLD_HIGH, ((uint32_t)Bemf_Threshold >> 4U));
    }
    else
    {
      LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD1, LL_ADC_AWD_THRESHOLD_HIGH, 0xFFFU);
      LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD1, LL_ADC_AWD_THRESHOLD_LOW, ((uint32_t)Bemf_Threshold >> 4U));
    }
  }

  BADC_SelectAdcChannel(step);

  /* Start injected conversion. */
  LL_ADC_ClearFlag_AWD1(ADC1);
  LL_ADC_EnableIT_AWD1(ADC1);
  LL_ADC_REG_StartConversion(ADC1);
#ifdef NULL_PTR_CHECK_BADC
  }
#endif
}

/**
 * @brief  Stops bemf ADC conversion
 */
__weak void BADC_Stop(void)
{
  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
  LL_ADC_DisableIT_EOC(ADC1);
  while (LL_ADC_REG_IsConversionOngoing(ADC1))
  {
    LL_ADC_REG_StopConversion(ADC1);
    while(LL_ADC_REG_IsStopConversionOngoing(ADC1));
  }

  /* Disable AWD */
  LL_ADC_DisableIT_AWD1(ADC1);

  /* Clear AWD */
  LL_ADC_ClearFlag_AWD1(ADC1);
}

/**
  * @brief  Configures the ADC for the current sampling.
  *         It sets the sampling point via TIM1_Ch4 value, the ADC sequence
  *         and channels.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component.
  * @param  pHandlePWMC: handler of the current instance of the PWMC component.
  * @param  BusVHandle: handler of the current instance of the Speed Control component.
  */
__weak void BADC_SetSamplingPoint(Bemf_ADC_Handle_t *pHandle, const PWMC_Handle_t *pHandlePWMC, BusVoltageSensor_Handle_t *BusVHandle)
{
#ifdef NULL_PTR_CHECK_BADC
  if ((MC_NULL == pHandle) || (MC_NULL == pHandlePWMC))
  {
    /* Nothing to do. */
  }
  else
  {
#endif
  uint16_t latest_busConv = VBS_GetAvBusVoltage_d(BusVHandle);
  if (VM == pHandle->DriveMode)
  {
      if (pHandlePWMC->CntPh > pHandle->OnSensingEnThres)
      {
        uint16_t Threshold_Pwm = (uint16_t)(pHandle->Pwm_H_L.AdcThresholdPwmPerc * latest_busConv/pHandle->Pwm_H_L.Bus2ThresholdConvFactor)
                               + pHandle->Pwm_H_L.ThresholdCorrectFactor;
        pHandle->IsOnSensingEnabled=true;
        pHandle->pSensing_Point = &(pHandle->Pwm_H_L.SamplingPointOn);
        pHandle->pSensing_Threshold_HSMod = Threshold_Pwm;
        pHandle->pSensing_Threshold_LSMod = Threshold_Pwm;
      }
      else if (pHandlePWMC->CntPh < pHandle->OnSensingDisThres)
      {
        pHandle->IsOnSensingEnabled=false;
        pHandle->pSensing_Point = &(pHandle->Pwm_H_L.SamplingPointOff);
        pHandle->pSensing_Threshold_HSMod = (uint16_t)(pHandle->Pwm_H_L.AdcThresholdLowPerc * latest_busConv/pHandle->Pwm_H_L.Bus2ThresholdConvFactor);
        pHandle->pSensing_Threshold_LSMod = (uint16_t)(pHandle->Pwm_H_L.AdcThresholdHighPerc * latest_busConv/pHandle->Pwm_H_L.Bus2ThresholdConvFactor)
                                          + pHandle->Pwm_H_L.ThresholdCorrectFactor;
      }
      else if (false == pHandle->IsOnSensingEnabled)
      {
        pHandle->pSensing_Threshold_HSMod = (uint16_t)(pHandle->Pwm_H_L.AdcThresholdLowPerc * latest_busConv/pHandle->Pwm_H_L.Bus2ThresholdConvFactor);
        pHandle->pSensing_Threshold_LSMod = (uint16_t)(pHandle->Pwm_H_L.AdcThresholdHighPerc * latest_busConv/pHandle->Pwm_H_L.Bus2ThresholdConvFactor)
                                          + pHandle->Pwm_H_L.ThresholdCorrectFactor;
      }
      else
      {
        uint16_t Threshold_Pwm = (uint16_t)(pHandle->Pwm_H_L.AdcThresholdPwmPerc * latest_busConv/pHandle->Pwm_H_L.Bus2ThresholdConvFactor)
                               + pHandle->Pwm_H_L.ThresholdCorrectFactor;
        pHandle->pSensing_Threshold_HSMod = Threshold_Pwm;
        pHandle->pSensing_Threshold_LSMod = Threshold_Pwm;
      }
  }
  else
  {
    pHandle->IsOnSensingEnabled=false;
    pHandle->pSensing_Point = &(pHandle->Pwm_H_L.SamplingPointOff);
    pHandle->pSensing_Threshold_HSMod = (uint16_t)(pHandle->Pwm_H_L.AdcThresholdLowPerc * latest_busConv/pHandle->Pwm_H_L.Bus2ThresholdConvFactor);
    pHandle->pSensing_Threshold_LSMod = (uint16_t)(pHandle->Pwm_H_L.AdcThresholdHighPerc * latest_busConv/pHandle->Pwm_H_L.Bus2ThresholdConvFactor)
                                        + pHandle->Pwm_H_L.ThresholdCorrectFactor;
  }
#ifdef NULL_PTR_CHECK_BADC
  }
#endif
}

/**
 * @brief  Gets last bemf value and checks for zero crossing detection.
 *         It updates speed loop timer and electrical angle accordingly.
 * @param  pHandle: handler of the current instance of the Bemf_ADC component.
 * @param  step: Current step of the 6-step sequence.
 * @retval none.
 */
__weak void BADC_IsZcDetected(Bemf_ADC_Handle_t *pHandle, uint8_t step)
{
#ifdef NULL_PTR_CHECK_BADC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    uint32_t TimerSpeed_Counter;
    uint32_t CounterAutoreload;
    uint32_t CC_Counter = 0U;  /* for switch case default before CounterAutoreload comparison. */
    uint32_t tStepTime;
    uint32_t wCaptBuf;

    if (LFTIM_COMMUTATION == pHandle->SpeedTimerState)
    {
      LL_ADC_DisableIT_AWD1(ADC1);
      TimerSpeed_Counter = LL_TIM_GetCounter(ADC_TIMER_TRIGGER);
      CounterAutoreload = LL_TIM_GetAutoReload(ADC_TIMER_TRIGGER) + 1U;
      if (TimerSpeed_Counter < pHandle->Last_TimerSpeed_Counter)
      {
        tStepTime = CounterAutoreload - (pHandle->Last_TimerSpeed_Counter - TimerSpeed_Counter);
      }
      else
      {
        tStepTime = TimerSpeed_Counter - pHandle->Last_TimerSpeed_Counter;
      }
      pHandle->ZcEvents++;

      if (0U == (step & 0x1U))
      {
        /* case STEP_1:
           case STEP_3:
           case STEP_5: */
         if (1 == pHandle->Direction)
        {
          pHandle->StepTime_Down =  tStepTime;
          if (2U == pHandle->ComputationDelay)
          {
            tStepTime = pHandle->StepTime_Last;
          }
          else
          {
            /* Nothing to do. */
          }
          CC_Counter = ((uint32_t)((pHandle->ZcFalling2CommDelay) * tStepTime)) >> 9;
          pHandle->StepTime_Last = pHandle->StepTime_Down;
        }
        else
        {
          pHandle->StepTime_Up =  tStepTime;
          if (2U == pHandle->ComputationDelay)
          {
            tStepTime = pHandle->StepTime_Last;
          }
          else
          {
            /* Nothing to do. */
          }
          CC_Counter = ((uint32_t)((pHandle->ZcRising2CommDelay) * tStepTime)) >> 9;
          pHandle->StepTime_Last = pHandle->StepTime_Up;
        }
      }
      else
      {
        /* case STEP_2:
           case STEP_4:
           case STEP_6: */
        if (1 == pHandle->Direction)
       {
           pHandle->StepTime_Up =  tStepTime;
          if (2U == pHandle->ComputationDelay)
          {
            tStepTime = pHandle->StepTime_Last;
          }
          else
          {
            /* Nothing to do. */
          }
          CC_Counter = ((uint32_t)((pHandle->ZcRising2CommDelay) * tStepTime)) >> 9;
          pHandle->StepTime_Last = pHandle->StepTime_Up;
        }
        else
        {
          pHandle->StepTime_Down =  tStepTime;
          if (2U == pHandle->ComputationDelay)
          {
            tStepTime = pHandle->StepTime_Last;
          }
          else
          {
            /* Nothing to do. */
          }
          CC_Counter = ((uint32_t)((pHandle->ZcFalling2CommDelay) * tStepTime)) >> 9;
          pHandle->StepTime_Last = pHandle->StepTime_Down;
        }
      }

      if (true == pHandle->IsLoopClosed)
      {
        BADC_SetSpeedTimer(pHandle, CC_Counter);
      }
      else
      {
        /* Nothing to do. */
      }
      wCaptBuf = pHandle->StepTime_Last;
      if (wCaptBuf < pHandle->MinPeriod)
      {
        /* Nothing to do */
      }
      else
      {
        pHandle->SpeedFIFOIdx++;
        if (pHandle->SpeedFIFOIdx == pHandle->SpeedBufferSize)
        {
          pHandle->SpeedFIFOIdx = 0U;
        }
        else
        {
          /* Nothing to do */
        }
        pHandle->ElPeriodSum -= pHandle->SpeedBufferDpp[pHandle->SpeedFIFOIdx]; /* value we gonna removed from the accumulator. */
        if (wCaptBuf >= pHandle->MaxPeriod)
        {
          pHandle->SpeedBufferDpp[pHandle->SpeedFIFOIdx] = (int32_t)pHandle->MaxPeriod * pHandle->Direction;
        }
        else
        {
          pHandle->SpeedBufferDpp[pHandle->SpeedFIFOIdx] = (int32_t)wCaptBuf ;
          pHandle->SpeedBufferDpp[pHandle->SpeedFIFOIdx] *= pHandle->Direction;
        }
        pHandle->ElPeriodSum += pHandle->SpeedBufferDpp[pHandle->SpeedFIFOIdx];
        /* Update pointers to speed buffer. */
      }
      /* Used to validate the average speed measurement. */
      if (pHandle->BufferFilled < pHandle->SpeedBufferSize)
      {
        pHandle->BufferFilled++;
      }
      else
      {
        /* Nothing to do. */
      }
      pHandle->Last_TimerSpeed_Counter = TimerSpeed_Counter;
      BADC_Stop();
    }
#ifdef NULL_PTR_CHECK_BADC
  }
#endif
}

/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and returns - through
  *         parameter hMecSpeedUnit - the rotor average mechanical speed,
  *         expressed in Unit. Average is computed considering a FIFO depth
  *         equal to SpeedBufferSizeUnit. Moreover it also computes and returns
  *         the reliability state of the sensor.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component.
  * @retval bool speed sensor reliability, measured with reference to parameters
  *         bMaximumSpeedErrorsNumber, VariancePercentage and SpeedBufferSize.
  *         true = sensor information is reliable.
  *         false = sensor information is not reliable.
  */
__weak bool BADC_CalcAvrgMecSpeedUnit(Bemf_ADC_Handle_t *pHandle)
{
  bool bReliability = true;
#ifdef NULL_PTR_CHECK_HALL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    bReliability = false;
  }
  else
  {
#endif

    uint32_t wCaptBuf;
    int16_t MecSpeedUnit = pHandle->_Super.hAvrMecSpeedUnit;

    if (pHandle->SpeedBufferDpp[pHandle->SpeedFIFOIdx] < 0)
    {
      wCaptBuf = (uint32_t) (- pHandle->SpeedBufferDpp[pHandle->SpeedFIFOIdx]);
    }
    else
    {
      wCaptBuf = (uint32_t) (pHandle->SpeedBufferDpp[pHandle->SpeedFIFOIdx]);
    }

    /* Filtering to0 fast speed... could be a glitch? */
    /* The MAX_PSEUDO_SPEED is temporary in the buffer, and never included in average computation. */
    if (wCaptBuf < pHandle->MinPeriod)
    {
      if (pHandle->BufferFilled < pHandle->SpeedBufferSize)
      {
        MecSpeedUnit = 0;
      }

    }
    else
    {
      if ((pHandle->BufferFilled < pHandle->SpeedBufferSize) || (false == pHandle->IsLoopClosed))
      {
        MecSpeedUnit = (int16_t)(((int32_t)pHandle->_Super.speedConvFactor) / pHandle->SpeedBufferDpp[pHandle->SpeedFIFOIdx]);
      }
      else
      {
        /* Average speed allow to smooth the mechanical sensors misalignement. */
        MecSpeedUnit = (int16_t)((int32_t)pHandle->_Super.speedConvFactor /
                       (pHandle->ElPeriodSum / (int32_t)pHandle->SpeedBufferSize)); /* Average value. */
      }
    }

    if (true == pHandle->IsLoopClosed)
    {
      bReliability = SPD_IsMecSpeedReliable(&pHandle->_Super, MecSpeedUnit);
    }
    else
    {
      /* Nothing to do. */
    }
    pHandle->_Super.hAvrMecSpeedUnit = MecSpeedUnit;

#ifdef NULL_PTR_CHECK_BADC
  }
#endif
  return (bReliability);
}

/**
  * @brief  Forces the rotation direction.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component.
  * @param  direction: imposed direction.
  */
__weak void BADC_SetDirection(Bemf_ADC_Handle_t *pHandle, int8_t direction)
{
#ifdef NULL_PTR_CHECK_BADC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    pHandle->Direction = direction;
#ifdef NULL_PTR_CHECK_BADC
  }

#endif
}

/**
  * @brief  Configures the proper ADC channel according to the current
  *         step corresponding to the floating phase. To be periodically called
  *         at least at every step change.
  * @param  step: current step of the six-step sequence.
  */
void BADC_SelectAdcChannel(uint8_t step)
{
  uint32_t pADCbemfChannel[6] = {ADC_CHANNEL_W, ADC_CHANNEL_V, ADC_CHANNEL_U,
                                 ADC_CHANNEL_W, ADC_CHANNEL_V, ADC_CHANNEL_U};

  /* Regular sequence configuration */
  LL_ADC_REG_SetSequencerChannels(ADC1, __LL_ADC_DECIMAL_NB_TO_CHANNEL(pADCbemfChannel[step]));
  LL_ADC_SetAnalogWDMonitChannels(ADC1, LL_ADC_AWD1, __LL_ADC_ANALOGWD_CHANNEL_GROUP(__LL_ADC_DECIMAL_NB_TO_CHANNEL(pADCbemfChannel[step]),LL_ADC_GROUP_REGULAR));
}

/**
  * @brief  Used to calculate instant speed during revup and to
  *         initialize parameters at step change.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component.
  */
void BADC_StepChangeEvent(Bemf_ADC_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_BADC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
  uint32_t tempReg;
  if (false == pHandle->IsLoopClosed)
  {
    tempReg = LL_TIM_GetCounter(ADC_TIMER_TRIGGER) + pHandle->DemagCounterThreshold;
  }
  else
  {
    tempReg = LL_TIM_OC_GetCompareCH1(ADC_TIMER_TRIGGER) + pHandle->DemagCounterThreshold;
  }
  if (true == pHandle->RequestLoopClosed)
  {
    pHandle->IsLoopClosed = true;
  }
  else
  {
    /* Nothing to do */
  }
  uint32_t CounterAutoreload = LL_TIM_GetAutoReload(ADC_TIMER_TRIGGER) + 1U;

  /* Stop Regular conversion or Injected Conversion in case of miss of BEMF Zero crossing event. */
  BADC_Stop();

  pHandle->SpeedTimerState = LFTIM_DEMAGNETIZATION;
  if (tempReg >= CounterAutoreload)
  {
    tempReg -= CounterAutoreload;
  }
  else
  {
    /* Nothing to do. */
  }
  LL_TIM_OC_SetCompareCH1(ADC_TIMER_TRIGGER, tempReg);
#ifdef NULL_PTR_CHECK_BADC
  }
#endif
}

/**
  * @brief  Calculates and stores in the corresponding variable the demagnetization
  *         time in open loop operation.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component.
  * @param  RevUpSpeed: current revup speed.
  */
void BADC_CalcRevUpDemagTime(Bemf_ADC_Handle_t *pHandle, int16_t RevUpSpeed)
{
#ifdef NULL_PTR_CHECK_BADC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
  int16_t tempRevUpSpeed = RevUpSpeed;
  if (tempRevUpSpeed < 0)
  {
    tempRevUpSpeed = - tempRevUpSpeed;
  }
  else
  {
    /* Nothing to do. */
  }
  if (0 == tempRevUpSpeed)
  {
    pHandle->DemagCounterThreshold = pHandle->DemagParams.DemagMinimumThreshold;
  }
  else
  {
    pHandle->DemagCounterThreshold = (pHandle->DemagParams.RevUpDemagSpeedConv / (uint32_t)tempRevUpSpeed);
  }

  if (pHandle->DemagCounterThreshold < pHandle->DemagParams.DemagMinimumThreshold)
  {
    pHandle->DemagCounterThreshold = pHandle->DemagParams.DemagMinimumThreshold;
  }
  else
  {
    /* Nothing to do. */
  }
#ifdef NULL_PTR_CHECK_BADC
  }
#endif
}

/**
  * @brief  Calculates and stores in the corresponding variable the demagnetization
  *         time in closed loop operation.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component.
  */
void BADC_CalcRunDemagTime(Bemf_ADC_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_BADC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
  int16_t hSpeed;
  hSpeed = pHandle->_Super.hAvrMecSpeedUnit;
  if (hSpeed < 0)
  {
    hSpeed = - hSpeed;
  }
  else
  {
    /* Nothing to do. */
  }

  if (hSpeed < (int16_t)pHandle->DemagParams.DemagMinimumSpeedUnit)
  {
    pHandle->DemagCounterThreshold = (pHandle->DemagParams.RunDemagSpeedConv / (uint32_t)hSpeed);
    if (pHandle->DemagCounterThreshold < pHandle->DemagParams.DemagMinimumThreshold)
    {
      pHandle->DemagCounterThreshold = pHandle->DemagParams.DemagMinimumThreshold;
    }
    else
    {
      /* Nothing to do. */
    }
  }
  else
  {
    pHandle->DemagCounterThreshold = pHandle->DemagParams.DemagMinimumThreshold;
  }
#ifdef NULL_PTR_CHECK_BADC
  }
#endif
}

/**
  * @brief  Must be called after switch-over procedure when
  *         virtual speed sensor transition is ended.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  */
void BADC_SetLoopClosed(Bemf_ADC_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_BADC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
  pHandle->RequestLoopClosed=true;
#ifdef NULL_PTR_CHECK_BADC
  }
#endif
}

/**
  * @brief  It is called tp set next speed timer interrupt
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  SpeedTimerCounter: delay in digits to schedule next speed timer interrupt
  */
void BADC_SetSpeedTimer(const Bemf_ADC_Handle_t *pHandle, uint32_t SpeedTimerCounter)
{
#ifdef NULL_PTR_CHECK_BADC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
  uint32_t CC_Counter;
  uint32_t CounterAutoreload = LL_TIM_GetAutoReload(ADC_TIMER_TRIGGER) + 1U;

  CC_Counter = LL_TIM_GetCounter(ADC_TIMER_TRIGGER);
  CC_Counter += SpeedTimerCounter;
  if (CC_Counter >= CounterAutoreload)
  {
    CC_Counter -= CounterAutoreload;
  }
  else
  {
    /* Nothing to do */
  }
  LL_TIM_OC_SetCompareCH1(ADC_TIMER_TRIGGER, CC_Counter);
#ifdef NULL_PTR_CHECK_BADC
  }
#endif
}

/**
  * @brief  It checks whether OC counter has been changed after BEMF detection and eventually increase
  * @brief  an error counter
  * @param  pHandle: handler of the current instance of the Bemf_ADC component.
  * @retval bool error counter is higher than maximum.
  */
bool BADC_CheckDetectionErrors(Bemf_ADC_Handle_t *pHandle)
{
  bool retVal = false;
  uint32_t TimerCC =   LL_TIM_OC_GetCompareCH1(ADC_TIMER_TRIGGER);
  if (TimerCC == pHandle->LastOCCounter)
  {
    pHandle->ZCDetectionErrors = pHandle->ZCDetectionErrors + 10;
  }
  else
  {
    if (pHandle->ZCDetectionErrors > 0)
    {
      pHandle->ZCDetectionErrors--;
    }
    else
    {
      /* Nothing to do */
    }
    pHandle->LastOCCounter = TimerCC;
  }
  if ( pHandle->ZCDetectionErrors >= pHandle->MaxZCDetectionErrors)
  {
    retVal = true;
  }
  else
  {
    /* Nothing to do */
  }

  return retVal;
}

/**
  * @brief  Configures the parameters for bemf sensing during pwm off-time.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component.
  * @param  BemfAdcConfig: thresholds and sampling time parameters.
  * @param  bemfAdcDemagConfig: demagnetization parameters.
  * @param  BemfRegIntParam: transfer parameters from/to motor Pilot.
  */
  void BADC_SetBemfSensorlessParam(Bemf_ADC_Handle_t *pHandle, Bemf_Sensing_Params *BemfAdcConfig,
                                   Bemf_Demag_Params *BemfAdcDemagConfig, Bemf_RegInterface_Param *BemfRegIntParam)
{
#ifdef NULL_PTR_CHECK_BADC
  if ((MC_NULL == pHandle) || (MC_NULL == BemfAdcConfig) || (MC_NULL == BemfAdcDemagConfig) || (MC_NULL == BemfRegIntParam))
  {
    /* Nothing to do. */
  }
  else
  {
#endif
  pHandle->Pwm_H_L.AdcThresholdPwmPerc = BemfAdcConfig->AdcThresholdPwmPerc;
  pHandle->Pwm_H_L.AdcThresholdHighPerc = BemfAdcConfig->AdcThresholdHighPerc;
  pHandle->Pwm_H_L.AdcThresholdLowPerc = BemfAdcConfig->AdcThresholdLowPerc;
  pHandle->Pwm_H_L.SamplingPointOff = BemfAdcConfig->SamplingPointOff;
  pHandle->Pwm_H_L.SamplingPointOn = BemfAdcConfig->SamplingPointOn;
  pHandle->ZcRising2CommDelay = BemfRegIntParam->ZcRising2CommDelay;
  pHandle->ZcFalling2CommDelay = BemfRegIntParam->ZcFalling2CommDelay;
  pHandle->DemagParams.DemagMinimumSpeedUnit = BemfAdcDemagConfig->DemagMinimumSpeedUnit;
  pHandle->DemagParams.DemagMinimumThreshold = BemfAdcDemagConfig->DemagMinimumThreshold;
  pHandle->OnSensingEnThres = BemfRegIntParam->OnSensingEnThres;
  pHandle->OnSensingDisThres = BemfRegIntParam->OnSensingDisThres;
  pHandle->ComputationDelay = (uint8_t)(BemfRegIntParam->ComputationDelay);
#ifdef NULL_PTR_CHECK_BADC
  }
#endif
}

/**
  * @brief  Gets the parameters for bemf sensing during pwm off-time.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component.
  * @param  BemfAdcConfig: thresholds and sampling time parameters.
  * @param  BemfAdcDemagConfig: demagnetization parameters.
  * @param  BemfRegIntParam: transfer parameters from/to motor Pilot.
  */
void BADC_GetBemfSensorlessParam(Bemf_ADC_Handle_t *pHandle, Bemf_Sensing_Params *BemfAdcConfig,
                                 Bemf_Demag_Params *BemfAdcDemagConfig, Bemf_RegInterface_Param *BemfRegIntParam)
{
#ifdef NULL_PTR_CHECK_BADC
  if ((MC_NULL == pHandle) || (MC_NULL == BemfAdcConfig) || (MC_NULL == BemfAdcDemagConfig) || (MC_NULL == BemfRegIntParam))
  {
    /* Nothing to do. */
  }
  else
  {
#endif
  BemfAdcConfig->AdcThresholdPwmPerc =   pHandle->Pwm_H_L.AdcThresholdPwmPerc;
  BemfAdcConfig->AdcThresholdHighPerc = pHandle->Pwm_H_L.AdcThresholdHighPerc;
  BemfAdcConfig->AdcThresholdLowPerc = pHandle->Pwm_H_L.AdcThresholdLowPerc;
  BemfAdcConfig->SamplingPointOff = pHandle->Pwm_H_L.SamplingPointOff;
  BemfAdcConfig->SamplingPointOn = pHandle->Pwm_H_L.SamplingPointOn;
  BemfRegIntParam->ZcRising2CommDelay = pHandle->ZcRising2CommDelay;
  BemfRegIntParam->ZcFalling2CommDelay = pHandle->ZcFalling2CommDelay;
  BemfAdcDemagConfig->DemagMinimumSpeedUnit = pHandle->DemagParams.DemagMinimumSpeedUnit;
  BemfAdcDemagConfig->DemagMinimumThreshold = pHandle->DemagParams.DemagMinimumThreshold;
  BemfRegIntParam->OnSensingEnThres = pHandle->OnSensingEnThres;
  BemfRegIntParam->OnSensingDisThres = pHandle->OnSensingDisThres;
  BemfRegIntParam->ComputationDelay = (uint16_t) pHandle->ComputationDelay;
#ifdef NULL_PTR_CHECK_BADC
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

/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/

