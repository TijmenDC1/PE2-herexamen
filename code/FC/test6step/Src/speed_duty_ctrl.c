/*
  ******************************************************************************
  * @file    speed_duty_ctrl.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the Speed Control component of the Motor Control SDK.
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
#include "speed_duty_ctrl.h"
#include "mc_type.h"

/* Global Definitions --------------------------------------------------------*/
#define CHECK_BOUNDARY

/* Local Functions -----------------------------------------------------------*/

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup SpeednDutyCtrl Speed Control
  * @brief Speed Control component of the Motor Control SDK
  *
  * @todo Document the Speed Control "module".
  *
  * @{
  */

/**
  * @brief  It should be called before each motor restart. If SDC is set in
            speed mode, this method resets the integral term of speed regulator.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  * @retval none.
  */
__weak void SDC_Clear(SpeednDutyCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SDC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif

    if (MCM_SPEED_MODE == pHandle->Mode)
    {
      PID_SetIntegralTerm(pHandle->PISpeed, 0);
    }
    else
    {
      /* Nothing to do. */
    }
    pHandle->DutyCycleRef = ((uint32_t)pHandle->DutyCycleRefDefault) * 65536U;
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
}

/**
  * @brief  Starts the execution of a ramp using new target and duration. This
  *         command interrupts the execution of any previous ramp command.
  *         The generated ramp will be in the modality previously set by
  *         SDC_SetControlMode method.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  * @param  hTargetFinal final value of command. This is different accordingly
  *         the STC modality.
  *         hTargetFinal is the value of mechanical rotor speed reference at the end
  *         of the ramp.Expressed in the unit defined by SPEED_UNIT
  * @param  hDurationms the duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the value.
  * @retval bool It return false if the absolute value of hTargetFinal is out of
  *         the boundary of the application (Above max application speed or below min
  *         application speed in this case the command is ignored and the
  *         previous ramp is not interrupted, otherwise it returns true.
  */
__weak bool SDC_ExecRamp(SpeednDutyCtrl_Handle_t *pHandle, int16_t hTargetFinal, uint32_t hDurationms)
{
  bool allowedRange = true;

#ifdef NULL_PTR_CHECK_SDC
  if (MC_NULL == pHandle)
  {
    allowedRange = false;
  }
  else
  {
#endif
    uint32_t wAux;
    int32_t wAux1;
    int16_t hCurrentReference;

    /* Check if the hTargetFinal is out of the bound of application. */
    if (MCM_DUTY_MODE == pHandle->Mode)
    {
      hCurrentReference = (int16_t)SDC_GetDutyCycleRef(pHandle);
#ifdef CHECK_BOUNDARY
      if ((int32_t)hTargetFinal > (int32_t)pHandle->MaxPositiveDutyCycle)
      {
        allowedRange = false;
      }
      else
      {
        /* Nothing to do. */
      }
#endif
    }
    else
    {
#ifdef NO_FULL_MISRA_C_COMPLIANCY_SPD_DUTY_CTRL
      //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      hCurrentReference = (int16_t)(pHandle->SpeedRefUnitExt >> 16);
#else
      hCurrentReference = (int16_t)(pHandle->SpeedRefUnitExt / 65536);
#endif

#ifdef CHECK_BOUNDARY
      if ((int32_t)hTargetFinal > (int32_t)pHandle->MaxAppPositiveMecSpeedUnit)
      {
        allowedRange = false;
      }
      else if (hTargetFinal < pHandle->MinAppNegativeMecSpeedUnit)
      {
        allowedRange = false;
      }
      else if ((int32_t)hTargetFinal < (int32_t)pHandle->MinAppPositiveMecSpeedUnit)
      {
        if (hTargetFinal > pHandle->MaxAppNegativeMecSpeedUnit)
        {
          allowedRange = false;
        }
      }
      else
      {
        /* Nothing to do */
      }
#endif
    }

    if (true == allowedRange)
    {
      /* Interrupts the execution of any previous ramp command */
      if (0U == hDurationms)
      {
        if (MCM_SPEED_MODE == pHandle->Mode)
        {
          pHandle->SpeedRefUnitExt = ((int32_t)hTargetFinal) * 65536;
        }
        else
        {
          pHandle->DutyCycleRef = ((uint32_t)hTargetFinal) * 65536U;
        }
        pHandle->RampRemainingStep = 0U;
        pHandle->IncDecAmount = 0;
      }
      else
      {
        /* Store the hTargetFinal to be applied in the last step */
        pHandle->TargetFinal = hTargetFinal;

        /* Compute the (wRampRemainingStep) number of steps remaining to complete
        the ramp. */
        wAux = ((uint32_t)hDurationms) * ((uint32_t)pHandle->SDCFrequencyHz);
        wAux /= 1000U;
        pHandle->RampRemainingStep = wAux;
        pHandle->RampRemainingStep++;

        /* Compute the increment/decrement amount (wIncDecAmount) to be applied to
        the reference value at each CalcSpeedReference. */
        wAux1 = (((int32_t)hTargetFinal) - ((int32_t)hCurrentReference)) * 65536;
        wAux1 /= ((int32_t)pHandle->RampRemainingStep);
        pHandle->IncDecAmount = wAux1;
      }
    }
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
  return (allowedRange);
}

/**
  * @brief  It is used to compute the new value of motor speed reference. It
  *         must be called at fixed time equal to hSDCFrequencyHz. It is called
  *         passing as parameter the speed sensor used to perform the speed
  *         regulation.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  * @retval int16_t motor dutycycle reference. This value represents actually the
  *         dutycycle expressed in digit.
  */
__weak uint16_t SDC_CalcSpeedReference(SpeednDutyCtrl_Handle_t *pHandle)
{
  uint16_t hDutyCycleReference = 0U;

#ifdef NULL_PTR_CHECK_SDC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    int32_t wCurrentReference;
    int16_t hMeasuredSpeed;
    int16_t hTargetSpeed;
    int16_t hError;

    if (MCM_DUTY_MODE == pHandle->Mode)
    {
      wCurrentReference = (int32_t)pHandle->DutyCycleRef;
    }
    else
    {
      wCurrentReference = pHandle->SpeedRefUnitExt;
    }

    /* Update the speed reference or the torque reference according to the mode
       and terminates the ramp if needed. */
    if (pHandle->RampRemainingStep > 1U)
    {
      /* Increment/decrement the reference value. */
      wCurrentReference += pHandle->IncDecAmount;

      /* Decrement the number of remaining steps */
      pHandle->RampRemainingStep--;
    }
    else if (1U == pHandle->RampRemainingStep)
    {
      /* Set the backup value of hTargetFinal. */
      wCurrentReference = ((int32_t)pHandle->TargetFinal) * 65536;
      pHandle->RampRemainingStep = 0U;
    }
    else
    {
      /* Do nothing. */
    }

    if (MCM_SPEED_MODE == pHandle->Mode)
    {
      /* Run the speed control loop */

      /* Compute speed error */
#ifdef NO_FULL_MISRA_C_COMPLIANCY_SPD_DUTY_CTRL
      //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      hTargetSpeed = (int16_t)(wCurrentReference >> 16);
#else
      hTargetSpeed = (int16_t)(wCurrentReference / 65536);
#endif
      hMeasuredSpeed = SPD_GetAvrgMecSpeedUnit(pHandle->SPD);
      if (hTargetSpeed < 0)
      {
        hError = hMeasuredSpeed - hTargetSpeed;
      }
      else
      {
        hError = hTargetSpeed - hMeasuredSpeed;
      }
      hDutyCycleReference = (uint16_t)PI_Controller(pHandle->PISpeed, (int32_t)hError);

      pHandle->SpeedRefUnitExt = wCurrentReference;
      pHandle->DutyCycleRef = ((uint32_t)hDutyCycleReference) * 65536U;
    }
    else
    {
      pHandle->DutyCycleRef = (uint32_t)wCurrentReference;
#ifdef NO_FULL_MISRA_C_COMPLIANCY_SPD_DUTY_CTRL
      //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      hDutyCycleReference = (uint16_t)((int16_t)(wCurrentReference >> 16));
#else
      hDutyCycleReference = (uint16_t)((int16_t)(wCurrentReference / 65536));
#endif
    }
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
  return (hDutyCycleReference);
}

/**
  * @brief  Stop the execution of speed ramp.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  * @retval bool It returns true if the command is executed, false otherwise.
  */
__weak bool SDC_StopSpeedRamp(SpeednDutyCtrl_Handle_t *pHandle)
{
  bool retVal = false;

#ifdef NULL_PTR_CHECK_SDC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if (MCM_SPEED_MODE == pHandle->Mode)
    {
      pHandle->RampRemainingStep = 0u;
      retVal = true;
    }
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
  return (retVal);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
