
/**
  ******************************************************************************
  * @file    mc_tasks_sixstep.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition
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
//cstat -MISRAC2012-Rule-21.1
#include "main.h"
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "digital_output.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "mcp_config.h"
#include "mc_app_hooks.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/

/* USER CODE END Private define */

/* Private variables----------------------------------------------------------*/
static volatile uint16_t hBootCapDelayCounterM1   = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM1 = ((uint16_t)0);

#define M1_CHARGE_BOOT_CAP_TICKS       (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M1_CHARGE_BOOT_CAP_DUTY_CYCLES ((uint32_t)0.000\
                                     * ((uint32_t)PWM_PERIOD_CYCLES / 2U))
#define M2_CHARGE_BOOT_CAP_TICKS       (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M2_CHARGE_BOOT_CAP_DUTY_CYCLES ((uint32_t)0\
                                      *((uint32_t)PWM_PERIOD_CYCLES2 / 2U))
#define SPEED_TIMER_IDLE_RATE_TICKS    (uint32_t) (REGULAR_CONVERSION_RATE_MS * SYSCLK_FREQ / (1000U * (LF_TIMER_PSC + 1U)))

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
void TSK_MF_StopProcessing(uint8_t motor);
MCI_Handle_t *GetMCI(uint8_t bMotor);
static void SixStep_StepCommution(void);

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  * @param  None
  */
__weak void SIX_STEP_Init(void)
{
  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */

    /**********************************************************/
    /*    PWM and current sensing component initialization    */
    /**********************************************************/
    pwmcHandle[M1] = &PWM_Handle_M1;
    PWMC_Init(&PWM_Handle_M1);

    /* USER CODE BEGIN MCboot 1 */

    /* USER CODE END MCboot 1 */

    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_HandleInit(&PIDSpeedHandle_M1);

    /******************************************************/
    /*   Main speed sensor component initialization       */
    /******************************************************/
    BADC_Init(&Bemf_ADC_M1);

    /******************************************************/
    /*   Speed & duty cycle component initialization          */
    /******************************************************/
    SIX_STEP_Clear(M1);
    MCI_ExecSpeedRamp(&Mci[M1],
    SDC_GetMecSpeedRefUnitDefault(pSDC[M1]),0); /* First command to SDC */

    /* USER CODE BEGIN MCboot 2 */

    /* USER CODE END MCboot 2 */
}

/**
 * @brief Performs stop process and update the state machine.This function
 *        shall be called only during medium frequency task.
 */
void TSK_MF_StopProcessing(uint8_t motor)
{
  SIX_STEP_Clear(motor);
  TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
  Mci[motor].State = STOP;
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the
  * present state of its state machine. In particular, duties requiring a periodic
  * execution at a medium frequency rate (such as the speed controller for instance)
  * are executed here.
  */
__weak void TSK_MediumFrequencyTaskM1(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */

  bool IsSpeedReliable = BADC_CalcAvrgMecSpeedUnit(&Bemf_ADC_M1);

  if (MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS)
  {
    if (MCI_GetOccurredFaults(&Mci[M1]) == MC_NO_FAULTS)
    {
      switch (Mci[M1].State)
      {
        case IDLE:
        {
          if (MCI_START == Mci[M1].DirectCommand)
          {
            RUC_6S_UpdatePulse(&RevUpControlM1, &BusVoltageSensor_M1._Super);
            PWMC_TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
            TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
            Mci[M1].State = CHARGE_BOOT_CAP;
          }
          else
          {
            /* Nothing to be done, FW stays in IDLE state. */
          }
          break;
        }

        case CHARGE_BOOT_CAP:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (TSK_ChargeBootCapDelayHasElapsedM1())
            {
              PWMC_SwitchOffPWM(pwmcHandle[M1]);
              BADC_SetDirection(&Bemf_ADC_M1, (int8_t)MCI_GetImposedMotorDirection(&Mci[M1]));
              SIX_STEP_Clear(M1);

              Mci[M1].State = START;
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
            }
            else
            {
              /* Nothing to be done, FW waits for bootstrap capacitor to charge */
            }
          }
          break;
        }
        case START:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
              /* Execute the IPD procedure. */
              /* Execute the Rev Up procedure. */
              if(! RUC_6S_Exec(&RevUpControlM1))
              {
                /* The time allowed for the startup sequence has expired */
                MCI_FaultProcessing(&Mci[M1], MC_START_UP, 0);
              }
              else
              {
              /* Rotor alignment */
                if (true == RUC_6S_IsAlignStageNow(&RevUpControlM1))
                {
                  PWMC_SetPhaseVoltage(pwmcHandle[M1], SixStepVars[M1].DutyCycleRef);
                  PWMC_LoadNextStep(&PWM_Handle_M1);
                }
                else
                {
                  /* Nothing to do */
                }

                /* Execute the open loop start-up ramp:
                 * Compute the duty cycle reference as configured in the Rev Up sequence */
                (void) BADC_CalcRevUpDemagTime (&Bemf_ADC_M1, SPD_GetAvrgMecSpeedUnit(&RevUpControlM1._Super));
                SixStepVars[M1].DutyCycleRef = SDC_CalcSpeedReference(pSDC[M1]);
              }

              /* Check that startup speed has reached the validation threshold*/
              if (true == RUC_6S_ObserverSpeedReached(&RevUpControlM1))
              {
#if PID_SPEED_INTEGRAL_INIT_DIV == 0
                PID_SetIntegralTerm(&PIDSpeedHandle_M1, 0);
#else
                PID_SetIntegralTerm(&PIDSpeedHandle_M1,
                                    (((int32_t)SixStepVars[M1].DutyCycleRef * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M1))
                                     / PID_SPEED_INTEGRAL_INIT_DIV));
#endif
                /* USER CODE BEGIN MediumFrequencyTask M1 1 */

                /* USER CODE END MediumFrequencyTask M1 1 */

                SixStepVars[M1].DutyCycleRef = SDC_CalcSpeedReference(pSDC[M1]);
                BADC_SetLoopClosed(&Bemf_ADC_M1);
                SDC_SetSpeedSensor(pSDC[M1], &Bemf_ADC_M1._Super);
                SDC_ForceSpeedReferenceToCurrentSpeed(pSDC[M1]); /* Init the reference speed to current speed */
                MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
                Mci[M1].State = RUN;
              }
              else
              {
                /* Nothing to do */
              }
          }
          break;
        }

        case RUN:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            /* USER CODE BEGIN MediumFrequencyTask M1 2 */

            /* USER CODE END MediumFrequencyTask M1 2 */

            MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
            SixStepVars[M1].DutyCycleRef = SDC_CalcSpeedReference(pSDC[M1]);
            (void) BADC_CalcRunDemagTime (&Bemf_ADC_M1);

            if(!IsSpeedReliable)
            {
              MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);
            }
            else
            {
              /* Nothing to do */
            }
          }
          break;
        }

        case STOP:
        {
          if (TSK_StopPermanencyTimeHasElapsedM1())
          {
            BADC_Clear(&Bemf_ADC_M1);
            /* USER CODE BEGIN MediumFrequencyTask M1 5 */

            /* USER CODE END MediumFrequencyTask M1 5 */

            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW waits for to stop */
          }
          break;
        }

        case FAULT_OVER:
        {
          if (MCI_ACK_FAULTS == Mci[M1].DirectCommand)
          {
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW stays in FAULT_OVER state until acknowledgement */
          }
          break;
        }

        case FAULT_NOW:
        {
          Mci[M1].State = FAULT_OVER;
          break;
        }

        default:
          break;
       }
    }
    else
    {
      Mci[M1].State = FAULT_OVER;
    }
  }
  else
  {
    Mci[M1].State = FAULT_NOW;
  }

  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */

}

/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void SIX_STEP_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN SixStep_Clear 0 */

  /* USER CODE END SixStep_Clear 0 */
  SDC_Clear(pSDC[bMotor]);
  SixStepVars[bMotor].DutyCycleRef = SDC_GetDutyCycleRef(pSDC[bMotor]);
  BADC_Stop();
  BADC_Clear(&Bemf_ADC_M1);
  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
  RUC_6S_Clear(&RevUpControlM1, MCI_GetImposedMotorDirection(&Mci[M1]));
  SDC_SetSpeedSensor(pSDC[M1], &RevUpControlM1._Super);
  Bemf_ADC_M1.SpeedTimerState = LFTIM_IDLE;

  /* USER CODE BEGIN SixStep_Clear 1 */

  /* USER CODE END SixStep_Clear 1 */
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

/**
  * @brief  This is the 6step commutation task. It configures the demagnetization period and
  * manages ADC regular conversions when motor is both running and stopped.
  */

__weak void TSK_SpeedTIM_task()
{

  /* USER CODE BEGIN SpeedTimerTask 0 */

  /* USER CODE END SpeedTimerTask 0 */

  if (true == BADC_CheckDetectionErrors(&Bemf_ADC_M1))
  {
    MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);
  }
  else
  {
    /* Nothing to do */
  }

  switch (Bemf_ADC_M1.SpeedTimerState)
  {
    case LFTIM_COMMUTATION:
    {
      BADC_StepChangeEvent(&Bemf_ADC_M1);
      (void)SixStep_StepCommution();
      RCM_ExecNextConv();
      BADC_SetSamplingPoint(&Bemf_ADC_M1, &PWM_Handle_M1, &BusVoltageSensor_M1._Super);
      PWMC_SetADCTriggerChannel(&PWM_Handle_M1, *Bemf_ADC_M1.pSensing_Point);
      break;
    }

    case LFTIM_DEMAGNETIZATION:
    {
      RCM_ReadOngoingConv();
      if (false == Bemf_ADC_M1.IsLoopClosed)
      {
        BADC_SetSpeedTimer(&Bemf_ADC_M1, RevUpControlM1.ElSpeedTimerDpp - Bemf_ADC_M1.DemagCounterThreshold);
      }
      else
      {
        /* Nothing to do, step commutation is scheduled only when bemf zero crossing is detected */
      }
      BADC_Start(&Bemf_ADC_M1, PWM_Handle_M1.Step, PWM_Handle_M1.LSModArray);
      Bemf_ADC_M1.SpeedTimerState = LFTIM_COMMUTATION;
      break;
    }

    case LFTIM_IDLE:
    default:
    {
      RCM_ExecNextConv();
      RCM_WaitForConv();
      RCM_ReadOngoingConv();
      BADC_SetSpeedTimer(&Bemf_ADC_M1, SPEED_TIMER_IDLE_RATE_TICKS);
      if (false == RUC_6S_IsAlignStageNow(&RevUpControlM1))
      {
       Bemf_ADC_M1.SpeedTimerState = LFTIM_COMMUTATION;
      }
      else
      {}
      break;
    }
  }

  /* USER CODE BEGIN SpeedTimerTask 1 */

  /* USER CODE END SpeedTimerTask 1 */
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
inline void SixStep_StepCommution(void)
{
  PWMC_SetPhaseVoltage(pwmcHandle[M1], SixStepVars[M1].DutyCycleRef);
  PWMC_ForceNextStep(&PWM_Handle_M1, RUC_6S_GetDirection(&RevUpControlM1), 0u);
  PWMC_LoadNextStep(&PWM_Handle_M1);
}

/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
