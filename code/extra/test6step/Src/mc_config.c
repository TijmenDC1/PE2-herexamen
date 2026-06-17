
/**
  ******************************************************************************
  * @file    mc_config.c
  * @author  Motor Control SDK Team,ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044,the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
//cstat -MISRAC2012-Rule-21.1
#include "main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "mc_config.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

/* USER CODE BEGIN Additional define */

/* USER CODE END Additional define */

/**
  * @brief  PI / PID Speed loop parameters Motor 1.
  */
PID_Handle_t PIDSpeedHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)(PERIODMAX * SP_KIDIV),
  .wLowerIntegralLimit = 0,
  .hUpperOutputLimit   = (int16_t)PERIODMAX,
  .hLowerOutputLimit   = 0,
  .hKpDivisor          = (uint16_t)SP_KPDIV,
  .hKiDivisor          = (uint16_t)SP_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

static SpeednDutyCtrl_Handle_t SpeednDutyCtrlM1 =
{
  .Mode                       = DEFAULT_CONTROL_MODE,                         /* Changed during RevUp phase */
  .TargetFinal                = 0,                                            /* Will be updated with SDC_ExecRamp and SDC_CalcSpeedReference. */
  .SpeedRefUnitExt            = (int32_t)(DEFAULT_TARGET_SPEED_UNIT) * 65536, /* Mechanical rotor speed reference. */
  .DutyCycleRef               = 0U * 65536U,                                  /* Will be updated during RevUp phase. */
  .RampRemainingStep          = 0U,
  .PISpeed                    = &PIDSpeedHandle_M1,                            /* Speed PID pointeur. */
  .SPD                        = &RevUpControlM1._Super,                        /* Speed sensor pointer. */
  .IncDecAmount               = 0U,
  .SDCFrequencyHz             = MEDIUM_FREQUENCY_TASK_RATE,
  .MaxAppPositiveMecSpeedUnit = (uint16_t)(MAX_APPLICATION_SPEED_UNIT),
  .MinAppPositiveMecSpeedUnit = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
  .MaxAppNegativeMecSpeedUnit = (int16_t)(-MIN_APPLICATION_SPEED_UNIT),
  .MinAppNegativeMecSpeedUnit = (int16_t)(-MAX_APPLICATION_SPEED_UNIT),
  .MaxPositiveDutyCycle       = (uint16_t)PERIODMAX / 2U,
  .ModeDefault                = DEFAULT_CONTROL_MODE,
  .MecSpeedRefUnitDefault     = (int16_t)(DEFAULT_TARGET_SPEED_UNIT),
  .DutyCycleRefDefault        = 0U,
};

RevUpCtrl_6S_Handle_t RevUpControlM1 =
{
  ._Super =
  {
    .bElToMecRatio             = POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit  = (uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit  = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber = M1_SS_MEAS_ERRORS_BEFORE_FAULTS,
    .speedConvFactor           = SPEED_TIMER_CONV_FACTOR,
  },

  .hRUCFrequencyHz         = MEDIUM_FREQUENCY_TASK_RATE,
  .hMinStartUpValidSpeed   = OBS_MINIMUM_SPEED_UNIT,
  .pSDC                    = &SpeednDutyCtrlM1,

  .ParamsData  =
  {
    {(uint16_t)PHASE1_DURATION,(int16_t)(PHASE1_FINAL_SPEED_UNIT),
    (uint16_t)PHASE1_VOLTAGE_DPP,&RevUpControlM1.ParamsData[1]},
    {(uint16_t)PHASE2_DURATION,(int16_t)(PHASE2_FINAL_SPEED_UNIT),
    (uint16_t)PHASE2_VOLTAGE_DPP,&RevUpControlM1.ParamsData[2]},
    {(uint16_t)PHASE3_DURATION,(int16_t)(PHASE3_FINAL_SPEED_UNIT),
    (uint16_t)PHASE3_VOLTAGE_DPP,&RevUpControlM1.ParamsData[3]},
    {(uint16_t)PHASE4_DURATION,(int16_t)(PHASE4_FINAL_SPEED_UNIT),
    (uint16_t)PHASE4_VOLTAGE_DPP,&RevUpControlM1.ParamsData[4]},
    {(uint16_t)PHASE5_DURATION,(int16_t)(PHASE5_FINAL_SPEED_UNIT),
    (uint16_t)PHASE5_VOLTAGE_DPP,(void*)MC_NULL},
  },
};

Bemf_ADC_Handle_t Bemf_ADC_M1 =
{
  ._Super =
  {
    .bElToMecRatio             = POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit  = (uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit  = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber = M1_SS_MEAS_ERRORS_BEFORE_FAULTS,
    .speedConvFactor           = SPEED_TIMER_CONV_FACTOR,
  },

  .Pwm_H_L =
  {
    .AdcThresholdPwmPerc       = 10 * BEMF_THRESHOLD_PWM_PERC,
    .AdcThresholdHighPerc      = 10 * BEMF_THRESHOLD_HIGH_PERC,
    .AdcThresholdLowPerc       = 10 * BEMF_THRESHOLD_LOW_PERC,
    .Bus2ThresholdConvFactor   = BEMF_BUS2THRES_FACTOR,
    .ThresholdCorrectFactor    = BEMF_CORRECT_FACTOR,
    .SamplingPointOff          = BEMF_ADC_TRIG_TIME,
    .SamplingPointOn           = BEMF_ADC_TRIG_TIME_ON,
  },
  .OnSensingEnThres            = BEMF_PWM_ON_ENABLE_THRES,
  .OnSensingDisThres           = BEMF_PWM_ON_DISABLE_THRES,
  .ComputationDelay            = (uint8_t) (- COMPUTATION_DELAY),
  .ZcRising2CommDelay          = ZCD_RISING_TO_COMM_9BIT,
  .ZcFalling2CommDelay         = ZCD_FALLING_TO_COMM_9BIT,
  .SpeedBufferSize             = BEMF_AVERAGING_FIFO_DEPTH,
  .StartUpConsistThreshold     = NB_CONSECUTIVE_TESTS,
  .DemagParams =
  {
    .DemagMinimumSpeedUnit     = DEMAG_MINIMUM_SPEED,
    .RevUpDemagSpeedConv       = DEMAG_REVUP_CONV_FACTOR,
    .RunDemagSpeedConv         = DEMAG_RUN_CONV_FACTOR,
    .DemagMinimumThreshold     = MIN_DEMAG_COUNTER_TIME,
  },
  .MaxZCDetectionErrors        = BEMF_ERRORS_SCORE,
  .DriveMode                   = DEFAULT_DRIVE_MODE,
};

PWMC_Handle_t PWM_Handle_M1 =
{
  .StartCntPh           = PWM_PERIOD_CYCLES,
  .PWMperiod            = PWM_PERIOD_CYCLES,
  .AlignStep            = ALIGN_STEP,
  .OverCurrentFlag      = false,
  .OverVoltageFlag      = false,
  .driverProtectionFlag = false,
  .TimerCfg             = &SixPwm_TimerCfgM1,
  .QuasiSynchDecay      = false,
  .LSModArray           = {0,0,0,0,0,0},
  .pParams_str          = &PWMC_ParamsM1,
  .LowSideOutputs       = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,
  .TurnOnLowSidesAction = false,
};

SixStepVars_t SixStepVars[NBR_OF_MOTORS];
SpeednDutyCtrl_Handle_t *pSDC[NBR_OF_MOTORS]    = {&SpeednDutyCtrlM1};
NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS] = {&TempSensor_M1};

MCI_Handle_t Mci[NBR_OF_MOTORS] =
{
  {
    .pSDC = &SpeednDutyCtrlM1,
    .pSixStepVars = &SixStepVars[0],
    .pPWM = &PWM_Handle_M1,
    .lastCommand = MCI_NOCOMMANDSYET,
    .hFinalSpeed = 0,
    .hFinalTorque = 0,
    .pScale = &scaleParams_M1,
    .hDurationms = 0,
    .DirectCommand = MCI_NO_COMMAND,
    .State = IDLE,
    .CurrentFaults = MC_NO_FAULTS,
    .PastFaults = MC_NO_FAULTS,
    .CommandState = MCI_BUFFER_EMPTY,
  },

};

/* USER CODE BEGIN Additional configuration */

/* USER CODE END Additional configuration */

/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/

