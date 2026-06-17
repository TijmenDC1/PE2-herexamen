
/**
  ******************************************************************************
  * @file    bemf_ADC_fdbk_sixstep.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Sensorless Bemf acquisition with ADC component of the Motor Control SDK.
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
  * @ingroup SpeednPosFdbk_Bemf
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BEMFADCFDBK_H
#define BEMFADCFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk_sixstep.h"
#include "speed_duty_ctrl.h"
#include "pwmc_sixstep.h"
#include "bus_voltage_sensor.h"
#include "parameters_conversion.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @addtogroup SpeednPosFdbk_Bemf
  * @{
  */

#define SPEED_BUFFER_LENGTH ((uint8_t)18) /*!< Length of buffer used to store the timer period measurements. */
#define ADC_TIMER_TRIGGER TIM2

/**
  * @brief This structure is used to handle the speed timer state
  *
  */
  typedef enum
{
  LFTIM_DEMAGNETIZATION,
  LFTIM_COMMUTATION,
  LFTIM_IDLE
} SpeedTimerState_t;

/**
  * @brief This structure is used to handle the thresholds for bemf zero crossing detection.
  *
  */
typedef struct
{
  uint16_t AdcThresholdPwmPerc;      /*!< BEMF voltage threshold for zero crossing detection when BEMF is decreasing. */
  uint16_t AdcThresholdHighPerc;     /*!< BEMF voltage threshold for zero crossing detection when BEMF is decreasing. */
  uint16_t AdcThresholdLowPerc;      /*!< BEMF voltage threshold for zero crossing detection when BEMF is increasing. */
  uint16_t Bus2ThresholdConvFactor;  /*!< BEMF voltage threshold for zero crossing detection when BEMF is increasing. */
  uint16_t ThresholdCorrectFactor;
  uint16_t SamplingPointOff;         /*!< Pulse value of the timer channel used to trig the ADC. */
  uint16_t SamplingPointOn;          /*!< Pulse value of the timer channel used to trig the ADC. */
  uint8_t AWDfiltering;
} Bemf_Sensing_Params;

/**
  * @brief This structure is used to transfer parameters from/to motor Pilot.
  *
  */
typedef struct
{
  uint16_t ZcRising2CommDelay;
  uint16_t ZcFalling2CommDelay;
  uint16_t OnSensingEnThres;
  uint16_t OnSensingDisThres;
  uint16_t ComputationDelay;
} Bemf_RegInterface_Param;

/**
  * @brief This structure is used to handle the demagnetization time before starting bemf acquisition.
  *
  */
typedef struct
{
  uint16_t DemagMinimumSpeedUnit;    /*!< Speed threshold for minimum demagnetization time. */
  uint32_t RevUpDemagSpeedConv;      /*!< Convertion factor between speed and demagnetization time. */
  uint32_t RunDemagSpeedConv;        /*!< Open loop convertion factor between speed and demagnetization time during. */
  uint32_t DemagMinimumThreshold;    /*!< Minimum demagnetization time. */
} Bemf_Demag_Params;

/**
  * @brief This structure is used to handle the data of an instance of the B-emf Feedback component.
  *
  */
typedef struct
{
  SpeednPosFdbk_6S_Handle_t _Super;
  uint8_t ZcEvents;                            /**< Number of bemf zero crossing events. */
  Bemf_Sensing_Params Pwm_H_L;                 /**< Parameters for zero crossing detection during ON time. */
  uint16_t pSensing_Threshold_HSMod;
  uint16_t pSensing_Threshold_LSMod;
  uint16_t *pSensing_Point;
  bool IsOnSensingEnabled;                     /*!< Value where 0 means BEMF is sensed during PWM OFF time and 1 or
                                                    greater means BEMF is sensed during PWM ON time. */
  uint16_t OnSensingEnThres;                   /*!< Pulse value of HF timer above which the PWM ON sensing is enabled. */
  uint16_t OnSensingDisThres;                  /*!< Pulse value of HF timer below which the PWM ON sensing is disabled. */
  uint16_t ZcRising2CommDelay;                 /*!< Rising Zero Crossing detection to commutation delay in 15/128 degrees. */
  uint16_t ZcFalling2CommDelay;                /*!< Falling Zero Crossing detection to commutation delay in 15/128 degrees. */
  uint8_t  SpeedBufferSize;                    /*!< Size of the buffer used to calculate the average
                                                    speed. It must be less than 18.*/
  int8_t Direction;                            /*!< Instantaneous direction of rotor between two
                                                    captures. */
  uint8_t StartUpConsistThreshold;             /*!< Number of consecutive tests on speed
                                                    consistency to be passed before
                                                    validating the start-up. */
  bool IsLoopClosed;                           /*!< Boolean variable containing speed
                                                    loop status. */
  bool RequestLoopClosed;                      /*!< Flag raised to close speed loop when next step commutations
                                                    occurs. */
  DrivingMode_t DriveMode;
  volatile uint8_t BufferFilled;               /*!< Indicates the number of speed measuremt
                                                    present in the buffer from the start.
                                                    It will be max bSpeedBufferSize and it
                                                    is used to validate the start of speed
                                                    averaging. If bBufferFilled is below
                                                    bSpeedBufferSize the instantaneous
                                                    measured speed is returned as average
                                                    speed.*/
  int32_t SpeedBufferDpp[SPEED_BUFFER_LENGTH]; /*!< Holding the last period captures. */
  int32_t  ElPeriodSum;                        /*!< Period accumulator used to speed up the average speed computation. */
  uint16_t SpeedFIFOIdx;                       /*!< Pointer of next element to be stored in
                                                    the speed sensor buffer. */
  uint16_t SatSpeed;                           /*!< Returned value if the measured speed is above the
                                                    maximum realistic. */
  uint32_t MaxPeriod;                          /*!< Time delay between two bemf zero crossing points when the speed
                                                    of the rotor is the minimum realistic in the
                                                    application: this allows to discriminate too low
                                                    freq for instance.
                                                    This period shoud be expressed in timer counts and
                                                    it will be:
                                                    wMaxPeriod = ((10 * CKTIM) / 6) / MinElFreq(0.1Hz). */
  uint32_t MinPeriod;                          /*!< Time delay between two bemf zero crossing points when the speed
                                                    of the rotor is the maximum realistic in the
                                                    application: this allows discriminating glitches
                                                    for instance.
                                                    This period shoud be expressed in timer counts and
                                                    it will be: wSpeedOverflow = ((10 * CKTIM) / 6) / MaxElFreq(0.1Hz). */
  uint8_t ComputationDelay;                    /*!< Selection of the period for computation of the delay between the
                                                    zero crossing and the step change. */
  uint8_t MaxZCDetectionErrors;               /*!< Counter to check motor stall */
  uint32_t LastOCCounter;                      /*!< Counter to check motor stall */
  uint8_t ZCDetectionErrors;                  /*!< Counter to check motor stall */
  uint32_t StepTime_Up;                        /*!< Low frequency timer counter value at zero crossing with increasing bemf. */
  uint32_t StepTime_Down;                      /*!< Low frequency timer counter value at zero crossing with increasing bemf. */
  uint32_t StepTime_Last;                      /*!< Low frequency timer counter value at zero crossing with increasing bemf. */
  uint32_t Last_TimerSpeed_Counter;            /*!< Last delay between zero crossing and step change. */
  uint32_t DemagCounterThreshold;              /*!< PWM cycles dedicated to windings demagnetization. */
  Bemf_Demag_Params DemagParams;               /*!< Demagnetization parameters. */
  SpeedTimerState_t SpeedTimerState;
} Bemf_ADC_Handle_t;

/* Exported functions --------------------------------------------------------*/

/* Initializes all the object variables. */
void BADC_Init(Bemf_ADC_Handle_t *pHandle);

/* Resets the ADC status and empties arrays. */
void BADC_Clear(Bemf_ADC_Handle_t *pHandle);

/* Gets ADC value and check for zero crossing detection. */
void BADC_IsZcDetected(Bemf_ADC_Handle_t *pHandle, uint8_t step);

/* Sets the trigger point of the ADC. */
void BADC_SetSamplingPoint(Bemf_ADC_Handle_t *pHandle, const PWMC_Handle_t *pHandlePWMC, BusVoltageSensor_Handle_t *BusVHandle);

/* Computes the rotor average mechanical speed in the unit defined by #SPEED_UNIT and returns it in pMecSpeedUnit. */
bool BADC_CalcAvrgMecSpeedUnit(Bemf_ADC_Handle_t *pHandle);

/* Forces the rotation direction. */
void BADC_SetDirection(Bemf_ADC_Handle_t *pHandle, int8_t direction);

/* Starts the bemf acquisition. */
void BADC_Start(const Bemf_ADC_Handle_t *pHandle, uint8_t step, const uint8_t LSModArray[6]);

/* Stops the bemf acquisition. */
void BADC_Stop(void);

/* Configures the sensorless parameters for the following step. */
void BADC_StepChangeEvent(Bemf_ADC_Handle_t *pHandle);

/* Computes the demagnetization time during revup procedure. */
void BADC_CalcRevUpDemagTime(Bemf_ADC_Handle_t *pHandle, int16_t RevUpSpeed);

/* Computes the demagnetization time in closed loop operation. */
void BADC_CalcRunDemagTime(Bemf_ADC_Handle_t *pHandle);

/* Sets the flag when switch over phase ends. */
void BADC_SetLoopClosed(Bemf_ADC_Handle_t *pHandle);

/* Configures speed timer interrupt */
void BADC_SetSpeedTimer(const Bemf_ADC_Handle_t *pHandle, uint32_t SpeedTimerCounter);

/* Checks whether zero crossing has been detected and OC counter updated*/
bool BADC_CheckDetectionErrors(Bemf_ADC_Handle_t *pHandle);

/* Sets the parameters for bemf sensing during pwm off-time. */
void BADC_SetBemfSensorlessParam(Bemf_ADC_Handle_t *pHandle, Bemf_Sensing_Params *BemfAdcConfig,
                                 Bemf_Demag_Params *BemfAdcDemagConfig, Bemf_RegInterface_Param *BemfRegIntParam);

/* Gets the parameters for bemf sensing during pwm off-time. */
void BADC_GetBemfSensorlessParam(Bemf_ADC_Handle_t *pHandle, Bemf_Sensing_Params *BemfAdcConfig,
                                 Bemf_Demag_Params *BemfAdcDemagConfig, Bemf_RegInterface_Param *BemfRegIntParam);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* BEMFADCFDBK_H */

/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/

