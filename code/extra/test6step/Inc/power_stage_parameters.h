
/**
  ******************************************************************************
  * @file    power_stage_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a power stage.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef POWER_STAGE_PARAMETERS_H
#define POWER_STAGE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/************* PWM Driving signals section **************/
/******** BEMF reading parameters section ******/

#define BEMF_ON_SENSING_DIVIDER              14.691176 /*!< BEMF voltage divider during PWM ON time
                                                   for zero crossing detection */
/*********** Bus voltage sensing section ****************/
#define VBUS_PARTITIONING_FACTOR             0.05219206680584551 /*!< It expresses how
                                                      much the Vbus is attenuated
                                                      before being converted into
                                                      digital value */
#define NOMINAL_BUS_VOLTAGE_V                17U
/******** Current reading parameters section ******/
/*** Topology ***/
#define None

#define RSHUNT                               0.010

/*  ICSs gains in case of isolated current sensors,
        amplification gain for shunts based sensing */
#define AMPLIFICATION_GAIN                   5.18

/************ Temperature sensing section ***************/
/* V[V]=V0+dV/dT[V/Celsius]*(T-T0)[Celsius]*/
#define V0_V                                 0 /*!< in Volts */
#define T0_C                                 25 /*!< in Celsius degrees */
#define dV_dT                                0.017 /*!< V/Celsius degrees */
#define T_MAX                                90 /*!< Sensor measured
                                                     temperature at maximum
                                                     power stage working
                                                     temperature, Celsius degrees */

#endif /*POWER_STAGE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
