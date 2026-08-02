/*
 * flightcontrol.h
 *
 *  Created on: 30 jul 2026
 *      Author: tijme
 *
 * Voert een FlightPlan_t uit op de motoren. Houdt het frame waterpas met een
 * angle-mode PID (MPU6050 + complementair filter + AnglePID), zelfde aanpak
 * als DshotFCMetMPU6050, en past daarbovenop per commando throttle/tilt aan.
 *
 * RelativeHeight/AbsoluteHeight/Move hebben de barometer/GPS nodig (zie
 * flightplan.c) en worden hier nog als stub afgehandeld: geparsed maar
 * overgeslagen met een printf, in afwachting van die sensoren.
 */

#ifndef INC_FLIGHTCONTROL_H_
#define INC_FLIGHTCONTROL_H_

#include <stdint.h>
#include "flightplan.h"

/* Initialiseert de IMU, de motoren (dshot) en de stabilisatie-PID's.
 * Kalibreert de gyro (plaat moet stil liggen) en de montage-offset van de IMU
 * (frame moet waterpas liggen), en arm de ESC's. Blokkerend, duurt enkele
 * seconden. Retourneert 0 bij succes. */
int FlightControl_Init(void);

/* Voert het volledige vluchtplan uit: blijft continu stabiliseren (roll/pitch
 * waterpas) terwijl de commando's na elkaar worden afgehandeld. Landt zelf
 * netjes af zodra het plan op is of bij een Land-commando. Blokkerend tot de
 * vlucht afgelopen is of de failsafe ingrijpt (te grote hoek of totale
 * vluchttijd voorbij, zie fc_config.h).
 *   get_tick_fn : bv. HAL_GetTick, voor het meten van commando-duur (ms)
 *   delay_fn    : bv. HAL_Delay, voor de regellus-pacing
 *   loop_dt_ms  : gewenste regellus-periode in ms
 */
void FlightControl_Run(FlightPlan_t *plan, uint32_t (*get_tick_fn)(void),
                        void (*delay_fn)(uint32_t), uint32_t loop_dt_ms);

#endif /* INC_FLIGHTCONTROL_H_ */
