/*
 * fc_config.h
 *
 * Alle instellingen die we tijdens het testen aanpassen.
 *
 *  Author: simon
 */

#ifndef INC_FC_CONFIG_H_
#define INC_FC_CONFIG_H_

/* teststanden */
#define ANGLE_CHECK_ONLY        0   /* 1 = motoren uit, alleen hoeken printen */
#define SET_MOTOR_DIRECTIONS    0   /* 1 = draairichting zetten, props eraf!   */
#define USE_OLD_DIRECTION_CMDS  0   /* 1 = cmd 7/8 i.p.v. 20/21 (oude BLHeli)  */

/* stand van de sensor op het frame.
 * Check: neus omlaag kantelen hoort pitch te veranderen, niet roll. */
#define IMU_ROTATED_90          1
#define IMU_UPSIDE_DOWN         1

/* complementair filter: hoger = meer gyro, lager = meer accel */
#define ATTITUDE_ALPHA          0.98f

#define GYRO_CALIB_SAMPLES      1000    /* stil houden     */
#define LEVEL_CALIB_SAMPLES     500     /* waterpas houden */

/* throttle. 0 = stop, 1..47 zijn commando's, 48..2047 is gas.
 * BASE moet tussen MIN en MAX liggen, anders klemt alles op MIN. */
#define THROTTLE_BASE           600
#define THROTTLE_MIN            200
#define THROTTLE_MAX            1200

/* PID, setpoint = 0 graden.
 * Tunen: eerst Kp tot hij begint te slingeren en dan terug, daarna Kd,
 * en Ki alleen als er een scheefstand blijft staan. */
#define PID_KP                  9.0f
#define PID_KI                  0.05f
#define PID_KD                  0.8f

#define PID_INTEGRAL_LIMIT      100.0f
#define PID_OUTPUT_LIMIT        500.0f
#define PID_D_CUTOFF_HZ         60.0f   /* filter op de D-term, lager = rustiger */

/* failsafe: motoren definitief uit bij te grote hoek of na deze tijd */
#define FAILSAFE_ANGLE          200.0f   /* graden, bij tunen met props ~55 */
#define FAILSAFE_TIMEOUT        200.0f   /* seconden */

#endif /* INC_FC_CONFIG_H_ */
