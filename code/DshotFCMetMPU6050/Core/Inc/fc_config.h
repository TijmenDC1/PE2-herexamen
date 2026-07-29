/*
 * fc_config.h
 *
 * Centrale instellingen van de vluchtcontroller.
 * Alle waarden die je tijdens het testen en tunen aanpast staan hier bij elkaar,
 * zodat je niet door main.c hoeft te zoeken.
 *
 *  Author: simon
 */

#ifndef INC_FC_CONFIG_H_
#define INC_FC_CONFIG_H_

/* ==========================================================================
 * TESTSTANDEN
 * ========================================================================== */

/* Op 1: motoren blijven UIT, alleen de hoeken worden geprint.
 * Gebruik dit om veilig de IMU-oriëntatie te controleren voor je iets laat draaien. */
#define ANGLE_CHECK_ONLY        0

/* Op 1: draairichting van M2 en M4 omkeren via DShot-commando's en opslaan in de
 * ESC. Eenmalig gebruiken, daarna terug op 0. PROPELLERS ERAF.
 * Werkte niet op deze ESC's; de richting is uiteindelijk omgezet door twee van
 * de drie motordraden te verwisselen. */
#define SET_MOTOR_DIRECTIONS    0

/* Op 1: gebruik DShot-commando 7/8 in plaats van 20/21.
 * Oudere BLHeli_S-firmware kent alleen die eerste twee. */
#define USE_OLD_DIRECTION_CMDS  0

/* ==========================================================================
 * IMU-ORIENTATIE
 *
 * Het sensormoduletje zit niet recht op het frame. Deze twee vlaggen rekenen de
 * sensor-assen om naar frame-assen.
 *
 * Controle: kantel de NEUS omlaag. Dan hoort 'pitch' te veranderen, niet 'roll'.
 * En de gemeten level-offset bij het opstarten hoort klein te zijn (een paar
 * graden). Is die bijna 180, dan staat de sensor ondersteboven.
 * ========================================================================== */

#define IMU_ROTATED_90          1   /* sensor 90 graden gedraaid t.o.v. frame  */
#define IMU_UPSIDE_DOWN         1   /* sensor ondersteboven gemonteerd         */

/* Weegfactor van het complementaire filter: hoger = meer vertrouwen op de gyro
 * (snel maar driftgevoelig), lager = meer op de accelerometer (stabiel maar
 * traag en gevoelig voor trillingen). */
#define ATTITUDE_ALPHA          0.98f

/* Aantal samples voor de kalibraties bij het opstarten. */
#define GYRO_CALIB_SAMPLES      1000    /* plaat moet STIL liggen              */
#define LEVEL_CALIB_SAMPLES     500     /* frame moet WATERPAS liggen          */

/* ==========================================================================
 * THROTTLE
 *
 * DShot-bereik: 0 = stop, 1..47 zijn commando's (nooit als gas versturen),
 * 48..2047 is echt gas.
 *
 * BASE moet tussen MIN en MAX liggen, met ruimte naar beide kanten voor de
 * PID-correcties. Staat BASE onder MIN, dan worden alle motoren naar MIN
 * geklemd en verdwijnt het onderlinge verschil volledig.
 *
 * Let op: stuwkracht loopt KWADRATISCH met het toerental. Op lage basis-throttle
 * levert een klein throttle-verschil nauwelijks koppel, en lijkt de regeling
 * te zwak terwijl de gains prima zijn.
 * ========================================================================== */

#define THROTTLE_BASE           600     /* throttle bij vlak liggen            */
#define THROTTLE_MIN            200     /* motoren blijven hieronder niet mooi draaien */
#define THROTTLE_MAX            1200    /* veiligheidsplafond voor deze tests   */

/* ==========================================================================
 * PID (angle mode: setpoint is 0 graden = waterpas)
 *
 * Afgestemd op de teststaaf, een as tegelijk.
 * Tuningvolgorde die gewerkt heeft:
 *   1. alleen Kp verhogen tot hij net begint te slingeren, dan terug naar ~60-70%
 *   2. Kd opbouwen tot hij in een of twee bewegingen tot rust komt
 *   3. eventueel Kp weer wat omhoog, want demping laat meer versterking toe
 *   4. Ki alleen als er een blijvende scheefstand overblijft
 * ========================================================================== */

#define PID_KP                  9.0f
#define PID_KI                  0.05f
#define PID_KD                  0.8f

#define PID_INTEGRAL_LIMIT      100.0f  /* anti-windup clamp op de integraal   */
#define PID_OUTPUT_LIMIT        500.0f  /* max correctie per motor (+/-)       */

/* Kantelfrequentie van het filter op de D-ingang. De D-term versterkt
 * hoogfrequente ruis, en dat hoor je als trillen in de motoren.
 * Lager = rustiger, maar ook iets tragere demping. */
#define PID_D_CUTOFF_HZ         60.0f

/* ==========================================================================
 * FAILSAFE
 *
 * Slaat de motoren definitief af (tot een herstart) bij een te grote hoek of na
 * een ingestelde testduur. De hoekbewaking vangt vooral een verkeerd mixer-teken
 * op: dan wordt de terugkoppeling positief en jaagt de regeling zichzelf op.
 *
 * Let op de wiskundige grenzen: pitch komt maar tot +/-90 graden en roll tot
 * +/-180. Een waarde boven 180 zet de hoekbewaking dus feitelijk uit.
 * ========================================================================== */

#define FAILSAFE_ANGLE          80.0f   /* graden; zet op ~55 bij tunen met props */
#define FAILSAFE_TIMEOUT        30.0f   /* seconden                            */

#endif /* INC_FC_CONFIG_H_ */
