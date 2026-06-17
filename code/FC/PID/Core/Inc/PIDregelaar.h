/*
 * PIDregelaar.h
 *
 *  Created on: 10 apr 2026
 *      Author: tijme
 */

#ifndef INC_PIDREGELAAR_H_
#define INC_PIDREGELAAR_H_



typedef struct {
    // De Tuning parameters aanpassen tijdens testen afhankelijk van externe factoren
    float Kp;
    float Ki;
    float Kd;			   // P
    float integraal;       // I  Som van de fouten over tijd
    float vorigefout;      // D  Fout van de vorige meting
    float limiet;      // extra bescherming als verkeerde berekening niet te veel kracht na motor
} PID_Controller;


extern PID_Controller pidRoll;
extern PID_Controller pidPitch;
extern PID_Controller pidYaw;

//initialiser bij opstart en alles op 0 zetten
void PID_Init(void);

// Berekent de PID-output op basis van setpoint en actuele waarde.
// De pointer naar de specifieke controller (Roll, Pitch en Yaw)
// setpoint gewenste waarde vanut sx1280
// actuele_waarde van gyro
// dt De verstreken tijd in seconden 0.001 (1khz) per pidloop maar dus 1/3 vanpidloop
// geeft berekende correctie voor de motoren die dus nog moet gecombineerd worde me throttle

float PID_Compute(PID_Controller *pid, float setpoint, float actuele_waarde, float dt);


#endif /* INC_PIDREGELAAR_H_ */
