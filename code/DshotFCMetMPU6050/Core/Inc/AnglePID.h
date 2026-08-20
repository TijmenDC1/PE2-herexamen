/*
 * AnglePID.h
 *
 *  Created on: 28 jul 2026
 *      Author: simon
 */

#ifndef INC_ANGLEPID_H_
#define INC_ANGLEPID_H_

typedef struct {
    float kp;
    float ki;
    float kd;

    float i_som;        // opgetelde integraal
    float i_limit;      // clamp tegen windup
    float vorige_fout;

    float out_limit;    // clamp op de output

    float d_filter;     // gefilterde D-ingang
    float d_cutoff_hz;
} PID_Angle_t;

void PID_Angle_Init(PID_Angle_t *pid, float kp, float ki, float kd,
                     float i_limit, float out_limit);

// alleen de interne waarden wissen, gains blijven staan
void PID_Angle_Reset(PID_Angle_t *pid);

// dt in seconden. D-term uit het verschil tussen twee fouten.
float PID_Angle_Update(PID_Angle_t *pid, float setpoint, float hoek, float dt);

// zelfde, maar D-term uit de gemeten gyro-snelheid (graden/s). Minder ruis.
float PID_Angle_UpdateRate(PID_Angle_t *pid, float setpoint, float hoek,
                            float rate, float dt);

#endif /* INC_ANGLEPID_H_ */
