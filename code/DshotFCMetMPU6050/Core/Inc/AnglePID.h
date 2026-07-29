/*
 * AnglePID.h
 *
 * Lichte, float-gebaseerde PID voor hoek-regeling (angle/stabilize-mode).
 * Setpoint = gewenste hoek in graden (0 = waterpas), measurement = gefuseerde
 * hoek uit Attitude_Update. Output = correctie in "throttle-eenheden" die de
 * mixer bij de basis-throttle optelt/aftrekt per motor.
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

    float integral;         // opgebouwde integraal-term
    float integral_limit;   // anti-windup: clamp op de integraal (in dezelfde eenheid als error*tijd)
    float prev_error;       // vorige fout, voor de afgeleide term

    float output_limit;     // clamp op de uiteindelijke output (+/-)

    float d_state;          // gefilterde D-ingang (laagdoorlaat, tegen trillen)
    float d_cutoff_hz;      // kantelfrequentie van dat filter; lager = rustiger
} PID_Angle_t;

// Initialiseert/reset een PID_Angle_t met de opgegeven gains en limieten.
void PID_Angle_Init(PID_Angle_t *pid, float kp, float ki, float kd,
                     float integral_limit, float output_limit);

// Reset alleen de interne toestand (integraal + vorige fout), gains blijven staan.
// Gebruik dit bv. vlak voor het armen, zodat oude opgebouwde integraal niet meetelt.
void PID_Angle_Reset(PID_Angle_t *pid);

// Berekent 1 PID-stap. dt in seconden (uit je eigen dt-meting).
// De D-term wordt hier afgeleid uit het verschil tussen twee foutwaarden.
float PID_Angle_Update(PID_Angle_t *pid, float setpoint, float measurement, float dt);

// Zelfde regelaar, maar de D-term komt uit de GEMETEN draaisnelheid (gyro, in
// graden/s) in plaats van uit het differentieren van de hoek. Dat is een directe
// meting in plaats van een berekening, dus veel minder ruis - en ruis is precies
// waar een D-term gevoelig voor is. Voor een vluchtregelaar is dit de juiste
// aanpak. 'rate' is de draaisnelheid van dezelfde as als 'measurement'.
float PID_Angle_UpdateRate(PID_Angle_t *pid, float setpoint, float measurement,
                            float rate, float dt);

#endif /* INC_ANGLEPID_H_ */
