/*
 * AnglePID.c
 *
 *  Created on: 28 jul 2026
 *      Author: simon
 */

#include "AnglePID.h"

// uit DshotFCMetMPU6050/Core/Src/AnglePID.c (auteur: simon)
void PID_Angle_Init(PID_Angle_t *pid, float kp, float ki, float kd,
                     float integral_limit, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->d_state = 0.0f;
    pid->d_cutoff_hz = 60.0f; // standaard filter op de D-term
}

// uit DshotFCMetMPU6050/Core/Src/AnglePID.c (auteur: simon)
void PID_Angle_Reset(PID_Angle_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->d_state = 0.0f;
}

// uit DshotFCMetMPU6050/Core/Src/AnglePID.c (auteur: simon)
float PID_Angle_UpdateRate(PID_Angle_t *pid, float setpoint, float measurement,
                            float rate, float dt)
{
    float error = setpoint - measurement;

    // integraal opbouwen, met anti-windup clamp
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit)  pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    pid->prev_error = error;

    // Bij een vaste setpoint geldt: d(fout)/dt = -d(hoek)/dt = -draaisnelheid.
    // We gebruiken dus de gyro rechtstreeks, met een minteken.
    // Eerst-orde laagdoorlaatfilter erop: de D-term versterkt hoogfrequente ruis,
    // en dat is wat je als trillen in de motoren hoort. Lagere d_cutoff_hz = rustiger,
    // maar ook iets tragere demping.
    if (pid->d_cutoff_hz > 0.0f && dt > 0.0f) {
        float rc = 1.0f / (6.2831853f * pid->d_cutoff_hz);
        float alpha = dt / (dt + rc);
        pid->d_state += alpha * (rate - pid->d_state);
    } else {
        pid->d_state = rate;
    }
    float derivative = -pid->d_state;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    if (output > pid->output_limit)  output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}

// uit DshotFCMetMPU6050/Core/Src/AnglePID.c (auteur: simon)
float PID_Angle_Update(PID_Angle_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    // integraal opbouwen, met anti-windup clamp
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit)  pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    // afgeleide: verandering van de fout per seconde
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - pid->prev_error) / dt;
    }
    pid->prev_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    if (output > pid->output_limit)  output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}
