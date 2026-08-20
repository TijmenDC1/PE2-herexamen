/*
 * AnglePID.c
 *
 *  Created on: 28 jul 2026
 *      Author: simon
 */

#include "AnglePID.h"

void PID_Angle_Init(PID_Angle_t *pid, float kp, float ki, float kd,
                     float i_limit, float out_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_limit = i_limit;
    pid->out_limit = out_limit;
    pid->i_som = 0.0f;
    pid->vorige_fout = 0.0f;
    pid->d_filter = 0.0f;
    pid->d_cutoff_hz = 60.0f;
}

void PID_Angle_Reset(PID_Angle_t *pid)
{
    pid->i_som = 0.0f;
    pid->vorige_fout = 0.0f;
    pid->d_filter = 0.0f;
}

float PID_Angle_UpdateRate(PID_Angle_t *pid, float setpoint, float hoek,
                            float rate, float dt)
{
    float fout = setpoint - hoek;

    pid->i_som += fout * dt;
    if (pid->i_som > pid->i_limit)  pid->i_som = pid->i_limit;
    if (pid->i_som < -pid->i_limit) pid->i_som = -pid->i_limit;

    pid->vorige_fout = fout;

    // setpoint is vast, dus d(fout)/dt = -draaisnelheid.
    // laagdoorlaatfilter erop, anders versterkt de D-term de ruis.
    if (pid->d_cutoff_hz > 0.0f && dt > 0.0f) {
        float rc = 1.0f / (6.2831853f * pid->d_cutoff_hz);
        float a = dt / (dt + rc);
        pid->d_filter += a * (rate - pid->d_filter);
    } else {
        pid->d_filter = rate;
    }
    float d = -pid->d_filter;

    float uit = pid->kp * fout + pid->ki * pid->i_som + pid->kd * d;

    if (uit > pid->out_limit)  uit = pid->out_limit;
    if (uit < -pid->out_limit) uit = -pid->out_limit;

    return uit;
}

float PID_Angle_Update(PID_Angle_t *pid, float setpoint, float hoek, float dt)
{
    float fout = setpoint - hoek;

    pid->i_som += fout * dt;
    if (pid->i_som > pid->i_limit)  pid->i_som = pid->i_limit;
    if (pid->i_som < -pid->i_limit) pid->i_som = -pid->i_limit;

    float d = 0.0f;
    if (dt > 0.0f) {
        d = (fout - pid->vorige_fout) / dt;
    }
    pid->vorige_fout = fout;

    float uit = pid->kp * fout + pid->ki * pid->i_som + pid->kd * d;

    if (uit > pid->out_limit)  uit = pid->out_limit;
    if (uit < -pid->out_limit) uit = -pid->out_limit;

    return uit;
}
