/*
 * PIDregelaar.c
 *
 *  Created on: 9 apr 2026
 *      Author: tijme
 */


typedef struct {
    // De Tuning parameters aanpassen tijdens testen afhankelijk van externe factoren
    float Kp;
    float Ki;
    float Kd;			   // P
    float integraal;       // I  Som van de fouten over tijd
    float vorigefout;      // D  Fout van de vorige meting
    float limiet;      // extra bescherming als verkeerde berekening niet te veel kracht na motor
} PID_Controller;



void PID_Init(void) {
    // ROLL moet dus nog ingesteld worden gevraagd aan AI om voorbeeld waarde te geven
    pidRoll.Kp = 1.2f;
    pidRoll.Ki = 0.01f;
    pidRoll.Kd = 0.05f;
    pidRoll.integraal = 0;
    pidRoll.vorigefout = 0;
    pidRoll.limiet = 100.0f; // Limiet om veilig te blijven

    // Pitch is vaak hetzelfde als Roll bij een symmetrische drone
    // gaat bij ons dus wss niet zijn omwille van batterij en flightcontroller
    pidPitch.Kp = 1.2f;
    pidPitch.Ki = 0.01f;
    pidPitch.Kd = 0.05f;
    pidPitch.integraal = 0;
    pidPitch.vorigefout = 0;
    pidPitch.limiet = 100.0f;

    // Yaw heeft meestal meer kracht (P) nodig om te draaien
    pidYaw.Kp = 2.0f;
    pidYaw.Ki = 0.01f;
    pidYaw.Kd = 0.0f;
    pidYaw.integraal = 0;
    pidYaw.vorigefout = 0;
    pidYaw.limiet = 50.0f;
}


float PID_Compute(PID_Controller *pid, float setpoint, float actuele_waarde, float dt) //dt zou idealiter 1khz zijn dus 0.001 1ms
{
    //ROLL
	// ERror berekening
    float fout = setpoint - actuele_waarde;

    // P term proportional
    float P_out = pid->Kp * fout;

    // 3. I-Term (Integral) - Geheugen
    pid->integraal = pid->integraal + (fout * dt);

    // limiet bescherming zowel + als -
    if (pid->integraal > pid->limiet)
    {
    	pid->integraal = pid->limiet;
    }
    if (pid->integraal < -pid->limiet)
    {
    	pid->integraal = -pid->limiet;
    }

    float I_out = pid->Ki * pid->integraal;

    //D: derrivetive of afgeleide
    //verandering fout per verlope tijd

    float afgeleide = (fout - pid->vorigefout) / dt;
    float D_out = pid->Kd * afgeleide;

    //nieuwe fout wordt oude fout
    pid->vorigefout = fout;

    float totaal = P_out + I_out + D_out;

    // totaal resultaat
    return totaal;



}

