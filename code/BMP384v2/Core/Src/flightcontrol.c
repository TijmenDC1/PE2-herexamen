/*
 * flightcontrol.c
 *
 *  Created on: 30 jul 2026
 *      Author: tijme
 *
 * Herkomst staat per functie hieronder vermeld: "overgenomen" = code/logica komt
 * (aangepast waar nodig) uit DshotFCMetMPU6050/Core/Src/main.c, "nieuw" = geschreven
 * voor deze flightplan-integratie en bestond niet in dat project.
 */

#include "flightcontrol.h"
#include "main.h"
#include "fc_config.h"
#include "MPU6050.h"
#include "attitude.h"
#include "AnglePID.h"
#include "dshot.h"
#include <stdio.h>

/* Tilt-hoek (graden) die Left/Right toepast bovenop de waterpas-regeling.
 * Vaste placeholder-waarde: pas aan/tune op de echte drone. Het teken hier is
 * een aanname; draai om als de drone de verkeerde kant op leunt. */
#define LEFT_RIGHT_TILT_DEG 15.0f

/* pid_roll/pid_pitch, roll_offset/pitch_offset, g_fused_roll/g_fused_pitch,
 * dshot_dma_complete en motors_killed: zelfde doel als de gelijknamige globals
 * in DshotFCMetMPU6050/Core/Src/main.c, hier enkel static gemaakt (bestandsscope
 * i.p.v. bestandsbreed zichtbaar) omdat main.c van sdkaart ze niet nodig heeft. */
static PID_Angle_t pid_roll;
static PID_Angle_t pid_pitch;

/* Vaste montage-offset van de IMU t.o.v. het frame (graden), gemeten bij het
 * opstarten in FlightControl_Init(). */
static float roll_offset  = 0.0f;
static float pitch_offset = 0.0f;

/* Gefuseerde hoeken (graden), bijgewerkt door Attitude_Update op elke tick. */
static float g_fused_roll  = 0.0f;
static float g_fused_pitch = 0.0f;

/* g_flight_time/g_cyc_prev: zelfde functie als de lokale variabelen test_time/
 * cyc_prev in de while(1)-lus van DshotFCMetMPU6050/Core/Src/main.c, hier
 * bestandsscope omdat run_control_tick() ze tussen aanroepen moet onthouden. */
static float    g_flight_time = 0.0f; /* opgeteld sinds start van FlightControl_Run, voor FAILSAFE_TIMEOUT */
static uint32_t g_cyc_prev    = 0;    /* vorige DWT->CYCCNT, voor de dt-meting */

static volatile uint8_t dshot_dma_complete = 1;

/* Nieuw: bestond niet in DshotFCMetMPU6050 (daar werd altijd rechtstreeks naar
 * THROTTLE_BASE geramped). Hier nodig om tussen twee flightplan-commando's te
 * onthouden vanaf welke throttle de volgende ramp moet starten. */
static int     current_base_throttle = 0;
static uint8_t motors_killed = 0;

/* Overgenomen uit DshotFCMetMPU6050/Core/Src/main.c, functie DWT_Init() (ongewijzigd). */
static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* Cortex-M7 vereist een software-unlock van de DWT voor CYCCNT telt. */
    *((volatile uint32_t*)0xE0001FB0) = 0xC5ACCE55; /* DWT->LAR unlock-key */
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Overgenomen uit DshotFCMetMPU6050/Core/Src/main.c, functie imu_to_frame() (ongewijzigd). */
static inline void imu_to_frame(float ax, float ay, float az, float gx, float gy,
                                 float *f_ax, float *f_ay, float *f_az,
                                 float *f_gx, float *f_gy)
{
    float x = ax, y = ay, z = az, p = gx, q = gy;

#if IMU_UPSIDE_DOWN
    y = -y; z = -z; q = -q;
#endif

#if IMU_ROTATED_90
    float tx = y, ty = -x;
    float tp = q, tq = -p;
    x = tx; y = ty; p = tp; q = tq;
#endif

    *f_ax = x; *f_ay = y; *f_az = z;
    *f_gx = p; *f_gy = q;
}

/* Overgenomen uit DshotFCMetMPU6050/Core/Src/main.c, functie motors_send() (ongewijzigd). */
static void motors_send(int t1, int t2, int t3, int t4)
{
    int t[4] = { t1, t2, t3, t4 };
    for (int i = 0; i < 4; i++) {
        if (t[i] < 0)    t[i] = 0;
        if (t[i] > 2047) t[i] = 2047;
        if (t[i] > 0 && t[i] < 48) t[i] = 0; /* 1..47 zijn dshot-commando's, geen gas */
    }
    while (dshot_dma_complete == 0) { }
    dshot_dma_complete = 0;
    update_motor_buffer(0, (uint16_t)t[0], 0); /* M1 */
    update_motor_buffer(1, (uint16_t)t[1], 0); /* M2 */
    update_motor_buffer(2, (uint16_t)t[2], 0); /* M3 */
    update_motor_buffer(3, (uint16_t)t[3], 0); /* M4 */
    send_dshot();
}

/* Overgenomen uit DshotFCMetMPU6050/Core/Src/main.c (ongewijzigd). Motor-index
 * -> fysieke positie, gemeten met test_motors_identify() in dat bestand:
 * M1 = links-onder, met de klok mee. */
static const int pitch_sign[4] = { +1, -1, -1, +1 }; /* voor(M2,M3) = -, achter(M1,M4) = + */
static const int roll_sign[4]  = { +1, +1, -1, -1 }; /* links(M1,M2) = +, rechts(M3,M4) = - */
static const int yaw_sign[4]   = {  0,  0,  0,  0 }; /* yaw-regeling nog niet geïmplementeerd */

/* Overgenomen uit DshotFCMetMPU6050/Core/Src/main.c, functie Update_Motors() (ongewijzigd). */
static void Update_Motors(int base, float roll_cmd, float pitch_cmd, float yaw_cmd)
{
    int mc[4];
    for (int i = 0; i < 4; i++) {
        int v = base
              + (int)(pitch_sign[i] * pitch_cmd)
              + (int)(roll_sign[i]  * roll_cmd)
              + (int)(yaw_sign[i]   * yaw_cmd);
        if (v < THROTTLE_MIN) v = THROTTLE_MIN;
        if (v > THROTTLE_MAX) v = THROTTLE_MAX;
        mc[i] = v;
    }
    motors_send(mc[0], mc[1], mc[2], mc[3]);
}

/* Overgenomen uit DshotFCMetMPU6050/Core/Src/main.c: de inhoud van de
 * while(1)-lus (daar geen losse functie, hier wel zodat elk vluchtplan-
 * commando 'm kan hergebruiken). Enige toevoeging t.o.v. het origineel is de
 * roll_setpoint-parameter, waarmee Left/Right het frame gericht laten tillen
 * (0 = waterpas, zoals in het origineel). Retourneert 0 als de failsafe net
 * is ingegrepen (motoren al op 0 gezet), anders 1. */
static uint8_t run_control_tick(int base_throttle, float roll_setpoint)
{
    float ax, ay, az, gx, gy, gz;
    MPU6050_Read_Accel(&ax, &ay, &az);
    MPU6050_Read_Gyro(&gx, &gy, &gz);

    uint32_t cyc_now = DWT->CYCCNT;
    float dt = (uint32_t)(cyc_now - g_cyc_prev) / (float)SystemCoreClock;
    g_cyc_prev = cyc_now;

    float f_ax, f_ay, f_az, f_gx, f_gy;
    imu_to_frame(ax, ay, az, gx, gy, &f_ax, &f_ay, &f_az, &f_gx, &f_gy);

    Attitude_Update(f_ax, f_ay, f_az, f_gx, f_gy, gz, dt, &g_fused_roll, &g_fused_pitch);

    float roll_level  = g_fused_roll  - roll_offset;
    float pitch_level = g_fused_pitch - pitch_offset;

    g_flight_time += dt;
    if (!motors_killed) {
        if (roll_level  >  FAILSAFE_ANGLE || roll_level  < -FAILSAFE_ANGLE ||
            pitch_level >  FAILSAFE_ANGLE || pitch_level < -FAILSAFE_ANGLE) {
            motors_killed = 1;
            printf("\n!!! FAILSAFE: hoek te groot (roll=%.1f pitch=%.1f) - MOTOREN UIT !!!\n",
                   roll_level, pitch_level);
        } else if (g_flight_time > FAILSAFE_TIMEOUT) {
            motors_killed = 1;
            printf("\n!!! FAILSAFE: vluchttijd van %.0f s voorbij - MOTOREN UIT !!!\n",
                   (double)FAILSAFE_TIMEOUT);
        }
    }

    if (motors_killed) {
        motors_send(0, 0, 0, 0);
        return 0;
    }

    float roll_cmd  = PID_Angle_UpdateRate(&pid_roll,  roll_setpoint, roll_level,  f_gx, dt);
    float pitch_cmd = PID_Angle_UpdateRate(&pid_pitch, 0.0f,          pitch_level, f_gy, dt);

    Update_Motors(base_throttle, roll_cmd, pitch_cmd, 0.0f);
    return 1;
}

/* Nieuw (bestond niet in DshotFCMetMPU6050 - dat project draaide 1 vaste
 * testvlucht i.p.v. losse, tijdgebonden vluchtplan-commando's). Houdt
 * base_throttle/roll_setpoint duration_ms lang aan, regelstap per regelstap.
 * Retourneert 0 als de failsafe onderweg ingreep. */
static uint8_t run_for_ms(int base_throttle, float roll_setpoint, uint32_t duration_ms,
                           uint32_t (*get_tick_fn)(void), void (*delay_fn)(uint32_t),
                           uint32_t loop_dt_ms)
{
    uint32_t t_start = get_tick_fn();
    while ((get_tick_fn() - t_start) < duration_ms) {
        if (!run_control_tick(base_throttle, roll_setpoint)) return 0;
        delay_fn(loop_dt_ms);
    }
    return 1;
}

/* Deels overgenomen, deels nieuw. Het stuk onder THROTTLE_MIN (alle 4 motoren
 * gelijk en ongestabiliseerd laten opspinnen vanuit 48, in kleine stapjes) is
 * de soft-start-ramp uit DshotFCMetMPU6050/Core/Src/main.c (daar inline in
 * main(), hier in een functie). Nieuw is dat dit herbruikbaar is voor een
 * willekeurig doel i.p.v. altijd naar de vaste THROTTLE_BASE, en het stuk
 * bóven THROTTLE_MIN (gestabiliseerd verder opbouwen naar `target`), nodig
 * omdat een vluchtplan tussen verschillende Throttle-commando's kan wisselen. */
static void ramp_base_throttle(int target, uint32_t (*get_tick_fn)(void),
                                void (*delay_fn)(uint32_t), uint32_t loop_dt_ms)
{
    (void)get_tick_fn;
    if (motors_killed) return;

    if (current_base_throttle < THROTTLE_MIN) {
        int first_target = (target < THROTTLE_MIN) ? target : THROTTLE_MIN;
        const int steps = 100;
        for (int i = 0; i <= steps; i++) {
            int t = 48 + ((first_target - 48) * i) / steps; /* 48 = laagste geldige gaswaarde */
            if (t < 0) t = 0;
            motors_send(t, t, t, t);
            delay_fn(loop_dt_ms);
        }
        current_base_throttle = first_target;
    }

    const int steps = 50;
    int start = current_base_throttle;
    for (int i = 0; i <= steps && !motors_killed; i++) {
        current_base_throttle = start + ((target - start) * i) / steps;
        if (!run_control_tick(current_base_throttle, 0.0f)) return;
        delay_fn(loop_dt_ms);
    }
    current_base_throttle = target;
}

/* Nieuw (bestond niet in DshotFCMetMPU6050 - dat project had geen Land-commando,
 * enkel een failsafe die alles direct op 0 zette). Bouwt de throttle rustig af
 * naar 0 en zet de motoren daarna definitief uit. */
static void land_and_stop(uint32_t (*get_tick_fn)(void), void (*delay_fn)(uint32_t),
                           uint32_t loop_dt_ms)
{
    ramp_base_throttle(THROTTLE_MIN, get_tick_fn, delay_fn, loop_dt_ms);
    for (int i = 0; i < 20; i++) {
        motors_send(0, 0, 0, 0);
        delay_fn(loop_dt_ms);
    }
    current_base_throttle = 0;
}

/* Grotendeels overgenomen uit DshotFCMetMPU6050/Core/Src/main.c: dit is de
 * initialisatie-sectie van main() daar (staat in dat bestand niet in een eigen
 * functie, maar inline vóór de while(1)-lus), hier verzameld in FlightControl_Init()
 * zodat flightrun.c 'm apart kan aanroepen vóór FlightControl_Run(). */
int FlightControl_Init(void)
{
    motors_killed = 0;
    current_base_throttle = 0;

    DWT_Init();
    dshot_init();
    MPU6050_Init();

    /* Overgenomen: schone throttle-0 stroom vóór de ESC's protocol-detectie doen. */
    printf("flightcontrol: DShot-signaal starten...\n");
    for (int i = 0; i < 100; i++) {
        motors_send(0, 0, 0, 0);
        HAL_Delay(20);
    }

    /* Overgenomen: gyro-kalibratie (MPU6050_Calibrate_Gyro). */
    printf("flightcontrol: gyro kalibreren, plaat stil houden...\n");
    MPU6050_Calibrate_Gyro(GYRO_CALIB_SAMPLES);
    printf("flightcontrol: gyro gekalibreerd.\n");

    /* Overgenomen: level-kalibratie (montage-offset van de IMU meten). */
    printf("flightcontrol: level kalibreren, frame WATERPAS en stil houden...\n");
    {
        float sum_r = 0.0f, sum_p = 0.0f;
        const int n = LEVEL_CALIB_SAMPLES;
        for (int i = 0; i < n; i++) {
            float ax, ay, az, r, p;
            float fax, fay, faz, dummy_gx, dummy_gy;
            MPU6050_Read_Accel(&ax, &ay, &az);
            imu_to_frame(ax, ay, az, 0.0f, 0.0f, &fax, &fay, &faz, &dummy_gx, &dummy_gy);
            Attitude_AccelAngles(fax, fay, faz, &r, &p);
            sum_r += r;
            sum_p += p;
            HAL_Delay(2);
        }
        roll_offset  = sum_r / (float)n;
        pitch_offset = sum_p / (float)n;
        g_fused_roll  = roll_offset;
        g_fused_pitch = pitch_offset;
    }
    printf("flightcontrol: level offset roll=%.2f pitch=%.2f graden\n", roll_offset, pitch_offset);

    /* Overgenomen: PID_Angle_Init-aanroepen met de gains uit fc_config.h. */
    PID_Angle_Init(&pid_roll,  PID_KP, PID_KI, PID_KD, PID_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT);
    PID_Angle_Init(&pid_pitch, PID_KP, PID_KI, PID_KD, PID_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT);
    pid_roll.d_cutoff_hz  = PID_D_CUTOFF_HZ;
    pid_pitch.d_cutoff_hz = PID_D_CUTOFF_HZ;

    /* Overgenomen: ESC-arming, de ESC accepteert pas gas na een langere periode throttle=0. */
    printf("flightcontrol: armen (throttle 0)...\n");
    for (int i = 0; i < 250; i++) {
        motors_send(0, 0, 0, 0);
        HAL_Delay(20);
    }
    printf("flightcontrol: armed.\n");

    PID_Angle_Reset(&pid_roll);
    PID_Angle_Reset(&pid_pitch);

    g_cyc_prev = DWT->CYCCNT;
    return 0;
}

/* Volledig nieuw - bestond niet in DshotFCMetMPU6050. Dat project had geen
 * vluchtplan, enkel 1 vaste testvlucht hardgecodeerd in main(). Dit is de
 * vluchtplan-orchestratie: leest FlightCmd_t's uit flightplan.h en vertaalt
 * elk commando naar aanroepen van de functies hierboven. */
void FlightControl_Run(FlightPlan_t *plan, uint32_t (*get_tick_fn)(void),
                        void (*delay_fn)(uint32_t), uint32_t loop_dt_ms)
{
    g_flight_time = 0.0f;
    FlightPlan_Reset(plan);

    while (FlightPlan_HasNext(plan) && !motors_killed) {
        FlightCmd_t *cmd = FlightPlan_Next(plan);

        switch (cmd->type) {

        case CMD_THROTTLE: {
            float percent = cmd->param[0];
            if (percent < 0.0f)   percent = 0.0f;
            if (percent > 100.0f) percent = 100.0f;
            int target = THROTTLE_MIN + (int)((percent / 100.0f) * (THROTTLE_MAX - THROTTLE_MIN));
            uint32_t duration_ms = (uint32_t)cmd->param[1];
            printf("flightcontrol: Throttle %.0f%% gedurende %lums\n",
                   (double)percent, (unsigned long)duration_ms);
            ramp_base_throttle(target, get_tick_fn, delay_fn, loop_dt_ms);
            run_for_ms(current_base_throttle, 0.0f, duration_ms, get_tick_fn, delay_fn, loop_dt_ms);
            break;
        }

        case CMD_HOVER: {
            uint32_t duration_ms = (uint32_t)cmd->param[0];
            printf("flightcontrol: Hover %lums (best effort, geen hoogteregeling zonder barometer)\n",
                   (unsigned long)duration_ms);
            if (current_base_throttle < THROTTLE_MIN) {
                ramp_base_throttle(THROTTLE_BASE, get_tick_fn, delay_fn, loop_dt_ms);
            }
            run_for_ms(current_base_throttle, 0.0f, duration_ms, get_tick_fn, delay_fn, loop_dt_ms);
            break;
        }

        case CMD_LEFT:
        case CMD_RIGHT: {
            /* param[0] wordt hier als duur in ms genomen (zelfde eenheid als
             * Hover). Teken van de tilt is een aanname, zie LEFT_RIGHT_TILT_DEG. */
            float tilt = (cmd->type == CMD_LEFT) ? -LEFT_RIGHT_TILT_DEG : LEFT_RIGHT_TILT_DEG;
            uint32_t duration_ms = (uint32_t)cmd->param[0];
            printf("flightcontrol: %s gedurende %lums\n",
                   (cmd->type == CMD_LEFT) ? "Left" : "Right", (unsigned long)duration_ms);
            if (current_base_throttle < THROTTLE_MIN) {
                ramp_base_throttle(THROTTLE_BASE, get_tick_fn, delay_fn, loop_dt_ms);
            }
            run_for_ms(current_base_throttle, tilt, duration_ms, get_tick_fn, delay_fn, loop_dt_ms);
            break;
        }

        case CMD_LAND:
            printf("flightcontrol: Land\n");
            land_and_stop(get_tick_fn, delay_fn, loop_dt_ms);
            return; /* vlucht is voorbij, geen commando's meer na Land */

        case CMD_RELATIVE_HEIGHT:
        case CMD_ABSOLUTE_HEIGHT:
        case CMD_MOVE:
        default:
            printf("flightcontrol: commando nog niet ondersteund (barometer/GPS nodig), overgeslagen\n");
            break;
        }
    }

    if (motors_killed) {
        printf("flightcontrol: failsafe actief, vluchtplan afgebroken\n");
        motors_send(0, 0, 0, 0);
    } else {
        /* Veiligheid: eindigt het plan zonder Land-commando, toch netjes landen. */
        printf("flightcontrol: einde vluchtplan zonder Land-commando, land alsnog\n");
        land_and_stop(get_tick_fn, delay_fn, loop_dt_ms);
    }
}

/* Overgenomen uit DshotFCMetMPU6050/Core/Src/main.c, functie
 * HAL_TIM_PWM_PulseFinishedCallback() (ongewijzigd). */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        dshot_dma_complete = 1; /* dshot-lijn is weer vrij voor een nieuw frame */
    }
}
