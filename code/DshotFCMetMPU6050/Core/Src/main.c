/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
//#include "BMP384.h"
//#include "BMM350.h"
//#include "BMI330.h"
#include "MPU6050.h"
#include "attitude.h"
#include "AnglePID.h"
#include "dshot.h"
//#include "pid_regulator.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_up_ch3;
DMA_HandleTypeDef hdma_tim4_ch1;
DMA_HandleTypeDef hdma_tim4_ch2;

UART_HandleTypeDef huart8;

/* USER CODE BEGIN PV */
/*
PID_Handle_t struct_PidRateRoll;
PID_Handle_t struct_PidRatePitch;
PID_Handle_t struct_PidRateYaw;
*/
//BMI330_Frame current_data;
//BMM350_MagData BMM350_Data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART8_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float g_hoogte;
float g_druk;
float g_hoek;
float g_pitch;
float g_roll;
float g_temp;
int g_acc;
int g_gyr;
float alpha = 0.98; // hoe hoger hoe meer de acc gaat doortellen
float dt = 0.02;   // voor 50hz
float gefilterde_hoogte = 0;
float verticale_snelheid = 0;
int g_new_bmm_data = 0;
int g_new_bmi_data = 0;

int _write(int file, char *ptr, int len) {
	for(int i = 0; i < len; i++){
		if(ptr[i]=='\n'){
			HAL_UART_Transmit(&huart8, (uint8_t*)"\r", 1, HAL_MAX_DELAY);
		}
		HAL_UART_Transmit(&huart8, (uint8_t*)&ptr[i], 1, HAL_MAX_DELAY);
	}
    return len;
}
/*
void Update_Altitude_Filter(float baro_hoogte, float acc_z_m_s2) {
    verticale_snelheid += acc_z_m_s2 * dt;
    float voorspelde_hoogte = gefilterde_hoogte + (verticale_snelheid * dt);
    gefilterde_hoogte = (alpha * voorspelde_hoogte) + ((1 - alpha) * baro_hoogte);
}
*/
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;

// gefuseerde hoeken (graden), worden door Attitude_Update bijgewerkt
float fused_roll  = 0.0f;
float fused_pitch = 0.0f;

// Angle-mode PID's: setpoint is altijd 0 graden (waterpas). Output is een
// throttle-correctie die de mixer per motor optelt/aftrekt.
// Start voorzichtig: eerst Ki=0, Kd klein, alleen Kp opbouwen tot hij actief
// terugregelt zonder heftig te overshooten/oscilleren. Pas daarna Kd/Ki bijstellen.
PID_Angle_t pid_roll;
PID_Angle_t pid_pitch;

// Vaste montage-offset van de IMU t.o.v. het frame (graden). De sensor zit niet
// perfect waterpas op de plaat, dus "frame waterpas" != "sensor leest 0".
// Deze offset wordt bij het opstarten gemeten en van de gefuseerde hoek afgetrokken.
float roll_offset  = 0.0f;
float pitch_offset = 0.0f;

// --- microseconden-timer via DWT cycle counter, voor een nauwkeurige dt ---
static void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;       // trace enablen
    // Cortex-M7 vereist een software-unlock van de DWT voor CYCCNT telt.
    // Zonder deze regel blijft CYCCNT op 0 -> dt zou altijd 0 zijn.
    *((volatile uint32_t*)0xE0001FB0) = 0xC5ACCE55;       // DWT->LAR unlock-key
    DWT->CYCCNT = 0;                                      // teller nul
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;                 // cycle counter aan
}
static inline uint32_t micros(void) {
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

// dshot
volatile uint8_t dshot_dma_complete = 1;

// tilt -> throttle mapping (test-instellingen, pas gerust aan)
// LET OP: BASE moet tussen MIN en MAX liggen, met genoeg ruimte naar beide kanten
// voor de PID-correcties. Staat BASE onder MIN, dan worden alle motoren naar MIN
// geklemd en zie je geen enkel verschil meer tussen de motoren.
// Stuwkracht loopt KWADRATISCH met toerental: op lage basis-throttle levert een
// klein throttle-verschil nauwelijks koppel. Voor echte stuurautoriteit moet de
// basis richting het zweefpunt liggen (grofweg 40-50% = 800-1000).
#define THROTTLE_BASE     600   // throttle bij vlak liggen (moet > MIN zijn!)
#define THROTTLE_MIN      200   // absoluut minimum (0-47 = dshot commando's) en 48-149 is schokkend = te lage bemf
#define THROTTLE_MAX      1200  // veiligheidsgrens voor deze test, met prop erop!

// --- FAILSAFE ---
// Slaat de motoren af als de drone te ver doorslaat (bv. door een verkeerd
// mixer-teken -> positieve terugkoppeling -> op hol slaan) of als de testtijd
// om is. Eenmaal afgeslagen blijven de motoren uit tot je opnieuw opstart.
// LET OP: pitch kan wiskundig maar tot +/-90 graden komen (atan2 met sqrt) en
// roll tot +/-180. Een grens van 200 betekent dus in de praktijk: hoek-failsafe
// staat UIT. Handig tijdens het uitzoeken op de teststaaf, want dan mag hij
// gerust omslaan en ondersteboven hangen. Zet dit terug naar ~45 zodra je
// echt gaat tunen, en zeker voordat er propellers op gaan.
#define FAILSAFE_ANGLE    80.0f // graden; daarboven direct motoren uit
#define FAILSAFE_TIMEOUT  30.0f  // seconden; automatische stop na deze tijd
static uint8_t motors_killed = 0;

// Zet 1 frame klaar voor alle 4 de motoren en verstuur het (met handshake op de
// vorige DMA). throttle-waarden worden hier hard begrensd op de dshot-range 0..2047.
static void motors_send(int t1, int t2, int t3, int t4) {
    int t[4] = { t1, t2, t3, t4 };
    for (int i = 0; i < 4; i++) {
        if (t[i] < 0)    t[i] = 0;
        if (t[i] > 2047) t[i] = 2047;
        // 1..47 zijn GEEN gas maar dshot-commando's (piepen, draairichting,
        // settings opslaan). Nooit per ongeluk versturen: 0 = stop, 48+ = gas.
        if (t[i] > 0 && t[i] < 48) t[i] = 0;
    }
    while (dshot_dma_complete == 0) { }   // wacht tot vorig frame verstuurd is
    dshot_dma_complete = 0;
    update_motor_buffer(0, (uint16_t)t[0], 0); // M1
    update_motor_buffer(1, (uint16_t)t[1], 0); // M2
    update_motor_buffer(2, (uint16_t)t[2], 0); // M3
    update_motor_buffer(3, (uint16_t)t[3], 0); // M4
    send_dshot();
}

// Houdt een vaste throttle-combinatie 'seconds' seconden aan.
static void hold_motors(int t1, int t2, int t3, int t4, float seconds) {
    int frames = (int)(seconds * 50.0f); // ~50 frames/s bij HAL_Delay(20)
    for (int f = 0; f < frames; f++) {
        motors_send(t1, t2, t3, t4);
        HAL_Delay(20);
    }
}

// --- DSHOT-COMMANDO'S (waarden 1..47, geen gas) ---
#define DSHOT_CMD_BEEP1                     1
#define DSHOT_CMD_SPIN_DIRECTION_1          7   // oudere variant: normaal
#define DSHOT_CMD_SPIN_DIRECTION_2          8   // oudere variant: omgekeerd
#define DSHOT_CMD_SAVE_SETTINGS            12
#define DSHOT_CMD_SPIN_DIRECTION_NORMAL    20
#define DSHOT_CMD_SPIN_DIRECTION_REVERSED  21

// Op 1: gebruik commando 7/8 i.p.v. 20/21. Oudere BLHeli_S-firmware kent
// alleen die eerste twee. Werkt 20/21 niet, probeer dan deze.
#define USE_OLD_DIRECTION_CMDS  0

// Stuurt een DShot-commando naar 1 motor. De overige motoren krijgen throttle 0.
// Let op: dit vult de buffer RECHTSTREEKS, want motors_send() klemt 1..47 naar 0.
// De telemetrie-bit moet 1 zijn, anders herkent de ESC het niet als commando.
// Commando's moeten herhaald worden (BLHeli wil er meestal 6 tot 10) en de ESC
// mag op dat moment NIET draaien.
static void dshot_command(int idx, uint16_t cmd, int repeats) {
    for (int i = 0; i < repeats; i++) {
        while (dshot_dma_complete == 0) { }
        dshot_dma_complete = 0;
        for (int m = 0; m < 4; m++) {
            if (m == idx) {
                update_motor_buffer(m, cmd, 1); // telemetrie-bit aan
            } else {
                update_motor_buffer(m, 0, 0);
            }
        }
        send_dshot();
        HAL_Delay(2);
    }
}

// Zet de draairichting van 1 motor en slaat die op in de ESC.
// reversed = 1 -> omgekeerd, 0 -> normaal.
static void dshot_set_direction(int idx, int reversed) {
#if USE_OLD_DIRECTION_CMDS
    uint16_t cmd = reversed ? DSHOT_CMD_SPIN_DIRECTION_2
                            : DSHOT_CMD_SPIN_DIRECTION_1;
#else
    uint16_t cmd = reversed ? DSHOT_CMD_SPIN_DIRECTION_REVERSED
                            : DSHOT_CMD_SPIN_DIRECTION_NORMAL;
#endif
    printf("M%d -> richting %s (cmd %u)\n", idx + 1,
           reversed ? "OMGEKEERD" : "normaal", (unsigned)cmd);

    dshot_command(idx, cmd, 10);                       // richting zetten (min. 6x)
    HAL_Delay(50);
    dshot_command(idx, DSHOT_CMD_SAVE_SETTINGS, 10);   // permanent opslaan (min. 6x)
    HAL_Delay(500);                                    // ESC bevestigt met piepjes
}

// DIAGNOSE: laat elke ESC om beurten piepen. Hoor je de piepjes, dan komen je
// DShot-commando's aan en werkt het mechanisme; dan ligt een mislukte
// richtingswissel aan de ESC-firmware. Hoor je NIETS, dan worden commando's
// helemaal niet herkend en heeft de richtingswissel ook geen kans.
// Beep-commando's moeten minstens 260 ms uit elkaar staan.
static void dshot_beep_test(void) {
    printf("\n=== PIEPTEST: elke motor hoort te piepen ===\n");
    for (int m = 0; m < 4; m++) {
        printf("  M%d piep...\n", m + 1);
        dshot_command(m, DSHOT_CMD_BEEP1, 10);
        HAL_Delay(400); // beep duurt ~260 ms
    }
    printf("=== Pieptest klaar ===\n\n");
}

// Draait elke motor 1 voor 1 op 'spin' throttle (~1,5 s per motor), de rest uit.
// Zo kun je controleren of M1..M4 op de juiste plek zitten en draairichting klopt.
static void test_motors_sequence(uint16_t spin) {
    for (int m = 0; m < 4; m++) {
        printf("Test motor M%d...\n", m + 1);
        int t[4] = { 0, 0, 0, 0 };
        t[m] = spin;
        hold_motors(t[0], t[1], t[2], t[3], 1.5f);
    }
    hold_motors(0, 0, 0, 0, 0.5f); // alles weer uit
}

// IDENTIFICATIE-TEST: draait TELKENS 3 MOTOREN en laat er 1 STILSTAAN.
// De stilstaande motor is degene die op dat moment wordt aangekondigd - veel
// makkelijker te zien/voelen dan 1 draaiende motor tussen 3 stille.
// Elke stap duurt 5 s, met 2 s alles-uit ertussen als duidelijke scheiding.
static void test_motors_identify(uint16_t spin) {
    printf("\n=== IDENTIFICATIE: de STILSTAANDE motor is de aangekondigde ===\n");
    for (int m = 0; m < 4; m++) {
        printf("\n>>> M%d STAAT STIL (de andere 3 draaien) - kijk welke stilstaat!\n", m + 1);
        int t[4] = { spin, spin, spin, spin };
        t[m] = 0; // deze staat stil
        hold_motors(t[0], t[1], t[2], t[3], 5.0f);

        printf("    ...alles uit...\n");
        hold_motors(0, 0, 0, 0, 2.0f);
    }
    printf("\n=== Identificatie klaar ===\n\n");
}

// RAMP-TEST voor 1 motor: bouwt langzaam op van t_start naar t_end en print
// elke stap. Zo vind je exact de throttle waarbij het misgaat, en na een reset
// zie je in de terminal tot hoever hij kwam. idx is 0..3 (M1..M4).
static void test_motor_ramp(int idx, int t_start, int t_end, int steps) {
    printf("\n>>> RAMP-TEST M%d: %d -> %d\n", idx + 1, t_start, t_end);
    for (int i = 0; i <= steps; i++) {
        int t = t_start + ((t_end - t_start) * i) / steps;
        int m[4] = { 0, 0, 0, 0 };
        m[idx] = t;
        motors_send(m[0], m[1], m[2], m[3]);
        printf("  M%d throttle = %d\n", idx + 1, t);
        HAL_Delay(50);
    }
    hold_motors(0, 0, 0, 0, 1.0f);
}

// PAAR-TEST: draait 2 gekozen motoren tegelijk, de andere 2 staan stil.
// Gebruik dit om te bevestigen dat je voor-paar / achter-paar klopt.
// Indices zijn 0..3 (dus M1=0, M2=1, M3=2, M4=3).
static void test_motor_pair(int idx_a, int idx_b, uint16_t spin, const char *label) {
    printf("\n>>> PAAR-TEST: %s (M%d + M%d draaien)\n", label, idx_a + 1, idx_b + 1);
    int t[4] = { 0, 0, 0, 0 };
    t[idx_a] = spin;
    t[idx_b] = spin;
    hold_motors(t[0], t[1], t[2], t[3], 5.0f);
    hold_motors(0, 0, 0, 0, 2.0f);
}

// Motor-index -> fysieke positie (buffer-index = motor_id in send_dshot).
// Gemeten met test_motors_identify(): M1 = links-onder, daarna met de klok mee.
// Van boven gezien, neus naar voren:
//
//        VOOR
//    M2 -------- M3
//     |          |
//     |          |     LINKS = M1, M2      RECHTS = M3, M4
//     |          |     VOOR  = M2, M3      ACHTER = M1, M4
//    M1 -------- M4
//        ACHTER
//
//   index 0 = M1 (TIM2_CH1, PA0)   -> ACHTER-LINKS
//   index 1 = M2 (TIM2_CH3, PA2)   -> VOOR-LINKS
//   index 2 = M3 (TIM4_CH2, PD13)  -> VOOR-RECHTS
//   index 3 = M4 (TIM4_CH1, PD12)  -> ACHTER-RECHTS
//
// REGEL OM DE TEKENS TE CONTROLEREN:
// de kant die naar BENEDEN kantelt moet HARDER gaan draaien (die duwt zichzelf
// terug omhoog). Gaat juist de omhoog-kant harder draaien, keer dan alle vier
// de tekens van die as om.
// Beide assen actief. Elke motor krijgt nu de som van twee correcties, dus de
// uitslagen kunnen groter worden dan bij het testen van een enkele as.
static const int pitch_sign[4] = { +1, -1, -1, +1 }; // voor(M2,M3) = -, achter(M1,M4) = +
static const int roll_sign[4]  = { +1, +1, -1, -1 }; // links(M1,M2) = +, rechts(M3,M4) = -
static const int yaw_sign[4]   = {  0,  0,  0,  0 }; // pas invullen bij yaw-regeling

// laatst verstuurde throttle per motor, puur om te kunnen meelezen tijdens testen
static int last_motor[4] = { 0, 0, 0, 0 };

// --- IMU-ORIENTATIE ---
// Het moduletje zit mogelijk gedraaid op het frame. Test: kantel de NEUS omlaag
// en kijk welke waarde verandert. Hoort 'pitch' te zijn. Verandert 'roll'
// i.p.v. pitch, dan staat de sensor 90 graden gedraaid -> zet deze op 1.
// Op 1: motoren blijven UIT en we printen alleen de hoeken. Gebruik dit om
// veilig te controleren of de IMU-oriëntatie klopt, VOOR je iets laat draaien.
// Zet terug op 0 zodra pitch/roll de juiste kant op bewegen.
#define ANGLE_CHECK_ONLY 0

// Op 1: bij het opstarten wordt de draairichting van M2 en M4 omgekeerd en in
// de ESC opgeslagen. Eenmalig gebruiken, daarna terug op 0. PROPS ERAF.
#define SET_MOTOR_DIRECTIONS 0

#define IMU_ROTATED_90   1  // 0 = sensor recht gemonteerd, 1 = 90 graden gedraaid
// Gemeten level-offset roll = -172 graden -> de Z-as wijst omlaag, de sensor
// staat dus ONDERSTEBOVEN. Met deze vlag op 1 draaien we hem 180 graden terug.
// Klopt het: na kalibratie moet de offset klein zijn (een paar graden), niet ~180.
#define IMU_UPSIDE_DOWN  1  // 0 = normaal, 1 = sensor ondersteboven gemonteerd

static inline void imu_to_frame(float ax, float ay, float az, float gx, float gy,
                                 float *f_ax, float *f_ay, float *f_az,
                                 float *f_gx, float *f_gy) {
    float x = ax, y = ay, z = az, p = gx, q = gy;

#if IMU_UPSIDE_DOWN
    // 180 graden om de X-as: Y en Z keren om (Z weer omhoog)
    y = -y;  z = -z;  q = -q;
#endif

#if IMU_ROTATED_90
    // 90 graden om de Z-as: (x,y) -> (y,-x)
    float tx = y, ty = -x;
    float tp = q, tq = -p;
    x = tx;  y = ty;  p = tp;  q = tq;
#endif

    *f_ax = x;  *f_ay = y;  *f_az = z;
    *f_gx = p;  *f_gy = q;
}

// Mixer: basis-throttle + correcties per as, met tekens uit de tabellen hierboven.
// Correcties in "throttle-eenheden". Verstuurt meteen 1 frame naar alle 4 motoren.
static void Update_Motors(int base, float roll_cmd, float pitch_cmd, float yaw_cmd) {
    int mc[4];
    for (int i = 0; i < 4; i++) {
        int v = base
              + (int)(pitch_sign[i] * pitch_cmd)
              + (int)(roll_sign[i]  * roll_cmd)
              + (int)(yaw_sign[i]   * yaw_cmd);
        if (v < THROTTLE_MIN) v = THROTTLE_MIN; // motoren blijven draaien
        if (v > THROTTLE_MAX) v = THROTTLE_MAX; // veiligheidsplafond
        mc[i] = v;
        last_motor[i] = v; // voor de debug-print
    }
    motors_send(mc[0], mc[1], mc[2], mc[3]);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
 /* Init_RateLoops();
  BMP384_Init();
  PID_HandleInit(struct_PidRateRoll);
  PID_HandleInit(struct_PidRatePitch);
  PID_HandleInit(struct_PidRateYaw);
  */
  //BMM350_Init();
  //BMI330_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_UART8_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  printf("Test1\n");

  // Waarom is de MCU (opnieuw) opgestart? BOR/POR = spanning weggezakt
  // (brownout) -> voedingsprobleem. PIN = resetknop/debugger. SFT = software.
  printf("--- RESET-OORZAAK: ");
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))  printf("BROWNOUT (BOR) - voeding zakt weg! ");
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))  printf("Power-on reset (POR) ");
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))  printf("Pin/debugger reset ");
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))  printf("Software reset ");
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) printf("Watchdog (IWDG) ");
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) printf("Watchdog (WWDG) ");
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) printf("Low-power reset ");
  printf("---\n");
  __HAL_RCC_CLEAR_RESET_FLAGS(); // wissen, anders blijven ze staan

  DWT_Init();       // microseconden-timer voor dt-meting
  dshot_init();     // CCR's op 0 + buffers geldig, voor het eerste frame uitgaat
  MPU6050_Init();

  // Meteen een schone stroom throttle-0 frames sturen, VOOR de kalibraties.
  // De ESC doet zijn protocol-detectie op de eerste frames die hij ziet; die
  // moeten dus geldig zijn en niet onderbroken worden.
  printf("DShot-signaal starten...\n");
  for (int i = 0; i < 100; i++) {
      motors_send(0, 0, 0, 0);
      HAL_Delay(20);
  }

  // gyro-offset wegkalibreren: plaat moet hierbij STIL liggen
  printf("Gyro kalibreren, plaat stil houden...\n");
  MPU6050_Calibrate_Gyro(1000);
  printf("Gyro gekalibreerd.\n");

  // Montage-offset van de IMU meten: het FRAME moet hierbij waterpas staan.
  // Wat de sensor dan leest is puur de scheefstand van het moduletje, en die
  // trekken we er voortaan af zodat "frame waterpas" ook echt 0 graden geeft.
  printf("Level kalibreren, frame WATERPAS en stil houden...\n");
  {
      float sum_r = 0.0f, sum_p = 0.0f;
      const int n = 500;
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

      // filter alvast op de gemeten stand zetten, scheelt insteltijd
      fused_roll  = roll_offset;
      fused_pitch = pitch_offset;
  }
  printf("Level offset: roll=%.2f  pitch=%.2f graden\n", roll_offset, pitch_offset);

  // Angle-PID's initialiseren: setpoint is 0 graden, gains hieronder zijn een
  // VOORZICHTIG startpunt. Begin met alleen Kp (Ki/Kd op 0), verhoog stap voor
  // stap tot de plaat actief terugregelt zonder hard te oscilleren. Voeg dan
  // pas een klein beetje Kd toe om overshoot te dempen, en tot slot een kleine
  // Ki om een blijvende scheve stand weg te regelen.
  // Argumenten: kp, ki, kd, integral_limit, output_limit
  // output_limit 300 = de PID mag een motor 300 boven/onder de basis zetten.
  // Stond op 100, dat was te weinig om het frame echt terug te duwen.
  //                        kp     ki     kd    i-limit  out-limit
  PID_Angle_Init(&pid_roll,  9.0f, 0.05f, 0.8f, 100.0f, 500.0f);
  PID_Angle_Init(&pid_pitch, 9.0f, 0.05f, 0.8f, 100.0f, 500.0f);

  // DShot arming: de ESC accepteert pas gas na een langere periode throttle=0.
  // Eerst wachten tot de ESC zelf klaar is met opstarten (piepjes), daarna een
  // ruime nul-periode. Te kort armen = motoren negeren het eerste gascommando,
  // precies het gedrag waarbij de eerste test niets deed.
  HAL_Delay(3000);
  printf("Start Arming (5 s throttle 0)...\n");
  for (int i = 0; i < 250; i++) {
      motors_send(0, 0, 0, 0); // alle 4 motoren throttle 0
      HAL_Delay(20);
  }
  printf("Armed!\n");

  // EENMALIG: draairichting van M2 en M4 omkeren en opslaan in de ESC.
  // Zet SET_MOTOR_DIRECTIONS op 1, flash, laat draaien, zet daarna terug op 0.
  // De instelling blijft in de ESC staan, ook na spanningloos maken.
  // Doe dit met de PROPELLERS ERAF.
#if SET_MOTOR_DIRECTIONS
  // Eerst piepen: hoor je dit niet, dan komen commando's sowieso niet aan
  // en heeft de richtingswissel hieronder geen enkele kans.
  dshot_beep_test();

  printf("\n=== DRAAIRICHTING INSTELLEN (props eraf!) ===\n");
  dshot_set_direction(0, 0); // M1 normaal
  dshot_set_direction(2, 0); // M3 normaal
  dshot_set_direction(1, 1); // M2 omgekeerd
  dshot_set_direction(3, 1); // M4 omgekeerd
  printf("=== Klaar. Zet SET_MOTOR_DIRECTIONS terug op 0 ===\n\n");
#endif

  // IDENTIFICATIE: telkens draaien er 3 en staat er 1 stil. De stilstaande is
  // degene die geprint wordt -> zo zie je welke fysieke motor M1..M4 is.
  // Zet deze regel in commentaar zodra je de indeling kent.
  // VERWIJDER PROPELLERS bij deze test!
  //test_motors_identify(THROTTLE_BASE);

  // Zodra je weet welke voor/achter zitten, kun je hiermee je paren bevestigen:
  // test_motor_pair(0, 3, THROTTLE_BASE, "vermoedelijk VOOR");
  // test_motor_pair(1, 2, THROTTLE_BASE, "vermoedelijk ACHTER");

  //printf("Motortest klaar, start PID-regeling.\n");

  // SOFT-START: throttle rustig opbouwen van 0 naar THROTTLE_BASE.
  // Een sprong van 0 naar volle basis-throttle laat sommige ESC's weigeren of
  // uit sync lopen. De identificatie-test deed dit vroeger onbedoeld voor ons.
  // Ramp begint bij 48 (laagste geldige gaswaarde), NIET bij 0: alles tussen
  // 1 en 47 zijn dshot-commando's en geen gas.
  // Ramp over ~4 s i.p.v. 1 s. Vier motoren die samen snel opspinnen trekken een
  // flinke stroompiek; zakt de accuspanning daardoor te ver weg, dan reset de
  // STM32 (brownout) en begint alles opnieuw. Langzaam opbouwen beperkt die piek
  // en laat je in de terminal zien BIJ WELKE throttle het misgaat.
  printf("Soft-start naar throttle %d...\n", THROTTLE_BASE);
  for (int i = 0; i <= 200; i++) {
      int t = 48 + ((THROTTLE_BASE - 48) * i) / 200;
      motors_send(t, t, t, t);
      if ((i % 20) == 0) printf("  ramp throttle = %d\n", t);
      HAL_Delay(20);
  }
  printf("Soft-start klaar, start PID-regeling.\n");

  // integraal/vorige-fout op 0: opgebouwde ruis uit arming/test mag niet meetellen
  PID_Angle_Reset(&pid_roll);
  PID_Angle_Reset(&pid_pitch);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t cyc_prev = DWT->CYCCNT; // ruwe cycle-teller, starttijd voor de eerste dt
  float test_time = 0.0f;          // looptijd sinds start regeling, voor de failsafe-timeout
  while (1)
  {
	  MPU6050_Read_Accel(&accel_x, &accel_y, &accel_z);
	  MPU6050_Read_Gyro(&gyro_x, &gyro_y, &gyro_z);

	  // werkelijke tijd sinds de vorige iteratie meten (in seconden).
	  // ruwe 32-bit cycle-teller -> unsigned aftrek is wrap-veilig
	  uint32_t cyc_now = DWT->CYCCNT;
	  float dt_meas = (uint32_t)(cyc_now - cyc_prev) / (float)SystemCoreClock;
	  cyc_prev = cyc_now;

	  // sensor-assen naar frame-assen mappen (zie IMU_ROTATED_90 bovenaan)
	  float f_ax, f_ay, f_az, f_gx, f_gy;
	  imu_to_frame(accel_x, accel_y, accel_z, gyro_x, gyro_y,
	               &f_ax, &f_ay, &f_az, &f_gx, &f_gy);

	  // sensorfusie: complementair filter blend gyro (snel) met accel (stabiel)
	  Attitude_Update(f_ax, f_ay, f_az,
	                  f_gx, f_gy, gyro_z,
	                  dt_meas, &fused_roll, &fused_pitch);

	  // montage-offset eraf: dit zijn de hoeken van het FRAME, niet van de sensor
	  float roll_level  = fused_roll  - roll_offset;
	  float pitch_level = fused_pitch - pitch_offset;

	  // FAILSAFE: te ver doorgeslagen of testtijd om -> motoren definitief uit.
	  // Dit vangt o.a. een verkeerd mixer-teken op, waarbij de regeling zichzelf
	  // opjaagt in plaats van terugregelt.
	  test_time += dt_meas;
	  if (!motors_killed) {
	      // Beide assen bewaken: de as die je test kan doorslaan, en de andere
	      // hoort juist vlak te blijven - loopt die weg, dan is er iets mis.
	      if (roll_level  >  FAILSAFE_ANGLE || roll_level  < -FAILSAFE_ANGLE ||
	          pitch_level >  FAILSAFE_ANGLE || pitch_level < -FAILSAFE_ANGLE) {
	          motors_killed = 1;
	          printf("\n!!! FAILSAFE: hoek te groot (roll=%.1f pitch=%.1f) - MOTOREN UIT !!!\n",
	                 roll_level, pitch_level);
	      } else if (test_time > FAILSAFE_TIMEOUT) {
	          motors_killed = 1;
	          printf("\n!!! FAILSAFE: testtijd van %.0f s voorbij - MOTOREN UIT !!!\n",
	                 (double)FAILSAFE_TIMEOUT);
	      }
	  }

	  if (motors_killed) {
	      motors_send(0, 0, 0, 0); // alles uit en uit houden
	      HAL_Delay(20);
	      continue;
	  }

	  // angle-mode PID: setpoint 0 graden (frame waterpas) vs de gemeten hoek.
	  // Output is een throttle-correctie die de mixer per motor toepast.
	  // D-term uit de gyro (f_gx = rolsnelheid, f_gy = pitchsnelheid, graden/s).
	  // Maakt de demping veel rustiger dan het differentieren van de hoek.
	  float roll_cmd  = PID_Angle_UpdateRate(&pid_roll,  0.0f, roll_level,  f_gx, dt_meas);
	  float pitch_cmd = PID_Angle_UpdateRate(&pid_pitch, 0.0f, pitch_level, f_gy, dt_meas);

#if ANGLE_CHECK_ONLY
	  // Controlestand: motoren blijven UIT, we printen alleen de hoeken.
	  // Hiermee check je veilig of de IMU-oriëntatie en de offsets kloppen.
	  motors_send(0, 0, 0, 0);
#else
	  // mixer verstuurt meteen een frame naar alle 4 de motoren (met handshake)
	  Update_Motors(THROTTLE_BASE, roll_cmd, pitch_cmd, 0.0f);
#endif

	  // pitch/roll in graden, de PID-correcties, en wat elke motor werkelijk krijgt.
	  // ACHTER = M1,M4   VOOR = M2,M3   LINKS = M1,M2   RECHTS = M3,M4
	  printf("pitch=%6.1f roll=%6.1f | p_cmd=%6.1f r_cmd=%6.1f | M1=%4d M2=%4d M3=%4d M4=%4d | dt=%.4f\n",
	         pitch_level, roll_level, pitch_cmd, roll_cmd,
	         last_motor[0], last_motor[1], last_motor[2], last_motor[3], dt_meas);

	  HAL_Delay(1); // pacing; Update_Motors doet zelf de dshot-handshake

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20404768;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 359;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 359;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
  huart8.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMI_CS_GPIO_Port, BMI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SX1280_CS_GPIO_Port, SX1280_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SX1280_nRST_GPIO_Port, SX1280_nRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BMP_SCK_Pin BMP_MISO_Pin BMP_MOSI_Pin */
  GPIO_InitStruct.Pin = BMP_SCK_Pin|BMP_MISO_Pin|BMP_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : Autonoom_Controller_Pin */
  GPIO_InitStruct.Pin = Autonoom_Controller_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Autonoom_Controller_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BMP_CS_Pin */
  GPIO_InitStruct.Pin = BMP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BMP_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BMP_INT_Pin BMI_INT1_Pin BMI_INT2_Pin */
  GPIO_InitStruct.Pin = BMP_INT_Pin|BMI_INT1_Pin|BMI_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Overcurrent_M1_Pin Overcurrent_M2_Pin */
  GPIO_InitStruct.Pin = Overcurrent_M1_Pin|Overcurrent_M2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BMI_CS_Pin */
  GPIO_InitStruct.Pin = BMI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BMI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BMI_SCK_Pin BMI_MISO_Pin BMI_MOSI_Pin */
  GPIO_InitStruct.Pin = BMI_SCK_Pin|BMI_MISO_Pin|BMI_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPS_Timer_Pin SD_SWA_Pin */
  GPIO_InitStruct.Pin = GPS_Timer_Pin|SD_SWA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GPS_RX_Pin GPS_TX_Pin */
  GPIO_InitStruct.Pin = GPS_RX_Pin|GPS_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART7;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SX1280_CS_Pin */
  GPIO_InitStruct.Pin = SX1280_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SX1280_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SX1280_SCK_Pin SX1280_MISO_Pin SX1280_MOSI_Pin */
  GPIO_InitStruct.Pin = SX1280_SCK_Pin|SX1280_MISO_Pin|SX1280_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SX1280_nRST_Pin */
  GPIO_InitStruct.Pin = SX1280_nRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SX1280_nRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SX1280_Bussy_Pin Overcurrent_M4_Pin Overcurrent_M3_Pin */
  GPIO_InitStruct.Pin = SX1280_Bussy_Pin|Overcurrent_M4_Pin|Overcurrent_M3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_D0_Pin SD_D1_Pin SD_D2_Pin SD_D3_Pin
                           SD_CK_Pin */
  GPIO_InitStruct.Pin = SD_D0_Pin|SD_D1_Pin|SD_D2_Pin|SD_D3_Pin
                          |SD_CK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CMD_Pin */
  GPIO_InitStruct.Pin = SD_CMD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(SD_CMD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BMM_INT_Pin */
  GPIO_InitStruct.Pin = BMM_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BMM_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
void Update_Motors(uint16_t throttle, int16_t roll, int16_t pitch, int16_t yaw) {
    int16_t m1 = throttle + pitch + roll - yaw; // Linksvoor
    int16_t m2 = throttle + pitch - roll + yaw; // Rechtsvoor
    int16_t m3 = throttle - pitch + roll + yaw; // Linksachter
    int16_t m4 = throttle - pitch - roll - yaw; // Rechtsachter
}
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
/*
    if (GPIO_Pin == GPIO_PIN_4)	//INT1 van BMI
    {
    	g_new_bmi_data = 1;
    	return;
    }

    if (GPIO_Pin == GPIO_PIN_5)	//INT2 van BMI
    {
    	return;
    }
    if (GPIO_Pin == GPIO_PIN_7)	//INT van BMM350
    {
    	g_new_bmm_data = 1;
    	return;
    }
    */
    /*
    if (GPIO_Pin == GPIO_PIN_13) //INT van BMP
    {
    	float druk = BMP384_ReadData();
    	float hoogte = BMP384_CalculateAltitude(druk);

    	printf("BMP384 Data: \n");
    	printf("BMP druk = %f, hoogte = %f", druk, hoogte);
    	g_druk = druk;
    	g_hoogte = hoogte;
    	return;
    }
    */
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        dshot_dma_complete = 1; // dshot-lijn is weer vrij voor een nieuw frame
    }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
