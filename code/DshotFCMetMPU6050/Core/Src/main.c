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
#include "fc_config.h"
#include "MPU6050.h"
#include "attitude.h"
#include "AnglePID.h"
#include "dshot.h"
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
int _write(int file, char *ptr, int len) {
	for(int i = 0; i < len; i++){
		if(ptr[i]=='\n'){
			HAL_UART_Transmit(&huart8, (uint8_t*)"\r", 1, HAL_MAX_DELAY);
		}
		HAL_UART_Transmit(&huart8, (uint8_t*)&ptr[i], 1, HAL_MAX_DELAY);
	}
    return len;
}

float ax, ay, az;
float gx, gy, gz;

float roll = 0.0f;
float pitch = 0.0f;

float roll_off = 0.0f;
float pitch_off = 0.0f;

PID_Angle_t pid_roll;
PID_Angle_t pid_pitch;

volatile uint8_t dshot_klaar = 1;
static uint8_t motoren_uit = 0;

// cycle counter aanzetten, daarmee meten we dt
static void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    *((volatile uint32_t*)0xE0001FB0) = 0xC5ACCE55;  // unlock, anders telt CYCCNT niet
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// 1 frame naar de 4 motoren sturen
static void motors_send(int t1, int t2, int t3, int t4) {
    int t[4] = { t1, t2, t3, t4 };
    for (int i = 0; i < 4; i++) {
        if (t[i] < 0)    t[i] = 0;
        if (t[i] > 2047) t[i] = 2047;
        if (t[i] > 0 && t[i] < 48) t[i] = 0;   // 1..47 zijn commando's, geen gas
    }
    while (dshot_klaar == 0) { }
    dshot_klaar = 0;
    update_motor_buffer(0, (uint16_t)t[0], 0);
    update_motor_buffer(1, (uint16_t)t[1], 0);
    update_motor_buffer(2, (uint16_t)t[2], 0);
    update_motor_buffer(3, (uint16_t)t[3], 0);
    send_dshot();
}

// zelfde throttle een aantal seconden aanhouden
void hold_motors(int t1, int t2, int t3, int t4, float sec) {
    int n = (int)(sec * 50.0f);
    for (int i = 0; i < n; i++) {
        motors_send(t1, t2, t3, t4);
        HAL_Delay(20);
    }
}

// dshot commando's (1..47)
#define CMD_BEEP        1
#define CMD_DIR_1       7
#define CMD_DIR_2       8
#define CMD_SAVE        12
#define CMD_DIR_NORM    20
#define CMD_DIR_REV     21

// commando naar 1 motor, rest op 0. Buffer rechtstreeks vullen want
// motors_send zet 1..47 op 0. Telemetriebit moet aan staan.
void dshot_command(int idx, uint16_t cmd, int keer) {
    for (int i = 0; i < keer; i++) {
        while (dshot_klaar == 0) { }
        dshot_klaar = 0;
        for (int m = 0; m < 4; m++) {
            if (m == idx) update_motor_buffer(m, cmd, 1);
            else          update_motor_buffer(m, 0, 0);
        }
        send_dshot();
        HAL_Delay(2);
    }
}

// draairichting zetten en opslaan in de ESC
void dshot_set_direction(int idx, int omgekeerd) {
#if USE_OLD_DIRECTION_CMDS
    uint16_t cmd = omgekeerd ? CMD_DIR_2 : CMD_DIR_1;
#else
    uint16_t cmd = omgekeerd ? CMD_DIR_REV : CMD_DIR_NORM;
#endif
    printf("M%d dir %u\n", idx + 1, (unsigned)cmd);
    dshot_command(idx, cmd, 10);
    HAL_Delay(50);
    dshot_command(idx, CMD_SAVE, 10);
    HAL_Delay(500);
}

// elke ESC laten piepen, om te zien of commando's aankomen
void dshot_beep_test(void) {
    for (int m = 0; m < 4; m++) {
        printf("piep M%d\n", m + 1);
        dshot_command(m, CMD_BEEP, 10);
        HAL_Delay(400);
    }
}

// motoren 1 voor 1 laten draaien
void test_motors_sequence(uint16_t gas) {
    for (int m = 0; m < 4; m++) {
        printf("M%d\n", m + 1);
        int t[4] = { 0, 0, 0, 0 };
        t[m] = gas;
        hold_motors(t[0], t[1], t[2], t[3], 1.5f);
    }
    hold_motors(0, 0, 0, 0, 0.5f);
}

// 3 draaien, 1 stil: de stilstaande is de geprinte motor
void test_motors_identify(uint16_t gas) {
    for (int m = 0; m < 4; m++) {
        printf("M%d staat stil\n", m + 1);
        int t[4] = { gas, gas, gas, gas };
        t[m] = 0;
        hold_motors(t[0], t[1], t[2], t[3], 5.0f);
        hold_motors(0, 0, 0, 0, 2.0f);
    }
}

// throttle langzaam opbouwen op 1 motor
void test_motor_ramp(int idx, int van, int tot, int stappen) {
    for (int i = 0; i <= stappen; i++) {
        int t = van + ((tot - van) * i) / stappen;
        int m[4] = { 0, 0, 0, 0 };
        m[idx] = t;
        motors_send(m[0], m[1], m[2], m[3]);
        printf("M%d gas %d\n", idx + 1, t);
        HAL_Delay(50);
    }
    hold_motors(0, 0, 0, 0, 1.0f);
}

// 2 motoren tegelijk laten draaien
void test_motor_pair(int a, int b, uint16_t gas) {
    printf("paar M%d + M%d\n", a + 1, b + 1);
    int t[4] = { 0, 0, 0, 0 };
    t[a] = gas;
    t[b] = gas;
    hold_motors(t[0], t[1], t[2], t[3], 5.0f);
    hold_motors(0, 0, 0, 0, 2.0f);
}

// M1 = achter-links (PA0), M2 = voor-links (PA2),
// M3 = voor-rechts (PD13), M4 = achter-rechts (PD12)
// De kant die naar beneden kantelt moet harder draaien; klopt dat niet,
// dan alle vier de tekens van die as omkeren.
static const int pitch_sign[4] = { +1, -1, -1, +1 };
static const int roll_sign[4]  = { +1, +1, -1, -1 };
static const int yaw_sign[4]   = {  0,  0,  0,  0 };

static int laatste_gas[4] = { 0, 0, 0, 0 };

// sensor-assen omrekenen naar frame-assen
static inline void imu_to_frame(float ax, float ay, float az, float gx, float gy,
                                float *fax, float *fay, float *faz,
                                float *fgx, float *fgy) {
    float x = ax, y = ay, z = az, p = gx, q = gy;

#if IMU_UPSIDE_DOWN
    y = -y;  z = -z;  q = -q;
#endif

#if IMU_ROTATED_90
    float tx = y, ty = -x;
    float tp = q, tq = -p;
    x = tx;  y = ty;  p = tp;  q = tq;
#endif

    *fax = x;  *fay = y;  *faz = z;
    *fgx = p;  *fgy = q;
}

// mixer: basisgas + de correcties per as
static void Update_Motors(int basis, float r_cmd, float p_cmd, float y_cmd) {
    int t[4];
    for (int i = 0; i < 4; i++) {
        int v = basis
              + (int)(pitch_sign[i] * p_cmd)
              + (int)(roll_sign[i]  * r_cmd)
              + (int)(yaw_sign[i]   * y_cmd);
        if (v < THROTTLE_MIN) v = THROTTLE_MIN;
        if (v > THROTTLE_MAX) v = THROTTLE_MAX;
        t[i] = v;
        laatste_gas[i] = v;
    }
    motors_send(t[0], t[1], t[2], t[3]);
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
  printf("start\n");

  // waarom is er gereset? BOR = spanning weggezakt
  printf("reset: ");
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))  printf("BOR ");
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))  printf("POR ");
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))  printf("PIN ");
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))  printf("SFT ");
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) printf("IWDG ");
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) printf("WWDG ");
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) printf("LPWR ");
  printf("\n");
  __HAL_RCC_CLEAR_RESET_FLAGS();

  DWT_Init();
  dshot_init();
  MPU6050_Init();

  // eerst een tijdje throttle 0 sturen, de ESC herkent daaraan het protocol
  printf("dshot aan\n");
  for (int i = 0; i < 100; i++) {
      motors_send(0, 0, 0, 0);
      HAL_Delay(20);
  }

  printf("gyro kalibreren, stil houden\n");
  MPU6050_Calibrate_Gyro(GYRO_CALIB_SAMPLES);

  // scheefstand van de sensor meten, frame moet waterpas liggen
  printf("level kalibreren, waterpas houden\n");
  float som_r = 0.0f, som_p = 0.0f;
  for (int i = 0; i < LEVEL_CALIB_SAMPLES; i++) {
      float x, y, z, r, p;
      float fax, fay, faz, d1, d2;
      MPU6050_Read_Accel(&x, &y, &z);
      imu_to_frame(x, y, z, 0.0f, 0.0f, &fax, &fay, &faz, &d1, &d2);
      Attitude_AccelAngles(fax, fay, faz, &r, &p);
      som_r += r;
      som_p += p;
      HAL_Delay(2);
  }
  roll_off  = som_r / LEVEL_CALIB_SAMPLES;
  pitch_off = som_p / LEVEL_CALIB_SAMPLES;
  roll  = roll_off;
  pitch = pitch_off;
  printf("offset r=%.2f p=%.2f\n", roll_off, pitch_off);

  PID_Angle_Init(&pid_roll,  PID_KP, PID_KI, PID_KD,
                 PID_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT);
  PID_Angle_Init(&pid_pitch, PID_KP, PID_KI, PID_KD,
                 PID_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT);
  pid_roll.d_cutoff_hz  = PID_D_CUTOFF_HZ;
  pid_pitch.d_cutoff_hz = PID_D_CUTOFF_HZ;

  // armen: de ESC wil eerst een langere periode throttle 0 zien
  HAL_Delay(3000);
  printf("armen\n");
  for (int i = 0; i < 250; i++) {
      motors_send(0, 0, 0, 0);
      HAL_Delay(20);
  }
  printf("armed\n");

#if SET_MOTOR_DIRECTIONS
  // eenmalig, met de propellers eraf
  dshot_beep_test();
  dshot_set_direction(0, 0);
  dshot_set_direction(2, 0);
  dshot_set_direction(1, 1);
  dshot_set_direction(3, 1);
#endif

  //test_motors_identify(THROTTLE_BASE);

  // gas rustig opbouwen, anders te grote stroompiek en brownout.
  // start bij 48 want daaronder zijn het commando's.
  printf("soft-start\n");
  for (int i = 0; i <= 200; i++) {
      int t = 48 + ((THROTTLE_BASE - 48) * i) / 200;
      motors_send(t, t, t, t);
      if ((i % 20) == 0) printf("gas %d\n", t);
      HAL_Delay(20);
  }
  printf("pid aan\n");

  PID_Angle_Reset(&pid_roll);
  PID_Angle_Reset(&pid_pitch);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t cyc_prev = DWT->CYCCNT;
  float looptijd = 0.0f;
  while (1)
  {
	  MPU6050_Read_Accel(&ax, &ay, &az);
	  MPU6050_Read_Gyro(&gx, &gy, &gz);

	  // echte tijd sinds vorige ronde (s), aftrek is wrap-veilig
	  uint32_t cyc_now = DWT->CYCCNT;
	  float dt = (uint32_t)(cyc_now - cyc_prev) / (float)SystemCoreClock;
	  cyc_prev = cyc_now;

	  float fax, fay, faz, fgx, fgy;
	  imu_to_frame(ax, ay, az, gx, gy, &fax, &fay, &faz, &fgx, &fgy);

	  // complementair filter: gyro + accel
	  Attitude_Update(fax, fay, faz, fgx, fgy, gz, dt, &roll, &pitch);

	  // hoeken van het frame, dus zonder de scheefstand van de sensor
	  float r = roll - roll_off;
	  float p = pitch - pitch_off;

	  // te ver doorgeslagen of tijd om -> motoren uit en uit laten
	  looptijd += dt;
	  if (!motoren_uit) {
	      if (r > FAILSAFE_ANGLE || r < -FAILSAFE_ANGLE ||
	          p > FAILSAFE_ANGLE || p < -FAILSAFE_ANGLE) {
	          motoren_uit = 1;
	          printf("failsafe hoek r=%.1f p=%.1f\n", r, p);
	      } else if (looptijd > FAILSAFE_TIMEOUT) {
	          motoren_uit = 1;
	          printf("failsafe tijd\n");
	      }
	  }

	  if (motoren_uit) {
	      motors_send(0, 0, 0, 0);
	      HAL_Delay(20);
	      continue;
	  }

	  // setpoint is 0 graden. D-term komt uit de gyro, dat is rustiger.
	  float r_cmd = PID_Angle_UpdateRate(&pid_roll,  0.0f, r, fgx, dt);
	  float p_cmd = PID_Angle_UpdateRate(&pid_pitch, 0.0f, p, fgy, dt);

#if ANGLE_CHECK_ONLY
	  motors_send(0, 0, 0, 0);
#else
	  Update_Motors(THROTTLE_BASE, r_cmd, p_cmd, 0.0f);
#endif

	  printf("p=%6.1f r=%6.1f | pc=%6.1f rc=%6.1f | %4d %4d %4d %4d | dt=%.4f\n",
	         p, r, p_cmd, r_cmd,
	         laatste_gas[0], laatste_gas[1], laatste_gas[2], laatste_gas[3], dt);

	  HAL_Delay(1);

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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        dshot_klaar = 1;
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
