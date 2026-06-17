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
#include "dshot.h"
//#include "BMM350.h"
//#include "BMI330.h"
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

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart8;

/* USER CODE BEGIN PV */
//BMP384_Calib calibData;


/*
PID_Handle_t struct_PidRateRoll;
PID_Handle_t struct_PidRatePitch;
PID_Handle_t struct_PidRateYaw;
BMM350_MagData BMM350_Data;
BMI330_Frame current_data;
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile int g_new_pressure_data = 0;

int _write(int file, char *ptr, int len) {
	for(int i = 0; i < len; i++){
		if(ptr[i]=='\n'){
			HAL_UART_Transmit(&huart8, (uint8_t*)"\r", 1, HAL_MAX_DELAY);
		}
		HAL_UART_Transmit(&huart8, (uint8_t*)&ptr[i], 1, HAL_MAX_DELAY);
	}
   return len;
}
volatile uint8_t dshot_dma_complete = 1;
/*
void Update_Altitude_Filter(float baro_hoogte, float acc_z_m_s2) {
    verticale_snelheid += acc_z_m_s2 * dt;
    float voorspelde_hoogte = gefilterde_hoogte + (verticale_snelheid * dt);
    gefilterde_hoogte = (alpha * voorspelde_hoogte) + ((1 - alpha) * baro_hoogte);
}
*/
/* USER CODE BEGIN 0 */

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
  BMM350_Init();
  PID_HandleInit(struct_PidRateRoll);
  PID_HandleInit(struct_PidRatePitch);
  PID_HandleInit(struct_PidRateYaw);
  */
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
  /* USER CODE BEGIN 2 */
  printf("Voor alles\n");
  /*
  for (int i = 48; i < 1000; i++) {
      while (dshot_dma_complete == 0) { } // Wacht tot vorig pakketje fysiek verstuurd is
      dshot_dma_complete = 0;

      update_motor_buffer(0, 0, 0);
      // ... update andere motoren ...
      send_dshot();
      HAL_Delay(20);
  }
  */
  // Buiten de while(1) - De Arming Sequence
    printf("Start Arming...\n");
    for (int i = 0; i < 100; i++) {
        while (dshot_dma_complete == 0) { }
        dshot_dma_complete = 0;

        update_motor_buffer(0, 0, 0);
        send_dshot();
        HAL_Delay(20);
    }
    printf("Armed! Start Motor...\n");

    // Binnen de while(1) - Vaste lage snelheid
      printf("Start\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*
  while (1)
  {
	  for (int i = 48; i < 2000; i++)
	      	{
		  printf("In loop\n");
	      	    // 1. Wacht tot de vorige DShot transfer 100% klaar is
	      	    while (dshot_dma_complete == 0)
	      	    {
	      	        // Doe hier niks, of doe andere nuttige dingen (zoals sensoren uitlezen!)
	      	    }

	      	    // 2. Zet de vlag op 0 (bezet) voordat we gaan zenden
	      	    dshot_dma_complete = 0;

	      	    // 3. Update de buffer met de nieuwe 'i' waarde
	      	    update_motor_buffer(0, i, 0); // Motor 1
	      	    update_motor_buffer(1, i,0); // Motor 2
	      	    update_motor_buffer(2, i, 0); // Motor 3
	      	    update_motor_buffer(3, i, 0); // Motor 4

	      	    // 4. Start de nieuwe DMA transfer
	      	    send_dshot();
	      	    printf("Test\n");

	      	    // Optioneel in een test-sweep: als je de motor de TIJD wilt geven om
	      	    // fysiek op toeren te komen, kun je hier nog steeds een kleine delay
	      	    // gebruiken, maar hij is niet meer nodig voor de DShot data-integriteit!
	      	    HAL_Delay(1);
	      	}
	  */

  while (1) {
      while (dshot_dma_complete == 0) { }
      printf("Int getriggerd\n");
      dshot_dma_complete = 0;

      // Throttle 150 is een mooie, relatief langzame draaisnelheid zonder prop
      update_motor_buffer(0, 150, 0);
      send_dshot();

      // Bij Dshot300 sturen we idealiter zo'n 500 tot 1000 frames per seconde
      HAL_Delay(2);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /*Configure GPIO pins : Dshot_M4_Pin Dshot_M3_Pin */
  GPIO_InitStruct.Pin = Dshot_M4_Pin|Dshot_M3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
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

  /*Configure GPIO pins : BMM_SCL_Pin BMM_SDA_Pin */
  GPIO_InitStruct.Pin = BMM_SCL_Pin|BMM_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

/*
    if (GPIO_Pin == GPIO_PIN_4)	//INT1 van BMI
    {
    	float data[15];
    	int len = 15;
    	float roll, pitch;
    	BMI330_ReadFIFO(data, len);
    	BMI330_ParseFIFOFrame(&data, &current_data);
    	BMI330_GetAngles(current_data, roll, pitch);

    	printf("Data van de BMI INT1 pin: \n");
    	printf("Gyro data: X = %f, Y = %f, Z = %f \n", current_data->gyr_x, current_data->gyr_y, current_data->gyr_z);
    	printf("acc  data: X = %f, Y = %f, Z = %f \n", current_data->acc_x, current_data->acc_y, current_data->acc_z);
    	printf("temp: %f \n\n\n\n", current_data->sensor_temp);
    	g_temp = current_data->sensor_temp;
    	g_roll = roll;
    	g_pitch = pitch;
    	return;
    }

    if (GPIO_Pin == GPIO_PIN_5)	//INT2 van BMI
    {
    	return;
    }
    if (GPIO_Pin == GPIO_PIN_7)	//INT van BMM350
    {
    	float roll, pitch;
    	BMI330_GetAngles(current_data, &roll, &pitch);
    	BMM350_GetData(&BMM350_Data, roll, pitch);
    	float hoek = BMM350_Data.richting;

    	// calibrate is iets wat we eenmalig moeten doen, hieruit halen we de vaste ofset waardes die we daarna kunnen gebruiken
    	BMM350_Calibrate(BMM350_Data.x, BMM350_Data.y, BMM350_Data.z, BMM350_Data.x_offset, BMM350_Data.y_offset, BMM350_Data.z_offset);
    	printf("BMM350 Data: \n");
    	printf("BMM350 Raw Data: x = %d, Y = %d, Z = %d \n\n", BMM350_Data.x, BMM350_Data.y, BMM350_Data.z);

    	// enkel de laatste offset meting is belangerijk want dan heb je de uiterste bereikt
    	printf("BMM350 offsets Data: x = %d, Y = %d, Z = %d \n\n\n\n", BMM350_Data.x_offset, BMM350_Data.y_offset, BMM350_Data.z_offset);

    	g_hoek = hoek;
    	return;
    }
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
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
	{
		//printf("In timer interupt\n");
	    // Controleer of dit de juiste timer en het juiste kanaal is (pas aan naar jouw setup!)
	    if(htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	    {
	        dshot_dma_complete = 1; // Zet de vlag op 1: de lijn is weer vrij!

	        // Optioneel: Stop de DMA transfer netjes
	        //HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
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
