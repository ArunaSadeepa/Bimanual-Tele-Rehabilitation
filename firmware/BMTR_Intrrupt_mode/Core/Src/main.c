/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

#include "string.h"
#include "stdio.h"
#include <stdbool.h>
#include "motor_encoder.h"
#include "moving_average_int16.h"
#include "math.h"
#include "motor.h"
//#include "fatfs_sd.h"
#include "dob.h"
#include "rtob.h"
#include <stdlib.h>

#include "DFR_i2c.h"

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

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

encoder_instance enc_instance_M1; // motor 1
encoder_instance enc_instance_M2; // motor 2

pid_instance_int16 pid_instance_M1;
pid_instance_int16 pid_instance_M2;

mov_aver_instance_int16 filter_instance1;
mov_aver_instance_int16 filter_instance2;

dob_instance dob1;
dob_instance dob2;

rtob_instance rtob1;
rtob_instance rtob2;

DFRobot_GP8XXX_IIC gp8211s_1;
DFRobot_GP8XXX_IIC gp8211s_2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float dt = 5;         // Update interval in milliseconds

float motor1_vel;       // Motor1 velocity
float motor2_vel;       // Motor2 velocity
float motor1_vel_u;       // Motor1 velocity unfiltered
float motor2_vel_u;

float motor1_prevel;       // Motor1 velocity
float motor2_prevel;       // Motor2 velocity

float motor1_pos; 		// Motor1 position
float motor2_pos; 		// Motor2 position

float T_Dis_AVG;

float Ia_ref1;
float T_Dis1;
float T_Rec1;
float T_P1;
float T_G1;
float volt1;

float Ia_ref2;
float T_Dis2;
float T_Rec2;
float T_P2;
float T_G2;
float volt2;

float T_Dis1_u;
float T_Rec1_u;

float T_Dis2_u;
float T_Rec2_u;

float TorqueDown1[100];
float TorqueDown2[100];
float PositionDown1[100];
float PositionDown2[100];

int ThetaConuterDown = 0;
int SD_DoneDown = 0;

float TorqueUp1[100];
float TorqueUp2[100];
float PositionUp1[100];
float PositionUp2[100];

int ThetaConuterUp = 81;
int SD_DoneUp = 0;

float Kp_M = 10.0;
float Ki_M = 0.10;
float Kd_M = 0.01; // set gains

float Kp_S = 2.0;
float Ki_S = 0.01;
float Kd_S = 0.01; // set gains

/************* Variables for controlling update rate and motor parameters *****************************/

float PPR = 512.0;       // Pulses per revolution
float gear_ratio = 26.0; // Gear ratio
float k_T = 0.0705;
float J_M = +3069.1e-7;
float I_max = 4.0;

float RPM_k;            // RPM constant
float RPM_to_Rads_per_sec; // RPM to Rads/sec
float Ticks_to_Deg;

/********* Variables for the START/STOP button *****************/

GPIO_PinState Enable = GPIO_PIN_SET;
GPIO_PinState SetPos = GPIO_PIN_RESET;
GPIO_PinState IsRotation = GPIO_PIN_RESET;

bool Running;
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;

/********* Variables for Mode *****************/
short Mode = 0;
// Exercise = 0;
// Friction = 1;
// Inertia = 2;

bool IsRo = 0;

bool IsDown = 0;

bool MST1 = 1;

/********* Variables for J and COG *****************/

bool IsMale = 0;

const float g = 9.81;

const float FAweight[] = { 0.0187, 0.0157 };
const float HAweight[] = { 0.0065, 0.005 };
const float FAlen[] = { 0.157, 0.16 };
const float HAlen[] = { 0.0575, 0.0575 };
const float FArog[] = { 0.526, 0.053 };
const float HArog[] = { 0.549, 0.54 };
const float FAcog[] = { 0.43, 0.434 };
const float HAcog[] = { 0.468, 0.468 };

float height = 1.7;
float weight = 55;
float J, J_Ro, G;
float g_dis = 20.0;

const float RoWeight = 0.25;
const float RoCOG = 0.02;

/******************************************/
int Status = -2;

int direction = 0;
int enable = 0;

/**********************************************/
/***************************** Variables for SD ********************************************/
/*FATFS fs; // file system
 FIL fil; // file
 FRESULT fresult; // to store the result
 char buffer[1024]; // to store the data
 UINT br, bw; // file read and write count*/
/**********************************************/

/*********************** to send the data to the uart ****************************/

void send_uart(char *string) {

	uint8_t len = strlen(string);
	HAL_UART_Transmit(&huart3, (uint8_t*) string, len, 2000); // transmit in blocking mode

}

/*************** saving user data ***********************/
struct user_metrics {
	float Weight;
	float Height;
	char Gender[20];
};

struct settings {
	float K;
	float g_dis;
};

/*************** saving user data to confirm ***********************/
struct data_to_send {
	struct user_metrics metrics;
	struct settings settings;
};

struct indicators {
	int exerciseMode;
	int MasterSide;
	int repetitions;
};

/**************************************************************/
float wgt;
float hgt;
char gen[7];

float J1;
float G1;

float springConstant;
float cutoffFrequency;

/**************************************************************/
/*void WriteToSD(float DATA1[], float DATA2[], uint32_t size, char fileName[]) {
 if (f_open(&fil, fileName,
 FA_OPEN_ALWAYS | FA_READ | FA_WRITE | FA_OPEN_APPEND) == FR_OK) {
 char buffer[100];
 for (int i = 0; i < size; i++) {
 // Format the string with floating-point values
 snprintf(buffer, sizeof(buffer), "%d, %d\n", (int) DATA1[i],
 (int) (DATA2[i] * 1000));
 // Write the string to the file
 f_puts(buffer, &fil);
 }
 // Close the file
 f_close(&fil);
 }
 }*/

/********************* Custom write function for ITM interface ****************************/

int _write(int file, char *ptr, int len) {
	int i;
	for (i = 0; i < len; i++) {
		ITM_SendChar((*ptr++));
	}
	return len;
}

/* to find the size of data in the buffer*/

int bufsize(char *buf) {

	int i = 0;
	while (*buf++ != '\0')
		i++;
	return i;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

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
	MX_USART3_UART_Init();
	MX_I2C3_Init();
	MX_SPI4_Init();
	MX_TIM1_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */

	/*fresult = f_mount(&fs, "", 0);
	 if (fresult != FR_OK)
	 send_uart("error in mounting SD Card...\n");
	 else
	 send_uart("SD Card mounted successfully...\n");

	 send_uart(".........................\n");
	 */
	HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);

	init_encoder(&enc_instance_M1);
	init_encoder(&enc_instance_M2);

	set_pid_gain(&pid_instance_M1, Kp_M, Ki_M, Kd_M); // set gains
	set_pid_gain(&pid_instance_M2, Kp_S, Ki_S, Kd_S); // set gains

	J1 = findJ(IsMale);
	J = 0.001;
	G1 = findG(IsMale);

	set_dob(&dob1, k_T, J, g_dis); // k = 70.5mNm/A  ,  j = 3069.1 gcm2  , g_dis = 50
	set_rtob(&rtob1, k_T, J, g_dis, 0.0129, 0.0003);

	set_dob(&dob2, k_T, J, g_dis); // k = 70.5mNm/A  ,  j = 3069.1 gcm2  , g_dis = 50
	set_rtob(&rtob2, k_T, J, g_dis, 0.0129, 0.0003);

	RPM_k = (float) 60.0 / (PPR * gear_ratio); // to calculate RPM form ticks/s
	RPM_to_Rads_per_sec = 2.0 * 3.14 / 60.0; // to calculate rads per sec from rpm
	Ticks_to_Deg = 360.0 / (512.0 * 26.0); // to calculate degrees form ticks

	/*********************************DAC configuration *******************************************************/
	GP8XXX_IIC_begin(&gp8211s_1, GP8211S_identifier, 0x58, GPIOB,
	GPIO_PIN_6,
	GPIOB, GPIO_PIN_9);
	GP8XXX_IIC_begin(&gp8211s_2, GP8211S_identifier, 0x58, GPIOF,
	GPIO_PIN_0,
	GPIOF, GPIO_PIN_1);

	GP8XXX_IIC_setDACOutRange(&gp8211s_1, eOutputRange10V); // 10V
	GP8XXX_IIC_setDACOutRange(&gp8211s_2, eOutputRange10V); // 10V

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); //BLUE LED
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET); //M_Driver1_Enable
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); //RED LED
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //GREEN LED
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET); // SD led
	/*************************/

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 144;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 6;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void) {

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.Timing = 0x00808CD2;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */

}

/**
 * @brief SPI4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI4_Init(void) {

	/* USER CODE BEGIN SPI4_Init 0 */

	/* USER CODE END SPI4_Init 0 */

	/* USER CODE BEGIN SPI4_Init 1 */

	/* USER CODE END SPI4_Init 1 */
	/* SPI4 parameter configuration*/
	hspi4.Instance = SPI4;
	hspi4.Init.Mode = SPI_MODE_MASTER;
	hspi4.Init.Direction = SPI_DIRECTION_2LINES;
	hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
	hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi4.Init.NSS = SPI_NSS_SOFT;
	hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi4.Init.CRCPolynomial = 7;
	hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI4_Init 2 */

	/* USER CODE END SPI4_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, MOTOR2_DIR_Pin | M_DRIVER2_ENABLE_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	SD_CS_Pin | LD3_Pin | MOTOR1_DIR_Pin | M_DRIVER1_ENABLE_Pin | LD2_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : USER_Btn_Pin */
	GPIO_InitStruct.Pin = USER_Btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
	GPIO_InitStruct.Pin = RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_REF_CLK_Pin RMII_CRS_DV_Pin */
	GPIO_InitStruct.Pin = RMII_REF_CLK_Pin | RMII_CRS_DV_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : MOTOR2_DIR_Pin M_DRIVER2_ENABLE_Pin */
	GPIO_InitStruct.Pin = MOTOR2_DIR_Pin | M_DRIVER2_ENABLE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : SD_CS_Pin LD3_Pin MOTOR1_DIR_Pin M_DRIVER1_ENABLE_Pin
	 LD2_Pin */
	GPIO_InitStruct.Pin = SD_CS_Pin | LD3_Pin | MOTOR1_DIR_Pin
			| M_DRIVER1_ENABLE_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : RMII_TXD1_Pin */
	GPIO_InitStruct.Pin = RMII_TXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OverCurrent_Pin */
	GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DATA_READY_Pin */
	GPIO_InitStruct.Pin = DATA_READY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DATA_READY_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_ID_Pin USB_DM_Pin USB_DP_Pin */
	GPIO_InitStruct.Pin = USB_ID_Pin | USB_DM_Pin | USB_DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
	GPIO_InitStruct.Pin = RMII_TX_EN_Pin | RMII_TXD0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
