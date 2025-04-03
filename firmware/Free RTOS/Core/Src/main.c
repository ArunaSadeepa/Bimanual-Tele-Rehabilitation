/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// FreeRTOS includes for multitasking functionality
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

// Standard libraries
#include "string.h"
#include "stdio.h"
#include <stdbool.h>
#include "math.h"
#include <stdlib.h>
#include "FLASH_SECTOR_F7.h"

// Custom libraries for motor control and other functionalities
#include "DFR_i2c.h"
#include "dob.h"
#include "rtob.h"
#include "motor.h"
#include "motor_encoder.h"
#include "moving_average_int16.h"
#include "fatfs_sd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Define GPIO pins for motor direction control */
#define MOTOR1_DIR_GPIO_Port GPIOB
#define MOTOR1_DIR_Pin GPIO_PIN_4
#define MOTOR2_DIR_GPIO_Port GPIOA
#define MOTOR2_DIR_Pin GPIO_PIN_2

// Macro to convert milliseconds to RTOS ticks, we use this for vTaskDelayUntil function, for precise timing
#define MS_TO_TICKS(ms) ((TickType_t)((((TickType_t)(ms)) * (TickType_t)configTICK_RATE_HZ) / (TickType_t)1000))

// Define data size for user metrics and settings structures
#define DATA_SIZE (sizeof(struct user_metrics) + sizeof(struct settings))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* tim1 for encoder 1
 * tim4 for encoder 2
 */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* Define instances for motor control */

// Encoder instances for motor 1 and motor 2 ( file included for this is "motor_encoder.h")
encoder_instance enc_instance_M1; // Encoder instance for motor 1
encoder_instance enc_instance_M2; // Encoder instance for motor 2

// PID controller instances for motor 1 and motor 2  ( file included for this is "motor.h")
pid_instance_int16 pid_instance_M1; // PID controller instance for motor 1
pid_instance_int16 pid_instance_M2; // PID controller instance for motor 2

// Moving average filter instances for smoothing data of T_rec ( file included for this is "moving_average_int16.h")
mov_aver_instance_int16 filter_instance1; // Filter instance for T_Rec1 data
mov_aver_instance_int16 filter_instance2; // Filter instance for T_Rec2 data

// Disturbance observer instances for motor 1 and motor 2 ( file included for this is "dob.h")
dob_instance dob1; // Disturbance observer for motor 1
dob_instance dob2; // Disturbance observer for motor 2

// Reaction torque observer instances for motor 1 and motor 2 ( file included for this is "rtob.h")
rtob_instance rtob1; //Reaction torque observer for motor 1
rtob_instance rtob2; //Reaction torque observer for motor 2

// Instances for I2C communication with GP8211 dac module
DFRobot_GP8XXX_IIC gp8211s_1;  // for module 1
DFRobot_GP8XXX_IIC gp8211s_2;  // for module 2
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI4_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

xTaskHandle BM_Task_Hnadler;
xTaskHandle SDCARD_Task_Hnadler;
xTaskHandle INITIALIZATION_Task_Hnadler;

/************* Variables for friction estimation *****************************/

float motor1_vel;       // Motor1 velocity
float motor2_vel;       // Motor2 velocity

float motor1_prevel;    // previous Motor1 velocity
float motor2_prevel;    // previous Motor2 velocity

float motor1_pos; 		// Motor1 position
float motor2_pos; 		// Motor2 position

float dt = 5; /* Update interval in milliseconds for encoder functions(velocity and position),
 DOB, RTOB and pid functions, after change this, every tuning parameters needs to changed accordingly*/

float Ia_ref1;     // estimated motor 1 reference current
float T_Dis1;      // filtered estimated motor 1 disturbance torque
float T_Dis1_u;    // un-filtered estimated motor 1 disturbance torque
float T_Rec1;      // filtered estimated motor 1 reaction torque
float T_Rec1_u;    // un-filtered estimated motor 1 reaction torque
float T_P1;        // Torque profile data relevant to motor 1 position
float T_G1;        // gravity compensation of 1
float volt1;       // output set voltage for dac 1

float Ia_ref2;     // estimated motor 2 reference current
float T_Dis2;      // filtered estimated motor 2 disturbance torque
float T_Dis2_u;    // un-filtered estimated motor 2 disturbance torque
float T_Rec2;      // filtered estimated motor 2 reaction torque
float T_Rec2_u;    // un-filtered estimated motor 2 reaction torque
float T_P2;        // Torque profile data relevant to motor 2 position
float T_G2;        // gravity compensation of 2
float volt2;       // output set voltage for dac 1

float k_s = 50.0; // Spring constant
float k_sd = 50.0; // spring damper constant

/****************** arrays to store data to be written to SD card  *******************************/
float TorqueDown1[100];
float TorqueDown2[100];
float PositionDown1[100];
float PositionDown2[100];

int ThetaConuterDown = 0;
int SD_DoneDown = 0; // flag

float TorqueUp1[100];
float TorqueUp2[100];
float PositionUp1[100];
float PositionUp2[100];

int ThetaConuterUp = 81;
int SD_DoneUp = 0; // flag
/*************************************************************************************************/

/*********************PID gains of master and follower *******************************************/
float Kp_M = 10.0; // set gains of master
float Ki_M = 0.10;
float Kd_M = 0.01;

float Kp_S = 2.0; // set gains of follower
float Ki_S = 0.001;
float Kd_S = 0.1;
/*************************************************************************************************/

/************* Variables for motor parameters ************************/

float PPR = 512.0;       // Pulses per revolution
float gear_ratio = 26.0; // Gear ratio
float k_T = 0.0705;      // Torque constant Nm/A
float J_M = +3069.1e-7;  // rotor inertia kgm^3
float I_max = 4.0;       // current limiter

float RPM_k;               // constant to  calculate RPM form ticks/s
float RPM_to_Rads_per_sec; // constant to to calculate rads per sec from rpm
float Ticks_to_Deg;        // constant to calculate degrees form ticks

/*************************************************************************************************/

/********************** Time variables for de-bounce of inbuilt user button***********************/

uint32_t previousMillis = 0;
uint32_t currentMillis = 0;

/*************************************************************************************************/

/********* Variables for buttons *****************************************************************/

// can be used to get several modes, there are pins for this on the pcb
short Mode = 0;
// Exercise = 0;
// Friction = 1;
// Inertia = 2;

bool IsRo = 0; // Is rotational movement
bool IsDown = 0; // Is the arm going down
bool MST1 = 1; // Is the master at side 1

/********* Variables to calculate inertia of hand and COG ( center of gravity) *******************/

const float g = 9.81;

// these values took from the research paper to estimate J and COG
const float FAweight[] = { 0.0187, 0.0157 };
const float HAweight[] = { 0.0065, 0.005 };
const float FAlen[] = { 0.157, 0.16 };
const float HAlen[] = { 0.0575, 0.0575 };
const float FArog[] = { 0.526, 0.053 };
const float HArog[] = { 0.549, 0.54 };
const float FAcog[] = { 0.43, 0.434 };
const float HAcog[] = { 0.468, 0.468 };

const float RoWeight = 0.25;
const float RoCOG = 0.02;

// default values of the patient ( this will be change from the values send by the web interface )
float height = 1.7;
float weight = 55;
bool IsMale = 0; // is the person is male
char gen[7];

float J, J_Ro, G;
float g_dis = 20.0;

/***************************** Variables for SD ********************************************/
FATFS fs; // file system
FIL fil; // file
FRESULT fresult; // to store the result
char buffer[1024]; // to store the data
UINT br, bw; // file read and write count
/**********************************************/

/*********************** to send the data to the uart *************************************/

void send_uart(char *string) {

	uint8_t len = strlen(string);
	HAL_UART_Transmit(&huart2, (uint8_t*) string, len, 2000);

}

/*************Timing**************/

TickType_t xLastWakeTime_BM, xLastWakeTime_SDCARD;

/*************** saving user data and settings ********************************************/
struct user_metrics {
	float Weight;
	float Height;
	char Gender[20];
};

struct settings {
	float K;
	float g_dis;
};

/*************** saving user data just to confirm *****************************************/
struct data_to_send {
	struct user_metrics metrics;
	struct settings settings;
};

/*************** Indicator parameters of web interface ************************************/
struct indicators {
	int exerciseMode;
	int MasterSide;
	int repetitions;
};

/*********** function prototypes *********************************************************/

void WriteToSD(float DATA1[], float DATA2[], uint32_t size, char fileName[]);
void Bimanual_MotorCtrl_M1(void);  // bi-manual function when motor 1 is master
void Bimanual_MotorCtrl_M2(void);  // bi-manual function when motor 2 is master
void Profile_Record(void);         // to record rotational torque profile

float findJ(bool i);  // to find the inertia of arm
float findG(bool i);  // to find the center of gravity of arm
float J1;             // inertia of arm
float G1;             // center of gravity of arm

void SDCARD_Task(void *argument) {

	//xLastWakeTime_SDCARD = xTaskGetTickCount(); // this is for precise timing

	while (1) {

		// get the reaction torques at every 1 degree angle when it going down direction and put it in to array
		if (motor1_pos > ThetaConuterDown && SD_DoneDown == 0) {

			PositionDown1[ThetaConuterDown] = motor1_pos;
			PositionDown2[ThetaConuterDown] = motor2_pos;
			TorqueDown1[ThetaConuterDown] = T_Rec1;
			TorqueDown2[ThetaConuterDown] = T_Rec2;

			ThetaConuterDown = ThetaConuterDown + 1;
		}

		// get the reaction torques at every 1 degree angle when it going up and put it in to array
		if (motor1_pos < ThetaConuterUp && SD_DoneDown == 1) {

			PositionUp1[ThetaConuterUp] = motor1_pos;
			PositionUp2[ThetaConuterUp] = motor2_pos;
			TorqueUp1[ThetaConuterUp] = T_Rec1;
			TorqueUp2[ThetaConuterUp] = T_Rec2;

			ThetaConuterUp = ThetaConuterUp - 1;
		}

		// write when at 81 degrees when going down direction
		if (ThetaConuterDown == 81) {

			WriteToSD(PositionDown1, TorqueDown1, 82, "T1_Down.TXT");
			WriteToSD(PositionDown2, TorqueDown2, 82, "T2_Down.TXT");
			SD_DoneDown = 1;
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET); // SD led
			ThetaConuterDown = 0;
		}

		// write when at 0 degree angle when going up direction
		if (ThetaConuterUp == 0) {

			WriteToSD(PositionUp1, TorqueUp1, 82, "T1_Up.TXT");
			WriteToSD(PositionUp2, TorqueUp2, 82, "T2_Up.TXT");
			SD_DoneDown = 0;
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET); // SD led
			ThetaConuterUp = 81;
		}

		vTaskDelay(100);

		//vTaskDelayUntil(&xLastWakeTime_SDCARD, pdMS_TO_TICKS(2)); // 500 Hz // for precise timing
	}

}

int xx = 0; // this variable should be replaced by the "Number of repetition" variable (replace xx with the repetions at COMMENT FLAG 1)

void INITIALIZATION_Task(void *argument) {

	uint8_t I2cBuffer[DATA_SIZE];
	float dataToSend = 0;
	static uint32_t lastIndicatorSendTime = 0;
	struct indicators ind;
	static float xx = 0;

	while (1) {

		/* Mode selection here */
		if (HAL_GPIO_ReadPin(Mode_Selection_3_GPIO_Port, Mode_Selection_3_Pin)
				== GPIO_PIN_RESET) {

			Mode = 1;

		} else if (HAL_GPIO_ReadPin(Mode_Selection_2_GPIO_Port,
		Mode_Selection_2_Pin) == GPIO_PIN_RESET) {

			Mode = 2;

		} else if (HAL_GPIO_ReadPin(Mode_Selection_1_GPIO_Port,
		Mode_Selection_1_Pin) == GPIO_PIN_RESET) {

			Mode = 3;
		}

		/* Change the exercise */

		if (HAL_GPIO_ReadPin(CHANGE_EXERCISE_GPIO_Port, CHANGE_EXERCISE_Pin)
				== GPIO_PIN_RESET) {

			// flexion and extension
			IsRo = 0;
			HAL_GPIO_WritePin(Error_Indication_2_GPIO_Port,
			Error_Indication_2_Pin, GPIO_PIN_SET); // indicate flexion and extension by turning on led
		} else {
			// rotational
			IsRo = 1;
			HAL_GPIO_WritePin(Error_Indication_2_GPIO_Port,
			Error_Indication_2_Pin, GPIO_PIN_RESET); // indicate rotational by turning off led
		}

		/* Master change */
		if (HAL_GPIO_ReadPin(MST_SLV_SHIFT_GPIO_Port, MST_SLV_SHIFT_Pin)
				== GPIO_PIN_RESET) {
			// master is at side 2
			MST1 = 0;
			HAL_GPIO_WritePin(Error_Indication_1_GPIO_Port,
			Error_Indication_1_Pin, GPIO_PIN_RESET); // indicate master is at side 2 by turning off led
		}

		else {
			// master is at side 1
			MST1 = 1;
			HAL_GPIO_WritePin(Error_Indication_1_GPIO_Port,
			Error_Indication_1_Pin, GPIO_PIN_SET); // indicate master is at side 1 by turning on led

		}

		/* DATA_READY pin is used to get data from esp32,
		 *  whenever the esp32 wants to send new data, this gpio will be set to high by the esp32*/
		if (HAL_GPIO_ReadPin(DATA_READY_GPIO_Port, DATA_READY_Pin)
				== GPIO_PIN_SET) {

			//NOTE; this is not complete, after reading this pin we need to read i2c bus for the new data
			// currently the data is reading every 1 second ( see COMMENT FLAG 2)

			// after new data we need to calculate J1 and G1 again for new user parameters
			J1 = findJ(IsMale);
			G1 = findG(IsMale);

			set_dob(&dob1, k_T, J, g_dis); // k = 70.5mNm/A  ,  J should be replace with J1
			set_rtob(&rtob1, k_T, J, g_dis, 0.0129, 0.0003); // dynamic friction coefficient = 0.0129 Nm, static fric. =  0.0003 Nm/rpm

			set_dob(&dob2, k_T, J, g_dis); // k = 70.5mNm/A  ,   J should be replace with J1
			set_rtob(&rtob2, k_T, J, g_dis, 0.0129, 0.0003); // dynamic friction coefficient = 0.0129 Nm, static fric. =  0.0003 Nm/rpm

			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET); // SD led

		} else {
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET); // SD led

		}

		// this is to send data to plot
		HAL_I2C_Master_Transmit(&hi2c3, 0, (uint8_t*) &dataToSend,
				sizeof(float), 100);

		dataToSend = T_Rec2 * 26.0; // To plot

		//replace xx with the repetions at COMMENT FLAG 1
		xx = xx + 1;

		// send indication data every 1 second
		uint32_t currentTime = HAL_GetTick();
		if (currentTime - lastIndicatorSendTime >= 1000) {

			ind.MasterSide = MST1;
			ind.exerciseMode = IsRo;
			// COMMENT FLAG 1
			ind.repetitions = xx; //replace xx with the repetions

			HAL_I2C_Master_Transmit(&hi2c3, 0, (uint8_t*) &ind,
					sizeof(struct indicators), 100);

			lastIndicatorSendTime = currentTime;
		}

		// COMMENT FLAG 2
		// read esp32 data at every 1 sec
		HAL_StatusTypeDef I2Cstatus = HAL_I2C_Master_Receive(&hi2c3, 0,
				I2cBuffer,
				DATA_SIZE, 100);

		if (I2Cstatus == HAL_OK) {

			struct user_metrics received_metrics;
			memcpy(&received_metrics, I2cBuffer, sizeof(struct user_metrics));

			struct settings received_settings;
			memcpy(&received_settings, I2cBuffer + sizeof(struct user_metrics),
					sizeof(struct settings));

			weight = received_metrics.Weight;
			height = received_metrics.Height;

			strncpy(gen, received_metrics.Gender, sizeof(gen) - 1);

			if (strcmp(gen, "male") == 0) {
				IsMale = 1;
			} else {
				IsMale = 0;
			}
			//k_s = received_settings.K;
			//g_dis = received_settings.g_dis;

			struct data_to_send confirmation_data;
			struct user_metrics sending_metrics;
			struct settings sending_settings;
			sending_metrics.Weight = weight;
			sending_metrics.Height = height;

			strncpy(sending_metrics.Gender, received_metrics.Gender,
					sizeof(sending_metrics.Gender) - 1);
			sending_metrics.Gender[sizeof(sending_metrics.Gender) - 1] = '\0';
			sending_settings.K = k_s;
			//sending_settings.g_dis = cutoffFrequency;
			sending_settings.g_dis = g_dis;

			confirmation_data.metrics = sending_metrics;
			confirmation_data.settings = sending_settings;

			// Send confirmation data back to ESP32
			HAL_StatusTypeDef send_status = HAL_I2C_Master_Transmit(&hi2c3, 0,
					(uint8_t*) &confirmation_data, sizeof(struct data_to_send),
					100);

			if (send_status == HAL_OK) {
				// Data sent successfully
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Turn on LED to indicate success
			} else {
				// Failed to send data
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // Turn off LED to indicate failure
			}
		} else {
		}
		vTaskDelay(1000);
		/*vTaskSuspend(SDCARD_Task_Hnadler);
		 vTaskDelay(100);
		 */
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7); // blue LED
		//vTaskSuspend(INITIALIZATION_Task_Hnadler);
	}

}

void BM_Task(void *argument) {
	xLastWakeTime_BM = xTaskGetTickCount();
//const TickType_t xFrequency = pdMS_TO_TICKS(2); // for 2kHz

	while (1) {
		//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, 1);
		//HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
		//Bimanual_MotorCtrl_M2(); // nearly 150us

		if (Mode == 1) {
			if (MST1) {
				set_pid_gain(&pid_instance_M1, Kp_M, Ki_M, Kd_M); // set gains for master
				set_pid_gain(&pid_instance_M2, Kp_S, Ki_S, Kd_S); // set gains for follower
				Bimanual_MotorCtrl_M1();

			} else {
				set_pid_gain(&pid_instance_M1, Kp_S, Ki_S, Kd_S); // set gains for follower
				set_pid_gain(&pid_instance_M2, Kp_M, Ki_M, Kd_M); // set gains for master
				Bimanual_MotorCtrl_M2();

			}
		}

		if (Mode == 2) {
			Profile_Record();
		}

		//vTaskDelayUntil(&xLastWakeTime_BM, xFrequency);
		//vTaskDelayUntil(&xLastWakeTime_BM, MS_TO_TICKS(5)); // 1 kHz
		vTaskDelay(5); // dt = 5

	}

}

/**************** function to write on SD card ********************************************/
void WriteToSD(float DATA1[], float DATA2[], uint32_t size, char fileName[]) {
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
}

/********************* Custom write function for ITM interface ****************************/
int _write(int file, char *ptr, int len) {
	int i;
	for (i = 0; i < len; i++) {
		ITM_SendChar((*ptr++));
	}
	return len;
}

/************* function to find the size of data in the buffer*****************************/
int bufsize(char *buf) {

	int i = 0;
	while (*buf++ != '\0')
		i++;
	return i;
}

/************* function to clean the buffer ***********************************************/
void bufclear(void) {

	for (int i = 0; i < 1024; i++)
		buffer[i] = '\0';
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	currentMillis = HAL_GetTick();
	//user button to toggle
	if (GPIO_Pin == GPIO_PIN_13 && (currentMillis - previousMillis > 150)) {

		// User Button was pressed
		previousMillis = currentMillis;
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7); // blue LED
	}
}

/**************** function to find the inertia of arm ************************************/
float findJ(bool i) {
	float FAJ = weight * FAweight[i] * (height * FAlen[i] * FArog[i])
			* (height * FAlen[i] * FArog[i]);
	float HAJ1 = weight * HAweight[i] * (height * HAlen[i] * HArog[i])
			* (height * HAlen[i] * HArog[i]);

	float HAJ = HAJ1
			+ (weight * HAweight[i])
					* ((height * (FAlen[i] + HAlen[i] * HAcog[i]))
							* (height * (FAlen[i] + HAlen[i] * HAcog[i]))
							- (height * HAlen[i] * HAcog[i])
									* (height * HAlen[i] * HAcog[i]));

	return (FAJ + HAJ + J_Ro) / (gear_ratio * gear_ratio) + J_M;
}

/**************** function to find the center of gravity of arm **************************/
float findG(bool i) {	// pos = angle from horizontal
	float w = FAweight[i] + HAweight[i];
	float TFA = FAweight[i] * FAlen[i] * HAcog[i];
	float THA = HAweight[i] * (HAlen[i] * HAcog[i] + FAlen[i]);

	return w * weight * g * (height * (TFA + THA) / w) + RoWeight * RoCOG;
}

/**************** Torque profile *********************************************************/
float torque_profile(float position, bool IsRo, bool IsDown) {
	if (IsRo) { // the following torque profiles need to replaced with torque profiles of the rotational movement
		if (!IsDown) {
			return -0.00006 * position * position * position
					+ 0.0045 * position * position + 0.3899 * position + 65.208;
		} else {
			return 0.00002 * position * position * position
					+ 0.0161 * position * position + 1.708 * position + 68.921;
		}
	} else {
		if (!IsDown) {
			return -0.00006 * position * position * position
					+ 0.0045 * position * position + 0.3899 * position + 65.208;
		} else {
			return 0.00002 * position * position * position
					+ 0.0161 * position * position + 1.708 * position + 68.921;
		}
	}
}

/**************** Gravity function *******************************************************/
float T_gravity(float position, bool IsDown, float G, bool IsRo) {

	position = 90 - position;

	if (IsRo) {
		return 0;
	} else {
		if (IsDown) {
			return G * cos(position) * G;
		} else {
			return -G * cos(position) * G;
		}

	}
}

/**************** function to estimate the rotational torque profile *********************/
void Profile_Record(void) {

	update_encoder(&enc_instance_M1, &htim1); // update the encoder1
	update_encoder(&enc_instance_M2, &htim4);

	if (motor1_vel > 0.5) {
		IsDown = 1;
	} else if (motor1_vel < -0.5) {
		IsDown = 0;
	}

	if (IsDown) {

		motor1_vel = -motor1_vel;
		motor2_vel = -motor2_vel;

	}

	motor1_vel = enc_instance_M1.velocity * RPM_k; // convert ticks per sec to RPM ----- RPM_k = 60/(PPR*gear_ratio)
	motor2_vel = -enc_instance_M2.velocity * RPM_k; // convert ticks per sec to RPM ----- RPM_k = 60/(PPR*gear_ratio)

	if (motor1_vel < -0.5) {
		Ia_ref1 += 0.1;
	}

	if (motor2_vel < -0.5) {
		Ia_ref2 += 0.1;
	}

	motor1_prevel = motor1_vel;
	motor2_prevel = motor2_vel;

	motor1_pos = enc_instance_M1.position * Ticks_to_Deg; // from tick to degrees -> 360.0/(512.0*26.0)
	motor2_pos = -enc_instance_M2.position * Ticks_to_Deg;

	T_Dis1 = dob1.T_dis;
	T_Rec1 = rtob1.T_ext;

	T_Dis2 = dob2.T_dis;
	T_Rec2 = rtob2.T_ext;

	if (fabs(Ia_ref1) > I_max) {
		Ia_ref1 = (Ia_ref1 / fabs(Ia_ref1)) * I_max;
	}
	if (fabs(Ia_ref2) > I_max) {
		Ia_ref2 = (Ia_ref2 / fabs(Ia_ref2)) * I_max;
	}

	if (fabs(Ia_ref1) < 0) {
		Ia_ref1 = 0;
	}
	if (fabs(Ia_ref2) < 0) {
		Ia_ref2 = 0;
	}

	if (motor1_vel > -1.0) {
		Ia_ref1 = 0;
		//Ia_ref2 = 0;
	}

	update_dob(&dob1, fabs(Ia_ref1), motor1_vel * RPM_to_Rads_per_sec * 26.0); //  &dob1,  Ia_ref,  velocity
	update_rtob(&rtob1, fabs(Ia_ref1), motor1_vel * RPM_to_Rads_per_sec * 26.0); // &rtob1, Ia_ref, velocity

	update_dob(&dob2, fabs(Ia_ref2), motor2_vel * RPM_to_Rads_per_sec * 26.0); //  &dob1,  Ia_ref,  velocity
	update_rtob(&rtob2, fabs(Ia_ref2), motor2_vel * RPM_to_Rads_per_sec * 26.0); // &rtob1, Ia_ref, velocity

	if (IsDown) {

		HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin, GPIO_PIN_RESET);

	} else {

		HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin, GPIO_PIN_RESET);
	}

	volt1 = fabs(Ia_ref1 * 32767 / I_max);
	volt2 = fabs(Ia_ref2 * 32767 / I_max);

	HAL_GPIO_WritePin(M_DRIVER1_ENABLE_GPIO_Port, M_DRIVER1_ENABLE_Pin,
			GPIO_PIN_SET); // M_Driver1_Enable
	HAL_GPIO_WritePin(M_DRIVER2_ENABLE_GPIO_Port, M_DRIVER2_ENABLE_Pin,
			GPIO_PIN_SET); // M_Driver1_Enable

	GP8XXX_IIC_setDACOutVoltage(&gp8211s_1, volt1, 0);
	GP8XXX_IIC_setDACOutVoltage(&gp8211s_2, volt2, 0); // offset 72 points

}

/**************** Function for bi-manual control when motor 1 is master ******************/
void Bimanual_MotorCtrl_M1(void) {

	update_encoder(&enc_instance_M1, &htim1); // update the encoder1
	update_encoder(&enc_instance_M2, &htim4); // update the encoder2

	motor1_vel = enc_instance_M1.velocity * RPM_k; // convert ticks per sec to RPM ----- RPM_k = 60/(PPR*gear_ratio)
	motor2_vel = -enc_instance_M2.velocity * RPM_k; // convert ticks per sec to RPM ----- RPM_k = 60/(PPR*gear_ratio)

	if (motor1_vel > 0.5) {
		IsDown = 1;
	} else if (motor1_vel < -0.5) {
		IsDown = 0;
	}

	if (IsDown) {

		motor1_vel = -motor1_vel;
		motor2_vel = -motor2_vel;

	}

	motor1_pos = enc_instance_M1.position * Ticks_to_Deg; // from tick to degrees -> 360.0/(512.0*26.0)
	motor2_pos = -enc_instance_M2.position * Ticks_to_Deg;

	T_Dis1 = dob1.T_dis;    // disturbance torque
	T_Rec1_u = rtob1.T_ext; // reaction torque

	T_Dis2 = dob2.T_dis;
	T_Rec2_u = rtob2.T_ext;

	apply_average_filter(&filter_instance1, T_Rec1_u, &T_Rec1); // filtering reaction torque
	apply_average_filter(&filter_instance2, T_Rec2_u, &T_Rec2);

	T_P1 = torque_profile(motor1_pos, IsRo, IsDown) / (26.0 * 80.0); // get the torque from profile
	T_P2 = torque_profile(motor2_pos, IsRo, IsDown) / (26.0 * 80.0);

	T_G1 = T_gravity(motor1_pos, IsDown, G1, IsRo) / 26; // get the gravity compensation
	T_G2 = T_gravity(motor2_pos, IsDown, G1, IsRo) / 26;

	T_G1 = 0;
	T_G2 = 0;

	if (IsDown) {
		k_s = -fabs(k_s);
		//k_sd = +fabs(k_sd);
	} else {
		k_s = fabs(k_s);
		//k_sd = -fabs(k_sd);
	}

	// PID
	apply_pid(&pid_instance_M1, (T_P1 + T_G1 - T_Rec1), dt); // PID for master
	apply_pid(&pid_instance_M2,
			(T_P2 + T_G2 - T_Rec2 + k_s * (motor1_pos - motor2_pos)
					+ k_sd * (motor1_vel - motor2_vel)), dt); // PID for follower

	Ia_ref1 = ((pid_instance_M1.output) * (0.25 / 5000.0) + T_Dis1) / k_T; // reference current of master
	Ia_ref2 = ((pid_instance_M2.output) * (0.25 / 5000.0) + T_Dis2) / k_T; // reference current of follower

	// limit the current
	if (fabs(Ia_ref1) > I_max) {
		Ia_ref1 = (Ia_ref1 / fabs(Ia_ref1)) * I_max;
	}
	if (fabs(Ia_ref2) > I_max) {
		Ia_ref2 = (Ia_ref2 / fabs(Ia_ref2)) * I_max;
	}

	// deactivate the master when there is no movement on it
	if (motor1_vel > -0.5) {
		Ia_ref1 = 0;
		//Ia_ref2 = 0;
	}

	// deactivate the follower when there is no position error
	if (fabs(motor1_pos - motor2_pos) < 0.5) {

		Ia_ref2 = 0;
	}

	//DOB and RTOB
	update_dob(&dob1, fabs(Ia_ref1), motor1_vel * RPM_to_Rads_per_sec * 26.0); //  &dob1,  Ia_ref,  velocity
	update_rtob(&rtob1, fabs(Ia_ref1), motor1_vel * RPM_to_Rads_per_sec * 26.0); // &rtob1, Ia_ref, velocity

	update_dob(&dob2, fabs(Ia_ref2), motor2_vel * RPM_to_Rads_per_sec * 26.0); //  &dob1,  Ia_ref,  velocity
	update_rtob(&rtob2, fabs(Ia_ref2), motor2_vel * RPM_to_Rads_per_sec * 26.0); // &rtob1, Ia_ref, velocity

	// set the direction pins accordingly
	if (IsDown) {
		if (Ia_ref1 < 0) {
			HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin,
					GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin,
					GPIO_PIN_RESET);
		}

		if (Ia_ref2 < 0) {
			HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin,
					GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin,
					GPIO_PIN_SET);
		}
	} else {
		if (Ia_ref1 > 0) {
			HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin,
					GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin,
					GPIO_PIN_RESET);
		}

		if (Ia_ref2 > 0) {
			HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin,
					GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin,
					GPIO_PIN_SET);
		}
	}

	// calculate the relevant voltage for DACs ( it is a value between 0 to 32767)
	volt1 = fabs(Ia_ref1 * 32767 / I_max);
	volt2 = fabs(Ia_ref2 * 32767 / I_max);

	HAL_GPIO_WritePin(M_DRIVER1_ENABLE_GPIO_Port, M_DRIVER1_ENABLE_Pin,
			GPIO_PIN_SET); // M_Driver1_Enable
	HAL_GPIO_WritePin(M_DRIVER2_ENABLE_GPIO_Port, M_DRIVER2_ENABLE_Pin,
			GPIO_PIN_SET); // M_Driver2_Enable

	// send data to DACs
	GP8XXX_IIC_setDACOutVoltage(&gp8211s_1, volt1, 0);
	GP8XXX_IIC_setDACOutVoltage(&gp8211s_2, volt2, 0); // has an offset of 72 points , need to check

}

/**************** Function for bi-manual control when motor 2 is master ******************/
void Bimanual_MotorCtrl_M2(void) {

	update_encoder(&enc_instance_M1, &htim1); // update the encoder1
	update_encoder(&enc_instance_M2, &htim4); // update the encoder2

	motor1_vel = enc_instance_M1.velocity * RPM_k; // convert ticks per sec to RPM ----- RPM_k = 60/(PPR*gear_ratio)
	motor2_vel = -enc_instance_M2.velocity * RPM_k; // convert ticks per sec to RPM ----- RPM_k = 60/(PPR*gear_ratio)

	if (motor2_vel > 0.5) {
		IsDown = 1;
	} else if (motor2_vel < -0.5) {
		IsDown = 0;
	}

	if (IsDown) {

		motor1_vel = -motor1_vel;
		motor2_vel = -motor2_vel;

	}

	motor1_pos = enc_instance_M1.position * Ticks_to_Deg; // from tick to degrees -> 360.0/(512.0*26.0)
	motor2_pos = -enc_instance_M2.position * Ticks_to_Deg;

	//motor1_pos = 180 - motor1_pos;
	//motor2_pos = 180 - motor2_pos;

	T_Dis1 = dob1.T_dis;    // disturbance torque
	T_Rec1_u = rtob1.T_ext; // reaction torque

	T_Dis2 = dob2.T_dis;
	T_Rec2_u = rtob2.T_ext;

	apply_average_filter(&filter_instance1, T_Rec1_u, &T_Rec1); // filtering reaction torque
	apply_average_filter(&filter_instance2, T_Rec2_u, &T_Rec2);

	T_P1 = torque_profile(motor1_pos, IsRo, IsDown) / (26.0 * 80.0); // get the torque from profile
	T_P2 = torque_profile(motor2_pos, IsRo, IsDown) / (26.0 * 80.0);

	T_G1 = T_gravity(motor1_pos, IsDown, G1, IsRo) / 26; // get the gravity compensation
	T_G2 = T_gravity(motor2_pos, IsDown, G1, IsRo) / 26;

	T_G1 = 0;
	T_G2 = 0;

	if (IsDown) {
		k_s = -fabs(k_s);
		//k_sd = +fabs(k_sd);
	} else {
		k_s = fabs(k_s);
		//k_sd = -fabs(k_sd);
	}

	// PID
	apply_pid(&pid_instance_M2, (T_P2 + T_G2 - T_Rec2), dt); // PID for master
	apply_pid(&pid_instance_M1,
			(T_P1 + T_G1 - T_Rec1 + k_s * (motor2_pos - motor1_pos)
					+ k_sd * (motor2_vel - motor1_vel)), dt); // PID for follower

	Ia_ref1 = ((pid_instance_M1.output) * (0.25 / 5000.0) + T_Dis1) / k_T; // reference current of master
	Ia_ref2 = ((pid_instance_M2.output) * (0.25 / 5000.0) + T_Dis2) / k_T; // reference current of follower

	// limit the current
	if (fabs(Ia_ref1) > I_max) {
		Ia_ref1 = (Ia_ref1 / fabs(Ia_ref1)) * I_max;
	}
	if (fabs(Ia_ref2) > I_max) {
		Ia_ref2 = (Ia_ref2 / fabs(Ia_ref2)) * I_max;
	}

	// deactivate the master when there is no movement on it
	if (motor2_vel > -0.5) {
		Ia_ref2 = 0;
		//Ia_ref2 = 0;
	}

	// deactivate the follower when there is no position error
	if (fabs(motor1_pos - motor2_pos) < 0.5) {

		Ia_ref1 = 0;
	}

	//DOB and RTOB
	update_dob(&dob1, fabs(Ia_ref1), motor1_vel * RPM_to_Rads_per_sec * 26.0); //  &dob1,  Ia_ref,  velocity
	update_rtob(&rtob1, fabs(Ia_ref1), motor1_vel * RPM_to_Rads_per_sec * 26.0); // &rtob1, Ia_ref, velocity

	update_dob(&dob2, fabs(Ia_ref2), motor2_vel * RPM_to_Rads_per_sec * 26.0); //  &dob1,  Ia_ref,  velocity
	update_rtob(&rtob2, fabs(Ia_ref2), motor2_vel * RPM_to_Rads_per_sec * 26.0); // &rtob1, Ia_ref, velocity

	// set the direction pins accordingly
	if (IsDown) {
		if (Ia_ref1 < 0) {
			HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin,
					GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin,
					GPIO_PIN_RESET);
		}

		if (Ia_ref2 < 0) {
			HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin,
					GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin,
					GPIO_PIN_SET);
		}
	} else {
		if (Ia_ref1 > 0) {
			HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin,
					GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin,
					GPIO_PIN_RESET);
		}

		if (Ia_ref2 > 0) {
			HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin,
					GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin,
					GPIO_PIN_SET);
		}
	}

	// calculate the relevant voltage for DACs ( it is a value between 0 to 32767)
	volt1 = fabs(Ia_ref1 * 32767 / I_max);
	volt2 = fabs(Ia_ref2 * 32767 / I_max);

	HAL_GPIO_WritePin(M_DRIVER1_ENABLE_GPIO_Port, M_DRIVER1_ENABLE_Pin,
			GPIO_PIN_SET); // M_Driver1_Enable
	HAL_GPIO_WritePin(M_DRIVER2_ENABLE_GPIO_Port, M_DRIVER2_ENABLE_Pin,
			GPIO_PIN_SET); // M_Driver2_Enable

	// send data to DACs
	GP8XXX_IIC_setDACOutVoltage(&gp8211s_1, volt1, 0);
	GP8XXX_IIC_setDACOutVoltage(&gp8211s_2, volt2, 0); // has an offset of 72 points , need to check

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
	MX_TIM1_Init();
	MX_TIM4_Init();
	MX_SPI4_Init();
	MX_FATFS_Init();
	MX_I2C3_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	fresult = f_mount(&fs, "", 0); // mount SD
	if (fresult != FR_OK)
		send_uart("error in mounting SD Card...\n");
	else
		send_uart("SD Card mounted successfully...\n");

	send_uart(".........................\n");


	xTaskCreate(BM_Task, "BM", 512, NULL, 5, &BM_Task_Hnadler);
	xTaskCreate(INITIALIZATION_Task, "INIT", 512, NULL, 2,
			&INITIALIZATION_Task_Hnadler);
	xTaskCreate(SDCARD_Task, "SD", 1024, NULL, 1, &SDCARD_Task_Hnadler);

	HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);

	init_encoder(&enc_instance_M1);
	init_encoder(&enc_instance_M2);

	set_pid_gain(&pid_instance_M1, Kp_M, Ki_M, Kd_M); // set gains
	set_pid_gain(&pid_instance_M2, Kp_S, Ki_S, Kd_S); // set gains

	J1 = findJ(IsMale); // calculate inertia of arm
	J = 0.001;
	G1 = findG(IsMale); // calculate center of gravity of arm

	set_dob(&dob1, k_T, J, g_dis); // k = 70.5mNm/A  ,  j = 3069.1 gcm2
	set_rtob(&rtob1, k_T, J, g_dis, 0.0129, 0.0003); // dynamic friction coefficient = 0.0129 Nm, static fric. =  0.0003 Nm/rpm

	set_dob(&dob2, k_T, J, g_dis); // k = 70.5mNm/A  ,  j = 3069.1 gcm2  , g_dis = 50
	set_rtob(&rtob2, k_T, J, g_dis, 0.0129, 0.0003); // dynamic friction coefficient = 0.0129 Nm, static fric. =  0.0003 Nm/rpm

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

	/****Flash memory********************************************************************************/

	/*char *data11 =
	 "hello world\
  				  This is a test to see how many words can we work with it";

	 uint32_t Rx_Data[30];

	 char string[100];

	 int number = 123;

	 float val = 123.456;

	 float RxVal;
	 float RxVal2;

	 Flash_Write_Data(0x08040000, (uint32_t*) data22, 9);
	 Flash_Read_Data(0x08040000, Rx_Data, 10);

	 int numofwords = (strlen(data11) / 4) + ((strlen(data11) % 4) != 0);

	 // dont use the address of 0x08080000,0x0800C100, it is already taken,need to try another locations

	 Flash_Write_Data(0x08080000, (uint32_t*) data11, numofwords);
	 Flash_Read_Data(0x08080000, Rx_Data, numofwords);
	 Convert_To_Str(Rx_Data, string);

	 Flash_Write_NUM(0x0800C100, number);
	 RxVal = Flash_Read_NUM(0x0800C100);

	 Flash_Write_NUM(0x0800D100, val);
	 RxVal2 = Flash_Read_NUM(0x0800D100);*/
	/************************************************************************************/

	vTaskStartScheduler();

	/* USER CODE END 2 */

	/* We should never get here as control is now taken by the scheduler */
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
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
	hi2c3.Init.Timing = 0x20404768;
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
	hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
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
	sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
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
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
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
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

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
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF,
	__Pin | SD_Indication_Pin | SD_Writing_Indicator_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, MOTOR2_DIREC_Pin | M_DRIVER2_ENABLE_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Error_Indication_2_GPIO_Port, Error_Indication_2_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			SD_SC_LD1_Pin | Error_Indication_1_Pin | For_Testing_Pin | LD3_Pin
					| MOTOR1_DIREC_Pin | M_DRIVER1_ENABLE_Pin | LD2_Pin
					| GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, USB_PowerSwitchOn_Pin | GPIO_PIN_9,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(NodeMCU_CS_GPIO_Port, NodeMCU_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : USER_BTN_Pin */
	GPIO_InitStruct.Pin = USER_BTN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : __Pin SD_Indication_Pin SD_Writing_Indicator_Pin */
	GPIO_InitStruct.Pin = __Pin | SD_Indication_Pin | SD_Writing_Indicator_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : MOTOR2_DIREC_Pin M_DRIVER2_ENABLE_Pin */
	GPIO_InitStruct.Pin = MOTOR2_DIREC_Pin | M_DRIVER2_ENABLE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : Mode_Selection_1_Pin Mode_Selection_2_Pin Mode_Selection_3_Pin DATA_READY_Pin
	 Limit_SW1_Pin Limit_SW2_Pin */
	GPIO_InitStruct.Pin = Mode_Selection_1_Pin | Mode_Selection_2_Pin
			| Mode_Selection_3_Pin | DATA_READY_Pin | Limit_SW1_Pin
			| Limit_SW2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : Error_Indication_2_Pin */
	GPIO_InitStruct.Pin = Error_Indication_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Error_Indication_2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SD_SC_LD1_Pin Error_Indication_1_Pin LD3_Pin MOTOR1_DIREC_Pin
	 M_DRIVER1_ENABLE_Pin LD2_Pin PB8 PB9 */
	GPIO_InitStruct.Pin = SD_SC_LD1_Pin | Error_Indication_1_Pin | LD3_Pin
			| MOTOR1_DIREC_Pin | M_DRIVER1_ENABLE_Pin | LD2_Pin | GPIO_PIN_8
			| GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : MST_SLV_SHIFT_Pin SD_WRITING_EN_SWITCH_PIN_Pin */
	GPIO_InitStruct.Pin = MST_SLV_SHIFT_Pin | SD_WRITING_EN_SWITCH_PIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pin : START_STOP_BTN_Pin */
	GPIO_InitStruct.Pin = START_STOP_BTN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(START_STOP_BTN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : CHANGE_EXERCISE_Pin */
	GPIO_InitStruct.Pin = CHANGE_EXERCISE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(CHANGE_EXERCISE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : For_Testing_Pin */
	GPIO_InitStruct.Pin = For_Testing_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(For_Testing_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_PowerSwitchOn_Pin PG9 */
	GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OverCurrent_Pin */
	GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Limit_SW3_Pin Limit_SW4_Pin */
	GPIO_InitStruct.Pin = Limit_SW3_Pin | Limit_SW4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : NodeMCU_CS_Pin */
	GPIO_InitStruct.Pin = NodeMCU_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(NodeMCU_CS_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

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
