/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f7xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32f7xx_it.h"
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
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

extern const float g;

extern const float FAweight[];
extern const float HAweight[];
extern const float FAlen[];
extern const float HAlen[];
extern const float FArog[];
extern const float HArog[];
extern const float FAcog[];
extern const float HAcog[];

extern float height;
extern float weight;
extern float J, J_Ro, G;
extern float g_dis;

extern const float RoWeight;
extern const float RoCOG;

extern float PPR;       // Pulses per revolution
extern float gear_ratio; // Gear ratio
extern float k_T;
extern float J_M;
extern float I_max;

extern float RPM_k;            // RPM constant
extern float RPM_to_Rads_per_sec; // RPM to Rads/sec
extern float Ticks_to_Deg;


extern float dt;         // Update interval in milliseconds
int indx = 0;

extern float motor1_vel;       // Motor1 velocity
extern float motor2_vel;       // Motor2 velocity

extern float motor1_pos; 		// Motor1 position
extern float motor2_pos; 		// Motor2 position

extern bool IsRo;

extern bool IsDown;

extern float Ia_ref1;
extern float T_Dis1;
extern float T_Rec1;
extern float T_P1;
extern float T_G1;
extern float volt1;

extern float Ia_ref2;
extern float T_Dis2;
extern float T_Rec2;
extern float T_P2;
extern float T_G2;
extern float volt2;

extern float T_Dis1_u;
extern float T_Rec1_u;

extern float T_Dis2_u;
extern float T_Rec2_u;

extern float G1;
extern float J1;

float k_s = 50.0;
float k_sd = 50.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Bimanual_MotorCtrl_M1(void);

volatile uint8_t FatFsCnt = 0;
volatile uint8_t Timer1, Timer2;

void SDTimer_Handler(void) {

	if (Timer1 > 0)
		Timer1--;

	if (Timer2 > 0)
		Timer2--;
}

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

float findG(bool i) {	// pos = angle from horizontal
	float w = FAweight[i] + HAweight[i];
	float TFA = FAweight[i] * FAlen[i] * HAcog[i];
	float THA = HAweight[i] * (HAlen[i] * HAcog[i] + FAlen[i]);

	return w * weight * g * (height * (TFA + THA) / w) + RoWeight * RoCOG;
}

float torque_profile(float position, bool IsRo, bool IsDown) {
	if (IsRo) {
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

float T_gravity(float position, bool IsDown, float G, bool IsRo) {

//float k_angle = 1; // coefficient to determine the angle, this would be a cos function
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


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

extern encoder_instance enc_instance_M1; // motor 1
extern encoder_instance enc_instance_M2; // motor 2

extern pid_instance_int16 pid_instance_M1;
extern pid_instance_int16 pid_instance_M2;

extern mov_aver_instance_int16 filter_instance1;
extern mov_aver_instance_int16 filter_instance2;

extern dob_instance dob1;
extern dob_instance dob2;

extern rtob_instance rtob1;
extern rtob_instance rtob2;

extern DFRobot_GP8XXX_IIC gp8211s_1;
extern DFRobot_GP8XXX_IIC gp8211s_2;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1) {
	}
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	FatFsCnt++;
	if (FatFsCnt >= 10) {
		FatFsCnt = 0;
		SDTimer_Handler();
	}

	indx++; // increment the indx in every millisecond
	if (indx == dt) {

		Bimanual_MotorCtrl_M1();

		indx = 0; // reset indx
	}
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

void Bimanual_MotorCtrl_M1(void) {

	update_encoder(&enc_instance_M1, &htim1); // update the encoder1
	update_encoder(&enc_instance_M2, &htim4);

	motor1_vel = enc_instance_M1.velocity * RPM_k; // convert ticks per sec to RPM ----- RPM_k = 60/(PPR*gear_ratio)
	motor2_vel = -enc_instance_M2.velocity * RPM_k; // convert ticks per sec to RPM ----- RPM_k = 60/(PPR*gear_ratio)

	//apply_average_filter(&filter_instance1, motor1_vel_u, &motor1_vel);
	//apply_average_filter(&filter_instance2, motor2_vel_u, &motor2_vel);

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

	//motor1_pos = 180 - motor1_pos;
	//motor2_pos = 180 - motor2_pos;

	T_Dis1 = dob1.T_dis;
	T_Rec1_u = rtob1.T_ext;

	T_Dis2 = dob2.T_dis;
	T_Rec2_u = rtob2.T_ext;

	apply_average_filter(&filter_instance1, T_Rec1_u, &T_Rec1);
	apply_average_filter(&filter_instance2, T_Rec2_u, &T_Rec2);

	T_P1 = torque_profile(motor1_pos, IsRo, IsDown) / (26.0 * 80.0);
	T_P2 = torque_profile(motor2_pos, IsRo, IsDown) / (26.0 * 80.0);

//T_P1 = 1;
//T_P2 = 1;

	T_G1 = T_gravity(motor1_pos, IsDown, G1, IsRo) / 26;
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

	apply_pid(&pid_instance_M1, (T_P1 + T_G1 - T_Rec1), dt);
	apply_pid(&pid_instance_M2,
			(T_P2 + T_G2 - T_Rec2 + k_s * (motor1_pos - motor2_pos)
					+ k_sd * (motor1_vel - motor2_vel)), dt);

	Ia_ref1 = ((pid_instance_M1.output) * (0.25 / 5000.0) + T_Dis1) / k_T; // master
	Ia_ref2 = ((pid_instance_M2.output) * (0.25 / 5000.0) + T_Dis2) / k_T; // slave

	if (fabs(Ia_ref1) > I_max) {
		Ia_ref1 = (Ia_ref1 / fabs(Ia_ref1)) * I_max;
	}
	if (fabs(Ia_ref2) > I_max) {
		Ia_ref2 = (Ia_ref2 / fabs(Ia_ref2)) * I_max;
	}

	if (motor1_vel > -0.5) {
		Ia_ref1 = 0;
		//Ia_ref2 = 0;
	}

	update_dob(&dob1, fabs(Ia_ref1), motor1_vel * RPM_to_Rads_per_sec * 26.0); //  &dob1,  Ia_ref,  velocity
	update_rtob(&rtob1, fabs(Ia_ref1), motor1_vel * RPM_to_Rads_per_sec * 26.0); // &rtob1, Ia_ref, velocity

	update_dob(&dob2, fabs(Ia_ref2), motor2_vel * RPM_to_Rads_per_sec * 26.0); //  &dob1,  Ia_ref,  velocity
	update_rtob(&rtob2, fabs(Ia_ref2), motor2_vel * RPM_to_Rads_per_sec * 26.0); // &rtob1, Ia_ref, velocity

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

	volt1 = fabs(Ia_ref1 * 32767 / I_max);
	volt2 = fabs(Ia_ref2 * 32767 / I_max);

	HAL_GPIO_WritePin(M_DRIVER1_ENABLE_GPIO_Port, M_DRIVER1_ENABLE_Pin,
			GPIO_PIN_SET); // M_Driver1_Enable
	HAL_GPIO_WritePin(M_DRIVER2_ENABLE_GPIO_Port, M_DRIVER2_ENABLE_Pin,
			GPIO_PIN_SET); // M_Driver1_Enable

	if (fabs(motor1_pos - motor2_pos) < 0.5) {
		HAL_GPIO_WritePin(M_DRIVER2_ENABLE_GPIO_Port, M_DRIVER2_ENABLE_Pin,
				GPIO_PIN_RESET); // M_Driver1_Enable
	} else {
		HAL_GPIO_WritePin(M_DRIVER2_ENABLE_GPIO_Port, M_DRIVER2_ENABLE_Pin,
				GPIO_PIN_SET); // M_Driver1_Enable
	}

	GP8XXX_IIC_setDACOutVoltage(&gp8211s_1, volt1, 0);
	GP8XXX_IIC_setDACOutVoltage(&gp8211s_2, volt2, 0); // offset 72 points

}



/* USER CODE END 1 */
