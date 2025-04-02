/*
 * DFR_i2c.c
 *
 *  Created on: Apr 9, 2024
 *
 */

#include "DFR_i2c.h"

void Delay_us(uint32_t delay) {
	// Calculate the number of CPU cycles for the delay
	// change CLOCK preprocessor if the SCL is not compatible with the micro-controller
	uint32_t cycles = CLOCK* delay;

	for (volatile uint32_t i = 0; i < cycles; i++) {
		__asm__ __volatile__("nop");
	}
}

void dfr_i2c_start(DFRobot_GP8XXX_IIC *gp8xxx) {

	/* Generate I2C start condition */
	HAL_GPIO_WritePin(gp8xxx->I2C_scl_port, gp8xxx->I2C_scl_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gp8xxx->I2C_sda_port, gp8xxx->I2C_sda_pin, GPIO_PIN_SET);
	Delay_us(I2C_CYCLE_BEFORE);
	HAL_GPIO_WritePin(gp8xxx->I2C_sda_port, gp8xxx->I2C_sda_pin, GPIO_PIN_RESET);
	Delay_us(I2C_CYCLE_AFTER);
	HAL_GPIO_WritePin(gp8xxx->I2C_scl_port, gp8xxx->I2C_scl_pin, GPIO_PIN_RESET);
	Delay_us(I2C_CYCLE_TOTAL);

}

void dfr_i2c_stop(DFRobot_GP8XXX_IIC *gp8xxx) {

	/* Generate I2C stop condition */
	HAL_GPIO_WritePin(gp8xxx->I2C_sda_port, gp8xxx->I2C_sda_pin, GPIO_PIN_RESET);
	Delay_us(I2C_CYCLE_BEFORE);
	HAL_GPIO_WritePin(gp8xxx->I2C_scl_port, gp8xxx->I2C_scl_pin, GPIO_PIN_SET);
	Delay_us(I2C_CYCLE_TOTAL);
	HAL_GPIO_WritePin(gp8xxx->I2C_sda_port, gp8xxx->I2C_sda_pin, GPIO_PIN_SET);
	Delay_us(I2C_CYCLE_TOTAL);

}

void dfr_i2c_write_1_bit(DFRobot_GP8XXX_IIC *gp8xxx,uint8_t bit) {

	// Write 1 bit to the I2C bus
	HAL_GPIO_WritePin(gp8xxx->I2C_sda_port, gp8xxx->I2C_sda_pin,
			bit ? GPIO_PIN_SET : GPIO_PIN_RESET);
	Delay_us(I2C_CYCLE_BEFORE);
	HAL_GPIO_WritePin(gp8xxx->I2C_scl_port, gp8xxx->I2C_scl_pin, GPIO_PIN_SET);
	Delay_us(I2C_CYCLE_TOTAL);
	HAL_GPIO_WritePin(gp8xxx->I2C_scl_port, gp8xxx->I2C_scl_pin, GPIO_PIN_RESET);
	Delay_us(I2C_CYCLE_AFTER);

}

HAL_StatusTypeDef dfr_i2c_write_byte(DFRobot_GP8XXX_IIC *gp8xxx,uint8_t data) {

	/* Write a byte to the I2C bus */
	for (int i = 0; i < 8; i++) {
		HAL_GPIO_WritePin(gp8xxx->I2C_sda_port, gp8xxx->I2C_sda_pin,
				(data & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		Delay_us(I2C_CYCLE_BEFORE);
		HAL_GPIO_WritePin(gp8xxx->I2C_scl_port, gp8xxx->I2C_scl_pin, GPIO_PIN_SET);
		Delay_us(I2C_CYCLE_TOTAL);
		HAL_GPIO_WritePin(gp8xxx->I2C_scl_port, gp8xxx->I2C_scl_pin, GPIO_PIN_RESET);
		Delay_us(I2C_CYCLE_AFTER);
		data <<= 1;
	}

	// Check for ACK from slave, expecting 0
	HAL_GPIO_WritePin(gp8xxx->I2C_sda_port, gp8xxx->I2C_sda_pin, GPIO_PIN_SET);
	Delay_us(I2C_CYCLE_BEFORE);
	HAL_GPIO_WritePin(gp8xxx->I2C_scl_port, gp8xxx->I2C_scl_pin, GPIO_PIN_SET);
	Delay_us(I2C_CYCLE_AFTER);

	uint16_t errorTime = 0;
	while (HAL_GPIO_ReadPin(gp8xxx->I2C_sda_port, gp8xxx->I2C_sda_pin) != GPIO_PIN_RESET) {
		Delay_us(I2C_CYCLE_BEFORE);
		errorTime++;
		if (errorTime > 100) { // No ACK from slave, generate stop condition and return error
			dfr_i2c_stop(gp8xxx);
			return HAL_ERROR;
		}
	}
	// ACK received as 0
	HAL_GPIO_WritePin(gp8xxx->I2C_scl_port, gp8xxx->I2C_scl_pin, GPIO_PIN_RESET);
	Delay_us(I2C_CYCLE_AFTER);
	return HAL_OK;
}

void GP8XXX_IIC_begin(DFRobot_GP8XXX_IIC *gp8xxx, eDAC_Name_t dac_name,
		uint8_t Addr, GPIO_TypeDef *SCL_port, uint16_t SCL_pin,
		GPIO_TypeDef *SDA_port, uint16_t SDA_pin) {

	 // Set I2C port and pin configurations

	gp8xxx->I2C_scl_port = SCL_port;
	gp8xxx->I2C_scl_pin = SCL_pin;
	gp8xxx->I2C_sda_port = SDA_port;
	gp8xxx->I2C_sda_pin = SDA_pin;

	/* Enable GPIO clock for SCL port */
	switch ((uint32_t) gp8xxx->I2C_scl_port) {
	case GPIOA_BASE:
		__HAL_RCC_GPIOA_CLK_ENABLE();
		break;
	case GPIOB_BASE:
		__HAL_RCC_GPIOB_CLK_ENABLE();
		break;
	case GPIOC_BASE:
		__HAL_RCC_GPIOC_CLK_ENABLE();
		break;
	case GPIOD_BASE:
		__HAL_RCC_GPIOD_CLK_ENABLE();
		break;
	case GPIOE_BASE:
		__HAL_RCC_GPIOE_CLK_ENABLE();
		break;
	case GPIOF_BASE:
		__HAL_RCC_GPIOF_CLK_ENABLE();
		break;
	case GPIOG_BASE:
		__HAL_RCC_GPIOG_CLK_ENABLE();
		break;
	case GPIOH_BASE:
		__HAL_RCC_GPIOH_CLK_ENABLE();
		break;
	case GPIOI_BASE:
		__HAL_RCC_GPIOI_CLK_ENABLE();
		break;
	case GPIOJ_BASE:
		__HAL_RCC_GPIOJ_CLK_ENABLE();
		break;
	case GPIOK_BASE:
		__HAL_RCC_GPIOK_CLK_ENABLE();
		break;
	default:
		break;
	}

	/* Enable GPIO clock for SDA port */
	switch ((uint32_t) gp8xxx->I2C_sda_port) {
	case GPIOA_BASE:
		__HAL_RCC_GPIOA_CLK_ENABLE();
		break;
	case GPIOB_BASE:
		__HAL_RCC_GPIOB_CLK_ENABLE();
		break;
	case GPIOC_BASE:
		__HAL_RCC_GPIOC_CLK_ENABLE();
		break;
	case GPIOD_BASE:
		__HAL_RCC_GPIOD_CLK_ENABLE();
		break;
	case GPIOE_BASE:
		__HAL_RCC_GPIOE_CLK_ENABLE();
		break;
	case GPIOF_BASE:
		__HAL_RCC_GPIOF_CLK_ENABLE();
		break;
	case GPIOG_BASE:
		__HAL_RCC_GPIOG_CLK_ENABLE();
		break;
	case GPIOH_BASE:
		__HAL_RCC_GPIOH_CLK_ENABLE();
		break;
	case GPIOI_BASE:
		__HAL_RCC_GPIOI_CLK_ENABLE();
		break;
	case GPIOJ_BASE:
		__HAL_RCC_GPIOJ_CLK_ENABLE();
		break;
	case GPIOK_BASE:
		__HAL_RCC_GPIOK_CLK_ENABLE();
		break;
	default:
		break;
	}

	// Configure GPIO pins for I2C
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = gp8xxx->I2C_scl_pin | gp8xxx->I2C_sda_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(gp8xxx->I2C_scl_port, &GPIO_InitStruct);
	HAL_GPIO_Init(gp8xxx->I2C_sda_port, &GPIO_InitStruct);


	// Set DAC model, resolution, and device address
	gp8xxx->deviceAddr = Addr;
	gp8xxx->DAC_Name = dac_name;

	switch (gp8xxx->DAC_Name) {
	case GP8211_identifier:
	case GP8211S_identifier:
	case GP8512_identifier:
		gp8xxx->resolution = RESOLUTION_15_BIT;
		gp8xxx->deviceAddr = DFGP8XXX_I2C_DEVICEADDR;

		break;
	case GP8413_identifier:
		gp8xxx->resolution = RESOLUTION_15_BIT;
		break;

	case GP8302_identifier:
		gp8xxx->resolution = RESOLUTION_12_BIT;
		gp8xxx->deviceAddr = DFGP8XXX_I2C_DEVICEADDR;
		break;

	case GP8503_identifier:
	case GP8403_identifier:
		gp8xxx->resolution = RESOLUTION_12_BIT;
		break;

	default:
		gp8xxx->resolution = RESOLUTION_15_BIT;
		gp8xxx->deviceAddr = DFGP8XXX_I2C_DEVICEADDR;
		break;

	}

}

int GP8XXX_IIC_setDACOutRange(DFRobot_GP8XXX_IIC *gp8xxx, eOutPutRange_t range) {

// only for 8402, 8413, 8211s, 8211, 8101

	uint8_t data = 0x00;

	switch (range) {

	case eOutputRange5V:
		break;

	case eOutputRange10V:
		data = 0x11;
		break;

	default:
		data = 0x00;
		break;

	}

	// Generate start condition
	dfr_i2c_start(gp8xxx);

	// Send device address
	dfr_i2c_write_byte(gp8xxx,gp8xxx->deviceAddr << 1);

	// Send register address
	dfr_i2c_write_byte(gp8xxx,GP8XXX_CONFIG_CURRENT_REG >> 1);

	// Send voltage data
	dfr_i2c_write_byte(gp8xxx,data);

	// Generate stop condition
	dfr_i2c_stop(gp8xxx);

	return HAL_OK;
}

int GP8XXX_IIC_setDACOutVoltage(DFRobot_GP8XXX_IIC *gp8xxx, uint16_t voltage,
		uint8_t channel) {

	// Set initial state of I2C lines
	if (voltage > gp8xxx->resolution)
		voltage = gp8xxx->resolution;

	if (gp8xxx->resolution == RESOLUTION_12_BIT) {
		voltage = voltage << 4;

	} else if (gp8xxx->resolution == RESOLUTION_15_BIT) {
		voltage = voltage << 1;

	}

	uint8_t buff[4] = { (uint8_t) (voltage & 0xff), (uint8_t) (voltage >> 8),
			(uint8_t) (voltage & 0xff), (uint8_t) (voltage >> 8) };

	if (channel == 0) {

		// Generate start condition
		dfr_i2c_start(gp8xxx);

		// Send device address
		dfr_i2c_write_byte(gp8xxx,gp8xxx->deviceAddr << 1);

		// Send register address
		dfr_i2c_write_byte(gp8xxx,GP8XXX_CONFIG_CURRENT_REG);

		// Send voltage data
		dfr_i2c_write_byte(gp8xxx,buff[0]);
		dfr_i2c_write_byte(gp8xxx,buff[1]);

		// Generate stop condition
		dfr_i2c_stop(gp8xxx);

		return HAL_OK;

	} else if (channel == 1) {

		// Generate start condition
		dfr_i2c_start(gp8xxx);

		// Send device address
		dfr_i2c_write_byte(gp8xxx,gp8xxx->deviceAddr << 1);

		// Send register address
		dfr_i2c_write_byte(gp8xxx,GP8XXX_CONFIG_CURRENT_REG << 1);

		// Send voltage data
		dfr_i2c_write_byte(gp8xxx,buff[0]);
		dfr_i2c_write_byte(gp8xxx,buff[1]);

		// Generate stop condition
		dfr_i2c_stop(gp8xxx);

		return HAL_OK;

	} else if (channel == 2) {

		// Generate start condition
		dfr_i2c_start(gp8xxx);

		// Send device address
		dfr_i2c_write_byte(gp8xxx,gp8xxx->deviceAddr << 1);

		// Send register address
		dfr_i2c_write_byte(gp8xxx,GP8XXX_CONFIG_CURRENT_REG);

		// Send voltage data
		dfr_i2c_write_byte(gp8xxx,buff[0]);
		dfr_i2c_write_byte(gp8xxx,buff[1]);
		dfr_i2c_write_byte(gp8xxx,buff[2]);
		dfr_i2c_write_byte(gp8xxx,buff[3]);

		// Generate stop condition
		dfr_i2c_stop(gp8xxx);
		return HAL_OK;
	} else
		return HAL_ERROR;

}

int GP8XXX_IIC_store(DFRobot_GP8XXX_IIC *gp8xxx) {

	uint16_t errorTime = 0;

	dfr_i2c_start(gp8xxx);

	for (uint8_t i = 0; i < 3; i++) {
		dfr_i2c_write_1_bit(gp8xxx,(GP8XXX_STORE_TIMING_HEAD << i) & 0x4);
	}

	// flag = false
	HAL_GPIO_WritePin(gp8xxx->I2C_sda_port, gp8xxx->I2C_sda_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gp8xxx->I2C_scl_port, gp8xxx->I2C_scl_pin, GPIO_PIN_SET);

	dfr_i2c_stop(gp8xxx);

	dfr_i2c_start(gp8xxx);

	dfr_i2c_write_byte(gp8xxx,GP8XXX_STORE_TIMING_ADDR);
	dfr_i2c_write_byte(gp8xxx,GP8XXX_STORE_TIMING_CMD1);

	dfr_i2c_stop(gp8xxx);

	dfr_i2c_start(gp8xxx);

	for (uint8_t i = 1; i < 9; i++) {
		dfr_i2c_write_1_bit(gp8xxx,(DFGP8XXX_I2C_DEVICEADDR << i) & 0x80);
	}

	// Check for ACK from slave waiting for 1
	HAL_GPIO_WritePin(gp8xxx->I2C_sda_port, gp8xxx->I2C_sda_pin, GPIO_PIN_SET);
	Delay_us(I2C_CYCLE_BEFORE);
	HAL_GPIO_WritePin(gp8xxx->I2C_scl_port, gp8xxx->I2C_scl_pin, GPIO_PIN_SET);
	Delay_us(I2C_CYCLE_AFTER);

	while (HAL_GPIO_ReadPin(gp8xxx->I2C_sda_port, gp8xxx->I2C_sda_pin) != GPIO_PIN_SET) {
		Delay_us(I2C_CYCLE_BEFORE);
		errorTime++;
		if (errorTime > 100) {
			dfr_i2c_stop(gp8xxx);
			return HAL_ERROR;
		}
	}
	errorTime = 0;

	HAL_GPIO_WritePin(gp8xxx->I2C_scl_port, gp8xxx->I2C_scl_pin, GPIO_PIN_RESET);
	Delay_us(I2C_CYCLE_AFTER);

	for (uint8_t j = 0; j < 8; j++) {

		for (uint8_t i = 0; i < 8; i++) {
			dfr_i2c_write_1_bit(gp8xxx,(GP8XXX_STORE_TIMING_CMD2 << i) & 0x80);
		}
		// Check for ACK from slave waiting for 100 cycles
		HAL_GPIO_WritePin(gp8xxx->I2C_sda_port, gp8xxx->I2C_sda_pin, GPIO_PIN_SET);
		Delay_us(I2C_CYCLE_BEFORE);
		HAL_GPIO_WritePin(gp8xxx->I2C_scl_port, gp8xxx->I2C_scl_pin, GPIO_PIN_SET);
		Delay_us(I2C_CYCLE_AFTER);

		while (HAL_GPIO_ReadPin(gp8xxx->I2C_sda_port, gp8xxx->I2C_sda_pin) != GPIO_PIN_SET) {
			Delay_us(I2C_CYCLE_BEFORE);
			errorTime++;
			if (errorTime > 100) {
				dfr_i2c_stop(gp8xxx);
				return HAL_ERROR;
			}
		}
		errorTime = 0;
		HAL_GPIO_WritePin(gp8xxx->I2C_scl_port, gp8xxx->I2C_scl_pin, GPIO_PIN_RESET);
		Delay_us(I2C_CYCLE_AFTER);

	}

	dfr_i2c_stop(gp8xxx);

	HAL_Delay(GP8XXX_STORE_TIMING_DELAY);

	dfr_i2c_start(gp8xxx);

	for (uint8_t i = 0; i < 3; i++) {
		dfr_i2c_write_1_bit(gp8xxx,(GP8XXX_STORE_TIMING_HEAD << i) & 0x4);
	}

	// flag = false
	HAL_GPIO_WritePin(gp8xxx->I2C_sda_port, gp8xxx->I2C_sda_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gp8xxx->I2C_scl_port, gp8xxx->I2C_scl_pin, GPIO_PIN_SET);

	dfr_i2c_stop(gp8xxx);

	dfr_i2c_start(gp8xxx);

	dfr_i2c_write_byte(gp8xxx,GP8XXX_STORE_TIMING_ADDR);
	dfr_i2c_write_byte(gp8xxx,GP8XXX_STORE_TIMING_CMD2);

	dfr_i2c_stop(gp8xxx);

	return HAL_OK;

}
