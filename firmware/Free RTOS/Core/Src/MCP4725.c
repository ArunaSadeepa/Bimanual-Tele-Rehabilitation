#include "MCP4725.h"

void init_DAC(dac_instance *dac, I2C_HandleTypeDef *hi2c, uint8_t addr,
		uint16_t vRef, uint8_t powerMode) {
	// Set the values of the parameters
	dac->hi2c_ptr = hi2c;
	dac->I2C_addr = addr;
	dac->refVoltage = vRef;
	dac->PowerMode = powerMode;
}

void outputVoltage(dac_instance *dac, uint16_t voltage) {
	int ret1;
	uint8_t data[3]; // Increase size to accommodate three bytes

	if (voltage > dac->refVoltage) {
		// Handle error
		return;
	} else {
		uint16_t dacValue = (uint16_t) (((float) voltage / dac->refVoltage)
				* 4095); // 1638 for 2000
		dac->dacTest = dacValue;
		// data[0] = MCP4725_Write_CMD | (dac->PowerMode << 1);     // dont need this since we operate at normal mode
		data[0] = MCP4725_Write_CMD; // MCP4725_Write_CMD - 0x40 = 64
		data[1] = (dacValue >> 4) & 0xFF; // 8 MSBs of data
		data[2] = dacValue & 0x0F; // 4 LSBs of data
		ret1 = HAL_I2C_Master_Transmit(dac->hi2c_ptr, dac->I2C_addr, data, 3,
		HAL_MAX_DELAY);
	}
}

void outputVoltageEEPROM(dac_instance *dac, uint16_t voltage) {
	int ret2;
	uint8_t data[3]; // Increase size to accommodate three bytes

	if (voltage > dac->refVoltage) {
		// Handle error
		return;
	} else {
		uint16_t dacValue = (uint16_t) (((float) voltage / dac->refVoltage)
				* 4095);
		dac->dacTest = dacValue;
		data[0] = MCP4725_WriteEEPROM_CMD | (dac->PowerMode << 1); // MCP4725_WriteEEPROM_CMD      0x60 = 96
		data[1] = (dacValue >> 4) & 0xFF;
		data[2] = dacValue & 0x0F;
		ret2 = HAL_I2C_Master_Transmit(dac->hi2c_ptr, dac->I2C_addr, data, 3,
		HAL_MAX_DELAY);
	}
}
