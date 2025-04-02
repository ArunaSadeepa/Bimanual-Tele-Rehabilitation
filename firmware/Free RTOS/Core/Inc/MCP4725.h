#ifndef INC_MCP4725_H_
#define INC_MCP4725_H_

#include "stdio.h"
#include "stdint.h"
#include <main.h>

#define MCP4725_Write_CMD            0x40
#define MCP4725_WriteEEPROM_CMD      0x60

typedef struct {
	I2C_HandleTypeDef *hi2c_ptr;
	uint8_t I2C_addr;
	uint16_t refVoltage;
	uint8_t PowerMode;
	uint16_t dacTest;
} dac_instance;

void init_DAC(dac_instance *dac, I2C_HandleTypeDef *hi2c, uint8_t addr,
		uint16_t vRef, uint8_t powerMode);
void outputVoltage(dac_instance *dac, uint16_t voltage);
void outputVoltageEEPROM(dac_instance *dac, uint16_t voltage);

#endif
