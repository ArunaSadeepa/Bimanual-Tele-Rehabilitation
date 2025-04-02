/*
 * DFR_i2c.h
 *
 *  Created on: Apr 9, 2024
 * 
 */

#ifndef INC_DFR_I2C_H_
#define INC_DFR_I2C_H_

#include <stdint.h>
#include <main.h>

// Definitions for DAC resolutions
#define RESOLUTION_12_BIT 0x0FFF
#define RESOLUTION_15_BIT 0x7FFF

// Configuration registers and addresses
#define GP8XXX_CONFIG_CURRENT_REG     0x02
#define DFGP8XXX_I2C_DEVICEADDR       0x58  //     7 bit i2c address

#define GP8XXX_STORE_TIMING_HEAD            0x02  ///< Store function timing start head
#define GP8XXX_STORE_TIMING_ADDR            0x10  ///< The first address for entering store timing
#define GP8XXX_STORE_TIMING_CMD1            0x03  ///< The command 1 to enter store timing
#define GP8XXX_STORE_TIMING_CMD2            0x00  ///< The command 2 to enter store timing
#define GP8XXX_STORE_TIMING_DELAY           10    ///< Store procedure interval delay time: 10ms, more than 7ms
#define I2C_CYCLE_TOTAL                     5     ///< Total I2C communication cycle
#define I2C_CYCLE_BEFORE                    1     ///< The first half cycle 2 of the total I2C communication cycle
#define I2C_CYCLE_AFTER                     2     ///< The second half cycle 3 of the total I2C communication cycle

#define CLOCK SystemCoreClock/18000000  // change this accordingly to match SCL frequency

// Enumeration for DAC output ranges
typedef enum {
	eOutputRange2_5V = 0,
	eOutputRange5V = 1,
	eOutputRange10V = 2,
	eOutputRangeVCC = 3
} eOutPutRange_t;

// Enumeration for DAC models
typedef enum {
	GP8211_identifier = 0,
	GP8211S_identifier = 1,
	GP8512_identifier = 2,
	GP8302_identifier = 3,
	GP8403_identifier = 4,
	GP8413_identifier = 5,
	GP8503_identifier = 6
} eDAC_Name_t;

// Struct to hold DAC configuration parameters
typedef struct {
    uint16_t resolution;        // Resolution of the DAC
    eDAC_Name_t DAC_Name;       // Identifier for the DAC model
    uint8_t deviceAddr;         // I2C device address
    GPIO_TypeDef *I2C_scl_port; // GPIO port for I2C SCL pin
    uint16_t I2C_scl_pin;       // Pin number for I2C SCL pin
    GPIO_TypeDef *I2C_sda_port; // GPIO port for I2C SDA pin
    uint16_t I2C_sda_pin;       // Pin number for I2C SDA pin
} DFRobot_GP8XXX_IIC;

/* Function prototypes */

/**
 * @fn GP8XXX_IIC_begin
 * @brief Initializes I2C communication for the DAC
 * @param gp8xxx Pointer to DFRobot_GP8XXX_IIC struct containing DAC configuration
 * @param dac_name Identifier for the DAC model
 * @param Addr I2C device address
 * @param SCL_port GPIO port for I2C SCL pin
 * @param SCL_pin Pin number for I2C SCL pin
 * @param SDA_port GPIO port for I2C SDA pin
 * @param SDA_pin Pin number for I2C SDA pin
 * @return None
 */
void GP8XXX_IIC_begin(DFRobot_GP8XXX_IIC *gp8xxx, eDAC_Name_t dac_name,
		uint8_t Addr, GPIO_TypeDef *SCL_port, uint16_t SCL_pin,
		GPIO_TypeDef *SDA_port, uint16_t SDA_pin);

/**
 * @fn GP8XXX_IIC_setDACOutRange
 * @brief Sets the output range for the DAC
 * @param gp8xxx Pointer to DFRobot_GP8XXX_IIC struct containing DAC configuration
 * @param range Desired output range for the DAC
 * @return 0 for success, other values for failure
 */
int GP8XXX_IIC_setDACOutRange(DFRobot_GP8XXX_IIC *gp8xxx, eOutPutRange_t range);

/**
 * @fn GP8XXX_IIC_setDACOutVoltage
 * @brief Sets the output voltage for the DAC
 * @param gp8xxx Pointer to DFRobot_GP8XXX_IIC struct containing DAC configuration
 * @param voltage Desired output voltage value
 * @param channel Output channel:
 *                0: Channel 0
 *                1: Channel 1
 *                2: All channels
 * @return 0 for success, other values for failure
 */
int GP8XXX_IIC_setDACOutVoltage(DFRobot_GP8XXX_IIC *gp8xxx, uint16_t voltage,
		uint8_t channel);

/**
 * @fn GP8XXX_IIC_store
 * @brief stores the voltage value
 * @param gp8xxx Pointer to DFRobot_GP8XXX_IIC struct containing DAC configuration
 * @return 0 for success, other values for failure
 */
int GP8XXX_IIC_store(DFRobot_GP8XXX_IIC *gp8xxx);

#endif /* INC_DFR_I2C_H_ */
