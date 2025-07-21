/**
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

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include "platform_zephyr.h"

LOG_MODULE_REGISTER(vl53l4cd_platform, LOG_LEVEL_DBG);

/* I2C device binding - will be set during initialization */
static const struct device *i2c_dev = NULL;

/* VL53L4CD I2C address (7-bit) */
#define VL53L4CD_I2C_ADDR 0x29

/* Check if VL53L4CD_I2C_FAST_MODE_PLUS is defined via Kconfig */
#ifdef CONFIG_VL53L4CD_I2C_FAST_MODE_PLUS
#define VL53L4CD_I2C_FAST_MODE_PLUS
#endif

int vl53l4cd_platform_init(void)
{
	/* Get I2C device binding */
	i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}
	
	LOG_INF("VL53L4CD platform initialized");
	return 0;
}

uint8_t VL53L4CD_RdDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t *value)
{
	uint8_t status = 0;
	uint8_t buffer[4];
	uint8_t reg_addr[2];
	
	if (i2c_dev == NULL) {
		LOG_ERR("I2C device not initialized");
		return 255;
	}
	
	/* Convert register address to big endian */
	reg_addr[0] = (RegisterAdress >> 8) & 0xFF;
	reg_addr[1] = RegisterAdress & 0xFF;
	
	/* Write register address and read data */
	status = i2c_write_read(i2c_dev, VL53L4CD_I2C_ADDR, 
			       reg_addr, 2, buffer, 4);
	
	if (status == 0) {
		/* Convert from big endian to host byte order */
		*value = ((uint32_t)buffer[0] << 24) | 
			 ((uint32_t)buffer[1] << 16) |
			 ((uint32_t)buffer[2] << 8) | 
			 (uint32_t)buffer[3];
	} else {
		LOG_ERR("I2C read DWord failed: %d", status);
		status = 255;
	}
	
	return status;
}

uint8_t VL53L4CD_RdWord(Dev_t dev, uint16_t RegisterAdress, uint16_t *value)
{
	uint8_t status = 0;
	uint8_t buffer[2];
	uint8_t reg_addr[2];
	
	if (i2c_dev == NULL) {
		LOG_ERR("I2C device not initialized");
		return 255;
	}
	
	/* Convert register address to big endian */
	reg_addr[0] = (RegisterAdress >> 8) & 0xFF;
	reg_addr[1] = RegisterAdress & 0xFF;
	
	/* Write register address and read data */
	status = i2c_write_read(i2c_dev, VL53L4CD_I2C_ADDR, 
			       reg_addr, 2, buffer, 2);
	
	if (status == 0) {
		/* Convert from big endian to host byte order */
		*value = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
	} else {
		LOG_ERR("I2C read Word failed: %d", status);
		status = 255;
	}
	
	return status;
}

uint8_t VL53L4CD_RdByte(Dev_t dev, uint16_t RegisterAdress, uint8_t *value)
{
	uint8_t status = 0;
	uint8_t reg_addr[2];
	
	if (i2c_dev == NULL) {
		LOG_ERR("I2C device not initialized");
		return 255;
	}
	
	/* Convert register address to big endian */
	reg_addr[0] = (RegisterAdress >> 8) & 0xFF;
	reg_addr[1] = RegisterAdress & 0xFF;
	
	/* Write register address and read data */
	status = i2c_write_read(i2c_dev, VL53L4CD_I2C_ADDR, 
			       reg_addr, 2, value, 1);
	
	if (status != 0) {
		LOG_ERR("I2C read Byte failed: %d", status);
		status = 255;
	}
	
	return status;
}

uint8_t VL53L4CD_WrByte(Dev_t dev, uint16_t RegisterAdress, uint8_t value)
{
	uint8_t status = 0;
	uint8_t buffer[3];
	
	if (i2c_dev == NULL) {
		LOG_ERR("I2C device not initialized");
		return 255;
	}
	
	/* Prepare data: register address (big endian) + value */
	buffer[0] = (RegisterAdress >> 8) & 0xFF;
	buffer[1] = RegisterAdress & 0xFF;
	buffer[2] = value;
	
	/* Write data */
	status = i2c_write(i2c_dev, buffer, 3, VL53L4CD_I2C_ADDR);
	
	if (status != 0) {
		LOG_ERR("I2C write Byte failed: %d", status);
		status = 255;
	}
	
	return status;
}

uint8_t VL53L4CD_WrWord(Dev_t dev, uint16_t RegisterAdress, uint16_t value)
{
	uint8_t status = 0;
	uint8_t buffer[4];
	
	if (i2c_dev == NULL) {
		LOG_ERR("I2C device not initialized");
		return 255;
	}
	
	/* Prepare data: register address (big endian) + value (big endian) */
	buffer[0] = (RegisterAdress >> 8) & 0xFF;
	buffer[1] = RegisterAdress & 0xFF;
	buffer[2] = (value >> 8) & 0xFF;
	buffer[3] = value & 0xFF;
	
	/* Write data */
	status = i2c_write(i2c_dev, buffer, 4, VL53L4CD_I2C_ADDR);
	
	if (status != 0) {
		LOG_ERR("I2C write Word failed: %d", status);
		status = 255;
	}
	
	return status;
}

uint8_t VL53L4CD_WrDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t value)
{
	uint8_t status = 0;
	uint8_t buffer[6];
	
	if (i2c_dev == NULL) {
		LOG_ERR("I2C device not initialized");
		return 255;
	}
	
	/* Prepare data: register address (big endian) + value (big endian) */
	buffer[0] = (RegisterAdress >> 8) & 0xFF;
	buffer[1] = RegisterAdress & 0xFF;
	buffer[2] = (value >> 24) & 0xFF;
	buffer[3] = (value >> 16) & 0xFF;
	buffer[4] = (value >> 8) & 0xFF;
	buffer[5] = value & 0xFF;
	
	/* Write data */
	status = i2c_write(i2c_dev, buffer, 6, VL53L4CD_I2C_ADDR);
	
	if (status != 0) {
		LOG_ERR("I2C write DWord failed: %d", status);
		status = 255;
	}
	
	return status;
}

uint8_t VL53L4CD_WaitMs(Dev_t dev, uint32_t TimeMs)
{
	k_msleep(TimeMs);
	return 0;
}
