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


#include "platform.h"

/* Helper function to get config data from device */
static inline const struct vl53l4cd_config *get_config(Dev_t dev)
{
    return dev->config;
}

uint8_t VL53L4CD_RdDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t *value)
{
	uint8_t status = 0;
	uint8_t data_write[2];
	uint8_t data_read[4];
	const struct vl53l4cd_config *config = get_config(dev);
	int ret;

	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	
	ret = i2c_write_read_dt(&config->i2c, data_write, 2, data_read, 4);
	if (ret == 0) {
		*value = ((uint32_t)data_read[0] << 24) | ((uint32_t)data_read[1] << 16) |
				 ((uint32_t)data_read[2] << 8) | (uint32_t)data_read[3];
		status = 0;
	} else {
		status = 255;
	}
	
	return status;
}

uint8_t VL53L4CD_RdWord(Dev_t dev, uint16_t RegisterAdress, uint16_t *value)
{
	uint8_t status = 0;
	uint8_t data_write[2];
	uint8_t data_read[2];
	const struct vl53l4cd_config *config = get_config(dev);
	int ret;

	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	
	ret = i2c_write_read_dt(&config->i2c, data_write, 2, data_read, 2);
	if (ret == 0) {
		*value = ((uint16_t)data_read[0] << 8) | (uint16_t)data_read[1];
		status = 0;
	} else {
		status = 255;
	}
	
	return status;
}

uint8_t VL53L4CD_RdByte(Dev_t dev, uint16_t RegisterAdress, uint8_t *value)
{
	uint8_t status = 0;
	uint8_t data_write[2];
	uint8_t data_read[1];
	const struct vl53l4cd_config *config = get_config(dev);
	int ret;

	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	
	ret = i2c_write_read_dt(&config->i2c, data_write, 2, data_read, 1);
	if (ret == 0) {
		*value = data_read[0];
		status = 0;
	} else {
		status = 255;
	}
	
	return status;
}

uint8_t VL53L4CD_WrByte(Dev_t dev, uint16_t RegisterAdress, uint8_t value)
{
	uint8_t data_write[3];
	uint8_t status = 0;
	const struct vl53l4cd_config *config = get_config(dev);
	int ret;

	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	data_write[2] = value & 0xFF;
	
	ret = i2c_write_dt(&config->i2c, data_write, 3);
	if (ret != 0) {
		status = 255;
	}
	
	return status;
}

uint8_t VL53L4CD_WrWord(Dev_t dev, uint16_t RegisterAdress, uint16_t value)
{
	uint8_t data_write[4];
	uint8_t status = 0;
	const struct vl53l4cd_config *config = get_config(dev);
	int ret;
	
	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	data_write[2] = (value >> 8) & 0xFF;
	data_write[3] = value & 0xFF;
	
	ret = i2c_write_dt(&config->i2c, data_write, 4);
	if (ret != 0) {
		status = 255;
	}
	
	return status;
}

uint8_t VL53L4CD_WrDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t value)
{
	uint8_t data_write[6];
	uint8_t status = 0;
	const struct vl53l4cd_config *config = get_config(dev);
	int ret;

	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	data_write[2] = (value >> 24) & 0xFF;
	data_write[3] = (value >> 16) & 0xFF;
	data_write[4] = (value >> 8) & 0xFF;
	data_write[5] = value & 0xFF;
	
	ret = i2c_write_dt(&config->i2c, data_write, 6);
	if (ret != 0) {
		status = 255;
	}
	
	return status;
}

uint8_t VL53L4CD_WaitMs(Dev_t dev, uint32_t TimeMs)
{
	/* Use Zephyr sleep function instead of HAL_Delay */
	k_msleep(TimeMs);
	return 0;
}
