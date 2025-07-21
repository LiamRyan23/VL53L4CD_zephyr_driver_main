/* vl53l4cd_platform.c - Zephyr platform implementation for VL53L4CD driver */

/*
 * Copyright (c) 2023 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "vl53l4cd_platform.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(vl53l4cd_platform, CONFIG_SENSOR_LOG_LEVEL);

#define VL53L4CD_POLLING_DELAY_MS	1

VL53L4CD_Error VL53L4CD_WriteMulti(VL53L4CD_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
	int ret;
	uint8_t buf[count + 2];
	
	if (!Dev || !Dev->i2c || !pdata) {
		return VL53L4CD_ERROR_INVALID_ARGUMENT;
	}
	
	/* Convert 16-bit register address to big-endian */
	buf[0] = (index >> 8) & 0xFF;
	buf[1] = index & 0xFF;
	memcpy(&buf[2], pdata, count);
	
	ret = i2c_write(Dev->i2c, buf, count + 2, Dev->I2cDevAddr);
	if (ret < 0) {
		LOG_ERR("I2C write error: %d", ret);
		return VL53L4CD_ERROR_CONTROL_INTERFACE;
	}
	
	return VL53L4CD_ERROR_NONE;
}

VL53L4CD_Error VL53L4CD_ReadMulti(VL53L4CD_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
	int ret;
	uint8_t reg_addr[2];
	
	if (!Dev || !Dev->i2c || !pdata) {
		return VL53L4CD_ERROR_INVALID_ARGUMENT;
	}
	
	/* Convert 16-bit register address to big-endian */
	reg_addr[0] = (index >> 8) & 0xFF;
	reg_addr[1] = index & 0xFF;
	
	ret = i2c_write_read(Dev->i2c, Dev->I2cDevAddr, reg_addr, 2, pdata, count);
	if (ret < 0) {
		LOG_ERR("I2C read error: %d", ret);
		return VL53L4CD_ERROR_CONTROL_INTERFACE;
	}
	
	return VL53L4CD_ERROR_NONE;
}

VL53L4CD_Error VL53L4CD_WrByte(VL53L4CD_DEV Dev, uint16_t index, uint8_t data)
{
	return VL53L4CD_WriteMulti(Dev, index, &data, 1);
}

VL53L4CD_Error VL53L4CD_WrWord(VL53L4CD_DEV Dev, uint16_t index, uint16_t data)
{
	uint8_t buf[2];
	
	/* Convert to big-endian */
	buf[0] = (data >> 8) & 0xFF;
	buf[1] = data & 0xFF;
	
	return VL53L4CD_WriteMulti(Dev, index, buf, 2);
}

VL53L4CD_Error VL53L4CD_WrDWord(VL53L4CD_DEV Dev, uint16_t index, uint32_t data)
{
	uint8_t buf[4];
	
	/* Convert to big-endian */
	buf[0] = (data >> 24) & 0xFF;
	buf[1] = (data >> 16) & 0xFF;
	buf[2] = (data >> 8) & 0xFF;
	buf[3] = data & 0xFF;
	
	return VL53L4CD_WriteMulti(Dev, index, buf, 4);
}

VL53L4CD_Error VL53L4CD_RdByte(VL53L4CD_DEV Dev, uint16_t index, uint8_t *data)
{
	return VL53L4CD_ReadMulti(Dev, index, data, 1);
}

VL53L4CD_Error VL53L4CD_RdWord(VL53L4CD_DEV Dev, uint16_t index, uint16_t *data)
{
	uint8_t buf[2];
	VL53L4CD_Error status;
	
	status = VL53L4CD_ReadMulti(Dev, index, buf, 2);
	if (status == VL53L4CD_ERROR_NONE) {
		/* Convert from big-endian */
		*data = ((uint16_t)buf[0] << 8) | buf[1];
	}
	
	return status;
}

VL53L4CD_Error VL53L4CD_RdDWord(VL53L4CD_DEV Dev, uint16_t index, uint32_t *data)
{
	uint8_t buf[4];
	VL53L4CD_Error status;
	
	status = VL53L4CD_ReadMulti(Dev, index, buf, 4);
	if (status == VL53L4CD_ERROR_NONE) {
		/* Convert from big-endian */
		*data = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
		        ((uint32_t)buf[2] << 8) | buf[3];
	}
	
	return status;
}

VL53L4CD_Error VL53L4CD_PollingDelay(VL53L4CD_DEV Dev)
{
	ARG_UNUSED(Dev);
	k_msleep(VL53L4CD_POLLING_DELAY_MS);
	return VL53L4CD_ERROR_NONE;
}

/* VL53L4CD API implementations */

VL53L4CD_Error VL53L4CD_GetSensorId(VL53L4CD_DEV Dev, uint16_t *p_id)
{
	VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
	
	if (!Dev || !p_id) {
		return VL53L4CD_ERROR_INVALID_ARGUMENT;
	}
	
	status = VL53L4CD_RdWord(Dev, 0x010F, p_id);
	
	return status;
}

VL53L4CD_Error VL53L4CD_SensorInit(VL53L4CD_DEV Dev)
{
	VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
	uint8_t temp;
	
	if (!Dev) {
		return VL53L4CD_ERROR_INVALID_ARGUMENT;
	}
	
	/* Basic sensor initialization sequence */
	status |= VL53L4CD_WrByte(Dev, 0x002E, 0x00);
	status |= VL53L4CD_WrByte(Dev, 0x002F, 0x00);
	status |= VL53L4CD_WrByte(Dev, 0x0030, 0x11);
	status |= VL53L4CD_WrByte(Dev, 0x0031, 0x02);
	status |= VL53L4CD_WrByte(Dev, 0x0032, 0x00);
	status |= VL53L4CD_WrByte(Dev, 0x0033, 0x02);
	
	/* Check if initialization was successful */
	status |= VL53L4CD_RdByte(Dev, 0x00E5, &temp);
	if (temp != 0x03) {
		status = VL53L4CD_ERROR_TIMEOUT;
	}
	
	return status;
}

VL53L4CD_Error VL53L4CD_StartRanging(VL53L4CD_DEV Dev)
{
	if (!Dev) {
		return VL53L4CD_ERROR_INVALID_ARGUMENT;
	}
	
	return VL53L4CD_WrByte(Dev, 0x0087, 0x40);
}

VL53L4CD_Error VL53L4CD_StopRanging(VL53L4CD_DEV Dev)
{
	if (!Dev) {
		return VL53L4CD_ERROR_INVALID_ARGUMENT;
	}
	
	return VL53L4CD_WrByte(Dev, 0x0087, 0x00);
}

VL53L4CD_Error VL53L4CD_CheckForDataReady(VL53L4CD_DEV Dev, uint8_t *p_is_data_ready)
{
	VL53L4CD_Error status;
	uint8_t temp;
	
	if (!Dev || !p_is_data_ready) {
		return VL53L4CD_ERROR_INVALID_ARGUMENT;
	}
	
	status = VL53L4CD_RdByte(Dev, 0x0031, &temp);
	*p_is_data_ready = (temp & 0x01) ? 1 : 0;
	
	return status;
}

VL53L4CD_Error VL53L4CD_GetResult(VL53L4CD_DEV Dev, VL53L4CD_ResultsData_t *p_result)
{
	VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
	
	if (!Dev || !p_result) {
		return VL53L4CD_ERROR_INVALID_ARGUMENT;
	}
	
	/* Read measurement results */
	status |= VL53L4CD_RdByte(Dev, 0x0089, &p_result->range_status);
	status |= VL53L4CD_RdWord(Dev, 0x0096, &p_result->distance_mm);
	status |= VL53L4CD_RdWord(Dev, 0x008E, &p_result->signal_rate_kcps);
	status |= VL53L4CD_RdWord(Dev, 0x0090, &p_result->ambient_rate_kcps);
	status |= VL53L4CD_RdWord(Dev, 0x0092, &p_result->sigma_mm);
	status |= VL53L4CD_RdWord(Dev, 0x008C, &p_result->number_of_spad);
	
	/* Calculate per-SPAD values */
	if (p_result->number_of_spad != 0) {
		p_result->signal_per_spad_kcps = p_result->signal_rate_kcps / p_result->number_of_spad;
		p_result->ambient_per_spad_kcps = p_result->ambient_rate_kcps / p_result->number_of_spad;
	} else {
		p_result->signal_per_spad_kcps = 0;
		p_result->ambient_per_spad_kcps = 0;
	}
	
	return status;
}

VL53L4CD_Error VL53L4CD_SetRangeTiming(VL53L4CD_DEV Dev, uint32_t timing_budget_ms, uint32_t inter_measurement_ms)
{
	VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
	
	if (!Dev) {
		return VL53L4CD_ERROR_INVALID_ARGUMENT;
	}
	
	/* Set timing budget (simplified implementation) */
	if (timing_budget_ms < 10 || timing_budget_ms > 200) {
		return VL53L4CD_ERROR_INVALID_ARGUMENT;
	}
	
	/* Set inter-measurement period */
	if (inter_measurement_ms > 0) {
		status |= VL53L4CD_WrWord(Dev, 0x006C, (uint16_t)inter_measurement_ms);
	}
	
	return status;
}

VL53L4CD_Error VL53L4CD_ClearInterrupt(VL53L4CD_DEV Dev)
{
	if (!Dev) {
		return VL53L4CD_ERROR_INVALID_ARGUMENT;
	}
	
	return VL53L4CD_WrByte(Dev, 0x0086, 0x01);
}
