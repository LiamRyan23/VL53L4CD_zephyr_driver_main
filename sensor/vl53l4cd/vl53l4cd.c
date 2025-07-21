/*
 * Copyright (c) 2023 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_vl53l4cd

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>

#include "VL53L4CD_ULD_Driver/VL53L4CD_api.h"
#include "Platform/platform.h"

LOG_MODULE_REGISTER(VL53L4CD, CONFIG_SENSOR_LOG_LEVEL);

struct vl53l4cd_data {
	VL53L4CD_ResultsData_t results;
	uint16_t sensor_id;
	bool ranging_active;
	uint32_t timing_budget_ms;
	uint32_t inter_measurement_ms;
	int8_t temperature_offset; /* For temperature compensation */
};

static int vl53l4cd_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct vl53l4cd_data *data = dev->data;
	VL53L4CD_Error status;
	uint8_t is_ready = 0;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_DISTANCE) {
		return -ENOTSUP;
	}

	if (!data->ranging_active) {
		LOG_ERR("Ranging not active, call sensor_attr_set first");
		return -EINVAL;
	}

	/* Check if data is ready */
	status = VL53L4CD_CheckForDataReady(dev, &is_ready);
	if (status != VL53L4CD_ERROR_NONE) {
		LOG_ERR("Failed to check data ready: %d", status);
		return -EIO;
	}

	if (!is_ready) {
		return -EAGAIN;
	}

	/* Get the ranging data */
	status = VL53L4CD_GetResult(dev, &data->results);
	if (status != VL53L4CD_ERROR_NONE) {
		LOG_ERR("Failed to get ranging result: %d", status);
		return -EIO;
	}

	/* Clear interrupt for next measurement */
	status = VL53L4CD_ClearInterrupt(dev);
	if (status != VL53L4CD_ERROR_NONE) {
		LOG_ERR("Failed to clear interrupt: %d", status);
		return -EIO;
	}

	return 0;
}

static int vl53l4cd_channel_get(const struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *val)
{
	struct vl53l4cd_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_DISTANCE:
		/* Distance is in mm, convert to meters */
		val->val1 = data->results.distance_mm / 1000;
		val->val2 = (data->results.distance_mm % 1000) * 1000;
		break;
	case SENSOR_CHAN_PROX:
		/* Return signal rate in kcps */
		val->val1 = data->results.signal_rate_kcps;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_AMBIENT_TEMP:
		/* Return ambient rate in kcps */
		val->val1 = data->results.ambient_rate_kcps;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_IR:
		/* Return sigma (noise estimate) in mm */
		val->val1 = data->results.sigma_mm;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_ALL:
		/* Return measurement status */
		val->val1 = data->results.range_status;
		val->val2 = 0;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int vl53l4cd_attr_set(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     const struct sensor_value *val)
{
	struct vl53l4cd_data *data = dev->data;
	VL53L4CD_Error status;

	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		if (val->val1 == 1) {
			/* Start ranging */
			if (!data->ranging_active) {
				status = VL53L4CD_StartRanging(dev);
				if (status != VL53L4CD_ERROR_NONE) {
					LOG_ERR("Failed to start ranging: %d", status);
					return -EIO;
				}
				data->ranging_active = true;
			}
		} else if (val->val1 == 0) {
			/* Stop ranging */
			if (data->ranging_active) {
				status = VL53L4CD_StopRanging(dev);
				if (status != VL53L4CD_ERROR_NONE) {
					LOG_ERR("Failed to stop ranging: %d", status);
					return -EIO;
				}
				data->ranging_active = false;
			}
		} else {
			return -EINVAL;
		}
		break;
	case SENSOR_ATTR_UPPER_THRESH:
		/* Set timing budget in milliseconds (10-200ms range) */
		if (val->val1 < 10 || val->val1 > 200) {
			LOG_ERR("Timing budget must be between 10-200ms");
			return -EINVAL;
		}
		/* Store the new timing budget, will be applied when intermeasurement is set */
		data->timing_budget_ms = val->val1;
		break;
	case SENSOR_ATTR_LOWER_THRESH:
		/* Set intermeasurement period in milliseconds */
		if (val->val1 < 0) {
			return -EINVAL;
		}
		/* Set new intermeasurement, must be > timing budget or 0 */
		if (val->val1 != 0 && val->val1 <= data->timing_budget_ms) {
			LOG_ERR("InterMeasurement (%d) must be > TimingBudget (%d) or 0", 
				val->val1, data->timing_budget_ms);
			return -EINVAL;
		}
		/* Apply both timing budget and intermeasurement together */
		status = VL53L4CD_SetRangeTiming(dev, data->timing_budget_ms, val->val1);
		if (status != VL53L4CD_ERROR_NONE) {
			LOG_ERR("Failed to set range timing (TB=%d, IM=%d): %d", 
				data->timing_budget_ms, val->val1, status);
			return -EIO;
		}
		data->inter_measurement_ms = val->val1;
		break;
	case SENSOR_ATTR_HYSTERESIS:
		/* Trigger temperature update if temperature changed >8°C */
		if (abs(val->val1 - data->temperature_offset) > 8) {
			/* Stop ranging if active */
			bool was_ranging = data->ranging_active;
			if (was_ranging) {
				status = VL53L4CD_StopRanging(dev);
				if (status != VL53L4CD_ERROR_NONE) {
					return -EIO;
				}
			}
			
			/* Perform temperature update */
			status = VL53L4CD_StartTemperatureUpdate(dev);
			if (status != VL53L4CD_ERROR_NONE) {
				LOG_ERR("Failed to update temperature: %d", status);
				return -EIO;
			}
			
			/* Restart ranging if it was active */
			if (was_ranging) {
				status = VL53L4CD_StartRanging(dev);
				if (status != VL53L4CD_ERROR_NONE) {
					return -EIO;
				}
			}
			
			data->temperature_offset = val->val1;
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int vl53l4cd_attr_get(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     struct sensor_value *val)
{
	struct vl53l4cd_data *data = dev->data;

	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		val->val1 = data->ranging_active ? 1 : 0;
		val->val2 = 0;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api vl53l4cd_api_funcs = {
	.sample_fetch = vl53l4cd_sample_fetch,
	.channel_get = vl53l4cd_channel_get,
	.attr_set = vl53l4cd_attr_set,
	.attr_get = vl53l4cd_attr_get,
};

static int vl53l4cd_init(const struct device *dev)
{
	const struct vl53l4cd_config *config = dev->config;
	struct vl53l4cd_data *data = dev->data;
	VL53L4CD_Error status;
	VL53L4CD_Version_t sw_version;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus %s not ready", config->i2c.bus->name);
		return -ENODEV;
	}

	if (!device_is_ready(config->xshut_gpio.port)) {
		LOG_ERR("XSHUT GPIO device not ready");
		return -ENODEV;
	}

	/* Configure XSHUT pin as output */
	int ret = gpio_pin_configure_dt(&config->xshut_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure XSHUT GPIO: %d", ret);
		return ret;
	}

	/* Power-up sequence: XSHUT active (low) -> delay -> XSHUT inactive (high) -> boot delay */
	gpio_pin_set_dt(&config->xshut_gpio, 1);  /* Set XSHUT active (low due to GPIO_ACTIVE_LOW) */
	k_msleep(2);                              /* Short delay in standby */
	gpio_pin_set_dt(&config->xshut_gpio, 0);  /* Set XSHUT inactive (high, device starts boot) */
	k_msleep(10);                             /* Wait for boot sequence (tBOOT = 1.2ms max) */

	/* Get software version */
	status = VL53L4CD_GetSWVersion(&sw_version);
	if (status != VL53L4CD_ERROR_NONE) {
		LOG_ERR("Failed to get SW version: %d", status);
		return -EIO;
	}

	LOG_INF("VL53L4CD driver version %d.%d.%d", 
		sw_version.major, sw_version.minor, sw_version.build);

	/* Check sensor ID */
	status = VL53L4CD_GetSensorId(dev, &data->sensor_id);
	if (status != VL53L4CD_ERROR_NONE || data->sensor_id != 0xEBAA) {
		LOG_ERR("VL53L4CD not detected, sensor_id=0x%04X, status=%d", 
			data->sensor_id, status);
		return -ENODEV;
	}

	/* Initialize sensor */
	status = VL53L4CD_SensorInit(dev);
	if (status != VL53L4CD_ERROR_NONE) {
		LOG_ERR("Failed to initialize sensor: %d", status);
		return -EIO;
	}

	/* Set default timing: 50ms timing budget, 0ms intermeasurement */
	status = VL53L4CD_SetRangeTiming(dev, 50, 0);
	if (status != VL53L4CD_ERROR_NONE) {
		LOG_ERR("Failed to set default timing: %d", status);
		return -EIO;
	}

	data->ranging_active = false;
	data->timing_budget_ms = 50;
	data->inter_measurement_ms = 0;
	data->temperature_offset = 0;

	LOG_INF("VL53L4CD sensor initialized successfully");

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int vl53l4cd_pm_action(const struct device *dev,
			      enum pm_device_action action)
{
	struct vl53l4cd_data *data = dev->data;
	VL53L4CD_Error status;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		if (data->ranging_active) {
			status = VL53L4CD_StopRanging(dev);
			if (status != VL53L4CD_ERROR_NONE) {
				return -EIO;
			}
		}
		break;
	case PM_DEVICE_ACTION_RESUME:
		/* Sensor will need re-initialization after resume */
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

#define VL53L4CD_INIT(inst)						\
	static struct vl53l4cd_data vl53l4cd_data_##inst;		\
	static const struct vl53l4cd_config vl53l4cd_config_##inst = {	\
		.i2c = I2C_DT_SPEC_INST_GET(inst),			\
		.i2c_addr = DT_INST_REG_ADDR(inst),			\
		IF_ENABLED(CONFIG_VL53L4CD_XSHUT_GPIO,			\
		(.xshut_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, xshut_gpios, {0}),)) \
	};								\
	PM_DEVICE_DT_INST_DEFINE(inst, vl53l4cd_pm_action);		\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
				     vl53l4cd_init,			\
				     PM_DEVICE_DT_INST_GET(inst),	\
				     &vl53l4cd_data_##inst,		\
				     &vl53l4cd_config_##inst,		\
				     POST_KERNEL,			\
				     CONFIG_SENSOR_INIT_PRIORITY,	\
				     &vl53l4cd_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(VL53L4CD_INIT)
