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

/***********************************/
/*   VL53L4CD Zephyr sensor example */
/***********************************/
/*
* This example shows how to use the VL53L4CD sensor through Zephyr's
* sensor API. The driver is instantiated via device tree and follows
* the standard Zephyr sensor driver pattern.
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(vl53l4cd_example, LOG_LEVEL_INF);

/* Get the VL53L4CD sensor device from device tree */
static const struct device *const vl53l4cd_dev = DEVICE_DT_GET_ANY(st_vl53l4cd);

/* GPIO configuration for LED (optional - for status indication) */
#define LED0_NODE DT_ALIAS(led0)
#if DT_NODE_EXISTS(LED0_NODE)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#endif

static void init_leds(void)
{
#if DT_NODE_EXISTS(LED0_NODE)
	if (!gpio_is_ready_dt(&led)) {
		LOG_ERR("LED device not ready");
		return;
	}
	
	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Cannot configure LED");
		return;
	}
	
	gpio_pin_set_dt(&led, 0);
#endif
}

static void blink_led(void)
{
#if DT_NODE_EXISTS(LED0_NODE)
	gpio_pin_toggle_dt(&led);
#endif
}

static int vl53l4cd_sensor_example(void)
{
	struct sensor_value distance, signal;
	int ret;
	int measurement_count = 0;
	
	LOG_INF("VL53L4CD Zephyr Sensor API Example");
	
	/* Check if the sensor device is ready */
	if (!device_is_ready(vl53l4cd_dev)) {
		LOG_ERR("VL53L4CD sensor device not ready");
		return -ENODEV;
	}
	
	LOG_INF("VL53L4CD sensor device ready");
	
	/* Measurement loop */
	while (measurement_count < 200) {
		/* Fetch sensor sample */
		ret = sensor_sample_fetch(vl53l4cd_dev);
		if (ret == -EAGAIN) {
			/* Data not ready yet, wait a bit */
			k_msleep(5);
			continue;
		} else if (ret != 0) {
			LOG_ERR("Failed to fetch sample: %d", ret);
			k_msleep(100);
			continue;
		}
		
		/* Get distance measurement */
		ret = sensor_channel_get(vl53l4cd_dev, SENSOR_CHAN_DISTANCE, &distance);
		if (ret != 0) {
			LOG_ERR("Failed to get distance: %d", ret);
			k_msleep(100);
			continue;
		}
		
		/* Get signal strength (optional) */
		ret = sensor_channel_get(vl53l4cd_dev, SENSOR_CHAN_LIGHT, &signal);
		if (ret != 0) {
			LOG_WRN("Failed to get signal strength: %d", ret);
			signal.val1 = 0;
			signal.val2 = 0;
		}
		
		measurement_count++;
		
		/* Convert distance from meters to millimeters for display */
		int32_t distance_mm = (distance.val1 * 1000) + (distance.val2 / 1000);
		
		LOG_INF("Measurement %3d: Distance=%d mm, Signal=%d kcps",
			measurement_count, distance_mm, signal.val1);
		
		/* Blink LED on each measurement */
		blink_led();
		
		/* Small delay between measurements */
		k_msleep(10);
	}
	
	LOG_INF("Completed 200 measurements");
	return 0;
}

int main(void)
{
	LOG_INF("VL53L4CD Sensor Example Application Starting...");
	
	/* Initialize LEDs */
	init_leds();
	
	/* Run the VL53L4CD sensor example */
	int ret = vl53l4cd_sensor_example();
	
	if (ret == 0) {
		LOG_INF("VL53L4CD sensor example completed successfully");
	} else {
		LOG_ERR("VL53L4CD sensor example failed with error: %d", ret);
	}
	
	/* Keep the application running */
	while (1) {
		k_msleep(1000);
	}
	
	return 0;
}
