/*
 * Copyright (c) 2023 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_VL53L4CD_VL53L4CD_ZEPHYR_H_
#define ZEPHYR_DRIVERS_SENSOR_VL53L4CD_VL53L4CD_ZEPHYR_H_

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_VL53L4CD_INT_GPIO
/**
 * @brief Custom sensor attributes for VL53L4CD detection thresholds
 * 
 * These attributes allow configuration of distance detection thresholds
 * that trigger interrupts when certain distance conditions are met.
 */

/* Set/get low distance threshold in millimeters (0-4000mm) */
#define SENSOR_ATTR_VL53L4CD_DETECTION_LOW_THRESHOLD    (SENSOR_ATTR_PRIV_START + 1)

/* Set/get high distance threshold in millimeters (0-4000mm) */ 
#define SENSOR_ATTR_VL53L4CD_DETECTION_HIGH_THRESHOLD   (SENSOR_ATTR_PRIV_START + 2)

/* Set/get detection window mode:
 * 0 = Below low threshold
 * 1 = Above high threshold  
 * 2 = Outside low/high window (below low OR above high)
 * 3 = Inside low/high window (above low AND below high)
 */
#define SENSOR_ATTR_VL53L4CD_DETECTION_WINDOW           (SENSOR_ATTR_PRIV_START + 3)

/**
 * @brief Detection window modes for VL53L4CD thresholds
 */
enum vl53l4cd_detection_window {
	VL53L4CD_WINDOW_BELOW_LOW = 0,      /* Below low threshold */
	VL53L4CD_WINDOW_ABOVE_HIGH = 1,     /* Above high threshold */
	VL53L4CD_WINDOW_OUTSIDE = 2,        /* Outside window (below low OR above high) */
	VL53L4CD_WINDOW_INSIDE = 3,         /* Inside window (above low AND below high) */
};

#endif /* CONFIG_VL53L4CD_INT_GPIO */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_VL53L4CD_VL53L4CD_ZEPHYR_H_ */
