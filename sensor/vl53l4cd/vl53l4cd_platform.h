/* vl53l4cd_platform.h - Zephyr platform definitions for VL53L4CD driver */

/*
 * Copyright (c) 2023 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef VL53L4CD_PLATFORM_H_
#define VL53L4CD_PLATFORM_H_

#include <zephyr/drivers/i2c.h>
#include <stdint.h>

/**
 * VL53L4CD device structure for Zephyr
 */
typedef struct {
	const struct device *i2c;
	uint8_t I2cDevAddr;
} VL53L4CD_Dev_t;

typedef VL53L4CD_Dev_t *VL53L4CD_DEV;
typedef uint16_t Dev_t;

/**
 * @brief Driver error type
 */
typedef uint8_t VL53L4CD_Error;

#define VL53L4CD_ERROR_NONE					((uint8_t)0U)
#define VL53L4CD_ERROR_XTALK_FAILED			((uint8_t)253U)
#define VL53L4CD_ERROR_INVALID_ARGUMENT		((uint8_t)254U)
#define VL53L4CD_ERROR_TIMEOUT				((uint8_t)255U)
#define VL53L4CD_ERROR_CONTROL_INTERFACE	((uint8_t)254U)

/**
 *  @brief defines Software Version
 */
typedef struct {
	uint8_t      major;
	uint8_t      minor;
	uint8_t      build;
	uint32_t     revision;
} VL53L4CD_Version_t;

/**
 *  @brief Packed reading results type
 */
typedef struct {
	uint8_t range_status;
	uint16_t distance_mm;
	uint16_t ambient_rate_kcps;
	uint16_t ambient_per_spad_kcps;
	uint16_t signal_rate_kcps;
	uint16_t signal_per_spad_kcps;
	uint16_t number_of_spad;
	uint16_t sigma_mm;
} VL53L4CD_ResultsData_t;

/* Platform function declarations */
VL53L4CD_Error VL53L4CD_WriteMulti(VL53L4CD_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count);
VL53L4CD_Error VL53L4CD_ReadMulti(VL53L4CD_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count);
VL53L4CD_Error VL53L4CD_WrByte(VL53L4CD_DEV Dev, uint16_t index, uint8_t data);
VL53L4CD_Error VL53L4CD_WrWord(VL53L4CD_DEV Dev, uint16_t index, uint16_t data);
VL53L4CD_Error VL53L4CD_WrDWord(VL53L4CD_DEV Dev, uint16_t index, uint32_t data);
VL53L4CD_Error VL53L4CD_RdByte(VL53L4CD_DEV Dev, uint16_t index, uint8_t *data);
VL53L4CD_Error VL53L4CD_RdWord(VL53L4CD_DEV Dev, uint16_t index, uint16_t *data);
VL53L4CD_Error VL53L4CD_RdDWord(VL53L4CD_DEV Dev, uint16_t index, uint32_t *data);
VL53L4CD_Error VL53L4CD_PollingDelay(VL53L4CD_DEV Dev);

/* API function declarations */
VL53L4CD_Error VL53L4CD_GetSensorId(VL53L4CD_DEV Dev, uint16_t *p_id);
VL53L4CD_Error VL53L4CD_SensorInit(VL53L4CD_DEV Dev);
VL53L4CD_Error VL53L4CD_ClearInterrupt(VL53L4CD_DEV Dev);
VL53L4CD_Error VL53L4CD_StartRanging(VL53L4CD_DEV Dev);
VL53L4CD_Error VL53L4CD_StopRanging(VL53L4CD_DEV Dev);
VL53L4CD_Error VL53L4CD_CheckForDataReady(VL53L4CD_DEV Dev, uint8_t *p_is_data_ready);
VL53L4CD_Error VL53L4CD_SetRangeTiming(VL53L4CD_DEV Dev, uint32_t timing_budget_ms, uint32_t inter_measurement_ms);
VL53L4CD_Error VL53L4CD_GetResult(VL53L4CD_DEV Dev, VL53L4CD_ResultsData_t *pResult);

#endif /* VL53L4CD_PLATFORM_H_ */
