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

#ifndef VL53L4CD_API_H_
#define VL53L4CD_API_H_

#include "platform_zephyr.h"

/**
 *  @brief VL53L4CD device structure for Zephyr
 */
typedef struct {
	const struct device *i2c;
	uint8_t I2cDevAddr;
} VL53L4CD_Dev_t;

typedef VL53L4CD_Dev_t *VL53L4CD_DEV;

/**
 *  @brief Driver version
 */

#define VL53L4CD_IMPLEMENTATION_VER_MAJOR       2
#define VL53L4CD_IMPLEMENTATION_VER_MINOR       2
#define VL53L4CD_IMPLEMENTATION_VER_BUILD       2
#define VL53L4CD_IMPLEMENTATION_VER_REVISION  	0

/**
 *  @brief Driver error type
 */

typedef uint8_t VL53L4CD_Error;

#define VL53L4CD_ERROR_NONE					((uint8_t)0U)
#define VL53L4CD_ERROR_XTALK_FAILED			((uint8_t)253U)
#define VL53L4CD_ERROR_INVALID_ARGUMENT		((uint8_t)254U)
#define VL53L4CD_ERROR_TIMEOUT				((uint8_t)255U)

/**
 *  @brief Inner Macro for API. Not for user, only for development.
 */

#define VL53L4CD_SOFT_RESET     ((uint16_t)0x0000)
#define VL53L4CD_I2C_SLAVE__DEVICE_ADDRESS      ((uint16_t)0x0001)
#define VL53L4CD_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND  ((uint16_t)0x0008)
#define VL53L4CD_XTALK_PLANE_OFFSET_KCPS ((uint16_t)0x0016)
#define VL53L4CD_XTALK_X_PLANE_GRADIENT_KCPS     ((uint16_t)0x0018)
#define VL53L4CD_XTALK_Y_PLANE_GRADIENT_KCPS     ((uint16_t)0x001A)
#define VL53L4CD_RANGE_OFFSET_MM     ((uint16_t)0x001E)
#define VL53L4CD_INNER_OFFSET_MM     ((uint16_t)0x0020)
#define VL53L4CD_OUTER_OFFSET_MM     ((uint16_t)0x0022)
#define VL53L4CD_GPIO_HV_MUX__CTRL      ((uint16_t)0x0030)
#define VL53L4CD_GPIO__TIO_HV_STATUS    ((uint16_t)0x0031)
#define VL53L4CD_SYSTEM__INTERRUPT  ((uint16_t)0x0046)
#define VL53L4CD_RANGE_CONFIG_A     ((uint16_t)0x005E)
#define VL53L4CD_RANGE_CONFIG_B      ((uint16_t)0x0061)
#define VL53L4CD_RANGE_CONFIG__SIGMA_THRESH     ((uint16_t)0x0064)
#define VL53L4CD_MIN_COUNT_RATE_RTN_LIMIT_MCPS    ((uint16_t)0x0066)
#define VL53L4CD_INTERMEASUREMENT_MS ((uint16_t)0x006C)
#define VL53L4CD_THRESH_HIGH    ((uint16_t)0x0072)
#define VL53L4CD_THRESH_LOW     ((uint16_t)0x0074)
#define VL53L4CD_SYSTEM__INTERRUPT_CLEAR        ((uint16_t)0x0086)
#define VL53L4CD_SYSTEM_START     ((uint16_t)0x0087)
#define VL53L4CD_RESULT__RANGE_STATUS   ((uint16_t)0x0089)
#define VL53L4CD_RESULT__SPAD_NB   ((uint16_t)0x008C)
#define VL53L4CD_RESULT__SIGNAL_RATE   ((uint16_t)0x008E)
#define VL53L4CD_RESULT__AMBIENT_RATE   ((uint16_t)0x0090)
#define VL53L4CD_RESULT__SIGMA   ((uint16_t)0x0092)
#define VL53L4CD_RESULT__DISTANCE   ((uint16_t)0x0096)

#define VL53L4CD_RESULT__OSC_CALIBRATE_VAL      ((uint16_t)0x00DE)
#define VL53L4CD_FIRMWARE__SYSTEM_STATUS        ((uint16_t)0x00E5)
#define VL53L4CD_IDENTIFICATION__MODEL_ID       ((uint16_t)0x010F)

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

/**
 * @brief This function programs the software driver version.
 * @param (VL53L4CD_Version_t) pVersion : Pointer of structure, containing the
 * software version.
 * @return (VL53L4CD_ERROR) status : 0 if SW version is OK.
 */
VL53L4CD_Error VL53L4CD_GetSWVersion(VL53L4CD_Version_t *pVersion);

/**
 * @brief This function sets a new I2C address to a sensor. It can be used for
 * example when multiple sensors share the same I2C bus.
 * @param (Dev_t) dev : Device instance to update.
 * @param (uint8_t) new_address : New I2C address.
 * @return (VL53L4CD_ERROR) status : 0 if I2C address has been correctly
 * programmed.
 */
VL53L4CD_Error VL53L4CD_SetI2CAddress(VL53L4CD_DEV Dev, uint8_t new_address);

/**
 * @brief This function is used to get the sensor id of VL53L4CD. The sensor id
 * should be 0xEBAA.
 * @param (Dev_t) dev : Device instance.
 * @param (uint16_t) *p_id : Sensor id.
 * @return (VL53L4CD_ERROR) status : 0 if OK.
 */
VL53L4CD_Error VL53L4CD_GetSensorId(Dev_t dev, uint16_t *p_id);

/**
 * @brief This function is used to initialize the sensor.
 * @param (Dev_t) dev : Device instance to initialize.
 * @return (VL53L4CD_ERROR) status : 0 if init is OK.
 */
VL53L4CD_Error VL53L4CD_SensorInit(Dev_t dev);

/**
 * @brief This function clears the interrupt. It needs to be called after a
 * ranging data reading to arm the interrupt for the next data ready event.
 * @param (Dev_t) dev : Device instance.
 * @return (VL53L4CD_ERROR) status : 0 if OK.
 */
VL53L4CD_Error VL53L4CD_ClearInterrupt(Dev_t dev);

/**
 * @brief This function starts a ranging session. The ranging operation is
 * continuous. The clear interrupt has to be done after each get data to allow
 * the interrupt to raise when the next data is ready.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @return (VL53L4CD_ERROR) status : 0 if OK.
 */
VL53L4CD_Error VL53L4CD_StartRanging(Dev_t dev);

/**
 * @brief This function stops the ranging in progress.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @return (VL53L4CD_ERROR) status : 0 if OK.
 */
VL53L4CD_Error VL53L4CD_StopRanging(Dev_t dev);

/**
 * @brief This function check if a new data is available by polling a dedicated
 * register.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint8_t) *p_is_data_ready : Pointer containing a flag to know if a
 * data is ready : 0 = no data ready, 1 = data ready.
 * @return (VL53L4CD_ERROR) status : 0 if OK.
 */
VL53L4CD_Error VL53L4CD_CheckForDataReady(Dev_t dev, uint8_t *p_is_data_ready);

/**
 * @brief This function sets new range timing. Timing are composed of
 * TimingBudget and InterMeasurement. TimingBudget represents the timing during
 * VCSEL enabled, and InterMeasurement the time between two measurements.
 * The sensor can have different ranging mode depending of the configuration,
 * please refer to the user manual for more information.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint32_t) timing_budget_ms :  New timing budget in ms. Value can be
 * between 10ms and 200ms. Default is 50ms.
 * @param (uint32_t) inter_measurement_ms :  New inter-measurement in ms. If the
 * value is equal to 0, the ranging period is defined by the timing budget.
 * Otherwise, inter-measurement must be > timing budget. When all the timing
 * budget is consumed, the device goes in low power mode until inter-measurement
 * is done.
 * @return (uint8_t) status : 0 if OK.
 */
VL53L4CD_Error VL53L4CD_SetRangeTiming(Dev_t dev, uint32_t timing_budget_ms, uint32_t inter_measurement_ms);

/**
 * @brief This function gets current range timing values.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint32_t) *p_timing_budget_ms : Timing budget in ms.
 * @param (uint32_t) *p_inter_measurement_ms : Inter-measurement in ms.
 * @return (uint8_t) status : 0 if OK.
 */
VL53L4CD_Error VL53L4CD_GetRangeTiming(Dev_t dev, uint32_t *p_timing_budget_ms, uint32_t *p_inter_measurement_ms);

/**
 * @brief This function gets the measurement results.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (VL53L4CD_ResultsData_t) *p_result : pointer to results data structure.
 * @return (uint8_t) status : 0 if OK.
 */
VL53L4CD_Error VL53L4CD_GetResult(Dev_t dev, VL53L4CD_ResultsData_t *p_result);

/**
 * @brief This function programs the offset value.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (int16_t) OffsetValueInMm : Offset value in mm.
 * @return (uint8_t) status : 0 if OK.
 */
VL53L4CD_Error VL53L4CD_SetOffset(Dev_t dev, int16_t OffsetValueInMm);

/**
 * @brief This function gets the offset value.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (int16_t) *p_offset : pointer to offset value in mm.
 * @return (uint8_t) status : 0 if OK.
 */
VL53L4CD_Error VL53L4CD_GetOffset(Dev_t dev, int16_t *p_offset);

/**
 * @brief This function programs the Xtalk value.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint16_t) XtalkValueKcps : Xtalk value in kcps.
 * @return (uint8_t) status : 0 if OK.
 */
VL53L4CD_Error VL53L4CD_SetXtalk(Dev_t dev, uint16_t XtalkValueKcps);

/**
 * @brief This function gets the Xtalk value.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint16_t) *p_xtalk_kcps : pointer to Xtalk value in kcps.
 * @return (uint8_t) status : 0 if OK.
 */
VL53L4CD_Error VL53L4CD_GetXtalk(Dev_t dev, uint16_t *p_xtalk_kcps);

/**
 * @brief This function programs the detection thresholds.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint8_t) window : Detection window.
 * @return (uint8_t) status : 0 if OK.
 */
VL53L4CD_Error VL53L4CD_SetDetectionThresholds(Dev_t dev, uint8_t window);

/**
 * @brief This function gets the detection thresholds.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint8_t) *p_window : pointer to detection window.
 * @return (uint8_t) status : 0 if OK.
 */
VL53L4CD_Error VL53L4CD_GetDetectionThresholds(Dev_t dev, uint8_t *p_window);

/**
 * @brief This function programs the signal threshold.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint16_t) signal_kcps : Signal threshold in kcps.
 * @return (uint8_t) status : 0 if OK.
 */
VL53L4CD_Error VL53L4CD_SetSignalThreshold(Dev_t dev, uint16_t signal_kcps);

/**
 * @brief This function gets the signal threshold.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint16_t) *p_signal_kcps : pointer to signal threshold in kcps.
 * @return (uint8_t) status : 0 if OK.
 */
VL53L4CD_Error VL53L4CD_GetSignalThreshold(Dev_t dev, uint16_t *p_signal_kcps);

/**
 * @brief This function programs the sigma threshold.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint16_t) sigma_mm : Sigma threshold in mm.
 * @return (uint8_t) status : 0 if OK.
 */
VL53L4CD_Error VL53L4CD_SetSigmaThreshold(Dev_t dev, uint16_t sigma_mm);

/**
 * @brief This function gets the sigma threshold.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint16_t) *p_sigma_mm : pointer to sigma threshold in mm.
 * @return (uint8_t) status : 0 if OK.
 */
VL53L4CD_Error VL53L4CD_GetSigmaThreshold(Dev_t dev, uint16_t *p_sigma_mm);

/**
 * @brief This function starts the temperature update process.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @return (uint8_t) status : 0 if OK.
 */
VL53L4CD_Error VL53L4CD_StartTemperatureUpdate(Dev_t dev);

#endif  //VL53L4CD_API_H_
