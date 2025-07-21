# VL53L4CD Driver Adaptation for nRF54L15 with Zephyr SDK

## Overview

This adaptation transforms the STMicroelectronics VL53L4CD Ultra Lite Driver v2.2.2 into a proper Zephyr sensor driver compatible with the nRF54L15 development kit. The implementation follows Zephyr's device driver model and sensor subsystem patterns.

## Architecture

### Driver Structure

The driver is split into several components following the VL53L0X reference implementation:

1. **vl53l4cd.c** - Main Zephyr sensor driver implementation
2. **vl53l4cd_platform.h** - Platform abstraction layer definitions
3. **vl53l4cd_platform.c** - Platform implementation with I2C functions
4. **Original API files** - Unchanged STMicroelectronics API (VL53L4CD_api.c, VL53L4CD_calibration.c)

### Key Features

- **Device Tree Integration**: Uses `DT_DRV_COMPAT` for proper device instantiation
- **Standard Sensor API**: Implements `sample_fetch`, `channel_get`, `attr_set`, and `trigger_set`
- **I2C Communication**: Proper Zephyr I2C driver integration with endianness handling
- **Power Management**: Optional PM support with suspend/resume functionality
- **GPIO Control**: Support for XSHUT (shutdown) and interrupt pins
- **Thread Safety**: Mutex protection for concurrent access
- **Error Handling**: Comprehensive error reporting and recovery

## Device Tree Configuration

### Binding File (st,vl53l4cd.yaml)

```yaml
description: STMicroelectronics VL53L4CD Time-of-Flight sensor

compatible: "st,vl53l4cd"

include: [sensor-device.yaml, i2c-device.yaml]

properties:
  xshut-gpios:
    type: phandle-array
    description: GPIO for XSHUT (shutdown) pin

  int-gpios:
    type: phandle-array
    description: GPIO for interrupt pin

  timing-budget-ms:
    type: int
    default: 50
    description: Timing budget in milliseconds (20-200ms)

  inter-measurement-ms:
    type: int
    default: 100
    description: Inter-measurement period in milliseconds
```

### Device Tree Overlay (nrf54l15dk_nrf54l15_cpuapp.overlay)

```dts
&i2c1 {
	status = "okay";
	pinctrl-0 = <&i2c1_default>;
	pinctrl-names = "default";
	
	vl53l4cd@29 {
		compatible = "st,vl53l4cd";
		reg = <0x29>;
		timing-budget-ms = <50>;
		inter-measurement-ms = <100>;
	};
};

&pinctrl {
	i2c1_default: i2c1_default {
		group1 {
			psels = <NRF_PSEL(TWIM_SDA, 1, 2)>,
				<NRF_PSEL(TWIM_SCL, 1, 3)>;
		};
	};
};
```

## Platform Abstraction Layer

The platform layer provides hardware abstraction following the VL53L0X pattern:

### I2C Functions
- `VL53L4CD_WriteMulti()` - Multi-byte write with proper endianness
- `VL53L4CD_ReadMulti()` - Multi-byte read with proper endianness  
- `VL53L4CD_WrByte/Word/DWord()` - Typed write functions
- `VL53L4CD_RdByte/Word/DWord()` - Typed read functions
- `VL53L4CD_PollingDelay()` - Timing delay function

### Error Codes
- `VL53L4CD_ERROR_NONE` - Success
- `VL53L4CD_ERROR_CONTROL_INTERFACE` - I2C communication error
- `VL53L4CD_ERROR_INVALID_ARGUMENT` - Invalid parameter
- `VL53L4CD_ERROR_TIMEOUT` - Operation timeout

## Sensor API Implementation

### Supported Channels
- `SENSOR_CHAN_DISTANCE` - Distance measurement in meters
- `SENSOR_CHAN_PROX` - Proximity status (valid/invalid)

### Sample Usage

```c
#include <zephyr/drivers/sensor.h>

const struct device *vl53l4cd = DEVICE_DT_GET_ANY(st_vl53l4cd);

// Fetch new measurement
int ret = sensor_sample_fetch(vl53l4cd);
if (ret == 0) {
    struct sensor_value distance;
    sensor_channel_get(vl53l4cd, SENSOR_CHAN_DISTANCE, &distance);
    
    // Convert to millimeters
    int distance_mm = distance.val1 * 1000 + distance.val2 / 1000;
    printk("Distance: %d mm\\n", distance_mm);
}
```

## Build Configuration

### Project Configuration (prj.conf)

```ini
# Sensor subsystem
CONFIG_SENSOR=y
CONFIG_VL53L4CD=y

# I2C support
CONFIG_I2C=y

# GPIO support  
CONFIG_GPIO=y

# Logging
CONFIG_LOG=y
CONFIG_SENSOR_LOG_LEVEL_INF=y

# Power management (optional)
CONFIG_PM_DEVICE=y
```

### Kconfig Options

```kconfig
config VL53L4CD
	bool "VL53L4CD Time-of-Flight sensor"
	default y
	depends on DT_HAS_ST_VL53L4CD_ENABLED
	select I2C
	help
	  Enable driver for STMicroelectronics VL53L4CD ToF sensor.
```

### CMake Integration

```cmake
# VL53L4CD sensor driver
target_sources_ifdef(CONFIG_VL53L4CD app PRIVATE
    vl53l4cd.c
    vl53l4cd_platform.c
    ../VL53L4CD_ULD_Driver/VL53L4CD_api.c
    ../VL53L4CD_ULD_Driver/VL53L4CD_calibration.c
)
```

## Hardware Setup

### I2C Connection (nRF54L15)
- **SDA**: P1.02 (I2C1 SDA)
- **SCL**: P1.03 (I2C1 SCL)
- **VDD**: 3.3V
- **GND**: Ground
- **XSHUT**: Optional GPIO for reset control
- **GPIO1**: Optional GPIO for interrupt

### Typical Board Configuration
```
nRF54L15 DK          VL53L4CD Module
+---------+          +-------------+
|   P1.02 |----------|    SDA      |
|   P1.03 |----------|    SCL      |
|   3.3V  |----------|    VDD      |
|   GND   |----------|    GND      |
+---------+          +-------------+
```

## Driver Initialization Flow

1. **Device Tree Parsing**: Extract I2C bus, address, and GPIO configurations
2. **I2C Bus Validation**: Ensure I2C controller is ready
3. **GPIO Setup**: Configure XSHUT and interrupt pins if present
4. **Sensor Reset**: Optional hardware reset via XSHUT pin
5. **Sensor ID Check**: Verify sensor responds with expected ID (0xEBAA)
6. **Sensor Initialization**: Call ST API initialization functions
7. **Timing Configuration**: Set measurement timing parameters
8. **Start Ranging**: Begin continuous distance measurements

## Power Management

The driver supports Zephyr's power management framework:

- **Suspend**: Stops ranging and enters low-power mode
- **Resume**: Restarts ranging after wake-up

## Error Handling

The driver provides comprehensive error handling:

- **I2C Communication Errors**: Detected and reported with specific error codes
- **Invalid Sensor ID**: Checks for proper sensor presence
- **Initialization Failures**: Graceful handling of setup errors
- **Runtime Errors**: Mutex protection and error recovery

## Performance Characteristics

- **Measurement Rate**: Configurable via inter-measurement timing
- **Accuracy**: Millimeter precision distance measurements
- **Range**: Typically 1mm to 1.3m (depending on target and conditions)
- **Power Consumption**: Optimized with power management support

## Integration with Existing Projects

To integrate this driver into an existing Zephyr project:

1. Copy driver files to your project sensor directory
2. Add device tree overlay for your target board
3. Update prj.conf with required configurations
4. Include sensor header in your application
5. Use standard Zephyr sensor API calls

## Testing and Validation

The provided example application demonstrates:

- Continuous distance measurements
- Error handling and recovery
- LED indication for measurement status
- Console output for debugging

## Future Enhancements

Potential improvements that could be added:

1. **Interrupt Support**: GPIO-based data ready interrupts
2. **Calibration Interface**: Expose calibration functions via sensor attributes
3. **Multi-Zone Support**: If supported by hardware variant
4. **Advanced Filtering**: Software filtering of measurements
5. **Dynamic Configuration**: Runtime timing parameter adjustment

## Conclusion

This adaptation successfully transforms the VL53L4CD Ultra Lite Driver into a full-featured Zephyr sensor driver that follows best practices and integrates seamlessly with the nRF54L15 development environment. The implementation maintains compatibility with the original ST API while providing modern Zephyr device driver patterns and robust error handling.
