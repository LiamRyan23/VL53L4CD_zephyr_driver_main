# VL53L4CD Interrupt Usage Guide

## Overview

The VL53L4CD sensor supports interrupt-based detection using configurable distance thresholds. The interrupt is only triggered when specific distance conditions are met, not for every measurement.

## Requirements

1. **Inter-measurement period must be enabled** (not equal to 0)
2. **Detection thresholds must be configured**
3. **Interrupt GPIO must be connected and configured in device tree**

## Configuration Steps

### 1. Device Tree Configuration

```dts
&i2c1 {
    vl53l4cd@29 {
        compatible = "st,vl53l4cd";
        reg = <0x29>;
        xshut-gpios = <&gpio0 3 GPIO_ACTIVE_LOW>;
        int-gpios = <&gpio0 4 GPIO_ACTIVE_LOW>;
    };
};
```

### 2. Enable Interrupt Support in Kconfig

```
CONFIG_VL53L4CD_INT_GPIO=y
```

### 3. Application Code

```c
#include <zephyr/drivers/sensor.h>
#include "vl53l4cd_zephyr.h"

const struct device *sensor = DEVICE_DT_GET_ONE(st_vl53l4cd);

static volatile bool detection_event_active = false;
static volatile uint16_t last_distance_mm = 0;
static volatile int64_t last_interrupt_time = 0;

// Handler function called for both sample ready and detection events
void sensor_trigger_handler(const struct device *dev, 
                           const struct sensor_trigger *trigger)
{
    struct sensor_value distance, signal, status;
    int ret;
    
    last_interrupt_time = k_uptime_get();
    
    // Get measurement results
    ret = sensor_sample_fetch(dev);
    if (ret != 0) {
        return; // Skip this reading if fetch failed
    }
    
    sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &distance);
    sensor_channel_get(dev, SENSOR_CHAN_ALL, &status);
    
    // Skip readings with errors
    if (status.val1 != 0) {
        return;
    }
    
    uint16_t distance_mm = distance.val1 * 1000 + distance.val2 / 1000;
    last_distance_mm = distance_mm;
    
    // Determine if this is a detection event or sample ready
    if (distance_mm <= 50) {  // Within detection threshold
        if (!detection_event_active) {
            detection_event_active = true;
            printk("*** DETECTION EVENT: Object within %dmm ***\n", distance_mm);
            // Don't clear interrupt - let pin stay HIGH for detection state
        }
    } else {
        if (detection_event_active) {
            detection_event_active = false;
            printk("Detection event cleared - object moved away\n");
        }
        // Clear interrupt for sample-ready events (pin will oscillate)
        VL53L4CD_ClearInterrupt(dev);
    }
}

int main(void)
{
    struct sensor_value val;
    struct sensor_trigger trig;
    
    // 1. Set timing budget (measurement accuracy vs speed)
    val.val1 = 50;  // 50ms timing budget
    val.val2 = 0;
    sensor_attr_set(sensor, SENSOR_CHAN_DISTANCE, 
                    SENSOR_ATTR_UPPER_THRESH, &val);
    
    // 2. Set inter-measurement period (MANDATORY for threshold detection)
    val.val1 = 1000;  // 1000ms between measurements
    val.val2 = 0;
    sensor_attr_set(sensor, SENSOR_CHAN_DISTANCE, 
                    SENSOR_ATTR_LOWER_THRESH, &val);
    
    // 3. Configure detection thresholds using custom sensor attributes
    
    // Set low threshold to 0mm (detect anything closer than high threshold)
    val.val1 = 0;
    sensor_attr_set(sensor, SENSOR_CHAN_DISTANCE, 
                    SENSOR_ATTR_VL53L4CD_DETECTION_LOW_THRESHOLD, &val);
    
    // Set high threshold to 50mm (trigger when object within 50mm)
    val.val1 = 50;
    sensor_attr_set(sensor, SENSOR_CHAN_DISTANCE,
                    SENSOR_ATTR_VL53L4CD_DETECTION_HIGH_THRESHOLD, &val);
    
    // Set detection window to mode 3 (INSIDE): interrupt when low < distance < high
    // For container monitoring: triggers when object is within 0-50mm range
    val.val1 = 3;  // Inside window mode
    sensor_attr_set(sensor, SENSOR_CHAN_DISTANCE,
                    SENSOR_ATTR_VL53L4CD_DETECTION_WINDOW, &val);
    
    // 4. Use direct API to ensure thresholds are properly configured
    VL53L4CD_Error vl_status = VL53L4CD_SetDetectionThresholds(sensor,
            0, 50, 3);  // low=0mm, high=50mm, window=inside
    if (vl_status != VL53L4CD_ERROR_NONE) {
        printk("Failed to set detection thresholds: %d\n", vl_status);
        return -1;
    }
    
    // 5. Set up trigger callback
    trig.type = SENSOR_TRIG_DATA_READY;
    trig.chan = SENSOR_CHAN_DISTANCE;
    sensor_trigger_set(sensor, &trig, sensor_trigger_handler);
    
    // 6. Start ranging
    val.val1 = 1;
    val.val2 = 0;
    sensor_attr_set(sensor, SENSOR_CHAN_DISTANCE, 
                    SENSOR_ATTR_SAMPLING_FREQUENCY, &val);
    
    printk("Interrupt behavior:\n");
    printk("  Pin OSCILLATES = Normal ranging (sample ready interrupts)\n");
    printk("  Pin STAYS HIGH = Detection event (object within 50mm)\n");
    
    while (1) {
        k_msleep(1000);
        
        // Check if interrupts have stopped (indicates detection event)
        int64_t now = k_uptime_get();
        bool interrupt_timeout = (now - last_interrupt_time) > 2000;
        
        if (detection_event_active && interrupt_timeout) {
            printk("CONTAINER FULL: Pin held HIGH, object within threshold\n");
            // Clear the detection interrupt to resume normal operation
            VL53L4CD_ClearInterrupt(sensor);
            detection_event_active = false;
        }
    }
}
```

## Detection Window Modes

- **0 (BELOW_LOW)**: Interrupt when distance < low_threshold
- **1 (ABOVE_HIGH)**: Interrupt when distance > high_threshold  
- **2 (OUTSIDE)**: Interrupt when distance < low_threshold OR > high_threshold
- **3 (INSIDE)**: Interrupt when low_threshold < distance < high_threshold

**Recommended for container monitoring**: Use mode 3 (INSIDE) with low=0mm and high=50mm to detect objects within the detection zone.

## Interrupt Pin Behavior

The VL53L4CD interrupt pin exhibits different behavior depending on the detection state:

### Normal Ranging (No Detection Event)
- Pin **OSCILLATES** between HIGH and LOW for each measurement
- Interrupts are generated regularly based on inter-measurement period
- Handler is called for each sample ready event

### Detection Event Active (Object Within Threshold)
- Pin **STAYS HIGH** when object is within detection threshold
- Interrupts stop being generated while condition persists
- Detection event remains active until object moves away or interrupt is manually cleared

### Application Usage Pattern
```c
// In interrupt handler:
if (distance_within_threshold) {
    // Don't clear interrupt - let pin stay HIGH
    detection_active = true;
} else {
    // Clear interrupt for normal sample ready events
    VL53L4CD_ClearInterrupt(dev);
    detection_active = false;
}

// In main loop:
if (detection_active && no_interrupts_for_timeout) {
    // Handle detection event (e.g., container full)
    VL53L4CD_ClearInterrupt(sensor);  // Resume normal operation
}
```

## Default Values

If thresholds are not explicitly configured, the driver uses:
- Low threshold: 500mm
- High threshold: 1000mm
- Window mode: 0 (below low threshold)

## Important Notes

1. **Dual interrupt behavior**: The sensor generates interrupts for both sample ready events AND detection threshold events
2. **Pin state indicates detection**: When pin stays HIGH (stops oscillating), it indicates a detection event is active
3. **Manual interrupt clearing**: For detection events, you must manually clear the interrupt to resume normal operation
4. **Inter-measurement period requirement**: MUST be > 0 for threshold detection to work
5. **Detection window mode 3 (INSIDE)**: Best for container monitoring - triggers when object enters detection zone
6. **Timing budget vs accuracy**: Lower timing budget = faster measurements but less accurate
7. **Error handling**: Always check status value - skip measurements with errors (status.val1 != 0)

## Troubleshooting

**Problem**: Interrupts fire for every measurement instead of only on threshold detection
**Solution**: Ensure detection window mode is set correctly (mode 3 for inside detection) and thresholds are properly configured

**Problem**: No interrupts generated at all
**Solution**: Verify inter-measurement period > 0 and interrupt GPIO is properly configured in device tree

**Problem**: Pin always stays HIGH
**Solution**: Check if object is permanently within detection threshold, or manually clear interrupt in application
