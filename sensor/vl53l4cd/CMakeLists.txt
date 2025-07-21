# SPDX-License-Identifier: Apache-2.0

# VL53L4CD sensor driver

zephyr_library()

# Add the main driver file
zephyr_library_sources_ifdef(CONFIG_VL53L4CD vl53l4cd.c)

# Add the platform abstraction layer
zephyr_library_sources_ifdef(CONFIG_VL53L4CD Platform/platform.c)

# Add the ST ULD driver sources
zephyr_library_sources_ifdef(CONFIG_VL53L4CD VL53L4CD_ULD_Driver/VL53L4CD_api.c)
zephyr_library_sources_ifdef(CONFIG_VL53L4CD VL53L4CD_ULD_Driver/VL53L4CD_calibration.c)

# Include directories
zephyr_library_include_directories_ifdef(CONFIG_VL53L4CD .)
zephyr_library_include_directories_ifdef(CONFIG_VL53L4CD Platform)
zephyr_library_include_directories_ifdef(CONFIG_VL53L4CD VL53L4CD_ULD_Driver)

# Add preprocessor definitions based on Kconfig
zephyr_library_compile_definitions_ifdef(CONFIG_VL53L4CD_I2C_FAST_MODE_PLUS VL53L4CD_I2C_FAST_MODE_PLUS)
