# Copyright (c) 2024 Goodix
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_I2C_GR5XXX i2c_gr5xxx.c gr55xx_hal_i2c_patch.c)
zephyr_include_directories(${ZEPHYR_BASE}/drivers/i2c)