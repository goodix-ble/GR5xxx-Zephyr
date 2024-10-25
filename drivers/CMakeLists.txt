# Copyright (c) 2024 Goodix
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_ADC               adc)
add_subdirectory_ifdef(CONFIG_BT_DRIVERS        bluetooth)
add_subdirectory_ifdef(CONFIG_FLASH             flash)
add_subdirectory_ifdef(CONFIG_GPIO              gpio)
add_subdirectory_ifdef(CONFIG_I2C               i2c)
add_subdirectory_ifdef(CONFIG_PINCTRL           pinctrl)
add_subdirectory_ifdef(CONFIG_PWM               pwm)
add_subdirectory_ifdef(CONFIG_SERIAL            serial)
add_subdirectory_ifdef(CONFIG_ENTROPY_GENERATOR trng)
add_subdirectory_ifdef(CONFIG_SYS_CLOCK_EXISTS  timer)