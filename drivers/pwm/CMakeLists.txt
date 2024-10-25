# Copyright (c) 2024 Goodix
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_PWM_GR5XXX pwm_gr5xxx.c)