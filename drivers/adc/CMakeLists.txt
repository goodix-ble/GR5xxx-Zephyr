# Copyright (c) 2024 Goodix
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_ADC_GR5XXX adc_gr5xxx.c)
zephyr_include_directories(${ZEPHYR_BASE}/drivers/adc)