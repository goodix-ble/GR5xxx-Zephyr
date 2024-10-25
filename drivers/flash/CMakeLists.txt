# Copyright (c) 2024 Goodix
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(${ZEPHYR_BASE}/drivers/flash)
zephyr_library_sources_ifdef(CONFIG_SOC_FLASH_GR55XX soc_flash_gr55xx.c)

