# Copyright (c) 2024 Goodix
# SPDX-License-Identifier: Apache-2.0

zephyr_sources(
    app_info.c
)

zephyr_linker_sources(ROM_START SORT_KEY 0x1app_info app_info.ld)
