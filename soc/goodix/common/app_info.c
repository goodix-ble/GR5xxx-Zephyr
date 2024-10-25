/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "custom_config.h"
#include "gr_plat.h"

#include <zephyr/linker/linker-defs.h>

#define APP_CODE_LOAD_ADDR_ZEPHYR (CONFIG_FLASH_BASE_ADDRESS + CONFIG_FLASH_LOAD_OFFSET)
#define APP_INFO_ADDR_ZEPHYR (APP_CODE_LOAD_ADDR_ZEPHYR + 0x200)
#define CHECKSUM_ZEPHYR (APP_INFO_PATTERN_VALUE + APP_INFO_VERSION + CHIP_VER + APP_CODE_LOAD_ADDR_ZEPHYR + APP_CODE_LOAD_ADDR_ZEPHYR)

const APP_INFO_t BUILD_IN_APP_INFO __attribute__((section(".app_info"))) =
{
    .app_pattern      = APP_INFO_PATTERN_VALUE,
    .app_info_version = APP_INFO_VERSION,
    .chip_ver         = CHIP_VER,
    .load_addr        = APP_CODE_LOAD_ADDR_ZEPHYR,
    .run_addr         = APP_CODE_LOAD_ADDR_ZEPHYR,
    .app_info_sum     = CHECKSUM_ZEPHYR,
    .check_img        = BOOT_CHECK_IMAGE,
    .boot_delay       = BOOT_LONG_TIME,
    .sec_cfg          = SECURITY_CFG_VAL,
#ifdef APP_INFO_COMMENTS
    .comments         = APP_INFO_COMMENTS,
#endif
};
