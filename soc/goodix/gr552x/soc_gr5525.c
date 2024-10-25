/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "platform_sdk.h"
#include "hal_flash.h"

#include "soc.h"

#define PUYA_FLASH_HP_CMD (0xA3)
#define PUYA_FLASH_HP_END_DUMMY (2)
#define FALSH_HP_MODE XQSPI_HP_MODE_DIS
#define FLASH_HP_CMD PUYA_FLASH_HP_CMD
#define FLASH_HP_END_DUMMY PUYA_FLASH_HP_END_DUMMY
#define SOFTWARE_REG_WAKEUP_FLAG_POS (8)
#define SOFTWARE_REG2_ULTRA_DEEP_SLEEP_FLAG_POS (9)

extern void platform_xqspi_env_init(void);

// static void ultra_deep_sleep_wakeup_handle(void);
static void sys_device_reset_reason_enable(void);
static void nvds_setup(void);

boot_mode_t pwr_mgmt_get_wakeup_flag()
{
    if (AON_CTL->SOFTWARE_REG0 & (1 << SOFTWARE_REG_WAKEUP_FLAG_POS))
    {
        return WARM_BOOT;
    }
    return COLD_BOOT;
}

void first_class_task(void)
{
    exflash_hp_init_t hp_init;

    platform_xqspi_env_init();
    if (!hal_flash_init())
    {
        /* Flash fault, cannot startup.
         * TODO: Output log via UART or Dump an error code to flash. */
        while (1)
            ;
    }

    hp_init.xqspi_hp_enable = FALSH_HP_MODE;
    hp_init.xqspi_hp_cmd = FLASH_HP_CMD;
    hp_init.xqspi_hp_end_dummy = FLASH_HP_END_DUMMY;
    platform_flash_enable_quad(&hp_init);

    /* Configure the sleep memory retention, it's modifed in ultra deep sleep.
     * GR5525 dosen't support configuring memory retention bank separately,
     * so we configure all the memory being retained during deep sleep.
     */
    AON_PWR->MEM_PWR_SLP0 = 0x0000FFFF;
    AON_PWR->MEM_PWR_SLP1 = 0x000003FF;

    /* Enable chip reset reason record. */
    sys_device_reset_reason_enable();
    /* nvds module init process. */
    nvds_setup();

    /* IO leakage protect configuration. */
    system_io_leakage_protect();

    /* platform init process. */
    platform_sdk_init();
}

void second_class_task(void)
{
#if CFG_LPCLK_INTERNAL_EN
    platform_clock_init((mcu_clock_type_t)SYSTEM_CLOCK, (sdk_clock_type_t)RNG_OSC_CLK2, CFG_LF_ACCURACY_PPM, 0);
#else
    platform_clock_init((mcu_clock_type_t)SYSTEM_CLOCK, RTC_OSC_CLK, CFG_LF_ACCURACY_PPM, 0);
#endif

    system_pmu_init((mcu_clock_type_t)SYSTEM_CLOCK);
    SystemCoreSetClock((mcu_clock_type_t)SYSTEM_CLOCK);
    SetSerialClock(SERIAL_N96M_CLK);

    // recover the default setting by temperature, should be called in the end
    pmu_calibration_handler(NULL);
}

__WEAK void nvds_init_error_handler(uint8_t err_code)
{
    /* nvds_deinit will erase the flash area and old data will be lost */
#ifdef NVDS_START_ADDR
    nvds_deinit(NVDS_START_ADDR, NVDS_NUM_SECTOR);
    nvds_init(NVDS_START_ADDR, NVDS_NUM_SECTOR);
#else
    nvds_deinit(0, NVDS_NUM_SECTOR);
    nvds_init(0, NVDS_NUM_SECTOR);
#endif
}

static void sys_device_reset_reason_enable(void)
{
    //Clear all status and enable reset record
    AON_CTL->DBG_REG_RST_SRC = (1UL << 24) | (1UL << 31);
    //Busy configuration, Wait
    while (AON_CTL->DBG_REG_RST_SRC & (1UL << 30));
    //Wait record ready
    while (!(AON_CTL->DBG_REG_RST_SRC & (1UL << 16)));
}

static void nvds_setup(void)
{
#ifndef ATE_TEST_ENABLE
    nvds_retention_size(4);

#ifdef NVDS_START_ADDR
    uint8_t err_code = nvds_init(NVDS_START_ADDR, NVDS_NUM_SECTOR);
#else
    uint8_t err_code = nvds_init(0, NVDS_NUM_SECTOR);
#endif

    switch(err_code)
    {
        case NVDS_SUCCESS:
            break;
        default:
            /* Nvds initialization errors.
             * For more information, please see NVDS_INIT_ERR_CODE. */
            nvds_init_error_handler(err_code);
            break;
    }
#endif //ATE_TEST_ENABLE
}
