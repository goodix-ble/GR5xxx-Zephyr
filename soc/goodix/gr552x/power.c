/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>

#include "gr55xx.h"
#include "gr55xx_sys.h"

static volatile bool s_warm_boot_second_flag = false;

SECTION_RAM_CODE static void pwr_mgmt_enter_sleep(void);

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
    (void)substate_id;
    if (state == PM_STATE_SUSPEND_TO_IDLE)
    {
        // idle
        k_cpu_idle();
    }
    else if (state == PM_STATE_SOFT_OFF)
    {
        // deel sleep
        pwr_mgmt_enter_sleep();
    }
    else
    {
        // Unsupported PM state
    }
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
    (void)substate_id;
    if (state == PM_STATE_SOFT_OFF)
    {
        if (s_warm_boot_second_flag)
        {
            extern void warm_boot_second();
            warm_boot_second();
        }
    }

    __enable_irq();
}

SECTION_RAM_CODE static void pwr_mgmt_enter_sleep(void)
{
    __disable_irq();
    s_warm_boot_second_flag = false;

    if (PMR_MGMT_SLEEP_MODE != pwr_mgmt_mode_get() || DEVICE_BUSY == pwr_mgmt_dev_suspend())
    {
        k_cpu_idle();
        __enable_irq();
        return;
    }

    uint8_t ret = pwr_mgmt_sleep();
    if (ret != PMR_MGMT_SUCCESS)
    {
        // failed to go to deep sleep
        k_cpu_idle();
        __enable_irq();
    }
    else
    {
        // woke-up from deep sleep and context loaded
        s_warm_boot_second_flag = true;
    }
}
