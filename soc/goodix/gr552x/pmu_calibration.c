/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include "platform_sdk.h"

#define FIRST_TIMER_MS (150)
#define DEFAULT_TIMER_MS (30 * 1000)

void pmu_calibration_timer_expiry(struct k_timer *p_timer)
{
    (void)p_timer;
    pmu_calibration_handler(NULL);
}

K_TIMER_DEFINE(pmu_calibration_timer, pmu_calibration_timer_expiry, NULL);

void system_pmu_calibration_init(uint32_t interval)
{
    if (interval)
    {
        k_timer_start(&pmu_calibration_timer, K_MSEC(FIRST_TIMER_MS), K_MSEC(interval));
    }
}

void system_pmu_calibration_stop(void)
{
    k_timer_stop(&pmu_calibration_timer);
}
