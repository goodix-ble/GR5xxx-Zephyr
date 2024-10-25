/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __PMU_CALIBRATION_H__
#define __PMU_CALIBRATION_H__

/**
 * PMU Calibration dedicated for zephyr
 */

#include <stdint.h>

void system_pmu_calibration_init(uint32_t interval);
void system_pmu_calibration_stop(void);

#endif // __PMU_CALIBRATION_H__
