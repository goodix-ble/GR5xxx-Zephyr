/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SOC_H__
#define __SOC_H__

#include "gr_includes.h"

typedef void (*FuncVector_t)(void);

boot_mode_t pwr_mgmt_get_wakeup_flag(void);

void first_class_task(void);

void second_class_task(void);

#endif // __SOC_H__
