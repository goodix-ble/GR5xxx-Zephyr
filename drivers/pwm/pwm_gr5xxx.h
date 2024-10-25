/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __PWM_GR5XXX_H__
#define __PWM_GR5XXX_H__

#include <zephyr/drivers/pinctrl.h>
#include "gr55xx_hal_pwm.h"

typedef struct
{
    const struct pinctrl_dev_config *pcfg;
} pwm_gr5xxx_config_t;

typedef struct
{
    /* simplified version of pwm_handle_t */
    pwm_regs_t *p_instance;
    uint32_t retention[12];
} pwm_gr5xxx_data_t;

#endif // __PWM_GR5XXX_H__
