/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __I2C_GR5XXX_H__
#define __I2C_GR5XXX_H__

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include "gr55xx_hal_i2c_patch.h"

typedef struct
{
    const struct pinctrl_dev_config *pcfg;
    uint32_t irqn;
    uint32_t dt_clock_frequency;
    void (*irq_config_func)(void);
} i2c_gr5xxx_config_t;

typedef struct
{
    i2c_handle_t handle;
    struct k_sem sync;
    struct k_sem lock;
    uint32_t dev_config;
    struct i2c_msg *curr_msg;
#ifdef CONFIG_I2C_CALLBACK
    i2c_callback_t cb;
    void *userdata;
    uint16_t slave_addr;
    uint8_t num_msgs;
#endif // CONFIG_I2C_CALLBACK
} i2c_gr5xxx_data_t;

#endif // __I2C_GR5XXX_H__
