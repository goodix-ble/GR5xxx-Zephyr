/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __GPIO_GR5XXX_H__
#define __GPIO_GR5XXX_H__

#include <zephyr/drivers/gpio.h>

typedef struct
{
    /* gpio_driver_config needs to be first */
    struct gpio_driver_config common;
    uint32_t reg;
    void (*irq_config_func)(const struct device *port);
} gpio_gr5xxx_config_t;

typedef struct
{
    /* gpio_driver_data needs to be first*/
    struct gpio_driver_data common;
    sys_slist_t cb;
} gpio_gr5xxx_data_t;

#endif // __GPIO_GR5XXX_H__
