/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __UART_GR5XXX_H__
#define __UART_GR5XXX_H__

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include "gr55xx_hal_uart.h"

typedef struct
{
    const struct pinctrl_dev_config *pcfg;
    uint32_t irqn;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
    uart_irq_config_func_t irq_config_func;
#endif // defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
} uart_gr5xxx_config_t;

typedef struct
{
    uart_handle_t handle;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    uart_irq_callback_user_data_t cb;
    void *user_data;
#endif // CONFIG_UART_INTERRUPT_DRIVEN
#ifdef CONFIG_UART_ASYNC_API
    uart_callback_t async_cb;
    void *async_cb_user_data;
#endif // CONFIG_UART_ASYNC_API
} uart_gr5xxx_data_t;

#endif // __UART_GR5XXX_H__