/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/pm/device.h>

#include "uart_gr5xxx.h"

#define DT_DRV_COMPAT goodix_gr5xxx_uart

#define ACQUIRE_DATA() uart_gr5xxx_data_t *data = (uart_gr5xxx_data_t *)dev->data
#define ACQUIRE_CONFIG() uart_gr5xxx_config_t *config = (uart_gr5xxx_config_t *)dev->config
#define ACQUIRE_PHANDLE() uart_handle_t *p_handle = &(((uart_gr5xxx_data_t *)dev->data)->handle)
#define ACQUIRE_UARTX() uart_regs_t *UARTx = (((uart_gr5xxx_data_t *)dev->data)->handle).p_instance

#define UART_CFG_NOT_SUPPORTED 0xFFFFFFFFU

#define _CFG_LUT_REVERSE(out, LUT, in)                      \
    do                                                      \
    {                                                       \
        for ((out) = 0; (out) < ARRAY_SIZE((LUT)); (out)++) \
        {                                                   \
            if ((LUT)[(out)] == (in))                       \
            {                                               \
                break;                                      \
            }                                               \
        }                                                   \
    } while (0)

static const uint32_t CFG_LUT_PARITY[] = {
    [UART_CFG_PARITY_NONE] = LL_UART_PARITY_NONE,
    [UART_CFG_PARITY_ODD] = LL_UART_PARITY_ODD,
    [UART_CFG_PARITY_EVEN] = LL_UART_PARITY_EVEN,
    [UART_CFG_PARITY_MARK] = UART_CFG_NOT_SUPPORTED,  // UART_CFG_PARITY_MARK is not supported by GR5xxx
    [UART_CFG_PARITY_SPACE] = UART_CFG_NOT_SUPPORTED, // UART_CFG_PARITY_SPACE is not supporte by GR5xxx
};

static const uint32_t CFG_LUT_STOPBITS[] = {
    [UART_CFG_STOP_BITS_0_5] = UART_CFG_NOT_SUPPORTED, // UART_CFG_STOP_BITS_0_5 is not supported by GR5xxx
    [UART_CFG_STOP_BITS_1] = LL_UART_STOPBITS_1,
    [UART_CFG_STOP_BITS_1_5] = LL_UART_STOPBITS_1_5,
    [UART_CFG_STOP_BITS_2] = LL_UART_STOPBITS_2,
};

static const uint32_t CFG_LUT_DATABIS[] = {
    [UART_CFG_DATA_BITS_5] = LL_UART_DATABITS_5B,
    [UART_CFG_DATA_BITS_6] = LL_UART_DATABITS_6B,
    [UART_CFG_DATA_BITS_7] = LL_UART_DATABITS_7B,
    [UART_CFG_DATA_BITS_8] = LL_UART_DATABITS_8B,
    [UART_CFG_DATA_BITS_9] = UART_CFG_NOT_SUPPORTED, // UART_CFG_DATA_BITS_9 is not supported by GR5xxx
};

static const uint32_t CFG_LUT_FLOWCTRL[] = {
    [UART_CFG_FLOW_CTRL_NONE] = LL_UART_HWCONTROL_NONE,
    [UART_CFG_FLOW_CTRL_RTS_CTS] = LL_UART_HWCONTROL_RTS_CTS,
    [UART_CFG_FLOW_CTRL_DTR_DSR] = UART_CFG_NOT_SUPPORTED, // UART_CFG_FLOW_CTRL_DTR_DSR is not supported by GR5xxx
    [UART_CFG_FLOW_CTRL_RS485] = UART_CFG_NOT_SUPPORTED,   // UART_CFG_FLOW_CTRL_RS485 is not supported by GR5xxx
};

extern uint32_t SystemCoreClock;

static int uart_gr5xxx_init(const struct device *dev)
{
    ACQUIRE_PHANDLE();
    ACQUIRE_CONFIG();

    int err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
    if (err < 0)
    {
        return err;
    }

    hal_status_t hal_err = hal_uart_init(p_handle);
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
    if (HAL_OK == hal_err)
    {
        config->irq_config_func(dev);
    }
#endif // defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
    return hal_err;
}

#ifdef CONFIG_UART_ASYNC_API
static int uart_gr5xxx_callback_set(const struct device *dev, uart_callback_t callback, void *user_data)
{
    ACQUIRE_DATA();

    data->async_cb = callback;
    data->async_cb_user_data = user_data;
#ifdef CONFIG_UART_EXCLUSIVE_API_CALLBACKS
    data->cb = NULL;
    data->user_data = NULL;
#endif // CONFIG_UART_EXCLUSIVE_API_CALLBACKS
}

static int uart_gr5xxx_tx(const struct device *dev, const uint8_t *buf, size_t len, int32_t timeout)
{
}

static int uart_gr5xxx_tx_abort(const struct device *dev)
{
}

static int uart_gr5xxx_rx_enable(const struct device *dev, uint8_t *buf, size_t len, int32_t timeout)
{
}

static int uart_gr5xxx_rx_buf_rsp(const struct device *dev, uint8_t *buf, size_t len)
{
}

static int uart_gr5xxx_rx_disable(const struct device *dev)
{
}

#endif // CONFIG_UART_ASYNC_API

static int uart_gr5xxx_poll_in(const struct device *dev, unsigned char *p_char)
{
    ACQUIRE_UARTX();

    if (!ll_uart_is_active_flag_rfne(UARTx))
    {
        return -1;
    }

    *p_char = ll_uart_receive_data8(UARTx);
    return 0;
}

static void uart_gr5xxx_poll_out(const struct device *dev, unsigned char out_char)
{
    ACQUIRE_UARTX();

    while (!ll_uart_is_active_flag_tfnf(UARTx))
        ;

    ll_uart_transmit_data8(UARTx, out_char);

    while (!ll_uart_is_active_flag_tfe(UARTx))
        ;
}

static int uart_gr5xxx_err_check(const struct device *dev)
{
    ACQUIRE_UARTX();

    uint32_t err = 0;
    uint32_t flag = ll_uart_get_line_status_flag(UARTx);

    if (flag & LL_UART_LSR_OE)
    {
        err |= UART_ERROR_OVERRUN;
    }

    if (flag & LL_UART_LSR_PE)
    {
        err |= UART_ERROR_PARITY;
    }

    if (flag & LL_UART_LSR_FE)
    {
        err |= UART_ERROR_FRAMING;
    }

    ll_uart_clear_line_status_flag(UARTx);

    return err;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_gr5xxx_configure(const struct device *dev, const struct uart_config *cfg)
{
    ACQUIRE_PHANDLE();
    ACQUIRE_UARTX();

    // Sanity check
    uint32_t parity = CFG_LUT_PARITY[cfg->parity];
    if (parity == UART_CFG_NOT_SUPPORTED)
    {
        return -ENOTSUP;
    }

    uint32_t stop_bits = CFG_LUT_STOPBITS[cfg->stop_bits];
    if (stop_bits == UART_CFG_NOT_SUPPORTED)
    {
        return -ENOTSUP;
    }

    uint32_t data_bits = CFG_LUT_DATABIS[cfg->data_bits];
    if (data_bits == UART_CFG_NOT_SUPPORTED)
    {
        return -ENOTSUP;
    }

    uint32_t flow_ctrl = CFG_LUT_FLOWCTRL[cfg->flow_ctrl];
    if (flow_ctrl == UART_CFG_NOT_SUPPORTED)
    {
        return -ENOTSUP;
    }

    ll_uart_disable_fifo(UARTx);

    ll_uart_config_character(UARTx, data_bits, parity, stop_bits);
    ll_uart_set_hw_flow_ctrl(UARTx, flow_ctrl);
    if (cfg->baudrate != p_handle->init.baud_rate)
    {
        ll_uart_set_baud_rate(UARTx, SystemCoreClock, cfg->baudrate);
        p_handle->init.baud_rate = cfg->baudrate;
    }

    ll_uart_flush_tx_fifo(UARTx);
    ll_uart_flush_rx_fifo(UARTx);

    ll_uart_enable_fifo(UARTx);

    return 0;
}

static int uart_gr5xxx_config_get(const struct device *dev, struct uart_config *cfg)
{
    ACQUIRE_PHANDLE();
    ACQUIRE_UARTX();

    cfg->baudrate = p_handle->init.baud_rate;

    _CFG_LUT_REVERSE(cfg->parity, CFG_LUT_PARITY, ll_uart_get_parity(UARTx));
    _CFG_LUT_REVERSE(cfg->stop_bits, CFG_LUT_STOPBITS, ll_uart_get_stop_bits_length(UARTx));
    _CFG_LUT_REVERSE(cfg->data_bits, CFG_LUT_DATABIS, ll_uart_get_data_bits_length(UARTx));
    _CFG_LUT_REVERSE(cfg->flow_ctrl, CFG_LUT_FLOWCTRL, ll_uart_get_hw_flow_ctrl(UARTx));

    return 0;
}
#endif // CONFIG_UART_USE_RUNTIME_CONFIGURE

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_gr5xxx_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
    ACQUIRE_UARTX();

    uint8_t idx = 0;
    while((len - idx > 0) && ll_uart_is_active_flag_tfnf(UARTx))
    {
        ll_uart_transmit_data8(UARTx, tx_data[idx]);
    }

    return idx;
}

static int uart_gr5xxx_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
    ACQUIRE_UARTX();

    uint8_t idx = 0;
    while((size - idx > 0) && ll_uart_is_active_flag_rfne(UARTx))
    {
        rx_data[idx++] = ll_uart_receive_data8(UARTx);
        if (ll_uart_get_line_status_flag(UARTx) & LL_UART_LSR_OE)
        {
            ll_uart_flush_rx_fifo(UARTx);
        }
    }

    return idx;
}

static void uart_gr5xxx_irq_tx_enable(const struct device *dev)
{
    ACQUIRE_UARTX();

    ll_uart_set_tx_fifo_threshold(UARTx, LL_UART_TX_FIFO_TH_EMPTY);
    ll_uart_enable_it(UARTx, LL_UART_IER_THRE);
}

static void uart_gr5xxx_irq_tx_disable(const struct device *dev)
{
    ACQUIRE_UARTX();

    ll_uart_disable_it(UARTx, LL_UART_IER_THRE);
}

static int uart_gr5xxx_irq_tx_ready(const struct device *dev)
{
    ACQUIRE_UARTX();

    return ll_uart_is_active_flag_tfnf(UARTx);
}

static void uart_gr5xxx_irq_rx_enable(const struct device *dev)
{
    ACQUIRE_UARTX();

    ll_uart_set_rx_fifo_threshold(UARTx, LL_UART_RX_FIFO_TH_HALF_FULL);
    ll_uart_enable_it(UARTx, LL_UART_IER_RDA);
}

static void uart_gr5xxx_irq_rx_disable(const struct device *dev)
{
    ACQUIRE_UARTX();

    ll_uart_disable_it(UARTx, LL_UART_IER_RDA);
}

static int uart_gr5xxx_irq_tx_complete(const struct device *dev)
{
    ACQUIRE_UARTX();

    return ll_uart_is_active_flag_tfe(UARTx);
}

static int uart_gr5xxx_irq_rx_ready(const struct device *dev)
{
    ACQUIRE_UARTX();

    return ll_uart_is_active_flag_rfne(UARTx);
}

static void uart_gr5xxx_irq_err_enable(const struct device *dev)
{
    ACQUIRE_UARTX();

    ll_uart_enable_it(UARTx, LL_UART_IER_RLS);
}

static void uart_gr5xxx_irq_err_disable(const struct device *dev)
{
    ACQUIRE_UARTX();

    ll_uart_disable_it(UARTx, LL_UART_IER_RLS);
}

static int uart_gr5xxx_irq_is_pending(const struct device *dev)
{
    return -ENOSYS;
}

static int uart_gr5xxx_irq_update(const struct device *dev)
{
    return -ENOSYS;
}

static void uart_gr5xxx_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb, void *user_data)
{
    ACQUIRE_DATA();

    data->cb = cb;
    data->user_data = user_data;
#ifdef CONFIG_UART_EXCLUSIVE_API_CALLBACKS
    data->async_cb = NULL;
    data->async_cb_user_data = NULL;
#endif // CONFIG_UART_EXCLUSIVE_API_CALLBACKS
}
#endif // CONFIG_UART_INTERRUPT_DRIVEN

static const struct uart_driver_api uart_gr5xxx_driver_api = {
#ifdef CONFIG_UART_ASYNC_API
    .callback_set = uart_gr5xxx_callback_set,
    .tx = uart_gr5xxx_tx,
    .tx_abort = uart_gr5xxx_tx_abort,
    .rx_enable = uart_gr5xxx_rx_enable,
    .rx_buf_rsp = uart_gr5xxx_rx_buf_rsp,
    .rx_disable = uart_gr5xxx_rx_disable,
#endif // CONFIG_UART_ASYNC_API

    .poll_in = uart_gr5xxx_poll_in,
    .poll_out = uart_gr5xxx_poll_out,

    .err_check = uart_gr5xxx_err_check,

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
    .configure = uart_gr5xxx_configure,
    .config_get = uart_gr5xxx_config_get,
#endif // CONFIG_UART_USE_RUNTIME_CONFIGURE

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    .fifo_fill = uart_gr5xxx_fifo_fill,
    .fifo_read = uart_gr5xxx_fifo_read,
    .irq_tx_enable = uart_gr5xxx_irq_tx_enable,
    .irq_tx_disable = uart_gr5xxx_irq_tx_disable,
    .irq_tx_ready = uart_gr5xxx_irq_tx_ready,
    .irq_rx_enable = uart_gr5xxx_irq_rx_enable,
    .irq_rx_disable = uart_gr5xxx_irq_rx_disable,
    .irq_tx_complete = uart_gr5xxx_irq_tx_complete,
    .irq_rx_ready = uart_gr5xxx_irq_rx_ready,
    .irq_err_enable = uart_gr5xxx_irq_err_enable,
    .irq_err_disable = uart_gr5xxx_irq_err_disable,
    .irq_is_pending = uart_gr5xxx_irq_is_pending,
    .irq_update = uart_gr5xxx_irq_update,
    .irq_callback_set = uart_gr5xxx_irq_callback_set,
#endif // CONFIG_UART_INTERRUPT_DRIVEN
};

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)

void uart_gr5xxx_isr(const struct device *dev)
{
    ACQUIRE_PHANDLE();
    hal_uart_irq_handler(p_handle);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    ACQUIRE_DATA();
    if (data->cb)
    {
        data->cb(dev, data->user_data);
    }
#endif // CONFIG_UART_INTERRUPT_DRIVEN
}

#endif // defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)

#ifdef CONFIG_PM_DEVICE

static int uart_gr5xxx_suspend(const struct device *dev)
{
    ACQUIRE_PHANDLE();

    hal_uart_state_t state = hal_uart_get_state(p_handle);
    if ((state != HAL_UART_STATE_RESET) && (state != HAL_UART_STATE_READY))
    {
        return -EBUSY;
    }
    hal_uart_suspend_reg(p_handle);
    return 0;
}

static int uart_gr5xxx_resume(const struct device *dev)
{
    ACQUIRE_PHANDLE();
    ACQUIRE_CONFIG();

    hal_uart_resume_reg(p_handle);
    hal_nvic_clear_pending_irq(config->irqn);
    hal_nvic_enable_irq(config->irqn);

    return 0;
}

static int uart_gr5xxx_pm_action(const struct device *dev, enum pm_device_action action)
{
    switch (action)
    {
        case PM_DEVICE_ACTION_SUSPEND:
            return uart_gr5xxx_suspend(dev);

        case PM_DEVICE_ACTION_RESUME:
            return uart_gr5xxx_resume(dev);

        default:
            return -ENOTSUP;
    }
}

#endif // CONFIG_PM_DEVICE

// Generate irq_config_func if CONFIG_UART_INTERRUPT_DRIVEN is present
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
// Declaration
#define GR5XXX_UART_IRQ_CONFIG_FUNC_DECL(index) static void uart_gr5xxx_irq_config_func_##index(const struct device *dev)
// Fill the slot in config struct
#define GR5XXX_UART_IRQ_CONFIG_FUNC_FILL(index) .irq_config_func = uart_gr5xxx_irq_config_func_##index,
// Implementation
#define GR5XXX_UART_IRQ_CONFIG_FUNC_IMPL(index)                                                                        \
    GR5XXX_UART_IRQ_CONFIG_FUNC_DECL(index)                                                                            \
    {                                                                                                                  \
        IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority), uart_gr5xxx_isr, DEVICE_DT_INST_GET(index), 0); \
        irq_enable(DT_INST_IRQN(index));                                                                               \
    }

#else

#define GR5XXX_UART_IRQ_CONFIG_FUNC_DECL(index)
#define GR5XXX_UART_IRQ_CONFIG_FUNC_FILL(index)
#define GR5XXX_UART_IRQ_CONFIG_FUNC_IMPL(index)

#endif // defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)

// Generate uart_gr5xxx_config_t instance and uart_gr5xxx_data_t instacne from devicetree
#define GR5XXX_UART_INIT(index)                                                                     \
    /* Define PINCTRL instance */                                                                   \
    PINCTRL_DT_INST_DEFINE(index);                                                                  \
    /* Declare irq_config_func (if exist)*/                                                         \
    GR5XXX_UART_IRQ_CONFIG_FUNC_DECL(index);                                                        \
    /* Declare device config */                                                                     \
    static const uart_gr5xxx_config_t uart_cfg_##index = {                                          \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                              \
        .irqn = DT_INST_IRQN(index),                                                                \
        GR5XXX_UART_IRQ_CONFIG_FUNC_FILL(index)                                                     \
    };                                                                                              \
    /* Declare device data */                                                                       \
    static uart_gr5xxx_data_t uart_data_##index = {                                                 \
        .handle = {                                                                                 \
            .p_instance = (uart_regs_t *)(DT_INST_REG_ADDR(index)),                                 \
            .init = {                                                                               \
                .baud_rate = DT_INST_PROP(index, current_speed),                                    \
                .data_bits = UART_DATABITS_8,                                                       \
                .stop_bits = UART_STOPBITS_1,                                                       \
                .parity = CFG_LUT_PARITY[DT_INST_ENUM_IDX_OR(index, parity, UART_CFG_PARITY_NONE)], \
                .hw_flow_ctrl = DT_INST_PROP(index, hw_flow_control),                               \
                .rx_timeout_mode = UART_RECEIVER_TIMEOUT_ENABLE,                                    \
            },                                                                                      \
        },                                                                                          \
    };                                                                                              \
    /* Define PM Device */                                                                          \
    PM_DEVICE_DT_INST_DEFINE(index, uart_gr5xxx_pm_action);                                         \
    /* Define device init */                                                                        \
    DEVICE_DT_INST_DEFINE(                                                                          \
        index,                                                                                      \
        uart_gr5xxx_init,                                                                           \
        PM_DEVICE_DT_INST_GET(index),                                                               \
        &uart_data_##index,                                                                         \
        &uart_cfg_##index,                                                                          \
        PRE_KERNEL_1,                                                                               \
        CONFIG_SERIAL_INIT_PRIORITY,                                                                \
        &uart_gr5xxx_driver_api);                                                                   \
    /* Implement uart_irq_config_func (if exist) */                                                 \
    GR5XXX_UART_IRQ_CONFIG_FUNC_IMPL(index)

// Generate init request for every used UART instance in devicetree
DT_INST_FOREACH_STATUS_OKAY(GR5XXX_UART_INIT);
