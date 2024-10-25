/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

#include "gpio_gr5xxx.h"
#include "gr5525_a0.h"
#include "gr5525_int_a0.h"
#include "gr55xx_hal_gpio.h"
#include "gr55xx_hal_aon_gpio.h"
#include "gr55xx_hal_msio.h"

#define PIN_MASK(pin) (1 << (pin))

#if CONFIG_SOC_GR5525
#define IO_MUX_GPIO GPIO_MUX_8
#else
#define IO_MUX_GPIO GPIO_MUX_7
#endif // CONFIG_SOC_GR5525

#define MASK_SHIFT(X) (31 - __builtin_clz((X) & -(X)))

#ifndef GPIO_PULL_MASK
#define GPIO_PULL_MASK (GPIO_PULL_UP | GPIO_PULL_DOWN)
#endif // GPIO_PULL_MASK

#ifndef GPIO_PULL_POS
#define GPIO_PULL_POS MASK_SHIFT(GPIO_PULL_MASK)
#endif // GPIO_PULL_POS

#ifndef GPIO_DIR_MASK
#define GPIO_DIR_MASK (GPIO_INPUT | GPIO_OUTPUT)
#endif // GPIO_DIR_MASK

#ifndef GPIO_DIR_POS
#define GPIO_DIR_POS MASK_SHIFT(GPIO_DIR_MASK)
#endif // GPIO_DIR_POS

#define ACQUIRE_CONFIG() gpio_gr5xxx_config_t *config = (gpio_gr5xxx_config_t *)port->config
#define ACQUIRE_DATA() gpio_gr5xxx_data_t *data = (gpio_gr5xxx_data_t *)port->data

static int gpio_gr5xxx_init(const struct device *port)
{
    ACQUIRE_CONFIG();
    if (config->irq_config_func)
    {
        config->irq_config_func(port);
    }
    return 0;
}

static int gpio_gr5xxx_pin_configure(const struct device *port, gpio_pin_t pin, gpio_flags_t flags)
{
    ACQUIRE_CONFIG();
    uint32_t pinmask = PIN_MASK(pin);
    // GPIO, AONIO, MSIO mode and pull shares exact same value
    uint32_t io_dir = ((flags & GPIO_DIR_MASK) >> GPIO_DIR_POS);
    uint32_t io_pull = ((flags & GPIO_PULL_MASK) >> GPIO_PULL_POS);

    if (config->reg == AON_IO_BASE)
    {
        // AON IO
        ll_aon_gpio_init_t aon_gpio_init = {
            .pin = pinmask,
            .mode = io_dir,
            .pull = io_pull,
            .mux = IO_MUX_GPIO,
            .speed = LL_AON_GPIO_SPEED_MEDIUM,
            .strength = LL_AON_GPIO_STRENGTH_MEDIUM,
            .input_type = LL_AON_GPIO_INPUT_TYPE_CMOS,
            .trigger = LL_AON_GPIO_TRIGGER_NONE,
        };
        error_status_t err = ll_aon_gpio_init(&aon_gpio_init);
        if (err == SUCCESS)
        {
            if (flags & GPIO_OUTPUT_INIT_HIGH)
            {
                ll_aon_gpio_set_output_pin(pinmask);
            }
            else if (flags & GPIO_OUTPUT_INIT_LOW)
            {
                ll_aon_gpio_reset_output_pin(pinmask);
            }
        }
        else
        {
            return -EIO;
        }
    }
    else if (config->reg == AON_MSIO_BASE)
    {
        // MSIO
        ll_msio_init_t msio_init = {
            .pin = pinmask,
            .direction = io_dir,
            .mode = LL_MSIO_MODE_DIGITAL,
            .pull = io_pull,
            .mux = IO_MUX_GPIO,
            .speed = LL_MSIO_SPEED_MEDIUM,
            .strength = LL_MSIO_STRENGTH_MEDIUM,
            .input_type = LL_MSIO_INPUT_TYPE_CMOS,
        };
        error_status_t err = ll_msio_init(MSIOA, &msio_init);
        if (err == SUCCESS)
        {
            if (flags & GPIO_OUTPUT_INIT_HIGH)
            {
                ll_msio_set_output_pin(MSIOA, pinmask);
            }
            else if (flags & GPIO_OUTPUT_INIT_LOW)
            {
                ll_msio_reset_output_pin(MSIOA, pinmask);
            }
        }
        else
        {
            return -EIO;
        }
    }
    else
    {
        // Normal GPIO
        gpio_regs_t *GPIOx = (gpio_regs_t *)config->reg;
        ll_gpio_init_t gpio_init = {
            .pin = pinmask,
            .mode = io_dir,
            .pull = io_pull,
            .mux = IO_MUX_GPIO,
            .speed = LL_GPIO_SPEED_MEDIUM,
            .strength = LL_GPIO_STRENGTH_MEDIUM,
            .input_type = LL_GPIO_INPUT_TYPE_CMOS,
            .trigger = LL_GPIO_TRIGGER_NONE,
        };
        error_status_t err = ll_gpio_init(GPIOx, &gpio_init);
        if (err == SUCCESS)
        {
            if (flags & GPIO_OUTPUT_INIT_HIGH)
            {
                ll_gpio_set_output_pin(GPIOx, pinmask);
            }
            else if (flags & GPIO_OUTPUT_INIT_LOW)
            {
                ll_gpio_reset_output_pin(GPIOx, pinmask);
            }
        }
        else
        {
            return -EIO;
        }
    }
    return 0;
}

#ifdef CONFIG_GPIO_GET_CONFIG
static int gpio_gr5xxx_pin_get_config(const struct device *port, gpio_pin_t pin, gpio_flags_t *flags)
{
    ACQUIRE_CONFIG();
    uint32_t pinmask = PIN_MASK(pin);
    if (config->reg == AON_IO_BASE)
    {
        // AON IO
        *flags = (ll_aon_gpio_get_pin_mode(pinmask) << GPIO_DIR_POS) | (ll_aon_gpio_get_pin_pull(pinmask) << GPIO_PULL_POS);
        if ((*flags) & GPIO_OUTPUT)
        {
            if (ll_aon_gpio_read_output_pin(pinmask))
            {
                *flags |= GPIO_OUTPUT_HIGH;
            }
            else
            {
                *flags |= GPIO_OUTPUT_LOW;
            }
        }
    }
    else if (config->reg == AON_MSIO_BASE)
    {
        // MSIO
        *flags = (ll_msio_get_pin_direction(MSIOA, pinmask) << GPIO_DIR_POS) | (ll_msio_get_pin_pull(MSIOA, pinmask) << GPIO_PULL_POS);
        if ((*flags) & GPIO_OUTPUT)
        {
            if (ll_msio_read_output_pin(pinmask))
            {
                *flags |= GPIO_OUTPUT_HIGH;
            }
            else
            {
                *flags |= GPIO_OUTPUT_LOW;
            }
        }
    }
    else
    {
        // Normal GPIO
        gpio_regs_t *GPIOx = (gpio_regs_t *)config->reg;
        *flags = (ll_gpio_get_pin_mode(GPIOx, pinmask) << GPIO_DIR_POS) | (ll_gpio_get_pin_pull(GPIOx, pinmask) << GPIO_PULL_POS);
        if ((*flags) & GPIO_OUTPUT)
        {
            if (ll_gpio_read_output_pin(GPIOx, pinmask))
            {
                *flags |= GPIO_OUTPUT_HIGH;
            }
            else
            {
                *flags |= GPIO_OUTPUT_LOW;
            }
        }
    }
    return 0;
}
#endif

static int gpio_gr5xxx_port_get_raw(const struct device *port, gpio_port_value_t *value)
{
    ACQUIRE_CONFIG();
    if (config->reg == AON_IO_BASE)
    {
        // AON IO
        *value = ll_aon_gpio_read_input_port();
    }
    else if (config->reg == AON_MSIO_BASE)
    {
        // MSIO
        *value = ll_msio_read_input_port(MSIOA);
    }
    else
    {
        // Normal GPIO
        gpio_regs_t *GPIOx = (gpio_regs_t *)config->reg;
        *value = ll_gpio_read_input_port(GPIOx);
    }
    return 0;
}

static int gpio_gr5xxx_port_set_masked_raw(const struct device *port, gpio_port_pins_t mask, gpio_port_value_t value)
{
    ACQUIRE_CONFIG();
    if (config->reg == AON_IO_BASE)
    {
        // AON IO
        uint32_t port_value = ll_aon_gpio_read_output_port();
        ll_aon_gpio_set_output_pin((port_value & ~mask) | (mask & value));
    }
    else if (config->reg == AON_MSIO_BASE)
    {
        // MSIO
        uint32_t port_value = ll_msio_read_output_port(MSIOA);
        ll_msio_set_output_pin(MSIOA, (port_value & ~mask) | (mask & value));
    }
    else
    {
        // Normal GPIO
        gpio_regs_t *GPIOx = (gpio_regs_t *)config->reg;
        uint32_t port_value = ll_gpio_read_output_port(GPIOx);
        ll_gpio_set_output_pin(GPIOx, (port_value & ~mask) | (mask & value));
    }
    return 0;
}

static int gpio_gr5xxx_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
    ACQUIRE_CONFIG();
    if (config->reg == AON_IO_BASE)
    {
        // AON IO
        ll_aon_gpio_set_output_pin(pins);
    }
    else if (config->reg == AON_MSIO_BASE)
    {
        // MSIO
        ll_msio_set_output_pin(MSIOA, pins);
    }
    else
    {
        // Normal GPIO
        gpio_regs_t *GPIOx = (gpio_regs_t *)config->reg;
        ll_gpio_set_output_pin(GPIOx, pins);
    }
    return 0;
}

static int gpio_gr5xxx_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
    ACQUIRE_CONFIG();
    if (config->reg == AON_IO_BASE)
    {
        // AON IO
        ll_aon_gpio_reset_output_pin(pins);
    }
    else if (config->reg == AON_MSIO_BASE)
    {
        // MSIO
        ll_msio_reset_output_pin(MSIOA, pins);
    }
    else
    {
        // Normal GPIO
        gpio_regs_t *GPIOx = (gpio_regs_t *)config->reg;
        ll_gpio_reset_output_pin(GPIOx, pins);
    }
    return 0;
}

static int gpio_gr5xxx_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
    ACQUIRE_CONFIG();
    if (config->reg == AON_IO_BASE)
    {
        // AON IO
        ll_aon_gpio_toggle_pin(pins);
    }
    else if (config->reg == AON_MSIO_BASE)
    {
        // MSIO
        ll_msio_toggle_pin(MSIOA, pins);
    }
    else
    {
        // Normal GPIO
        gpio_regs_t *GPIOx = (gpio_regs_t *)config->reg;
        ll_gpio_toggle_pin(GPIOx, pins);
    }
    return 0;
}

static int gpio_gr5xxx_pin_interrupt_configure(const struct device *port, gpio_pin_t pin, enum gpio_int_mode int_mode, enum gpio_int_trig int_trig)
{
    ACQUIRE_CONFIG();
    uint32_t pinmask = PIN_MASK(pin);
    if (config->reg == AON_IO_BASE)
    {
        // AON IO
        if (int_mode == GPIO_INT_MODE_DISABLED)
        {
            ll_aon_gpio_disable_it(pinmask);
        }
        else
        {
            if (int_mode == GPIO_INT_MODE_LEVEL)
            {
                // Level trigger
                switch (int_trig)
                {
                    case GPIO_INT_TRIG_LOW:
                        ll_aon_gpio_enable_low_trigger(pinmask);
                        break;

                    case GPIO_INT_TRIG_HIGH:
                        ll_aon_gpio_enable_high_trigger(pinmask);
                        break;

                    default:
                        return -ENOTSUP;
                }
            }
            else
            {
                // Edge trigger
                switch (int_trig)
                {
                    case GPIO_INT_TRIG_LOW:
                        ll_aon_gpio_enable_falling_trigger(pinmask);
                        break;

                    case GPIO_INT_TRIG_HIGH:
                        ll_aon_gpio_enable_rising_trigger(pinmask);
                        break;

                    case GPIO_INT_TRIG_BOTH:
                        ll_aon_gpio_enable_both_trigger(pinmask);
                        break;

                    default:
                        return -ENOTSUP;
                }
            }

            ll_aon_gpio_enable_it(pinmask);
        }
    }
    else if (config->reg == AON_MSIO_BASE)
    {
        // MSIO do not suport interrupt
        return -ENOTSUP;
    }
    else
    {
        // Normal GPIO
        gpio_regs_t *GPIOx = (gpio_regs_t *)config->reg;
        if (int_mode == GPIO_INT_MODE_DISABLED)
        {
            ll_gpio_disable_it(GPIOx, pinmask);
        }
        else
        {
            if (int_mode == GPIO_INT_MODE_LEVEL)
            {
                // Level trigger
                switch (int_trig)
                {
                    case GPIO_INT_TRIG_LOW:
                        ll_gpio_enable_low_trigger(GPIOx, pinmask);
                        break;

                    case GPIO_INT_TRIG_HIGH:
                        ll_gpio_enable_high_trigger(GPIOx, pinmask);
                        break;

                    default:
                        return -ENOTSUP;
                }
            }
            else
            {
                // Edge trigger
                switch (int_trig)
                {
                    case GPIO_INT_TRIG_LOW:
                        ll_gpio_enable_falling_trigger(GPIOx, pinmask);
                        break;

                    case GPIO_INT_TRIG_HIGH:
                        ll_gpio_enable_rising_trigger(GPIOx, pinmask);
                        break;

                    case GPIO_INT_TRIG_BOTH:
                        ll_gpio_enable_both_edge_trigger(GPIOx, pinmask);
                        break;

                    default:
                        return -ENOTSUP;
                }
            }

            ll_gpio_enable_it(GPIOx, pinmask);
        }
    }
    return 0;
}

static int gpio_gr5xxx_manage_callback(const struct device *port, struct gpio_callback *cb, bool set)
{
    ACQUIRE_DATA();
    return gpio_manage_callback(&data->cb, cb, set);
}

static const struct gpio_driver_api gpio_gr5xxx_driver_api = {
    .pin_configure = gpio_gr5xxx_pin_configure,
#ifdef CONFIG_GPIO_GET_CONFIG
    .pin_get_config = gpio_gr5xxx_pin_get_config,
#endif
    .port_get_raw = gpio_gr5xxx_port_get_raw,
    .port_set_masked_raw = gpio_gr5xxx_port_set_masked_raw,
    .port_set_bits_raw = gpio_gr5xxx_port_set_bits_raw,
    .port_clear_bits_raw = gpio_gr5xxx_port_clear_bits_raw,
    .port_toggle_bits = gpio_gr5xxx_port_toggle_bits,
    .pin_interrupt_configure = gpio_gr5xxx_pin_interrupt_configure,
    .manage_callback = gpio_gr5xxx_manage_callback,
};

void gpio_gr5xxx_isr(const struct device *port)
{
    ACQUIRE_CONFIG();
    ACQUIRE_DATA();
    uint32_t pinmask_triggered = 0;
    if (config->reg == AON_IO_BASE)
    {
        // AON IO
        pinmask_triggered = ll_aon_gpio_read_flag_it(LL_AON_GPIO_PIN_ALL);
        hal_aon_gpio_irq_handler();
    }
    else
    {
        // Normal GPIO
        gpio_regs_t *GPIOx = (gpio_regs_t *)config->reg;
        pinmask_triggered = ll_gpio_read_flag_it(GPIOx, LL_GPIO_PIN_ALL);
        hal_gpio_exti_irq_handler(GPIOx);
    }

    gpio_fire_callbacks(&data->cb, port, pinmask_triggered);
}

// Since we are having gpioa/b/c/aonio/msio not gpio0/1/2/3/4, DO NOT use any DT macros that contains _INST_

// Declaration
#define GR5XXX_GPIO_IRQ_CONFIG_FUNC_DECL(name) static void gpio_gr5xxx_irq_config_func_##name(const struct device *dev)
// Fill the slot in config struct
#define GR5XXX_GPIO_IRQ_CONFIG_FUNC_FILL(name) .irq_config_func = gpio_gr5xxx_irq_config_func_##name
// Implementation
#define GR5XXX_GPIO_IRQ_CONFIG_FUNC_IMPL(name)                                                       \
    GR5XXX_GPIO_IRQ_CONFIG_FUNC_DECL(name)                                                           \
    {                                                                                                \
        IRQ_CONNECT(DT_IRQN(name), DT_IRQ(name, priority), gpio_gr5xxx_isr, DEVICE_DT_GET(name), 0); \
        irq_enable(DT_IRQN(name));                                                                   \
    }

#define GR5XXX_GPIO_INIT(name)                                                                 \
    /* Declare irq_config_func (if gpio has irq) */                                            \
    COND_CODE_1(DT_IRQ_HAS_IDX(name, 0), (GR5XXX_GPIO_IRQ_CONFIG_FUNC_DECL(name);), ())        \
    /* Declare device config */                                                                \
    static const gpio_gr5xxx_config_t DT_CAT(gpio_cfg_, name) = {                              \
        .common = {                                                                            \
            .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_NODE(name),                            \
        },                                                                                     \
        .reg = DT_REG_ADDR(name),                                                              \
        COND_CODE_1(DT_IRQ_HAS_IDX(name, 0), (GR5XXX_GPIO_IRQ_CONFIG_FUNC_FILL(name), ), ())   \
    };                                                                                         \
    /* Declare device data */                                                                  \
    static gpio_gr5xxx_data_t DT_CAT(gpio_data_, name);                                        \
    /* Define device init */                                                                   \
    DEVICE_DT_DEFINE(                                                                          \
        name,                                                                                  \
        gpio_gr5xxx_init,                                                                      \
        NULL, /* No PM device needed for gpio */                                               \
        &DT_CAT(gpio_data_, name),                                                             \
        &DT_CAT(gpio_cfg_, name),                                                              \
        PRE_KERNEL_1,                                                                          \
        CONFIG_GPIO_INIT_PRIORITY,                                                             \
        &gpio_gr5xxx_driver_api);                                                              \
    /* Implement gpio_irq_config_func (if gpio has irq) */                                     \
    COND_CODE_1(DT_IRQ_HAS_IDX(name, 0), (GR5XXX_GPIO_IRQ_CONFIG_FUNC_IMPL(name)), ())

#define GPIO_DEVICE_INIT_GR5XXX_IF_OKAY(name)                 \
    COND_CODE_1(DT_NODE_HAS_STATUS(DT_NODELABEL(name), okay), \
                (GR5XXX_GPIO_INIT(DT_NODELABEL(name))),       \
                ())

// Generate init request for every used GPIO instance in devicetree
GPIO_DEVICE_INIT_GR5XXX_IF_OKAY(gpioa);
GPIO_DEVICE_INIT_GR5XXX_IF_OKAY(gpiob);
GPIO_DEVICE_INIT_GR5XXX_IF_OKAY(gpioc);
GPIO_DEVICE_INIT_GR5XXX_IF_OKAY(aonio);
GPIO_DEVICE_INIT_GR5XXX_IF_OKAY(msio);
