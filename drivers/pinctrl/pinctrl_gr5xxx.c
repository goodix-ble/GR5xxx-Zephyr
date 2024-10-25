/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/gr5xxx_pinctrl.h>

#include "gr55xx_hal_gpio.h"
#include "gr55xx_hal_aon_gpio.h"
#include "gr55xx_hal_msio.h"

#define PIN_MASK(pin) (1 << (pin))

#if CONFIG_SOC_GR5525
#define IO_MUX_GPIO GPIO_MUX_8
#else
#define IO_MUX_GPIO GPIO_MUX_7
#endif // CONFIG_SOC_GR5525

static gpio_regs_t *const GPIOx_LUT[] = {GPIO0, GPIO1, GPIO2};

static void gr5xxx_pin_config(const pinctrl_soc_pin_t *pin)
{
    switch (pin->fields.port)
    {
        case GR5XXX_PORT_GPIOA:
        case GR5XXX_PORT_GPIOB:
        case GR5XXX_PORT_GPIOC:
        {
            gpio_init_t init = {
                .pin = PIN_MASK(pin->fields.pin),
                .mode = pin->fields.mux == IO_MUX_GPIO ? GPIO_MODE_OUTPUT : GPIO_MODE_MUX,
                .pull = pin->fields.pull,
                .mux = pin->fields.mux,
#if CONFIG_SOC_GR5525
                .speed = GPIO_SPEED_MEDIUM,
                .strength = pin->fields.strength,
                .input_type = pin->fields.is_schmitt,
#endif // CONFIG_SOC_GR5525
            };
            hal_gpio_init(GPIOx_LUT[pin->fields.port], &init);
        }
        break;

        case GR5XXX_PORT_AONIO:
        {
            aon_gpio_init_t init = {
                .pin = PIN_MASK(pin->fields.pin),
                .mode = pin->fields.mux == IO_MUX_GPIO ? AON_GPIO_MODE_OUTPUT : AON_GPIO_MODE_MUX,
                .pull = pin->fields.pull,
                .mux = pin->fields.mux,
#if CONFIG_SOC_GR5525
                .speed = AON_GPIO_SPEED_MEDIUM,
                .strength = pin->fields.strength,
                .input_type = pin->fields.is_schmitt,
#endif // CONFIG_SOC_GR5525
            };
            hal_aon_gpio_init(&init);
        }
        break;

        case GR5XXX_PORT_MSIO:
        {
            msio_init_t init = {
                .pin = PIN_MASK(pin->fields.pin),
                .direction = pin->fields.mux == IO_MUX_GPIO ? MSIO_DIRECTION_OUTPUT : MSIO_DIRECTION_INPUT,
                .mode = pin->fields.is_analog,
                .pull = pin->fields.pull,
                .mux = pin->fields.mux,
#if CONFIG_SOC_GR5525
                .speed = AON_GPIO_SPEED_MEDIUM,
                .strength = pin->fields.strength,
                .input_type = pin->fields.is_schmitt,
#endif // CONFIG_SOC_GR5525
            };
            hal_msio_init(MSIOA, &init);
        }
    }
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
    ARG_UNUSED(reg);

    for (uint32_t i = 0; i < pin_cnt; i++)
    {
        gr5xxx_pin_config(&pins[i]);
    }

    return 0;
}
