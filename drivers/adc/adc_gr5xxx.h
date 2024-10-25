/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ADC_GR5XXX_H__
#define __ADC_GR5XXX_H__

#include <zephyr/drivers/pinctrl.h>
#include "gr55xx_hal_adc.h"

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

typedef struct
{
    const struct pinctrl_dev_config *pcfg;
} adc_gr5xxx_config_t;

typedef struct
{
    struct adc_context ctx;
    adc_handle_t handle;
    struct {
        uint8_t input_positive;
        uint8_t input_negative;
        uint8_t reference;
        uint8_t differential;
    } channels_cfg[CONFIG_ADC_GR5XXX_ADC_CHANNEL_COUNT];
#ifdef CONFIG_ADC_GR5XXX_SUPPORT_ZEPHYR_CONVERT
    adc_trim_info_t adc_trim;
#endif // CONFIG_ADC_GR5XXX_SUPPORT_ZEPHYR_CONVERT
} adc_gr5xxx_data_t;

#ifndef CONFIG_ADC_GR5XXX_SUPPORT_ZEPHYR_CONVERT
int gdx_adc_raw_to_voltage(uint16_t *raw, double *voltage, uint32_t length);
#endif // CONFIG_ADC_GR5XXX_SUPPORT_ZEPHYR_CONVERT

#endif // __ADC_GR5XXX_H__
