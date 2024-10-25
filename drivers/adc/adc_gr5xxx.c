/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/pm/device.h>

#include "adc_gr5xxx.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_gr5xxx, CONFIG_ADC_LOG_LEVEL);

#define DT_DRV_COMPAT goodix_gr5xxx_adc

#ifdef CONFIG_ADC_GR5XXX_SUPPORT_ZEPHYR_CONVERT
#define GR5XXX_ADC_REF_VOLT_MV_0P8 (800 * 2)
#define GR5XXX_ADC_REF_VOLT_MV_1P2 (1200 * 2)
#define GR5XXX_ADC_REF_VOLT_MV_1P6 (1600 * 2)

#define GET_ADC_TRIM_OFFSET(p_trim, ref_src) (p_trim)->UTIL_CAT(offset_int_, ref_src)
#define GET_ADC_TRIM_SLOPE(p_trim, ref_src) (p_trim)->UTIL_CAT(slope_int_, ref_src)
#else
#define GR5XXX_ADC_REF_VOLT_MV_0P8 (800)
#define GR5XXX_ADC_REF_VOLT_MV_1P2 (1200)
#define GR5XXX_ADC_REF_VOLT_MV_1P6 (1600)
#endif // CONFIG_ADC_GR5XXX_SUPPORT_ZEPHYR_CONVERT

#define GR5XXX_ADC_INPUT_SRC_MASK ( \
    BIT(ADC_INPUT_SRC_IO0) |        \
    BIT(ADC_INPUT_SRC_IO1) |        \
    BIT(ADC_INPUT_SRC_IO2) |        \
    BIT(ADC_INPUT_SRC_IO3) |        \
    BIT(ADC_INPUT_SRC_IO4) |        \
    BIT(ADC_INPUT_SRC_TMP) |        \
    BIT(ADC_INPUT_SRC_BAT))

#define ACQUIRE_DATA() adc_gr5xxx_data_t *data = (adc_gr5xxx_data_t *)dev->data
#define ACQUIRE_CONFIG() adc_gr5xxx_config_t *config = (adc_gr5xxx_config_t *)dev->config
#define ACQUIRE_PHANDLE() adc_handle_t *p_handle = &(((adc_gr5xxx_data_t *)dev->data)->handle)

uint16_t adc_get_trim_func(adc_trim_info_t *p_adc_trim)
{
    return sys_adc_trim_get(p_adc_trim);
}

static int check_buffer(const struct adc_sequence *sequence)
{
    size_t channel_count = POPCOUNT(sequence->channels);
    size_t min_bufsiz = channel_count * sizeof(uint16_t);
    if (sequence->options)
    {
        min_bufsiz *= (1 + sequence->options->extra_samplings);
    }

    if (min_bufsiz > sequence->buffer_size)
    {
        return -ENOMEM;
    }

    return 0;
}

static int check_sequence(const struct adc_sequence *sequence)
{
    // Check channel map
    uint32_t channel_mask = sequence->channels;
    if (!channel_mask || (channel_mask & ~BIT_MASK(CONFIG_ADC_GR5XXX_ADC_CHANNEL_COUNT)))
    {
        LOG_ERR("Invalid channels: 0x%08x\n", channel_mask);
        return -EINVAL;
    }

    // Check (or ignore?) other unsupported parameters
    if (sequence->resolution != 13)
    {
        LOG_ERR("Invalid resolution: %d\n", sequence->resolution);
        return -EINVAL;
    }

    if (sequence->oversampling > CONFIG_ADC_GR5XXX_MAX_OVERSAMPLING_EXP)
    {
        LOG_ERR("Invalid oversampling: %d, max %d\n", sequence->oversampling, CONFIG_ADC_GR5XXX_MAX_OVERSAMPLING_EXP);
        return -EINVAL;
    }

    if (sequence->calibrate)
    {
        return -ENOTSUP;
    }

    return check_buffer(sequence);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
#ifdef CONFIG_ADC_ASYNC
    if (ctx->asynchronous)
    {
        // TODO: Async using DMA

    }
    else
#endif // CONFIG_ADC_ASYNC
    {
        adc_gr5xxx_data_t *data = (adc_gr5xxx_data_t *)CONTAINER_OF(ctx, adc_gr5xxx_data_t, ctx);
        adc_handle_t *p_handle = &data->handle;

#if CONFIG_ADC_GR5XXX_MAX_OVERSAMPLING_EXP > 0
        uint16_t oversampling_buffer[(1 << CONFIG_ADC_GR5XXX_MAX_OVERSAMPLING_EXP)];
#endif // CONFIG_ADC_GR5XXX_MAX_OVERSAMPLING_EXP > 0

        uint16_t *p_buffer = (uint16_t *)ctx->sequence.buffer;
        uint16_t sample_length = 1 << ctx->sequence.oversampling; // 2 ^ oversampling
        uint32_t channel_mask = ctx->sequence.channels;


        uint32_t channel_id = 0;
        while(channel_mask)
        {
            if (channel_mask & BIT(0))
            {
                // Select corresponding channel(s)
                if (data->channels_cfg[channel_id].differential)
                {
                    p_handle->init.input_mode = ADC_INPUT_DIFFERENTIAL;
                    ll_adc_set_input_mode(LL_ADC_INPUT_DIFFERENTIAL);
                    ll_adc_set_channelp(data->channels_cfg[channel_id].input_positive);
                    ll_adc_set_channeln(data->channels_cfg[channel_id].input_negative);
                }
                else
                {
                    p_handle->init.input_mode = ADC_INPUT_SINGLE;
                    ll_adc_set_input_mode(LL_ADC_INPUT_SINGLE);
                    // This is ON PURPOSE since in single mode GR5xxx's ADC choose to capture on channel_n
                    ll_adc_set_channeln(data->channels_cfg[channel_id].input_positive);
                }

                // Start conversion
                hal_status_t err;
#if CONFIG_ADC_GR5XXX_MAX_OVERSAMPLING_EXP > 0
                if (sample_length > 1)
                {
                    // oversampling
                    err = hal_adc_poll_for_conversion(p_handle, oversampling_buffer, sample_length);
                    uint16_t sample_avg = oversampling_buffer[0];
                    for (int i = 1; i < sample_length; i++)
                    {
                        sample_avg += oversampling_buffer[i];
                        sample_avg /= 2;
                    }
                    *p_buffer = sample_avg;
                }
                else
#endif // CONFIG_ADC_GR5XXX_MAX_OVERSAMPLING_EXP
                {
                    err = hal_adc_poll_for_conversion(p_handle, p_buffer, 1);
                }

                if (HAL_OK != err)
                {
                    LOG_ERR("hal_adc_poll_for_conversion() return %d\n", err);
                    break;
                }

#ifdef CONFIG_ADC_GR5XXX_SUPPORT_ZEPHYR_CONVERT
                // Compensate raw value so that adc_raw_to_millivolts can get correct value
                const int ref_internal = -1 * UTIL_CAT(GR5XXX_ADC_REF_VOLT_MV_, DT_INST_STRING_UPPER_TOKEN(0, ref_voltage)) / 100;
                int raw_int = (int)(*p_buffer);
                int trim_slope = (int)GET_ADC_TRIM_SLOPE(&data->adc_trim, DT_INST_STRING_TOKEN(0, ref_voltage));
                int trim_offset = (int)GET_ADC_TRIM_OFFSET(&data->adc_trim, DT_INST_STRING_TOKEN(0, ref_voltage));
                raw_int = (81920 * (raw_int - trim_offset)) / (ref_internal * trim_slope);
                *p_buffer = (uint16_t)(raw_int);
#endif // CONFIG_ADC_GR5XXX_SUPPORT_ZEPHYR_CONVERT

                p_buffer++;
            }
            channel_mask >>= 1;
            channel_id++;
        }

        const struct device *dev = DEVICE_DT_INST_GET(0);
        adc_context_on_sampling_done(ctx, dev);
    }
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
    if (!repeat_sampling)
    {
        uint16_t *p_buffer = (uint16_t *)ctx->sequence.buffer;
        p_buffer += POPCOUNT(ctx->sequence.channels);
        ctx->sequence.buffer = (void *)p_buffer;
    }
}

static int adc_gr5xxx_channel_setup(const struct device *dev, const struct adc_channel_cfg *channel_cfg)
{
#ifndef ADC_GR5XXX_IGNORE_UNSUPPORTED_PARAMETERS
    // Parameters check
    if (channel_cfg->gain != ADC_GAIN_1)
    {
        // Should we implement software gain?
        return -ENOTSUP;
    }
    // TODO: external reference
    if (channel_cfg->reference != ADC_REF_INTERNAL)
    {
        return -ENOTSUP;
    }
    // Acquisiton time not configurable
    if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT)
    {
        return -ENOTSUP;
    }
#endif // ADC_GR5XXX_IGNORE_UNSUPPORTED_PARAMETERS

    // Logical channel_id greater than maximum supported
    if (channel_cfg->channel_id >= CONFIG_ADC_GR5XXX_ADC_CHANNEL_COUNT)
    {
        return -EINVAL;
    }

    if ((BIT(channel_cfg->input_positive) & GR5XXX_ADC_INPUT_SRC_MASK) == 0)
    {
        // input_positive pin invalid
        return -EINVAL;
    }

    if (channel_cfg->differential)
    {
        if ((BIT(channel_cfg->input_negative) & GR5XXX_ADC_INPUT_SRC_MASK) == 0)
        {
            // input_negative pin invalid
            return -EINVAL;
        }
    }

    ACQUIRE_DATA();
    ACQUIRE_CONFIG();

    int err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
    if (err < 0)
    {
        return err;
    }

    uint8_t channel_id = channel_cfg->channel_id;
    data->channels_cfg[channel_id].input_positive = channel_cfg->input_positive;
    data->channels_cfg[channel_id].input_negative = channel_cfg->input_negative;
    data->channels_cfg[channel_id].reference = channel_cfg->reference; // incase we supported external reference in the future
    data->channels_cfg[channel_id].differential = channel_cfg->differential;

    return 0;
}

static int adc_gr5xxx_read(const struct device *dev, const struct adc_sequence *sequence)
{
    ACQUIRE_DATA();

    // Sanity check
    int err = check_sequence(sequence);
    if (err < 0)
    {
        return err;
    }

    adc_context_lock(&data->ctx, false, NULL);

    adc_context_start_read(&data->ctx, sequence);
    int error = adc_context_wait_for_completion(&data->ctx);

    adc_context_release(&data->ctx, error);

    return error;
}

#ifndef CONFIG_ADC_GR5XXX_SUPPORT_ZEPHYR_CONVERT
int gdx_adc_raw_to_voltage(uint16_t *raw, double *voltage, uint32_t length)
{
    if (!raw || !voltage || !length)
    {
        return -EINVAL;
    }

    const struct device *dev = DEVICE_DT_INST_GET(0);

    ACQUIRE_PHANDLE();

    hal_adc_voltage_intern(p_handle, raw, voltage, length);
    return 0;
}
#endif // CONFIG_ADC_GR5XXX_SUPPORT_ZEPHYR_CONVERT

#ifdef CONFIG_ADC_ASYNC

static int adc_gr5xxx_read_async(const struct device *dev, const struct adc_sequnece *sequence, struct k_poll_signal *async)
{
    ACQUIRE_DATA();

    // Sanity check
    int err = check_sequence(sequence);
    if (err < 0)
    {
        return err;
    }

    adc_context_lock(&data->ctx, true, async);

    adc_context_start_read(&data->ctx, sequence);
    int error = adc_context_wait_for_completion(&data->ctx);

    adc_context_release(&data->ctx, error);

    return error;
}

void hal_adc_conv_cplt_callback(adc_handle_t *p_adc)
{
}

#endif // CONFIG_ADC_ASYNC

static int adc_gr5xxx_init(const struct device *dev)
{
    ACQUIRE_PHANDLE();
    ACQUIRE_DATA();

    hal_status_t hal_err = hal_adc_init(p_handle);
    if (HAL_OK == hal_err)
    {
#ifdef CONFIG_ADC_GR5XXX_SUPPORT_ZEPHYR_CONVERT
        uint16_t ret = adc_get_trim_func(&data->adc_trim);
        if (ret != 0)
        {
            LOG_ERR("Failed to load ADC_TRIM information, error_code = %d\n", ret);
            data->adc_trim.slope_int_0p8 = 1;
            data->adc_trim.slope_int_1p2 = 1;
            data->adc_trim.slope_int_1p6 = 1;
            data->adc_trim.slope_int_2p0 = 1;
            data->adc_trim.slope_ext_1p0 = 1;
        }
#endif // CONFIG_ADC_GR5XXX_SUPPORT_ZEPHYR_CONVERT
        adc_context_unlock_unconditionally(&data->ctx);
    }
    return hal_err;
}

static const struct adc_driver_api adc_gr5xxx_driver_api = {
	.channel_setup = adc_gr5xxx_channel_setup,
	.read = adc_gr5xxx_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_gr5xxx_read_async,
#endif // CONFIG_ADC_ASYNC
	.ref_internal = UTIL_CAT(GR5XXX_ADC_REF_VOLT_MV_, DT_INST_STRING_UPPER_TOKEN(0, ref_voltage)),
};

#ifdef CONFIG_PM_DEVICE

static int adc_gr5xxx_suspend(const struct device *dev)
{
    ACQUIRE_PHANDLE();

    hal_adc_state_t state = hal_adc_get_state(p_handle);
    if ((state != HAL_ADC_STATE_RESET) && (state != HAL_ADC_STATE_READY))
    {
        return -EBUSY;
    }
    hal_adc_suspend_reg(p_handle);
    return 0;
}

static int adc_gr5xxx_resume(const struct device *dev)
{
    ACQUIRE_PHANDLE();
    hal_adc_resume_reg(p_handle);
    return 0;
}

static int adc_gr5xxx_pm_action(const struct device *dev, enum pm_device_action action)
{
    switch (action)
    {
    case PM_DEVICE_ACTION_SUSPEND:
        return adc_gr5xxx_suspend(dev);

    case PM_DEVICE_ACTION_RESUME:
        return adc_gr5xxx_resume(dev);

    default:
        return -ENOTSUP;
    }
}

#endif // CONFIG_PM_DEVICE

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
// Make sure only 1 instance is declared
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1, "GR5xxx only support one ADC instance");
// Define PINCTRL instance
PINCTRL_DT_INST_DEFINE(0);
// Declare device config
static const adc_gr5xxx_config_t adc_cfg_0 = {
    .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
};
// Declare device data
static adc_gr5xxx_data_t adc_data_0 = {
    ADC_CONTEXT_INIT_TIMER(adc_data_0, ctx),
    ADC_CONTEXT_INIT_LOCK(adc_data_0, ctx),
    ADC_CONTEXT_INIT_SYNC(adc_data_0, ctx),
    .handle = {
        .init = {
            .channel_p = ADC_INPUT_SRC_IO0,
            .channel_n = ADC_INPUT_SRC_IO1,
            .input_mode = ADC_INPUT_DIFFERENTIAL,
            .ref_source = ADC_REF_SRC_BUF_INT,
            .ref_value = UTIL_CAT(ADC_REF_VALUE_, DT_INST_STRING_UPPER_TOKEN(0, ref_voltage)),
            .clock = UTIL_CAT(ADC_CLK_, DT_INST_STRING_UPPER_TOKEN(0, clock)),
        },
    },
};
// Define PM device
PM_DEVICE_DT_INST_DEFINE(0, adc_gr5xxx_pm_action);
// Define device init
DEVICE_DT_INST_DEFINE(
    0,
    adc_gr5xxx_init,
    PM_DEVICE_DT_INST_GET(0),
    &adc_data_0,
    &adc_cfg_0,
    POST_KERNEL,
    CONFIG_ADC_INIT_PRIORITY,
    &adc_gr5xxx_driver_api
);

#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
