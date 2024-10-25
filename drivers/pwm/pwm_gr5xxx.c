/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pwm.h>
#include <zephyr/pm/device.h>

#include "pwm_gr5xxx.h"
#include "gr55xx_ll_cgc.h"

#define DT_DRV_COMPAT goodix_gr5xxx_pwm

#define ACQUIRE_DATA() pwm_gr5xxx_data_t *data = (pwm_gr5xxx_data_t *)dev->data
#define ACQUIRE_CONFIG() pwm_gr5xxx_config_t *config = (pwm_gr5xxx_config_t *)dev->config
#define ACQUIRE_PWMX() pwm_regs_t *PWMx = ((pwm_gr5xxx_data_t *)dev->data)->p_instance

typedef enum
{
    PWM_CH_A = 0,
    PWM_CH_B,
    PWM_CH_C,
} pwm_channel_t;

__STATIC_INLINE void pwm_set_action_event_cmp_x0(pwm_regs_t *PWMx, pwm_channel_t ch, uint32_t action_event)
{
    /*
     * ll_pwm_set_action_event_cmp_a0
     * ll_pwm_set_action_event_cmp_b0
     * ll_pwm_set_action_event_cmp_c0
     * PWM_AQCTRL_X0_Pos = X * 4
     */
    const uint32_t PWM_AQCTRL_X0_Pos = (uint32_t)ch * 4;
    const uint32_t PWM_AQCTRL_X0_Msk = 0x3U << PWM_AQCTRL_X0_Pos;
    MODIFY_REG(PWMx->AQCTRL, PWM_AQCTRL_X0_Msk, action_event << PWM_AQCTRL_X0_Pos);
}

__STATIC_INLINE void pwm_set_action_event_cmp_x1(pwm_regs_t *PWMx, pwm_channel_t ch, uint32_t action_event)
{
    /*
     * ll_pwm_set_action_event_cmp_a1
     * ll_pwm_set_action_event_cmp_b1
     * ll_pwm_set_action_event_cmp_c1
     * PWM_AQCTRL_X1_Pos = X * 4 + 2
     */
    const uint32_t PWM_AQCTRL_X1_Pos = (uint32_t)ch * 4 + 2;
    const uint32_t PWM_AQCTRL_X1_Msk = 0x3U << PWM_AQCTRL_X1_Pos;
    MODIFY_REG(PWMx->AQCTRL, PWM_AQCTRL_X1_Msk, action_event << PWM_AQCTRL_X1_Pos);
}

__STATIC_INLINE void pwm_enable_positive_drive_channel_x(pwm_regs_t *PWMx, pwm_channel_t ch)
{
    /*
     * ll_pwm_enable_positive_drive_channel_a
     * ll_pwm_enable_positive_drive_channel_b
     * ll_pwm_enable_positive_drive_channel_c
     * PWM_MODE_DPENX_Pos = X + 3
     */
    const uint32_t PWM_MODE_DPENX_Pos = (uint32_t)ch + 3;
    const uint32_t PWM_MODE_DPENX_Msk = 0x1U << PWM_MODE_DPENX_Pos;
    SET_BITS(PWMx->MODE, PWM_MODE_DPENX_Msk);
}

__STATIC_INLINE void pwm_disable_positive_drive_channel_x(pwm_regs_t *PWMx, pwm_channel_t ch)
{
    /*
     * ll_pwm_disable_positive_drive_channel_a
     * ll_pwm_disable_positive_drive_channel_b
     * ll_pwm_disable_positive_drive_channel_c
     * PWM_MODE_DPENX_Pos = X + 3
     */
    const uint32_t PWM_MODE_DPENX_Pos = (uint32_t)ch + 3;
    const uint32_t PWM_MODE_DPENX_Msk = 0x1U << PWM_MODE_DPENX_Pos;
    CLEAR_BITS(PWMx->MODE, PWM_MODE_DPENX_Msk);
}

__STATIC_INLINE void pwm_set_compare_x0(pwm_regs_t *PWMx, pwm_channel_t ch, uint32_t compare)
{
    /*
     * ll_pwm_set_compare_a0
     * ll_pwm_set_compare_b0
     * ll_pwm_set_compare_c0
     * &PWMx->CMPX0 = (&PWMx->CMPA0) + X * 2
     */
    volatile uint32_t *PMWx_CMPX0 = &(PWMx->CMPA0) + (uint32_t)ch * 2;
    WRITE_REG(*PMWx_CMPX0, compare);
}

__STATIC_INLINE void pwm_set_compare_x1(pwm_regs_t *PWMx, pwm_channel_t ch, uint32_t compare)
{
    /*
     * ll_pwm_set_compare_a1
     * ll_pwm_set_compare_b1
     * ll_pwm_set_compare_c1
     * &PWMx->CMPX1 = (&PWMx->CMPA1) + X * 2
     */
    volatile uint32_t *PMWx_CMPX1 = &(PWMx->CMPA1) + (uint32_t)ch * 2;
    WRITE_REG(*PMWx_CMPX1, compare);
}

static int pwm_gr5xxx_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles, uint32_t pulse_cycles, pwm_flags_t flags)
{
    // Param check
    if (channel >= 3)
    {
        return -EINVAL;
    }

    ACQUIRE_PWMX();
    if (pulse_cycles == 0)
    {
        // Inactive
        // Since the counter will most likely not count to end, disable sync update to make sure clear operation takes effect asap.
        ll_pwm_disable_update_all(PWMx);
        // Configure X0 & X1 to 0 to avoid 100% period inactive stays high issue
        pwm_set_compare_x0(PWMx, (pwm_channel_t)channel, 0);
        pwm_set_compare_x1(PWMx, (pwm_channel_t)channel, 0);
        pwm_set_action_event_cmp_x0(PWMx, (pwm_channel_t)channel, LL_PWM_ACTIONEVENT_CLEAR);
        pwm_set_action_event_cmp_x1(PWMx, (pwm_channel_t)channel, LL_PWM_ACTIONEVENT_NONE);
        ll_pwm_enable_update_all(PWMx);

        if (PWMx->AQCTRL == 0x111)
        {
            /**
             * If PWMx->AQCTRL == 0x111, it means all channels are deactivated,
             * and only in this scenario that we could suspend
             */
            pm_device_busy_clear(dev);
        }
    }
    else
    {
        // Active
        pm_device_busy_set(dev);

        ll_pwm_disable_pause(PWMx);

        if (flags & PWM_POLARITY_INVERTED)
        {
            pwm_disable_positive_drive_channel_x(PWMx, (pwm_channel_t)channel);
        }
        else
        {
            pwm_enable_positive_drive_channel_x(PWMx, (pwm_channel_t)channel);
        }

        pwm_set_compare_x0(PWMx, (pwm_channel_t)channel, 0);
        pwm_set_compare_x1(PWMx, (pwm_channel_t)channel, pulse_cycles);

        pwm_set_action_event_cmp_x0(PWMx, (pwm_channel_t)channel, LL_PWM_ACTIONEVENT_SET);
        pwm_set_action_event_cmp_x1(PWMx, (pwm_channel_t)channel, LL_PWM_ACTIONEVENT_CLEAR);

        ll_pwm_set_prescaler(PWMx, period_cycles);
    }

    return 0;
}

static int pwm_gr5xxx_get_cycles_per_sec(const struct device *dev, uint32_t channel, uint64_t *cycles)
{
    (void)dev;
    (void)channel;

    SystemCoreUpdateClock();
    *cycles = SystemCoreClock;

    return 0;
}

static const struct pwm_driver_api pwm_gr5xxx_driver_api = {
    .set_cycles = pwm_gr5xxx_set_cycles,
    .get_cycles_per_sec = pwm_gr5xxx_get_cycles_per_sec,
};

static int pwm_gr5xxx_init(const struct device *dev)
{
    ACQUIRE_CONFIG();
    ACQUIRE_PWMX();

    // Extracted from hal_pwm_init()
    /* Enable Clock for Serial blocks and Automatic turn off Serial blocks clock during WFI. */
    ll_cgc_disable_force_off_serial_hclk();
    ll_cgc_disable_wfi_off_serial_hclk();

    /* Enable PWM Clock and Automatic turn off PWM clock during WFI. */
    if (PWMx == PWM0)
    {
        ll_cgc_disable_force_off_pwm0_hclk();
        ll_cgc_disable_pwm0_slp_wfi();
    }
    else if (PWMx == PWM1)
    {
        ll_cgc_disable_force_off_pwm1_hclk();
        ll_cgc_disable_pwm1_slp_wfi();
    }

    ll_pwm_enable(PWMx);

    // Reset state for every channel
    ll_pwm_set_compare_a0(PWMx, 0);
    ll_pwm_set_compare_a1(PWMx, 0);
    ll_pwm_set_action_event_cmp_a0(PWMx, LL_PWM_ACTIONEVENT_CLEAR);
    ll_pwm_set_action_event_cmp_a1(PWMx, LL_PWM_ACTIONEVENT_NONE);
    ll_pwm_set_flicker_stop_level_a(PWMx, LL_PWM_STOP_LVL_LOW);

    ll_pwm_set_compare_b0(PWMx, 0);
    ll_pwm_set_compare_b1(PWMx, 0);
    ll_pwm_set_action_event_cmp_b0(PWMx, LL_PWM_ACTIONEVENT_CLEAR);
    ll_pwm_set_action_event_cmp_b1(PWMx, LL_PWM_ACTIONEVENT_NONE);
    ll_pwm_set_flicker_stop_level_b(PWMx, LL_PWM_STOP_LVL_LOW);

    ll_pwm_set_compare_c0(PWMx, 0);
    ll_pwm_set_compare_c1(PWMx, 0);
    ll_pwm_set_action_event_cmp_c0(PWMx, LL_PWM_ACTIONEVENT_CLEAR);
    ll_pwm_set_action_event_cmp_b1(PWMx, LL_PWM_ACTIONEVENT_NONE);
    ll_pwm_set_flicker_stop_level_c(PWMx, LL_PWM_STOP_LVL_LOW);

    // Pause before configure
    ll_pwm_enable_pause(PWMx);

    // Enable sync-update
    ll_pwm_enable_update_all(PWMx);

    // Init pinctrl at last to prevent uncertain GPIO state
    return pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
}

#ifdef CONFIG_PM_DEVICE

static int pwm_gr5xxx_suspend(const struct device *dev)
{
    ACQUIRE_DATA();
    ACQUIRE_PWMX();

    data->retention[0] = READ_REG(PWMx->MODE);
    data->retention[1] = READ_REG(PWMx->PRD);
    data->retention[2] = READ_REG(PWMx->BRPRD);
    data->retention[3] = READ_REG(PWMx->HOLD);
    data->retention[4] = READ_REG(PWMx->CMPA0);
    data->retention[5] = READ_REG(PWMx->CMPA1);
    data->retention[6] = READ_REG(PWMx->CMPB0);
    data->retention[7] = READ_REG(PWMx->CMPB1);
    data->retention[8] = READ_REG(PWMx->CMPC0);
    data->retention[9] = READ_REG(PWMx->CMPC1);
    data->retention[10] = READ_REG(PWMx->AQCTRL);

    return 0;
}

static int pwm_gr5xxx_resume(const struct device *dev)
{
    ACQUIRE_DATA();
    ACQUIRE_PWMX();

    SET_BITS(PWMx->UPDATE, PWM_UPDATE_SA);
    MODIFY_REG(PWMx->MODE, PWM_MODE_BREATHEN, (data->retention[0] & PWM_MODE_BREATHEN_Msk));
    SET_BITS(PWMx->MODE, PWM_MODE_FLICKER_PAUSE_LEVEL_A);
    SET_BITS(PWMx->MODE, PWM_MODE_FLICKER_PAUSE_LEVEL_B);
    SET_BITS(PWMx->MODE, PWM_MODE_FLICKER_PAUSE_LEVEL_C);
    SET_BITS(PWMx->MODE, PWM_MODE_BREATH_PAUSE_LEVEL);
    MODIFY_REG(PWMx->MODE, PWM_MODE_PAUSE, (data->retention[0] & PWM_MODE_PAUSE_Msk)); // pause or on-going
    MODIFY_REG(PWMx->MODE, PWM_MODE_DPENA, (data->retention[0] & PWM_MODE_DPENA_Msk));
    MODIFY_REG(PWMx->MODE, PWM_MODE_DPENB, (data->retention[0] & PWM_MODE_DPENB_Msk));
    MODIFY_REG(PWMx->MODE, PWM_MODE_DPENC, (data->retention[0] & PWM_MODE_DPENC_Msk));
    WRITE_REG(PWMx->PRD, data->retention[1]);
    WRITE_REG(PWMx->BRPRD, data->retention[2]);
    MODIFY_REG(PWMx->HOLD, PWM_HOLD_HOLD, (data->retention[3] & PWM_HOLD_HOLD_Msk));
    WRITE_REG(PWMx->CMPA0, data->retention[4]);
    WRITE_REG(PWMx->CMPA1, data->retention[5]);
    WRITE_REG(PWMx->CMPB0, data->retention[6]);
    WRITE_REG(PWMx->CMPB1, data->retention[7]);
    WRITE_REG(PWMx->CMPC0, data->retention[8]);
    WRITE_REG(PWMx->CMPC1, data->retention[9]);
    MODIFY_REG(PWMx->AQCTRL, PWM_AQCTRL_A0, (data->retention[10] & PWM_AQCTRL_A0_Msk));
    MODIFY_REG(PWMx->AQCTRL, PWM_AQCTRL_A1, (data->retention[10] & PWM_AQCTRL_A1_Msk));
    MODIFY_REG(PWMx->AQCTRL, PWM_AQCTRL_B0, (data->retention[10] & PWM_AQCTRL_B0_Msk));
    MODIFY_REG(PWMx->AQCTRL, PWM_AQCTRL_B1, (data->retention[10] & PWM_AQCTRL_B1_Msk));
    MODIFY_REG(PWMx->AQCTRL, PWM_AQCTRL_C0, (data->retention[10] & PWM_AQCTRL_C0_Msk));
    MODIFY_REG(PWMx->AQCTRL, PWM_AQCTRL_C1, (data->retention[10] & PWM_AQCTRL_C1_Msk));
    MODIFY_REG(PWMx->MODE, PWM_MODE_EN, (data->retention[0] & PWM_MODE_EN_Msk)); // enable or disable

    return 0;
}

static int pwm_gr5xxx_pm_action(const struct device *dev, enum pm_device_action action)
{
    switch (action)
    {
    case PM_DEVICE_ACTION_SUSPEND:
        return pwm_gr5xxx_suspend(dev);

    case PM_DEVICE_ACTION_RESUME:
        return pwm_gr5xxx_resume(dev);

    default:
        return -ENOTSUP;
    }
}

#endif // CONFIG_PM_DEVICE

// Generate pwm_gr5xxx_config_t instance and pwm_gr5xxx_data_t instance from devicetree
#define GR5XXX_PWM_INIT(index)                                 \
    /* Define PINCTRL instance */                              \
    PINCTRL_DT_INST_DEFINE(index);                             \
    /* Declare device config */                                \
    static const pwm_gr5xxx_config_t pwm_cfg_##index = {       \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),         \
    };                                                         \
    /* Declare device data */                                  \
    static pwm_gr5xxx_data_t pwm_data_##index = {              \
        .p_instance = (pwm_regs_t *)(DT_INST_REG_ADDR(index)), \
    };                                                         \
    /* Define PM Device */                                     \
    PM_DEVICE_DT_INST_DEFINE(index, pwm_gr5xxx_pm_action);     \
    /* Define device init */                                   \
    DEVICE_DT_INST_DEFINE(                                     \
        index,                                                 \
        pwm_gr5xxx_init,                                       \
        PM_DEVICE_DT_INST_GET(index),                          \
        &pwm_data_##index,                                     \
        &pwm_cfg_##index,                                      \
        PRE_KERNEL_1,                                          \
        CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                    \
        &pwm_gr5xxx_driver_api);

// Generate init request for every used PWM instance in devicetree
DT_INST_FOREACH_STATUS_OKAY(GR5XXX_PWM_INIT);
