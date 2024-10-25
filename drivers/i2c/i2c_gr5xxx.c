/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/pm/device.h>

#include "i2c_gr5xxx.h"

#define DT_DRV_COMPAT goodix_gr5xxx_i2c

#define ACQUIRE_DATA() i2c_gr5xxx_data_t *data = (i2c_gr5xxx_data_t *)dev->data
#define ACQUIRE_CONFIG() i2c_gr5xxx_config_t *config = (i2c_gr5xxx_config_t *)dev->config
#define ACQUIRE_PHANDLE() i2c_handle_t *p_handle = &(((i2c_gr5xxx_data_t *)dev->data)->handle)
#define ACQUIRE_I2CX() i2c_regs_t *I2Cx = (((i2c_gr5xxx_data_t *)dev->data)->handle).p_instance

#define DEVICE_LOCK(dev)                                                                  \
    do                                                                                    \
    {                                                                                     \
        int err = k_sem_take(&((i2c_gr5xxx_data_t *)dev->data)->lock, Z_TIMEOUT_NO_WAIT); \
        if (err < 0)                                                                      \
        {                                                                                 \
            return err;                                                                   \
        }                                                                                 \
        pm_device_busy_set(dev);                                                          \
    } while (0)

#define DEVICE_UNLOCK(dev)                                   \
    do                                                       \
    {                                                        \
        k_sem_give(&((i2c_gr5xxx_data_t *)dev->data)->lock); \
        pm_device_busy_clear(dev);                           \
    } while (0)

#ifdef CONFIG_I2C_CALLBACK
#define I2C_DATA_CB_PLACEHOLDER ((i2c_callback_t)0xFFFFFFFF)
#endif // CONFIG_I2C_CALLBACK

static int i2c_gr5xxx_init(const struct device *dev)
{
    ACQUIRE_CONFIG();

    int err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
    if (err < 0)
    {
        return err;
    }

    config->irq_config_func();

    return 0;
}

static int i2c_gr5xxx_master_start_transfer(i2c_handle_t *p_handle, struct i2c_msg *msg, uint16_t addr)
{
    i2c_gr5xxx_data_t *data = CONTAINER_OF(p_handle, i2c_gr5xxx_data_t, handle);

    int err = 0;
    data->curr_msg = msg;

    if (msg->flags & I2C_MSG_ADDR_10_BITS)
    {
        ll_i2c_set_master_addressing_mode(p_handle->p_instance, LL_I2C_OWNADDRESS_10BIT);
    }
    else
    {
        ll_i2c_set_master_addressing_mode(p_handle->p_instance, LL_I2C_OWNADDRESS_7BIT);
    }

    hal_status_t ret;
    uint32_t extra_cmd = 0;

    if (msg->flags & I2C_MSG_STOP)
    {
        extra_cmd |= LL_I2C_CMD_MST_GEN_STOP;
    }

    if (msg->flags & I2C_MSG_READ)
    {
        ret = hal_i2c_master_receive_it_patch(p_handle, addr, msg->buf, msg->len, extra_cmd);
        if (HAL_OK != ret)
        {
            err = EIO;
        }
    }
    else
    {
        ret = hal_i2c_master_transmit_it_patch(p_handle, addr, msg->buf, msg->len, extra_cmd);
        if (HAL_OK != ret)
        {
            err = EIO;
        }
    }

    return err;
}

#ifdef CONFIG_I2C_CALLBACK
static inline void i2c_gr5xxx_call_user_cb(const struct device *dev)
{
    ACQUIRE_DATA();
    ACQUIRE_PHANDLE();
    if (data->cb != I2C_DATA_CB_PLACEHOLDER)
    {
        int result = p_handle->error_code == HAL_I2C_ERROR_NONE ? 0 : -EIO;
        data->cb(dev, result, data->userdata);
    }
    DEVICE_UNLOCK(dev);
    data->cb = NULL;
}
#endif // CONFIG_I2C_CALLBACK

static int i2c_gr5xxx_configure(const struct device *dev, uint32_t dev_config)
{
    ACQUIRE_CONFIG();
    ACQUIRE_DATA();
    ACQUIRE_PHANDLE();

    uint32_t speed = 0;
    switch(I2C_SPEED_GET(dev_config))
    {
        case I2C_SPEED_STANDARD:
            speed = I2C_SPEED_100K;
            break;

        case I2C_SPEED_FAST:
            speed = I2C_SPEED_400K;
            break;

        case I2C_SPEED_FAST_PLUS:
            speed = I2C_SPEED_1000K;
            break;

        case I2C_SPEED_HIGH:
            speed = 3400000;
            break;

        case I2C_SPEED_DT:
            speed = config->dt_clock_frequency;
            break;

        case I2C_SPEED_ULTRA:
            return -ENOTSUP;
    }

    p_handle->init.speed = speed;
    // Deprecated flag, only for backward compatibility
    p_handle->init.addressing_mode = (dev_config & I2C_ADDR_10_BITS) ? I2C_ADDRESSINGMODE_10BIT:I2C_ADDRESSINGMODE_7BIT;
    data->dev_config = dev_config;

    if (HAL_OK == hal_i2c_init(p_handle))
    {
        return 0;
    }
    return -EIO;
}

static int i2c_gr5xxx_get_config(const struct device *dev, uint32_t *dev_config)
{
    ACQUIRE_DATA();
    *dev_config = data->dev_config;
    return 0;
}

static int i2c_gr5xxx_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs, uint16_t addr)
{
    ACQUIRE_PHANDLE();
    ACQUIRE_DATA();

    if (!(data->dev_config & I2C_MODE_CONTROLLER))
    {
        return -ENOTSUP;
    }

    DEVICE_LOCK(dev);

    int err = 0;
    for (uint8_t i = 0; i < num_msgs; i++)
    {
        struct i2c_msg *p_msg = &msgs[i];

        err = i2c_gr5xxx_master_start_transfer(p_handle, p_msg, addr);
        if (err < 0)
        {
            break;
        }

        k_sem_take(&data->sync, K_FOREVER);
        if (p_handle->error_code != HAL_I2C_ERROR_NONE)
        {
            err = EIO;
            break;
        }
    }

    DEVICE_UNLOCK(dev);

    return -err;
}

#ifdef CONFIG_I2C_CALLBACK
static int i2c_gr5xxx_transfer_cb(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs, uint16_t addr, i2c_callback_t cb, void *userdata)
{
    ACQUIRE_DATA();
    ACQUIRE_PHANDLE();

    if (!(data->dev_config & I2C_MODE_CONTROLLER))
    {
        return -ENOTSUP;
    }

    DEVICE_LOCK(dev); // unlock after transfer done (or error)

    if (!cb)
    {
        /**
         * Maybe the developer is just not care about result, but we
         * use cb to check if this is an async transfer, so give it
         * a dummy value.
         */
        cb = I2C_DATA_CB_PLACEHOLDER;
    }

    data->cb = cb;
    data->num_msgs = num_msgs;
    data->userdata = userdata;
    data->slave_addr = addr;

    int err = i2c_gr5xxx_master_start_transfer(p_handle, msgs, addr);
    if (err < 0)
    {
        i2c_gr5xxx_call_user_cb(dev);
    }
    return err;
}
#endif /* CONFIG_I2C_CALLBACK */

static int i2c_gr5xxx_recover_bus(const struct device *dev)
{
    // GR5xxx I2C driver will automatically do recover once detected SDA_STUCK_AT_LOW
    return 0;
}

static const struct i2c_driver_api i2c_gr5xxx_driver_api = {
	.configure = i2c_gr5xxx_configure,
	.get_config = i2c_gr5xxx_get_config,
	.transfer = i2c_gr5xxx_transfer,
#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb = i2c_gr5xxx_transfer_cb,
#endif
	.recover_bus = i2c_gr5xxx_recover_bus,
};

void i2c_gr5xxx_isr(const struct device *dev)
{
    ACQUIRE_PHANDLE();
    hal_i2c_irq_handler(p_handle);
#ifdef CONFIG_I2C_CALLBACK
    ACQUIRE_DATA();
    if (data->cb)
    {
        if (data->num_msgs == 0)
        {
            i2c_gr5xxx_call_user_cb(dev);
        }
    }
#endif // CONFIG_I2C_CALLBACK
}

void hal_i2c_master_transfer_cplt_callback(i2c_handle_t *p_i2c)
{
    i2c_gr5xxx_data_t *data = CONTAINER_OF(p_i2c, i2c_gr5xxx_data_t, handle);
#ifdef CONFIG_I2C_CALLBACK
    if (data->cb)
    {
        data->num_msgs--;
        if (data->num_msgs > 0)
        {
            data->curr_msg++;
            int err = i2c_gr5xxx_master_start_transfer(p_i2c, data->curr_msg, data->slave_addr);
            if (err < 0)
            {
                data->num_msgs = 0;
                if (p_i2c->error_code == HAL_I2C_ERROR_NONE)
                {
                    // Busy, technically this should not happen.
                    p_i2c->error_code = 0xFFFFFFFFUL;
                }
            }
        }
    }
#endif // CONFIG_I2C_CALLBACK
    {
        k_sem_give(&data->sync);
    }
}

void hal_i2c_error_callback(i2c_handle_t *p_i2c)
{
    i2c_gr5xxx_data_t *data = CONTAINER_OF(p_i2c, i2c_gr5xxx_data_t, handle);
#ifdef CONFIG_I2C_CALLBACK
    if (data->cb)
    {
        data->num_msgs = 0;
    }
    else
#endif // CONFIG_I2C_CALLBACK
    {
        k_sem_give(&data->sync);
    }
}

#ifdef CONFIG_PM_DEVICE

static int i2c_gr5xxx_suspend(const struct device *dev)
{
    ACQUIRE_PHANDLE();

    hal_i2c_state_t state = hal_i2c_get_state(p_handle);
    if ((state != HAL_I2C_STATE_RESET) && (state != HAL_I2C_STATE_READY))
    {
        return -EBUSY;
    }
    hal_i2c_suspend_reg(p_handle);
    return 0;
}

static int i2c_gr5xxx_resume(const struct device *dev)
{
    ACQUIRE_PHANDLE();
    ACQUIRE_CONFIG();

    hal_i2c_resume_reg(p_handle);
    hal_nvic_clear_pending_irq(config->irqn);
    hal_nvic_enable_irq(config->irqn);

    return 0;
}

static int i2c_gr5xxx_pm_action(const struct device *dev, enum pm_device_action action)
{
    switch (action)
    {
    case PM_DEVICE_ACTION_SUSPEND:
        return i2c_gr5xxx_suspend(dev);

    case PM_DEVICE_ACTION_RESUME:
        return i2c_gr5xxx_resume(dev);

    default:
        return -ENOTSUP;
    }
}

#endif // CONFIG_PM_DEVICE

// Generate i2c_gr5xxx_config_t instance and i2c_gr5xxx_data_t instance from devicertree
#define GR5XXX_I2C_INIT(index)                                                         \
    /* Define PINCTRL instance */                                                      \
    PINCTRL_DT_INST_DEFINE(index);                                                     \
    /* Impement i2c_gr5xxx_irq_config_func */                                          \
    static void i2c_gr5xxx_irq_config_func_##index(void)                               \
    {                                                                                  \
        IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority),                 \
                    i2c_gr5xxx_isr, DEVICE_DT_INST_GET(index), 0);                     \
        irq_enable(DT_INST_IRQN(index));                                               \
    }                                                                                  \
    /* Declare device config */                                                        \
    static i2c_gr5xxx_config_t i2c_cfg_##index = {                                     \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                 \
        .irqn = DT_INST_IRQN(index),                                                   \
        .dt_clock_frequency = DT_INST_PROP_OR(index, clock_frequency, I2C_SPEED_100K), \
        .irq_config_func = i2c_gr5xxx_irq_config_func_##index,                         \
    };                                                                                 \
    /* Declare device data */                                                          \
    static i2c_gr5xxx_data_t i2c_data_##index = {                                      \
        .handle = {                                                                    \
            .p_instance = (i2c_regs_t *)(DT_INST_REG_ADDR(index)),                     \
            .init = {                                                                  \
                .speed = DT_INST_PROP_OR(index, clock_frequency, I2C_SPEED_100K),      \
                .own_address = 0,                                                      \
                .addressing_mode = I2C_ADDRESSINGMODE_7BIT,                            \
                .general_call_mode = I2C_GENERALCALL_DISABLE,                          \
                .tx_hold_time = 0,                                                     \
                .rx_hold_time = 0,                                                     \
            },                                                                         \
        },                                                                             \
        .sync = Z_SEM_INITIALIZER(UTIL_CAT(i2c_data_, index).sync, 0, 1),              \
        .lock = Z_SEM_INITIALIZER(UTIL_CAT(i2c_data_, index).lock, 1, 1),              \
    };                                                                                 \
    /* Define PM Device */                                                             \
    PM_DEVICE_DT_INST_DEFINE(index, i2c_gr5xxx_pm_action);                             \
    /* Define device init */                                                           \
    DEVICE_DT_INST_DEFINE(                                                             \
        index,                                                                         \
        i2c_gr5xxx_init,                                                               \
        PM_DEVICE_DT_INST_GET(index),                                                  \
        &i2c_data_##index,                                                             \
        &i2c_cfg_##index,                                                              \
        PRE_KERNEL_1,                                                                  \
        CONFIG_I2C_INIT_PRIORITY,                                                      \
        &i2c_gr5xxx_driver_api);

// Generate init request for every used I2C instance in devicetre
DT_INST_FOREACH_STATUS_OKAY(GR5XXX_I2C_INIT);
