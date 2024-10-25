/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "gr55xx_hal.h"
#include "gr55xx_hal_i2c_patch.h"
#include <stdbool.h>

#define I2C_TIMEOUT_ADDR    (10000U)       /*!< 10 s  */
#define I2C_TIMEOUT_BUSY    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_RXNE    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_STOP    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_TFNF    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_FLAG    (25U)          /*!< 25 ms */

#define I2C_TXFIFO_SIZE                    (32U)
#define I2C_RXFIFO_SIZE                    (32U)

/* Private define for @ref PreviousState usage */
#define I2C_STATE_MSK ((uint32_t)((HAL_I2C_STATE_BUSY_TX | HAL_I2C_STATE_BUSY_RX) & (~((uint32_t)HAL_I2C_STATE_READY)))) /*!< Mask State define, keep only RX and TX bits            */
#define I2C_STATE_NONE ((uint32_t)(HAL_I2C_MODE_NONE))                                                                   /*!< Default Value                                          */
#define I2C_STATE_MASTER_BUSY_TX ((uint32_t)((HAL_I2C_STATE_BUSY_TX & I2C_STATE_MSK) | HAL_I2C_MODE_MASTER))             /*!< Master Busy TX, combinaison of State LSB and Mode enum */
#define I2C_STATE_MASTER_BUSY_RX ((uint32_t)((HAL_I2C_STATE_BUSY_RX & I2C_STATE_MSK) | HAL_I2C_MODE_MASTER))             /*!< Master Busy RX, combinaison of State LSB and Mode enum */
#define I2C_STATE_SLAVE_BUSY_TX ((uint32_t)((HAL_I2C_STATE_BUSY_TX & I2C_STATE_MSK) | HAL_I2C_MODE_SLAVE))               /*!< Slave Busy TX, combinaison of State LSB and Mode enum  */
#define I2C_STATE_SLAVE_BUSY_RX ((uint32_t)((HAL_I2C_STATE_BUSY_RX & I2C_STATE_MSK) | HAL_I2C_MODE_SLAVE))               /*!< Slave Busy RX, combinaison of State LSB and Mode enum  */

/* Private define to centralize the enable/disable of Interrupts */
#define I2C_XFER_ERROR_IT (LL_I2C_INTR_MASK_TX_ABRT | LL_I2C_INTR_MASK_SCL_STUCK_AT_LOW)

#define I2C_MST_XFER_TX_IT (I2C_XFER_ERROR_IT |         \
                            LL_I2C_INTR_MASK_TX_EMPTY | \
                            LL_I2C_INTR_MASK_STOP_DET)

#define I2C_MST_XFER_RX_IT (I2C_XFER_ERROR_IT |         \
                            LL_I2C_INTR_MASK_TX_EMPTY | \
                            LL_I2C_INTR_MASK_RX_FULL |  \
                            LL_I2C_INTR_MASK_RX_OVER |  \
                            LL_I2C_INTR_MASK_STOP_DET)

#define I2C_SLV_XFER_TX_IT (I2C_XFER_ERROR_IT |         \
                            LL_I2C_INTR_MASK_TX_EMPTY | \
                            LL_I2C_INTR_MASK_STOP_DET)

#define I2C_SLV_XFER_RX_IT (I2C_XFER_ERROR_IT |        \
                            LL_I2C_INTR_MASK_RX_FULL | \
                            LL_I2C_INTR_MASK_RX_OVER | \
                            LL_I2C_INTR_MASK_STOP_DET)

#define I2C_XFER_LISTEN_IT (I2C_XFER_ERROR_IT |         \
                            LL_I2C_INTR_MASK_STOP_DET | \
                            LL_I2C_INTR_MASK_RD_REQ)

#define I2C_XFER_CPLT_IT (LL_I2C_INTR_MASK_STOP_DET)

/* Private define to Abort Source */
#define I2C_TX_ABRT_NOACK (LL_I2C_ABRT_GCALL_NOACK |   \
                           LL_I2C_ABRT_TXDATA_NOACK |  \
                           LL_I2C_ABRT_10ADDR2_NOACK | \
                           LL_I2C_ABRT_10ADDR1_NOACK | \
                           LL_I2C_ABRT_7B_ADDR_NOACK)

extern void i2c_it_master_cplt(i2c_handle_t *p_i2c);
extern void i2c_it_slave_cplt(i2c_handle_t *p_i2c);
extern void i2c_it_error(i2c_handle_t *p_i2c, uint32_t error_code);
extern void i2c_set_device_state(i2c_handle_t *p_i2c, hal_i2c_state_t state);
extern void i2c_set_renew_flag(i2c_handle_t *p_i2c);
extern void i2c_resume_before_using(i2c_handle_t *p_i2c, bool init_flag);

__STATIC_INLINE hal_status_t i2c_master_transfer_config(i2c_handle_t *p_i2c, uint16_t dev_address)
{
    /* Enable Master Mode and Set Slave Address */
    ll_i2c_disable(p_i2c->p_instance);
    ll_i2c_enable_master_mode(p_i2c->p_instance);
    ll_i2c_set_slave_address(p_i2c->p_instance, dev_address);
    ll_i2c_enable(p_i2c->p_instance);

    /* CLear all interrupt */
    ll_i2c_clear_flag_intr(p_i2c->p_instance);

    i2c_set_renew_flag(p_i2c);

    return HAL_OK;
}

__WEAK void hal_i2c_master_transfer_cplt_callback(i2c_handle_t *p_i2c)
{
    UNUSED(p_i2c);
}

static void i2c_transfer_done(i2c_handle_t *p_i2c)
{
    i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
    p_i2c->mode = HAL_I2C_MODE_NONE;
    hal_i2c_master_transfer_cplt_callback(p_i2c);
}

static hal_status_t i2c_master_isr_it_patch(i2c_handle_t *p_i2c, uint32_t it_source, uint32_t abort_sources)
{
    uint32_t curxfercnt = 0U;
    uint32_t cmd = LL_I2C_CMD_MST_WRITE;

    if (RESET != (it_source & LL_I2C_INTR_STAT_TX_ABRT))
    {
        if (RESET != (abort_sources & I2C_TX_ABRT_NOACK))
        {
            /* Set corresponding Error Code */
            p_i2c->error_code |= HAL_I2C_ERROR_NOACK;
        }
    }
    else if (RESET != (it_source & LL_I2C_INTR_STAT_RX_FULL))
    {
        /* Get data count in RX FIFO */
        curxfercnt = ll_i2c_get_rx_fifo_level(p_i2c->p_instance);

        /* Read data from RX FIFO */
        while ((curxfercnt--) && (0U != p_i2c->xfer_count))
        {
            (*p_i2c->p_buffer++) = ll_i2c_receive_data8(p_i2c->p_instance);
            p_i2c->xfer_count--;
        }

        if (p_i2c->xfer_count < (ll_i2c_get_rx_fifo_threshold(p_i2c->p_instance) + 1))
        {
            ll_i2c_set_rx_fifo_threshold(p_i2c->p_instance, LL_I2C_RX_FIFO_TH_CHAR_1);
        }

        if (0U == p_i2c->xfer_count)
        {
            ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_RX_FULL);
        }
    }
    else if (RESET != (it_source & LL_I2C_INTR_STAT_TX_EMPTY))
    {
        /* Get free data count in TX FIFO */
        curxfercnt = I2C_TXFIFO_SIZE - ll_i2c_get_tx_fifo_level(p_i2c->p_instance);

        /* Master transmit process */
        if (HAL_I2C_STATE_BUSY_TX == p_i2c->state)
        {
            /* Write data into TX FIFO */
            while ((curxfercnt--) && (0U != p_i2c->xfer_count))
            {
                if (1U == p_i2c->xfer_count)
                {
                    cmd |= p_i2c->xfer_options;
                }

                ll_i2c_transmit_data8(p_i2c->p_instance, *p_i2c->p_buffer++, cmd);
                p_i2c->xfer_count--;
            }

            if (0U == p_i2c->xfer_count)
            {
                ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_TX_EMPTY);
            }
        }
        /* Master receive process */
        else if (HAL_I2C_STATE_BUSY_RX == p_i2c->state)
        {
            cmd = LL_I2C_CMD_MST_READ;

            while ((curxfercnt--) && (0U != p_i2c->master_ack_count))
            {
                if (1U == p_i2c->master_ack_count)
                {
                    cmd |= p_i2c->xfer_options;
                }

                ll_i2c_transmit_data8(p_i2c->p_instance, 0U, cmd);
                p_i2c->master_ack_count--;
            }

            if (0U == p_i2c->master_ack_count)
            {
                ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_TX_EMPTY);
            }
        }
    }

    if (p_i2c->xfer_count == 0)
    {
        i2c_transfer_done(p_i2c);
    }

    if (RESET != (it_source & LL_I2C_INTR_STAT_STOP_DET))
    {
        /* Call I2C Master complete process */
        i2c_it_master_cplt(p_i2c);
    }

    return HAL_OK;
}


hal_status_t hal_i2c_master_transmit_it_patch(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size, uint32_t extra_cmd)
{
    /* Check the I2C handle allocation */
    if (NULL == p_i2c)
    {
        return HAL_ERROR;
    }

    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        /* Process Locked */
        __HAL_LOCK(p_i2c);

        i2c_resume_before_using(p_i2c, false);

        i2c_set_renew_flag(p_i2c);

        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_TX);
        p_i2c->mode          = HAL_I2C_MODE_MASTER;
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer = p_data;
        p_i2c->xfer_size = size;
        p_i2c->xfer_count = size;
        p_i2c->xfer_options = extra_cmd;
        p_i2c->xfer_isr = i2c_master_isr_it_patch;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);

        /* Enable the selected I2C peripheral */
        ll_i2c_enable(p_i2c->p_instance);

        extern hal_status_t i2c_master_start_transmit_it(i2c_handle_t *p_i2c);
        return i2c_master_start_transmit_it(p_i2c);
    }
    else
    {
        return HAL_BUSY;
    }
}

hal_status_t hal_i2c_master_receive_it_patch(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size, uint32_t extra_cmd)
{
    /* Check the I2C handle allocation */
    if (NULL == p_i2c)
    {
        return HAL_ERROR;
    }

    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        /* Process Locked */
        __HAL_LOCK(p_i2c);

        i2c_resume_before_using(p_i2c, false);

        i2c_set_renew_flag(p_i2c);

        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_RX);
        p_i2c->mode = HAL_I2C_MODE_MASTER;
        p_i2c->error_code = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer = p_data;
        p_i2c->xfer_size = size;
        p_i2c->xfer_count = size;
        p_i2c->master_ack_count = size;
        p_i2c->xfer_options = extra_cmd;
        p_i2c->xfer_isr = i2c_master_isr_it_patch;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);

        /* Enable the selected I2C peripheral */
        ll_i2c_enable(p_i2c->p_instance);

        extern hal_status_t i2c_master_start_receive_it(i2c_handle_t * p_i2c);
        return i2c_master_start_receive_it(p_i2c);
    }
    else
    {
        return HAL_BUSY;
    }
}
