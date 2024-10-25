/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __GR55XX_HAL_I2C_EX_H__
#define __GR55XX_HAL_I2C_EX_H__

#include "gr55xx_hal_i2c.h"

hal_status_t hal_i2c_master_transmit_it_patch(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size, uint32_t extra_cmd);
hal_status_t hal_i2c_master_receive_it_patch(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size, uint32_t extra_cmd);

#endif // __GR55XX_HAL_I2C_EX_H__