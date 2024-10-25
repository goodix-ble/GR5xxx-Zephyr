/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT goodix_gr5xxx_trng

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/drivers/entropy.h>
#include <string.h>

#include "grx_hal.h"

#define LOG_LEVEL CONFIG_FLASH_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(trng_gr55xx);

static rng_handle_t s_rng_handle =
{
    .p_instance     = RNG,
    .init.seed_mode = RNG_SEED_FR0_S0,
    .init.lfsr_mode = RNG_LFSR_MODE_59BIT,
    .init.out_mode  = RNG_OUTPUT_FR0_S0,
    .init.post_mode = RNG_POST_PRO_NEUMANN,
};

static void trng_gr55xx_init(void)
{
    hal_rng_init(&s_rng_handle);
}
static void trng_gr55xx_deinit(void)
{
    hal_rng_deinit(&s_rng_handle);
}

static int entropy_gr5xx_get_entropy(const struct device *dev, uint8_t *buf, uint16_t len)
{
    trng_gr55xx_init();

    uint8_t *buf_bytes = buf;

    while (len > 0) {
        uint32_t word = 0;
        hal_rng_generate_random_number(&s_rng_handle, NULL, &word);

        uint32_t to_copy = MIN(sizeof(word), len);

        memcpy(buf_bytes, &word, to_copy);
        buf_bytes += to_copy;
        len -= to_copy;
    }

    trng_gr55xx_deinit();

    return 0;
}

static int entropy_gr5xx_init(const struct device *dev)
{
    return 0;
}

static const struct entropy_driver_api trng_gr55xx_api = {
    .get_entropy = entropy_gr5xx_get_entropy,
};

DEVICE_DT_INST_DEFINE(0, entropy_gr5xx_init, NULL, NULL, NULL,
            POST_KERNEL, CONFIG_ENTROPY_INIT_PRIORITY,
            &trng_gr55xx_api);
