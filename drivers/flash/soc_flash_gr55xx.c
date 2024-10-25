/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "gr55xx_hal_exflash.h"
#include "hal_flash.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/drivers/flash.h>
#include <string.h>


#define LOG_LEVEL CONFIG_FLASH_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(soc_flash_gr55xx);

#if DT_NODE_HAS_STATUS(DT_INST(0, goodix_gr5xxx_flash_controller), okay)
#define DT_DRV_COMPAT goodix_gr5xxx_flash_controller
#else
#define DT_DRV_COMPAT FLASH_GOODIX
#endif

#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)

#if defined(CONFIG_MULTITHREADING)
/* semaphore for locking flash resources (tickers) */
static struct k_sem sem_lock;
#define SYNC_INIT() k_sem_init(&sem_lock, 1, 1)
#define SYNC_LOCK() k_sem_take(&sem_lock, K_FOREVER)
#define SYNC_UNLOCK() k_sem_give(&sem_lock)
#else
#define SYNC_INIT()
#define SYNC_LOCK()
#define SYNC_UNLOCK()
#endif

static int flash_gr55xx_read(const struct device *dev, off_t offset,
                void *data,
                size_t len)
{
    int rc = 0;

    if (!len) {
        return 0;
    }

    SYNC_LOCK();
    if(!hal_flash_read((EXFLASH_START_ADDR + offset), data, len))
    {
        rc = -1;
    }
    SYNC_UNLOCK();
    return rc;
}

static int flash_gr55xx_erase(const struct device *dev, off_t offset,
                size_t len)
{
    int rc = 0;

    if (!len) {
        return 0;
    }

    SYNC_LOCK();

    if(true != hal_flash_erase((EXFLASH_START_ADDR + offset), len))
    {
        rc = -1;
    }

    SYNC_UNLOCK();
    return rc;
}

static int flash_gr55xx_write(const struct device *dev, off_t offset,
                const void *data, size_t len)
{
    int rc = 0;

    if (!len) {
        return 0;
    }

    SYNC_LOCK();

    if(!hal_flash_write((EXFLASH_START_ADDR + offset), (uint8_t*)data, len))
    {
        rc = -1;
    }

    SYNC_UNLOCK();
    return rc;
}

static const struct flash_parameters flash_gr55xx_parameters = {
    .write_block_size = 4,
    .erase_value = 0xff,
};

static const struct flash_parameters *
flash_gr55xx_get_parameters(const struct device *dev)
{
    ARG_UNUSED(dev);

    return &flash_gr55xx_parameters;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static struct flash_pages_layout dev_layout;

static void flash_gr55xx_pages_layout(const struct device *dev,
                     const struct flash_pages_layout **layout,
                     size_t *layout_size)
{
    *layout = &dev_layout;
    *layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_driver_api flash_gr55xx_api = {
    .erase = flash_gr55xx_erase,
    .write = flash_gr55xx_write,
    .read = flash_gr55xx_read,
    .get_parameters = flash_gr55xx_get_parameters,
    #if defined(CONFIG_FLASH_PAGE_LAYOUT)
    .page_layout = flash_gr55xx_pages_layout,
    #endif
};

static int goodix_flash_init(const struct device *dev)
{
    SYNC_INIT();

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
    dev_layout.pages_count = 256;
    dev_layout.pages_size = 4096;
#endif

    return 0;
}

DEVICE_DT_INST_DEFINE(0, goodix_flash_init, NULL,
            NULL, NULL,
            POST_KERNEL,CONFIG_FLASH_INIT_PRIORITY,
            &flash_gr55xx_api);
