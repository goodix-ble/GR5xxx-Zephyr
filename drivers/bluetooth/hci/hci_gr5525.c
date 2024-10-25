/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/bluetooth.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/irq.h>


#include "ble.h"
#include "ble_event.h"
#include "ble_cfg.h"
#include "gr55xx.h"
#include "gr55xx_hal.h"
#include "gr55xx_pwr.h"
#include "gr55xx_sys.h"
#include "gr_soc.h"

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hci_gr5525);

#define DT_DRV_COMPAT goodix_bt_hci

#define HCI_CACHE_BUF_LEN      1024

STACK_HEAP_INIT(s_heaps_table);


/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern void ble_sdk_schedule(void);
extern void sys_prevent_sleep_clear(uint16_t prv_slp_bit);
extern void BLE_IRQHandler(void);
extern void BLESLP_IRQHandler(void);
extern void BLE_PWR_ON_IRQHandler(void);
extern void BLE_SDK_Handler(void);
extern void ble_sdk_patch_env_init(void);
extern void ble_feature_init(void);
extern void ble_task_tab_init(void);
extern void ble_stack_controller_init(stack_heaps_table_t *p_heaps_table);
extern sdk_err_t ble_hci_init(ble_hci_rx_channel_t *p_rx_channel, ble_hci_host_recv_cb_t host_recv_cb);

struct hci_data {
    bt_hci_recv_t recv;
};

static uint8_t               s_hci_cache_buffer[HCI_CACHE_BUF_LEN];
static ble_hci_rx_channel_t  s_hci_rx_channel =
{
    .p_channel  = s_hci_cache_buffer,
    .cache_size = HCI_CACHE_BUF_LEN,
};

static struct net_buf *get_rx(const uint8_t *buf)
{
    bool discardable = false;
    k_timeout_t timeout = K_FOREVER;
    switch (buf[0])
    {
    case BT_HCI_H4_EVT:
        if (buf[1] == BT_HCI_EVT_LE_META_EVENT &&
           (buf[3] == BT_HCI_EVT_LE_ADVERTISING_REPORT ||
            buf[3] == BT_HCI_EVT_LE_EXT_ADVERTISING_REPORT))
        {
            discardable = true;
            timeout = K_NO_WAIT;
        }
        return bt_buf_get_evt(buf[1], discardable, timeout);

    case BT_HCI_H4_ACL:
        return bt_buf_get_rx(BT_BUF_ACL_IN, K_FOREVER);

    case BT_HCI_H4_SCO:
        if (IS_ENABLED(CONFIG_BT_ISO))
        {
            return bt_buf_get_rx(BT_BUF_ISO_IN, K_FOREVER);
        }
        __fallthrough;

    default:
        return NULL;
    }

    return NULL;
}

static void host_recv_data_process(uint8_t *data, uint16_t len)
{
    const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
    struct hci_data *hci = dev->data;

    pwr_mgmt_ble_wakeup();
    struct net_buf *buf = get_rx(data);
    if(buf)
    {
        net_buf_add_mem(buf, &data[1], len - 1);
        hci->recv(dev, buf);
        NVIC_SetPendingIRQ(BLE_SDK_IRQn);
        NVIC_EnableIRQ(BLE_SDK_IRQn);
    }
}

void zephyr_ble_sdk_irq_handler(void *args)
{
    NVIC_ClearPendingIRQ(BLE_SDK_IRQn);
    pwr_mgmt_ble_wakeup();
    ble_sdk_schedule();
    /* Clear RW_PLF_DEEP_SLEEP_DISABLED & RW_TL_RX_ONGOING. */
    sys_prevent_sleep_clear(0x0104);
}

static int vuart_open(const struct device *dev, bt_hci_recv_t recv)
{
    unsigned int key;

    key = irq_lock();

    struct hci_data *data = dev->data;
    data->recv = recv;

    ble_sdk_patch_env_init();
    ble_feature_init();
    ble_task_tab_init();

    ble_hci_init(&s_hci_rx_channel, host_recv_data_process);
    ble_stack_controller_init(&s_heaps_table);

    IRQ_DIRECT_CONNECT(BLE_IRQn, (2 - 1), BLE_IRQHandler, 0);
    IRQ_DIRECT_CONNECT(BLESLP_IRQn, (2 - 1), BLESLP_IRQHandler, 0);
    IRQ_DIRECT_CONNECT(BLE_PWR_ON_IRQn, (2 - 1), BLE_PWR_ON_IRQHandler, 0);
    IRQ_CONNECT(BLE_SDK_IRQn, IRQ_PRIO_LOWEST, zephyr_ble_sdk_irq_handler, NULL, 0);
    irq_enable(BLE_IRQn);
    irq_enable(BLESLP_IRQn);
    irq_enable(BLE_SDK_IRQn);

    irq_unlock(key);
    return 0;
}

static int vuart_send(const struct device *dev, struct net_buf *buf)
{
    LOG_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf), buf->len);
    switch (bt_buf_get_type(buf))
    {
    case BT_BUF_ACL_OUT:
        net_buf_push_u8(buf, BT_HCI_H4_ACL);
        break;

    case BT_BUF_CMD:
        net_buf_push_u8(buf, BT_HCI_H4_CMD);
        break;

    case BT_BUF_ISO_OUT:
        if (IS_ENABLED(CONFIG_BT_ISO))
        {
            net_buf_push_u8(buf, BT_HCI_H4_ISO);
            break;
        }
        __fallthrough;

    default:
        LOG_ERR("Unknown type %u", bt_buf_get_type(buf));
        return -EINVAL;
    }

    LOG_HEXDUMP_DBG(buf->data, buf->len, "Final HCI buffer:");
    while(ble_hci_rx_channel_surplus_space_get() <= buf->len);
    ble_hci_host_packet_send(buf->data, buf->len);
    NVIC_SetPendingIRQ(BLE_SDK_IRQn);
    NVIC_EnableIRQ(BLE_SDK_IRQn);
    net_buf_unref(buf);
    return 0;
}

static const struct bt_hci_driver_api drv = {
    .open       = vuart_open,
    .send       = vuart_send,
};


#define HCI_DEVICE_INIT(inst) \
    static struct hci_data hci_data_##inst = { \
    }; \
    DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &hci_data_##inst, NULL, \
                POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &drv)

/* Only one instance supported */
HCI_DEVICE_INIT(0)

