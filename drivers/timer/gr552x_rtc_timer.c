/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "gr55xx_hal_rtc.h"

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/timer/system_timer.h>

#define COUNTER_MAX 0x00ffffff
#define TIMER_STOPPED 0xff000000

#define CYC_PER_TICK (sys_clock_hw_cycles_per_sec() / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define MAX_TICKS ((COUNTER_MAX / CYC_PER_TICK) - 1)
#define MAX_CYCLES (MAX_TICKS * CYC_PER_TICK)
#if IS_ENABLED(CONFIG_TICKLESS_KERNEL)
#define INITIAL_TIMEOUT (MAX_TICKS)
#else
#define INITIAL_TIMEOUT (CYC_PER_TICK)
#endif // #if IS_ENABLED(CONFIG_TICKLESS_KERNEL)

#define MIN_DELAY MAX(1024, (CYC_PER_TICK / 16))

static rtc_handle_t s_rtc1_handle = {
    .p_instance = RTC1,
    .init = {
        .prescaler_div = RTC_DIV_NONE,
        .overflow_det_state = OPENED,
        .start_value = 0,
    },
};

static uint32_t s_cycle_count = 0;
static uint32_t s_announced_cycles = 0;
static struct k_spinlock s_spinlock = {};

static void gr55xx_rtc1_irq_handler(void *args)
{
    ARG_UNUSED(args);

    hal_rtc_irq_handler(&s_rtc1_handle);
#if IS_ENABLED(CONFIG_TICKLESS_KERNEL)
    s_cycle_count = hal_rtc_get_cur_count(&s_rtc1_handle);
    uint32_t delta = (s_cycle_count - s_announced_cycles) / CYC_PER_TICK;
    s_announced_cycles += delta * CYC_PER_TICK;
    sys_clock_announce(delta);
#else
    sys_clock_announce(1);
#endif // IS_ENABLED(CONFIG_TICKLESS_KERNEL)

    z_arm_int_exit();
}

static int gr552x_rtc_timer_init(void)
{
    hal_rtc_deinit(&s_rtc1_handle);
    hal_rtc_init(&s_rtc1_handle);
    IRQ_DIRECT_CONNECT(RTC1_IRQn, IRQ_PRIO_LOWEST, gr55xx_rtc1_irq_handler, 0);
    hal_rtc_set_tick_and_start(&s_rtc1_handle, ONE_TIME, INITIAL_TIMEOUT - 1);
    return 0;
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
#if IS_ENABLED(CONFIG_TICKLESS_KERNEL)
    if (idle && ticks == K_TICKS_FOREVER)
    {
        hal_rtc_stop_tick(&s_rtc1_handle);
    }
    else
    {
        ticks = (ticks == K_TICKS_FOREVER) ? MAX_TICKS : ticks;
        ticks = CLAMP(ticks - 1, 0, (int32_t)MAX_TICKS);

        K_SPINLOCK(&s_spinlock)
        {
            hal_rtc_stop_tick(&s_rtc1_handle);
            s_cycle_count = hal_rtc_get_cur_count(&s_rtc1_handle);
            uint32_t unannounced = s_cycle_count - s_announced_cycles;
            if ((int32_t)unannounced <= 0)
            {
                NVIC_SetPendingIRQ(RTC1_IRQn);
            }
            else
            {
                uint32_t next_timeout_cycle = ticks * CYC_PER_TICK;
                if (next_timeout_cycle < MIN_DELAY)
                {
                    next_timeout_cycle = MIN_DELAY;
                }
                hal_rtc_set_tick_and_start(&s_rtc1_handle, ONE_TIME, next_timeout_cycle - 1);
            }
        }
    }
#endif // IS_ENABLED(CONFIG_TICKLESS_KERNEL)
}

uint32_t sys_clock_elapsed()
{
#if IS_ENABLED(CONFIG_TICKLESS_KERNEL)
    uint32_t cyc;
    K_SPINLOCK(&s_spinlock)
    {
        cyc = s_cycle_count - s_announced_cycles;
    }
    return cyc / CYC_PER_TICK;
#else
    return 0;
#endif // IS_ENABLED(CONFIG_TICKLESS_KERNEL)
}

uint32_t sys_clock_cycle_get_32()
{
    uint32_t rtc_count;
    K_SPINLOCK(&s_spinlock)
    {
        rtc_count = hal_rtc_get_cur_count(&s_rtc1_handle);
    }
    return rtc_count;
}

SYS_INIT(gr552x_rtc_timer_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);