/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * INCLUDE FILES
 */

#include "soc.h"
#include "platform_sdk.h"
#include "pmu_calibration.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/linker/linker-defs.h>

#define LOCAL_BASEPRI_DISABLE()                     \
	do                                              \
	{                                               \
		uint32_t __basepri_value = __get_BASEPRI(); \
		__set_BASEPRI(0);

#define LOCAL_BASEPRI_ENABLE()      \
	__set_BASEPRI(__basepri_value); \
	}                               \
	while (0)                       \
		;

typedef void (*FuncVector_t)(void);

static volatile uint32_t s_zephyr_msp_addr = 0;
__ALIGNED(0x400) FuncVector_t FuncVector_table[MAX_NUMS_IRQn + NVIC_USER_IRQ_OFFSET] = {};
uint32_t SystemCoreClock = CLK_64M;
K_KERNEL_STACK_ARRAY_DECLARE(z_interrupt_stacks, CONFIG_MP_MAX_NUM_CPUS, CONFIG_ISR_STACK_SIZE);

// Copied from zephyr/arch/arm/core/cortex_m/prep_c.c
#if defined(CONFIG_CPU_HAS_FPU)
static inline void z_arm_floating_point_init(void)
{
	/*
	 * Upon reset, the Co-Processor Access Control Register is, normally,
	 * 0x00000000. However, it might be left un-cleared by firmware running
	 * before Zephyr boot.
	 */
	SCB->CPACR &= (~(CPACR_CP10_Msk | CPACR_CP11_Msk));

#if defined(CONFIG_FPU)
	/*
	 * Enable CP10 and CP11 Co-Processors to enable access to floating
	 * point registers.
	 */
#if defined(CONFIG_USERSPACE)
	/* Full access */
	SCB->CPACR |= CPACR_CP10_FULL_ACCESS | CPACR_CP11_FULL_ACCESS;
#else
	/* Privileged access only */
	SCB->CPACR |= CPACR_CP10_PRIV_ACCESS | CPACR_CP11_PRIV_ACCESS;
#endif /* CONFIG_USERSPACE */
	/*
	 * Upon reset, the FPU Context Control Register is 0xC0000000
	 * (both Automatic and Lazy state preservation is enabled).
	 */
#if defined(CONFIG_MULTITHREADING) && !defined(CONFIG_FPU_SHARING)
	/* Unshared FP registers (multithreading) mode. We disable the
	 * automatic stacking of FP registers (automatic setting of
	 * FPCA bit in the CONTROL register), upon exception entries,
	 * as the FP registers are to be used by a single context (and
	 * the use of FP registers in ISRs is not supported). This
	 * configuration improves interrupt latency and decreases the
	 * stack memory requirement for the (single) thread that makes
	 * use of the FP co-processor.
	 */
	FPU->FPCCR &= (~(FPU_FPCCR_ASPEN_Msk | FPU_FPCCR_LSPEN_Msk));
#else
	/*
	 * FP register sharing (multithreading) mode or single-threading mode.
	 *
	 * Enable both automatic and lazy state preservation of the FP context.
	 * The FPCA bit of the CONTROL register will be automatically set, if
	 * the thread uses the floating point registers. Because of lazy state
	 * preservation the volatile FP registers will not be stacked upon
	 * exception entry, however, the required area in the stack frame will
	 * be reserved for them. This configuration improves interrupt latency.
	 * The registers will eventually be stacked when the thread is swapped
	 * out during context-switch or if an ISR attempts to execute floating
	 * point instructions.
	 */
	FPU->FPCCR = FPU_FPCCR_ASPEN_Msk | FPU_FPCCR_LSPEN_Msk;
#endif /* CONFIG_FPU_SHARING */

	/* Make the side-effects of modifying the FPCCR be realized
	 * immediately.
	 */
	__DSB();
	__ISB();

	/* Initialize the Floating Point Status and Control Register. */
#if defined(CONFIG_ARMV8_1_M_MAINLINE)
	/*
	 * For ARMv8.1-M with FPU, the FPSCR[18:16] LTPSIZE field must be set
	 * to 0b100 for "Tail predication not applied" as it's reset value
	 */
	__set_FPSCR(4 << FPU_FPDSCR_LTPSIZE_Pos);
#else
	__set_FPSCR(0);
#endif

	/*
	 * Note:
	 * The use of the FP register bank is enabled, however the FP context
	 * will be activated (FPCA bit on the CONTROL register) in the presence
	 * of floating point instructions.
	 */

#endif /* CONFIG_FPU */

	/*
	 * Upon reset, the CONTROL.FPCA bit is, normally, cleared. However,
	 * it might be left un-cleared by firmware running before Zephyr boot.
	 * We must clear this bit to prevent errors in exception unstacking.
	 *
	 * Note:
	 * In Sharing FP Registers mode CONTROL.FPCA is cleared before switching
	 * to main, so it may be skipped here (saving few boot cycles).
	 *
	 * If CONFIG_INIT_ARCH_HW_AT_BOOT is set, CONTROL is cleared at reset.
	 */
#if (!defined(CONFIG_FPU) || !defined(CONFIG_FPU_SHARING)) && \
	(!defined(CONFIG_INIT_ARCH_HW_AT_BOOT))

	__set_CONTROL(__get_CONTROL() & (~(CONTROL_FPCA_Msk)));
#endif
}
#endif /* CONFIG_CPU_HAS_FPU */

static int goodix_gr55xx_init(void);
static void patch_init(void);
static void vector_table_init(void);
static void vector_table_sync(void);
static void vector_table_register(IRQn_Type irqn, FuncVector_t func);

void z_arm_platform_init(void)
{
	if (COLD_BOOT != pwr_mgmt_get_wakeup_flag())
	{
		__set_MSP(s_zephyr_msp_addr);
		vector_table_init();
#if defined(CONFIG_CPU_HAS_FPU)
		z_arm_floating_point_init();
#endif // defined(CONFIG_CPU_HAS_FPU)
		warm_boot_first();
	}
}

static int goodix_gr55xx_init(void)
{
	if (COLD_BOOT == pwr_mgmt_get_wakeup_flag())
	{
		LOCAL_BASEPRI_DISABLE();

		vector_table_sync();
		vector_table_init();

		extern void CLK_CALIB_IRQHandler(void);
		extern void CPLL_DRIFT_IRQHandler(void);
		vector_table_register(CLK_CALIB_IRQn, CLK_CALIB_IRQHandler);
		vector_table_register(CPLL_DRIFT_IRQn, CPLL_DRIFT_IRQHandler);

		patch_init();
		first_class_task();
		second_class_task();

		s_zephyr_msp_addr = (uint32_t)(K_KERNEL_STACK_BUFFER(z_interrupt_stacks[0])) + K_KERNEL_STACK_SIZEOF(z_interrupt_stacks[0]);

#ifdef CONFIG_PM
		// Unregister warm_boot_second(). Call warm_boot_second in pm_state_exit_post_ops
		// No idea why but
		extern void pwr_mgmt_warm_boot_reg(void (*func)(void));
		pwr_mgmt_warm_boot_reg(NULL);
		pwr_mgmt_mode_set(PMR_MGMT_SLEEP_MODE);
#endif // CONFIG_PM

		LOCAL_BASEPRI_ENABLE();
	}
	return 0;
}

static int goodix_gr55xx_pmu_calibration_init(void)
{
	system_pmu_calibration_init(30 * 1000);
	return 0;
}

static void patch_init(void)
{
	gr5xx_fpb_init(FPB_MODE_PATCH_AND_DEBUG);
}

static void vector_table_init()
{
	__DMB();
	FuncVector_table[0] = *(FuncVector_t *)(SCB->VTOR);
	SCB->VTOR = (uint32_t)FuncVector_table;
	__DSB();
}

static void vector_table_sync()
{
	const uint32_t vector_size = ((uint32_t)(_vector_end) - (uint32_t)(_vector_start));
	const uint32_t vector_num = MIN((vector_size + 3) >> 2, CONFIG_NUM_IRQS);
	for (uint32_t i = 0; i < vector_num; i++)
	{
		FuncVector_table[i] = ((FuncVector_t *)_vector_start)[i];
	}
}

static inline void vector_table_register(IRQn_Type irqn, FuncVector_t func)
{
	// Assume that input irqn is valid
	FuncVector_table[irqn + NVIC_USER_IRQ_OFFSET] = func;
}

SYS_INIT(goodix_gr55xx_init, PRE_KERNEL_1, 0);
SYS_INIT(goodix_gr55xx_pmu_calibration_init, POST_KERNEL, 0);
