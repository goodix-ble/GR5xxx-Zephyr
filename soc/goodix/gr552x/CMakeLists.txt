# Copyright (c) 2024 Goodix
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_ARM)
  set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "SoC Linker script")
endif()

zephyr_linker_sources(RWDATA rwdata.ld)

zephyr_sources(
  soc.c
  soc_${CONFIG_SOC}.c
  pmu_calibration.c
)

zephyr_sources_ifdef(CONFIG_PM power.c)