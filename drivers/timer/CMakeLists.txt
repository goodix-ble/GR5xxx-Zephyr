# Copyright (c) 2024 Goodix
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_GR552X_RTC_TIMER gr552x_rtc_timer.c)