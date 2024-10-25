# Copyright (c) 2024 Goodix
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_TRNG_GR5XXX trng_gr55xx.c)