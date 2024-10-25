/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

. = ALIGN(0x200);
.app_info = .;
KEEP(*(.app_info))