/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

. = ALIGN(32);
*(RAM_CODE)
*(TINY_RAM_SPACE)
. = ALIGN(128);
*(FPB)
*(SVC_TABLE)