# Copyright (c) 2024 Goodix
# SPDX-License-Identifier: Apache-2.0

board_runner_args(gprogrammer)
board_runner_args(jlink "--device=CORTEX-M4")

include(${BOARD_DIR}/gprogrammer.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)