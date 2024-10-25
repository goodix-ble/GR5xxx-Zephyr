/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __PINCTRL_SOC_H__
#define __PINCTRL_SOC_H__

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

#include <zephyr/dt-bindings/pinctrl/gr5xxx_pinctrl.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t port : 4;
        uint32_t pin : 6;
        uint32_t pull : 2;
        uint32_t mux : 8;
        uint32_t is_analog : 1;
        uint32_t is_schmitt : 1;
        uint32_t strength : 2;
    } fields;
} pinctrl_soc_pin_t;

#define _BIN(high, low) ((high << 1) | (low))

#if 0
#define DT_PINCTRL_NODE_ID DT_PROP_BY_IDX(node_id, prop, idx)
#define DT_PINCTRL_PINMUX(node_id, prop, idx) DT_PROP(DT_PROP_BY_IDX(node_id, prop, idx), pinmux)

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                                                                       \
{.fields = {                                                                                                                               \
     .port = GET_FIELD(PORT, DT_PINCTRL_PINMUX(node_id, prop, idx)),                                                                       \
     .pin = GET_FIELD(PIN, DT_PINCTRL_PINMUX(node_id, prop, idx)),                                                                         \
     .pull = _BIN(DT_PROP(DT_PROP_BY_IDX(node_id, prop, idx), bias_pull_up), DT_PROP(DT_PROP_BY_IDX(node_id, prop, idx), bias_pull_down)), \
     .mux = GET_FIELD(MUX, DT_PINCTRL_PINMUX(node_id, prop, idx)),                                                                         \
     .is_analog = GET_FIELD(IS_ANALOG, DT_PINCTRL_PINMUX(node_id, prop, idx)),                                                             \
     .is_schmitt = DT_PROP(DT_PROP_BY_IDX(node_id, prop, idx), input_schmitt_enable),                                                      \
     .strength = DT_ENUM_IDX(DT_PROP_BY_IDX(node_id, prop, idx), drive_strength),                                                          \
 }},
#endif // 0

#define Z_PINCTRL_STATE_PIN_INIT_IMPL(node_id)                                       \
{.fields = {                                                                         \
     .port = DT_ENUM_IDX(node_id, port),                                             \
     .pin = DT_PROP(node_id, pin),                                                   \
     .pull = _BIN(DT_PROP(node_id, bias_pull_down), DT_PROP(node_id, bias_pull_up)), \
     .mux = DT_PROP(node_id, mux),                                                   \
     .is_analog = DT_PROP(node_id, is_analog),                                       \
     .is_schmitt = DT_PROP(node_id, input_schmitt_enable),                           \
     .strength = DT_ENUM_IDX(node_id, drive_strength),                               \
 }},

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx) Z_PINCTRL_STATE_PIN_INIT_IMPL(DT_PROP_BY_IDX(node_id, prop, idx))

// #define Z_PINCTRL_STATE_PINS_INIT(node_id, prop) {DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop), DT_FOREACH_PROP_ELEM, pinmux, Z_PINCTRL_STATE_PIN_INIT)}
// #define Z_PINCTRL_STATE_PINS_INIT(node_id, prop) {DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)}
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop) {DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __PINCTRL_SOC_H__