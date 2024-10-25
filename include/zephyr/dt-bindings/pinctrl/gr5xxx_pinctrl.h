/*
 * Copyright (c) 2024, Goodix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __GR5XXX_PINCTRL_H__
#define __GR5XXX_PINCTRL_H__

#define GR5XXX_PINCTRL_PORT_POS (0)        // port
#define GR5XXX_PINCTRL_PIN_POS (4)         // pin
#define GR5XXX_PINCTRL_PULL_POS (10)       // pull
#define GR5XXX_PINCTRL_MUX_POS (12)        // mux
#define GR5XXX_PINCTRL_IS_ANALOG_POS (20)  // is_analog
#define GR5XXX_PINCTRL_IS_SCHMITT_POS (21) // is_schmitt
#define GR5XXX_PINCTRL_STRENGTH_POS (22)   // strength

#define GR5XXX_PINCTRL_PORT_MSK (0xF)       // 4 bits for port
#define GR5XXX_PINCTRL_PIN_MSK (0x3F)       // 6 bits for pin
#define GR5XXX_PINCTRL_PULL_MSK (0x3)       // 2 bits for pull
#define GR5XXX_PINCTRL_MUX_MSK (0xFF)       // 8 bits for mux
#define GR5XXX_PINCTRL_IS_ANALOG_MSK (0x1)  // 1 bit for is_analog
#define GR5XXX_PINCTRL_IS_SCHMITT_MSK (0x1) // 1 bit for is_schmitt
#define GR5XXX_PINCTRL_STRENGTH_MSK (0x3)   // 2 bits for strength

#define GR5XXX_PORT_GPIOA (0)
#define GR5XXX_PORT_GPIOB (1)
#define GR5XXX_PORT_GPIOC (2)
#define GR5XXX_PORT_AONIO (3)
#define GR5XXX_PORT_MSIO (4)

#define SET_FIELD(__FIELD__, __VAL__) (((__VAL__) & (GR5XXX_PINCTRL_##__FIELD__##_MSK)) << GR5XXX_PINCTRL_##__FIELD__##_POS)
#define GET_FIELD(__FIELD__, __VAL__) (((__VAL__) >> GR5XXX_PINCTRL_##__FIELD__##_POS) & GR5XXX_PINCTRL_##__FIELD__##_MSK)

#define GR5XXX_PINMUX(port, pin, mux, is_analog) (SET_FIELD(PORT, port) | SET_FIELD(PIN, pin) | SET_FIELD(MUX, mux) | SET_FIELD(IS_ANALOG, is_analog))

#endif // __GR5XXX_PINCTRL_H__