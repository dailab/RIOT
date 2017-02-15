/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 * Copyright (C) 2014 Freie Universität Berlin
 * Copyright (C) 2013 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_cc1x0x
 * @{
 *
 * @file
 * @brief       internal declarations for cc1x0x driver
 *
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 */

#ifndef CC1X0X_INTERFACE_H
#define CC1X0X_INTERFACE_H

#include <stdint.h>
#include "cc1x0x.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name cc1x0x raw low-level interface
 * @internal
 * @{
 */
char *cc1x0x_get_marc_state(cc1x0x_t *dev);
char *cc1x0x_state_to_text(uint8_t state);
int cc1x0x_rd_set_mode(cc1x0x_t *dev, int mode);
uint8_t cc1x0x_get_buffer_pos(cc1x0x_t *dev);
void cc1x0x_isr_handler(cc1x0x_t *dev, void(*callback)(void*), void*arg);
void cc1x0x_set_base_freq_raw(cc1x0x_t *dev, const char* freq_array);
void cc1x0x_setup_rx_mode(cc1x0x_t *dev);
void cc1x0x_switch_to_pwd(cc1x0x_t *dev);
void cc1x0x_switch_to_rx(cc1x0x_t *dev);
void cc1x0x_wakeup_from_rx(cc1x0x_t *dev);
void cc1x0x_write_register(cc1x0x_t *dev, uint8_t r, uint8_t value);

extern const char cc1x0x_default_conf[];
extern const uint8_t cc1x0x_default_conf_size;
extern const uint8_t cc1x0x_pa_table[];

#ifdef MODULE_CC1X0X_HOOKS
void cc1x0x_hooks_init(void);
void cc1x0x_hook_rx(void);
void cc1x0x_hook_tx(void);
void cc1x0x_hook_off(void);
#endif
/* @} */

#ifdef __cplusplus
}
#endif

/** @} */
#endif /* CC1X0X_INTERFACE_H */
