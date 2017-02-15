/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup   driver_cc1x0x
 * @{
 *
 * @file
 * @brief     cc1x0x default settings override
 *
 * By setting either CC1X0X_DEFAULT_PATABLE or CC1X0X_DEFAULT_FREQ in board.h,
 * it is possible to override the default pa table or base frequency registers
 * on a per-device basis.
 *
 * @author    Kaspar Schleiser <kaspar@schleiser.de>
 */
#ifndef CC1X0X_DEFAULTSETTINGS_H
#define CC1X0X_DEFAULTSETTINGS_H

#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CC1X0X_DEFAULT_PATABLE
#define CC1X0X_DEFAULT_PATABLE cc1x0x_default_pa_table
extern const char cc1x0x_default_pa_table[8];
#endif

#ifndef CC1X0X_DEFAULT_FREQ
#define CC1X0X_DEFAULT_FREQ cc1x0x_default_base_freq
extern const char cc1x0x_default_base_freq[3];
#endif

#ifdef __cplusplus
}
#endif

#endif /* CC1X0X_DEFAULTSETTINGS_H */
/** @} */
