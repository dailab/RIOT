/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup   board_remote_revb
 * @{
 *
 * @file
 * @brief     cc1x0x board specific configuration
 *
 * @author    Kaspar Schleiser <kaspar@schleiser.de>
 */

#ifndef CC110X_PARAMS_H
#define CC110X_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "board.h"

/**
 * @name CC1X0X configuration
 */
const cc110x_params_t cc110x_params[] = {
    {
        .spi  = 0,
        //.cs   = CC1200_CSN_GPIO,
        .cs   = GPIO_PA5,
        .gdo0 = CC1200_GPD0_GPIO,
        .gdo1 = CC1200_MISO_GPIO,
        .gdo2 = CC1200_GPD2_GPIO
    },
};
/** @} */

#ifdef __cplusplus
}
#endif
#endif /* CC110X_PARAMS_H */
/** @} */
