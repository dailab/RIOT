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

#ifndef CC1X0X_PARAMS_H
#define CC1X0X_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "board.h"

/**
 * @name CC1X0X configuration
 */
const cc1x0x_params_t cc1x0x_params[] = {
    {
        .spi  = SPI_0,
        .cs   = CC1200_CSN_GPIO,
        .gdo0 = CC1200_GPD0_GPIO,
        .gdo1 = CC1200_MISO_GPIO,
        .gdo2 = CC1200_GPD2_GPIO
    },
};
/** @} */

#ifdef __cplusplus
}
#endif
#endif /* CC1X0X_PARAMS_H */
/** @} */
