/*
 * Copyright (C) 2014 Freie Universität Berlin
 *               2015 Kaspar Schleiser <kaspar@schleiser.de>
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
 * @brief       Variables for the cc1x0x ng_netdev base interface
 *
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 */

#ifndef CC1X0X_NETDEV_H
#define CC1X0X_NETDEV_H

#include "periph/gpio.h"
#include "periph/spi.h"
#include "net/netdev2.h"
#include "cc1x0x.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Implementation of netdev2_driver_t for CC1X0X device
 */
extern const netdev2_driver_t netdev2_cc1x0x_driver;

/**
 * @brief cc1x0x netdev2 struct
 */
typedef struct netdev2_cc1x0x {
    netdev2_t netdev;       /**< writing obious */
    cc1x0x_t cc1x0x;        /**< documentation here */
} netdev2_cc1x0x_t;

/**
 * @brief   Received packet status information for cc1x0x radios
 */
typedef struct netdev2_radio_rx_info netdev2_cc1x0x_rx_info_t;

/**
 * @brief netdev2 <-> cc1x0x glue code initialization function
 *
 * @param[out]      netdev2_cc1x0x  ptr to netdev2_cc1x0x struct ti initialize
 * @param[in]       params          cc1x0x IO parameter struct to use
 *
 * @return          0               on success
 * @return          -1              on error
 */
int netdev2_cc1x0x_setup(netdev2_cc1x0x_t *netdev2_cc1x0x, const cc1x0x_params_t *params);

#ifdef __cplusplus
}
#endif

#endif /* CC1X0X_NETDEV_H */
