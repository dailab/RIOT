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
 * @brief     cc1x0x gnrc glue code interface
 *
 * @author    Kaspar Schleiser <kaspar@schleiser.de>
 */

#include "net/gnrc/netdev2.h"
#include "cc1x0x-netdev2.h"

#ifndef GNRC_CC1X0X_H
#define GNRC_CC1X0X_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief initialize cc1x0x gnrc glue code
 *
 * @param[in] gnrc_netdev2  gnrc_netdev2 state structure to initialize
 * @param[in] dev           cc1x0x device structure to setup
 *
 * @return 1    on sucess
 * @return <=0  on error
 */
int gnrc_netdev2_cc1x0x_init(gnrc_netdev2_t *gnrc_netdev2, netdev2_cc1x0x_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* GNRC_CC1X0X_H */
/** @} */
