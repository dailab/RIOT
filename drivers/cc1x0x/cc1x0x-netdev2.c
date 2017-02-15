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
 * @file
 * @brief       Implementation of netdev2 interface for cc1x0x
 *
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @}
 */

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "cc1x0x.h"
#include "cc1x0x-netdev2.h"
#include "cc1x0x-internal.h"
#include "cc1x0x-interface.h"
#include "net/eui64.h"

#include "periph/cpuid.h"
#include "periph/gpio.h"
#include "net/netdev2.h"
#include "net/gnrc/nettype.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

static int _send(netdev2_t *dev, const struct iovec *vector, unsigned count)
{
    DEBUG("%s:%u\n", __func__, __LINE__);

    netdev2_cc1x0x_t *netdev2_cc1x0x = (netdev2_cc1x0x_t*) dev;
    cc1x0x_pkt_t *cc1x0x_pkt = vector[0].iov_base;

    return cc1x0x_send(&netdev2_cc1x0x->cc1x0x, cc1x0x_pkt);
}

static int _recv(netdev2_t *dev, void *buf, size_t len, void *info)
{
    DEBUG("%s:%u\n", __func__, __LINE__);

    cc1x0x_t *cc1x0x = &((netdev2_cc1x0x_t*) dev)->cc1x0x;

    cc1x0x_pkt_t *cc1x0x_pkt = &cc1x0x->pkt_buf.packet;
    if (cc1x0x_pkt->length > len) {
        return -ENOSPC;
    }

    memcpy(buf, (void*)cc1x0x_pkt, cc1x0x_pkt->length);
    if (info != NULL) {
        netdev2_cc1x0x_rx_info_t *cc1x0x_info = info;

        cc1x0x_info->rssi = cc1x0x->pkt_buf.rssi;
        cc1x0x_info->lqi = cc1x0x->pkt_buf.lqi;
    }
    return cc1x0x_pkt->length;
}

static inline int _get_iid(netdev2_t *netdev, eui64_t *value, size_t max_len)
{
    cc1x0x_t *cc1x0x = &((netdev2_cc1x0x_t*) netdev)->cc1x0x;
    uint8_t *eui64 = (uint8_t*) value;

    if (max_len < sizeof(eui64_t)) {
        return -EOVERFLOW;
    }

    /* make address compatible to https://tools.ietf.org/html/rfc6282#section-3.2.2*/
    memset(eui64, 0, sizeof(eui64_t));
    eui64[3] = 0xff;
    eui64[4] = 0xfe;
    eui64[7] = cc1x0x->radio_address;

    return sizeof(eui64_t);
}

static int _get(netdev2_t *dev, netopt_t opt, void *value, size_t value_len)
{
    cc1x0x_t *cc1x0x = &((netdev2_cc1x0x_t*) dev)->cc1x0x;

    switch (opt) {
        case NETOPT_DEVICE_TYPE:
            assert(value_len == 2);
            *((uint16_t *) value) = NETDEV2_TYPE_CC1X0X;
            return 2;
        case NETOPT_PROTO:
            assert(value_len == sizeof(gnrc_nettype_t));
            *((gnrc_nettype_t *)value) = cc1x0x->proto;
            return sizeof(gnrc_nettype_t);
        case NETOPT_CHANNEL:
            assert(value_len > 1);
            *((uint16_t *)value) = (uint16_t)cc1x0x->radio_channel;
            return 2;
        case NETOPT_ADDRESS:
            assert(value_len > 0);
            *((uint8_t *)value) = cc1x0x->radio_address;
            return 1;
        case NETOPT_MAX_PACKET_SIZE:
            assert(value_len > 0);
            *((uint8_t *)value) = CC1X0X_PACKET_LENGTH;
            return 1;
        case NETOPT_IPV6_IID:
            return _get_iid(dev, value, value_len);
        default:
            break;
    }

    return -ENOTSUP;
}

static int _set(netdev2_t *dev, netopt_t opt, void *value, size_t value_len)
{
    cc1x0x_t *cc1x0x = &((netdev2_cc1x0x_t*) dev)->cc1x0x;

    switch (opt) {
        case NETOPT_CHANNEL:
            {
                if (value_len != sizeof(uint16_t)) {
                    return -EINVAL;
                }

                /* to account for endianess */
                uint16_t arg_16 = *((uint16_t*)value);
                //uint8_t *arg = (uint8_t*)value;
                //uint8_t channel = arg[value_len-1];
                uint8_t channel = arg_16;
                //uint8_t channel = arg[value_len-1];
                DEBUG("%s:%s:%u channel = %u value_len=%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__, channel, value_len);
                if ((channel < CC1X0X_MIN_CHANNR) || (channel > CC1X0X_MAX_CHANNR)) {
                    return -EINVAL;
                }
                if (cc1x0x_set_channel(cc1x0x, channel) == -1) {
                    return -EINVAL;
                }
                return 1;
            }
        case NETOPT_ADDRESS:
            if (value_len < 1) {
                return -EINVAL;
            }
            if (!cc1x0x_set_address(cc1x0x, *(uint8_t*)value)) {
                return -EINVAL;
            }
            return 1;
        case NETOPT_PROTO:
            if (value_len != sizeof(gnrc_nettype_t)) {
                return -EINVAL;
            }
            else {
                cc1x0x->proto = (gnrc_nettype_t) value;
                return sizeof(gnrc_nettype_t);
            }
            break;
        default:
            return -ENOTSUP;
    }

    return 0;
}

static void _netdev2_cc1x0x_isr(void *arg)
{
    netdev2_t *netdev2 = (netdev2_t*) arg;
    netdev2->event_callback(netdev2, NETDEV2_EVENT_ISR);
}

static void _netdev2_cc1x0x_rx_callback(void *arg)
{
    netdev2_t *netdev2 = (netdev2_t*) arg;
    cc1x0x_t *cc1x0x = &((netdev2_cc1x0x_t*) arg)->cc1x0x;
    gpio_irq_disable(cc1x0x->params.gdo2);
    netdev2->event_callback(netdev2, NETDEV2_EVENT_RX_COMPLETE);
}

static void _isr(netdev2_t *dev)
{
    DEBUG("%s:%u\n", __func__, __LINE__);
    cc1x0x_t *cc1x0x = &((netdev2_cc1x0x_t*) dev)->cc1x0x;
    DEBUG("%s:%u\n", __func__, __LINE__);
    cc1x0x_isr_handler(cc1x0x, _netdev2_cc1x0x_rx_callback, (void*)dev);
}

static int _init(netdev2_t *dev)
{
    DEBUG("%s:%u\n", __func__, __LINE__);

    cc1x0x_t *cc1x0x = &((netdev2_cc1x0x_t*) dev)->cc1x0x;

    gpio_init_int(cc1x0x->params.gdo2, GPIO_IN, GPIO_BOTH,
            &_netdev2_cc1x0x_isr, (void*)dev);

    gpio_set(cc1x0x->params.gdo2);
    gpio_irq_disable(cc1x0x->params.gdo2);

    /* Switch to RX mode */
    cc1x0x_rd_set_mode(cc1x0x, RADIO_MODE_ON);

    return 0;
}

const netdev2_driver_t netdev2_cc1x0x_driver = {
    .send=_send,
    .recv=_recv,
    .init=_init,
    .get=_get,
    .set=_set,
    .isr=_isr
};

int netdev2_cc1x0x_setup(netdev2_cc1x0x_t *netdev2_cc1x0x, const cc1x0x_params_t *params)
{
    DEBUG("netdev2_cc1x0x_setup()\n");
    netdev2_cc1x0x->netdev.driver = &netdev2_cc1x0x_driver;

    /* set default protocol */
#ifdef MODULE_GNRC_NETIF
# ifdef MODULE_GNRC_SIXLOWPAN
    netdev2_cc1x0x->cc1x0x.proto = GNRC_NETTYPE_SIXLOWPAN;
# else
    netdev2_cc1x0x->cc1x0x.proto = GNRC_NETTYPE_UNDEF;
# endif
#endif

    return cc1x0x_setup(&netdev2_cc1x0x->cc1x0x, params);
}
