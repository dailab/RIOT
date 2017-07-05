/*
 * Copyright (C) 2014 Freie Universität Berlin
 *               2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_cc1200
 * @{
 * @file
 * @brief       Implementation of netdev interface for cc1200
 *
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @}
 */

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "cc1200.h"
#include "cc1200-netdev.h"
#include "cc1200-internal.h"
#include "cc1200-interface.h"
#include "net/eui64.h"

#include "periph/gpio.h"
#include "net/netdev.h"
#include "net/gnrc/nettype.h"
#include "led.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

static int _send(netdev_t *dev, const struct iovec *vector, unsigned count)
{
    DEBUG("%s:%u\n", __func__, __LINE__);
    LED_ON(0);
    int result = 0;
    netdev_cc1200_t *netdev_cc1200 = (netdev_cc1200_t*) dev;
#ifdef MODULE_CC1200
    int size = 0;
    for(int i = 0; i<count; i++) size += vector[i].iov_len;
    if(size > CC1200_PACKET_LENGTH-1){
        return -EOVERFLOW;
    }

    char package[size+1];
    package[0] = size;
    uint pos = 1;


    DEBUG("%s:%u SIZE: %u\n", __func__, __LINE__, size);
    for(int i = 0; i< count; i++){
        uint pkt_len = vector[i].iov_len;

        DEBUG("%s:%u Package to send: \n", __func__, __LINE__);
        for(int j = 0; j < pkt_len; j++){
            DEBUG("0x%x ", *(((char*) vector[i].iov_base)+j));
        }
        DEBUG("\n");

        memcpy(package+pos, vector[i].iov_base, pkt_len);
        pos += pkt_len;
    }
    cc1200_pkt_t *cc1200_pkt = (cc1200_pkt_t*) package;
#else
    cc1200_pkt_t *cc1200_pkt = vector[0].iov_base;

#endif  /* MODULE_CC1200 */
    result = cc1200_send(&netdev_cc1200->cc1200, cc1200_pkt );
    LED_OFF(0);
    return result;
}

static int _recv(netdev_t *dev, void *buf, size_t len, void *info)
{
    DEBUG("%s:%u\n", __func__, __LINE__);

    cc1200_t *cc1200 = &((netdev_cc1200_t*) dev)->cc1200;
    cc1200_pkt_t *cc1200_pkt = &cc1200->pkt_buf.packet;
#ifdef MODULE_CC1200
    if(buf == NULL){
        /* GNRC wants to know how many bytes we got */
        
        /* Make sure pkt_len is sane */
        if (cc1200_pkt->length > CC1200_MAX_DATA_LENGTH) {
            return -EOVERFLOW;
        }

        //return cc1200_pkt->length - IEEE802154_FCS_LEN;
        return cc1200_pkt->length;
    }

#endif

    if (cc1200_pkt->length > len) {
        return -ENOSPC;
    }

    //memcpy(buf, (void*)cc1200_pkt, cc1200_pkt->length);
    memcpy(buf, (void*)cc1200_pkt+1, cc1200_pkt->length);
    if (info != NULL) {
        netdev_cc1200_rx_info_t *cc1200_info = info;

        cc1200_info->rssi = cc1200->pkt_buf.rssi;
        cc1200_info->lqi = cc1200->pkt_buf.lqi;
    }
    DEBUG("%s:%u pkt->length: %u\n", __func__, __LINE__, cc1200_pkt->length);
    return cc1200_pkt->length;
}

static inline int _get_iid(netdev_t *netdev, eui64_t *value, size_t max_len)
{
    cc1200_t *cc1200 = &((netdev_cc1200_t*) netdev)->cc1200;
    uint8_t *eui64 = (uint8_t*) value;

    if (max_len < sizeof(eui64_t)) {
        return -EOVERFLOW;
    }

    /* make address compatible to https://tools.ietf.org/html/rfc6282#section-3.2.2*/
    memset(eui64, 0, sizeof(eui64_t));
    eui64[3] = 0xff;
    eui64[4] = 0xfe;
    eui64[7] = cc1200->radio_address;

    return sizeof(eui64_t);
}

static int _get(netdev_t *dev, netopt_t opt, void *value, size_t value_len)
{
#ifndef MODULE_CC1200
    cc1200_t *cc1200 = &((netdev_cc1200_t*) dev)->cc1200;
#endif

    int res = -ENOTSUP;
    switch (opt) {

#ifndef MODULE_CC1200
#ifdef MODULE_GNRC_NETIF
        case NETOPT_PROTO:
            assert(value_len == sizeof(gnrc_nettype_t));
            *((gnrc_nettype_t *)value) = cc1200->proto;
            return sizeof(gnrc_nettype_t);
#endif
        case NETOPT_CHANNEL:
            assert(value_len > 1);
            *((uint16_t *)value) = (uint16_t)cc1200->radio_channel;
            return 2;
        case NETOPT_ADDRESS:
            assert(value_len > 0);
            *((uint8_t *)value) = cc1200->radio_address;
            return 1;
        case NETOPT_IPV6_IID:
            return _get_iid(dev, value, value_len);
        case NETOPT_DEVICE_TYPE:
            assert(value_len == 2);
            *((uint16_t *) value) = NETDEV_TYPE_CC1200;
            return 2;
#else
        case NETOPT_DEVICE_TYPE:
            assert(value_len == 2);
            *((uint16_t *) value) = NETDEV_TYPE_IEEE802154;
            res = 2;
            break;


#endif /* MODULE_CC1200 */

        case NETOPT_MAX_PACKET_SIZE:
            assert(value_len > 0);
            *((uint8_t *)value) = CC1200_PACKET_LENGTH;
            return 1;
        default:
            break;
    }

#ifdef MODULE_CC1200
    if (((res = netdev_ieee802154_get((netdev_ieee802154_t *)dev, opt, value,
                                      value_len)) >= 0) || (res != -ENOTSUP)) {
        return res;
    }
#endif /* MODULE_CC1200 */
    return res;
}

static int _set(netdev_t *dev, netopt_t opt, void *value, size_t value_len)
{
    cc1200_t *cc1200 = &((netdev_cc1200_t*) dev)->cc1200;
    int res = -ENOTSUP;

    switch (opt) {
        case NETOPT_CHANNEL:
            {
                uint8_t *arg = (uint8_t*)value;
                uint8_t channel = arg[value_len-1];
                if ((channel < CC1200_MIN_CHANNR) || (channel > CC1200_MAX_CHANNR)) {
                    return -EINVAL;
                }
                if (cc1200_set_channel(cc1200, channel) == -1) {
                    return -EINVAL;
                }
                break;
            }
        case NETOPT_ADDRESS:
            if (value_len < 1) {
                return -EINVAL;
            }
            if (!cc1200_set_address(cc1200, *(uint8_t*)value)) {
                return -EINVAL;
            }
            if (!cc1200_set_address_short(cc1200, *(uint16_t*)value)) {
                return -EINVAL;
            }
            break;
        case NETOPT_NID:
            if (value_len > sizeof(uint16_t)) {
                res = -EOVERFLOW;
            }
            if (value_len < 1) {
                return -EINVAL;
            }
            cc1200->pan_id = *(uint16_t*) value;
            break;
        case NETOPT_ADDRESS_LONG:
            if (value_len > sizeof(uint64_t)) {
                res = -EOVERFLOW;
            }
            else {
                res = cc1200_set_address_long(cc1200, *((uint64_t*)value));
            }
            break;
#ifdef MODULE_GNRC_NETIF
        case NETOPT_PROTO:
            if (value_len != sizeof(gnrc_nettype_t)) {
                return -EINVAL;
            }
            else {
                cc1200->proto = (gnrc_nettype_t) value;
                return sizeof(gnrc_nettype_t);
            }
            break;
#endif
        default:
            break;
    }

    if (res == -ENOTSUP) {
        res = netdev_ieee802154_set((netdev_ieee802154_t *)dev, opt,
                                    value, value_len);
    }
    return res;
}

static void _netdev_cc1200_isr(void *arg)
{
    DEBUG("%s:%u\n", __func__, __LINE__);
    netdev_t *netdev = (netdev_t*) arg;
    unsigned state = irq_disable();
    netdev->event_callback(netdev, NETDEV_EVENT_ISR);
    irq_restore(state);
    DEBUG("%s:%u\n", __func__, __LINE__);
}

static void _netdev_cc1200_rx_callback(void *arg)
{
    DEBUG("%s:%u\n", __func__, __LINE__);
    netdev_t *netdev = (netdev_t*) arg;
    cc1200_t *cc1200 = &((netdev_cc1200_t*) arg)->cc1200;
    gpio_irq_disable(cc1200->params.gdo2);
    netdev->event_callback(netdev, NETDEV_EVENT_RX_COMPLETE);
}

static void _isr(netdev_t *dev)
{
    DEBUG("%s:%u\n", __func__, __LINE__);
    cc1200_t *cc1200 = &((netdev_cc1200_t*) dev)->cc1200;
    unsigned state = irq_disable();
    cc1200_isr_handler(cc1200, _netdev_cc1200_rx_callback, (void*)dev);
    irq_restore(state);
}

static int _init(netdev_t *dev)
{
    DEBUG("%s:%u\n", __func__, __LINE__);
    cc1200_t *cc1200 = &((netdev_cc1200_t*) dev)->cc1200;

    gpio_init_int(cc1200->params.gdo2, GPIO_IN_PD, GPIO_BOTH,
            &_netdev_cc1200_isr, (void*)dev);
    gpio_init(cc1200->params.gdo0, GPIO_IN_PD);

    gpio_set(cc1200->params.gdo2);
    gpio_irq_disable(cc1200->params.gdo2);

    /* Switch to RX mode */
    cc1200_rd_set_mode(cc1200, RADIO_MODE_OFF);
    cc1200_rd_set_mode(cc1200, RADIO_MODE_ON);
    DEBUG("%s:%u\n", __func__, __LINE__);

    return 0;
}

const netdev_driver_t netdev_cc1200_driver = {
    .send=_send,
    .recv=_recv,
    .init=_init,
    .get=_get,
    .set=_set,
    .isr=_isr
};

int netdev_cc1200_setup(netdev_cc1200_t *netdev_cc1200, const cc1200_params_t *params)
{
    DEBUG("netdev_cc1200_setup()\n");
#if MODULE_CC1200
    netdev_cc1200->netdev.netdev.driver = &netdev_cc1200_driver;
#ifdef MODULE_GNRC_NETIF
# ifdef MODULE_GNRC_SIXLOWPAN
    netdev_cc1200->cc1200.proto = GNRC_NETTYPE_SIXLOWPAN;
    netdev_cc1200->netdev.proto = GNRC_NETTYPE_SIXLOWPAN;
# else
    netdev_cc1200->cc1200.proto = GNRC_NETTYPE_UNDEF;
    netdev_cc1200->netdev.proto = GNRC_NETTYPE_UNDEF;
# endif
#endif
#else
    netdev_cc1200->netdev.driver = &netdev_cc1200_driver;

    /* set default protocol */
#ifdef MODULE_GNRC_NETIF
# ifdef MODULE_GNRC_SIXLOWPAN
    netdev_cc1200->cc1200.proto = GNRC_NETTYPE_SIXLOWPAN;
# else
    netdev_cc1200->cc1200.proto = GNRC_NETTYPE_UNDEF;
# endif
#endif
#endif

    cc1200_setup(&netdev_cc1200->cc1200, params);

    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);

    /* Initialise netdev_ieee802154_t struct */
    cc1200_t *cc1200 = &netdev_cc1200->cc1200; 
    netdev_ieee802154_set((netdev_ieee802154_t *)netdev_cc1200, NETOPT_NID, &cc1200->pan_id,
                          sizeof(uint16_t));
    netdev_ieee802154_set((netdev_ieee802154_t *)netdev_cc1200, NETOPT_CHANNEL, &cc1200->radio_channel,
                          sizeof(uint8_t));
    netdev_ieee802154_set((netdev_ieee802154_t *)netdev_cc1200, NETOPT_ADDRESS,
                          &cc1200->radio_address_short, sizeof(uint16_t));
    netdev_ieee802154_set((netdev_ieee802154_t *)netdev_cc1200, NETOPT_ADDRESS_LONG,
                          &cc1200->radio_address_long, sizeof(uint64_t));
    return 0;
}
