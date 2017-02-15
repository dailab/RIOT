/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include <assert.h>

#include <sys/uio.h>

#include "net/netdev2.h"
#include "net/gnrc.h"
#include "cc1x0x.h"
#include "cc1x0x-netdev2.h"
#include "net/gnrc/netdev2.h"
#include "od.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

static int _send(gnrc_netdev2_t *gnrc_netdev2, gnrc_pktsnip_t *pkt)
{
    cc1x0x_pkt_t cc1x0x_pkt;
    netdev2_t *dev = gnrc_netdev2->dev;
    netdev2_cc1x0x_t *netdev_cc1x0x = (netdev2_cc1x0x_t *) dev;
    cc1x0x_t *cc1x0x = &netdev_cc1x0x->cc1x0x;

    assert(pkt != NULL);
    assert(dev->driver == &netdev2_cc1x0x_driver);

    gnrc_netif_hdr_t *netif_hdr;
    gnrc_pktsnip_t *payload;

    payload = pkt->next;

    if (pkt->type != GNRC_NETTYPE_NETIF) {
        DEBUG("gnrc_netdev2_cc1x0x: First header was not generic netif header\n");
        gnrc_pktbuf_release(pkt);
        return -EBADMSG;
    }

    netif_hdr = (gnrc_netif_hdr_t *) pkt->data;

    /* set up header */
    if (netif_hdr->src_l2addr_len == 1) {
        uint8_t *_src_addr = gnrc_netif_hdr_get_src_addr(netif_hdr);
        cc1x0x_pkt.phy_src = *_src_addr;
    }
    else {
        cc1x0x_pkt.phy_src = cc1x0x->radio_address;
    }

    if (netif_hdr->flags & (GNRC_NETIF_HDR_FLAGS_BROADCAST |
                GNRC_NETIF_HDR_FLAGS_MULTICAST)) {
        cc1x0x_pkt.address = 0;
    }
    else {
        uint8_t *_dst_addr = gnrc_netif_hdr_get_dst_addr(netif_hdr);
        cc1x0x_pkt.address = _dst_addr[netif_hdr->dst_l2addr_len-1];
    }

    switch (payload->type) {
#ifdef MODULE_GNRC_SIXLOWPAN
        case GNRC_NETTYPE_SIXLOWPAN:
            cc1x0x_pkt.flags = 1;
            break;
#endif
        default:
            cc1x0x_pkt.flags = 0;
    }

    struct iovec vector;
    vector.iov_base = (char*)&cc1x0x_pkt;
    vector.iov_len = sizeof(cc1x0x_pkt_t);

    unsigned payload_len = 0;
    uint8_t *pos = cc1x0x_pkt.data;

    while (payload) {
        payload_len += payload->size;

        if (payload_len > CC1X0X_MAX_DATA_LENGTH) {
            DEBUG("gnrc_netdev2_cc1x0x: payload length exceeds maximum"
                    "(%u>%u)\n", payload_len, CC1X0X_MAX_DATA_LENGTH);
            gnrc_pktbuf_release(pkt);
            return -EBADMSG;
        }

        memcpy(pos, payload->data, payload->size);
        pos += payload->size;
        payload = payload->next;
    }

    /* pkt has been copied into iovec, we're done with it. */
    gnrc_pktbuf_release(pkt);

    cc1x0x_pkt.length = (uint8_t) payload_len + CC1X0X_HEADER_LENGTH;

    DEBUG("gnrc_netdev2_cc1x0x: sending packet from %u to %u with payload "
            "length %u\n",
            (unsigned)cc1x0x_pkt.phy_src,
            (unsigned)cc1x0x_pkt.address,
            (unsigned)cc1x0x_pkt.length);

    return dev->driver->send(dev, &vector, 1);
}

static gnrc_pktsnip_t *_recv(gnrc_netdev2_t *gnrc_netdev2)
{
    netdev2_t *dev = gnrc_netdev2->dev;
    cc1x0x_t *cc1x0x = &((netdev2_cc1x0x_t*) dev)->cc1x0x;

    cc1x0x_pkt_t *cc1x0x_pkt = &cc1x0x->pkt_buf.packet;

    int payload_length = cc1x0x_pkt->length - CC1X0X_HEADER_LENGTH;

    int nettype;

    int addr_len;
    switch (cc1x0x_pkt->flags) {
#ifdef MODULE_GNRC_SIXLOWPAN
        case 1:
            addr_len = 8;
            nettype = GNRC_NETTYPE_SIXLOWPAN;
            break;
#endif
        default:
            addr_len = 1;
            nettype = GNRC_NETTYPE_UNDEF;
    }

    /* copy packet payload into pktbuf */
    gnrc_pktsnip_t *pkt = gnrc_pktbuf_add(NULL, cc1x0x_pkt->data,
            payload_length, nettype);

    if(!pkt) {
        DEBUG("cc1x0x: _recv: cannot allocate pktsnip.\n");
        return NULL;
    }


    gnrc_pktsnip_t *netif_hdr;
    netif_hdr = gnrc_pktbuf_add(NULL, NULL,
            sizeof(gnrc_netif_hdr_t) + 2*addr_len,
            GNRC_NETTYPE_NETIF);

    if (netif_hdr == NULL) {
        DEBUG("gnrc_netdev2_cc1x0x: no space left in packet buffer\n");
        gnrc_pktbuf_release(pkt);
        return NULL;
    }

    gnrc_netif_hdr_init(netif_hdr->data, addr_len, addr_len);
    if (addr_len == 8) {
        uint64_t src_addr = cc1x0x_pkt->phy_src;
        uint64_t dst_addr = cc1x0x_pkt->address;
        gnrc_netif_hdr_set_src_addr(netif_hdr->data, (uint8_t*)&src_addr, addr_len);
        gnrc_netif_hdr_set_dst_addr(netif_hdr->data, (uint8_t*)&dst_addr, addr_len);
    }
    else {
        gnrc_netif_hdr_set_src_addr(netif_hdr->data, (uint8_t*)&cc1x0x_pkt->phy_src, addr_len);
        gnrc_netif_hdr_set_dst_addr(netif_hdr->data, (uint8_t*)&cc1x0x_pkt->address, addr_len);
    }

    ((gnrc_netif_hdr_t *)netif_hdr->data)->if_pid = thread_getpid();
    ((gnrc_netif_hdr_t *)netif_hdr->data)->lqi = cc1x0x->pkt_buf.lqi;
    ((gnrc_netif_hdr_t *)netif_hdr->data)->rssi = cc1x0x->pkt_buf.rssi;

    DEBUG("gnrc_netdev2_cc1x0x: received packet from %02x"
            " of length %u\n",
            (unsigned)cc1x0x_pkt->phy_src,
            (unsigned)cc1x0x_pkt->length-CC1X0X_HEADER_LENGTH);
#if defined(MODULE_OD) && ENABLE_DEBUG
    od_hex_dump(cc1x0x_pkt->data, payload_length, OD_WIDTH_DEFAULT);
#endif


    pkt->next = netif_hdr;

    return pkt;
}

int gnrc_netdev2_cc1x0x_init(gnrc_netdev2_t *gnrc_netdev2, netdev2_t *dev)
{
    gnrc_netdev2->send = _send;
    gnrc_netdev2->recv = _recv;
    gnrc_netdev2->dev = dev;

    return 0;
}
