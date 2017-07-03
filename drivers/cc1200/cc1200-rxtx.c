/*
 * Copyright (C) 2014 Freie Universität Berlin
 * Copyright (C) 2013 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_cc1200
 * @{
 * @file
 * @brief       Functions for packet reception and transmission on cc1200 devices
 *
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @}
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "cc1200.h"
#include "cc1200-spi.h"
#include "cc1200-internal.h"
#include "cc1200-interface.h"
#include "cc1200-defines.h"

#include "periph/gpio.h"
#include "irq.h"

#include "kernel_types.h"
#include "msg.h"
#include "xtimer.h"

#include "cpu_conf.h"
#include "cpu.h"

#include "log.h"
#include "led.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

static void _rx_abort(cc1200_t *dev)
{
    gpio_irq_disable(dev->params.gdo2);

    cc1200_strobe(dev, CC1200_SIDLE);    /* Switch to IDLE (should already be)... */
    cc1200_strobe(dev, CC1200_SFRX);     /* ...for flushing the RX FIFO */

    cc1200_switch_to_rx(dev);
}



static void _rx_read_data(cc1200_t *dev, void(*callback)(void*), void*arg)
{
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
    //printf("%s:%u\n", __func__, __LINE__);
    //int fifo = cc1200_get_reg_robust(dev, 0xfb);
    uint8_t fifo = cc1200_read_reg(dev, CC1200_NUM_RXBYTES);
    //DEBUG("%s:%s:%u FIFO: %u\n", RIOT_FILE_RELATIVE, __func__, __LINE__, fifo);

    //#if ENABLE_DEBUG
    //int m_state = cc1200_read_reg(dev, CC1200_MODEM_STATUS1);
    //DEBUG("%s:%u ModemSt1: %u\n", __func__, __LINE__, m_state);
    //#endif
    /*
    int bytes = cc1200_read_reg(dev, CC1200_MARC_STATUS1);
    DEBUG("%s:%u MarcSt1: %u\n", __func__, __LINE__, bytes);
    //if (m_state & 0x08) {
    */
    uint8_t overflow = cc1200_strobe(dev, CC1200_SNOP);
    //DEBUG("%s:%s:%u STATE: %u\n", RIOT_FILE_RELATIVE, __func__, __LINE__, overflow);
    if((overflow & 0x70) == 0x60){
        //DEBUG("%s:%s:%u rx overflow\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
        _rx_abort(dev);
        return;
    }



    if (!fifo) {
    //DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
    printf("%s:%u\n", __func__, __LINE__);
        gpio_irq_enable(dev->params.gdo2);
        return;
    }

    //printf("%s:%u\n", __func__, __LINE__);
    cc1200_pkt_buf_t *pkt_buf = &dev->pkt_buf;
    //DEBUG("%s:%s:%u pkt_buf->pos: %u\n", RIOT_FILE_RELATIVE, __func__, __LINE__, pkt_buf->pos);
    if (!pkt_buf->pos) {
        pkt_buf->pos = 1;

    //printf("%s:%u\n", __func__, __LINE__);
    //printf("%s:%u\n", __func__, __LINE__);
        pkt_buf->packet.length = cc1200_read_reg(dev, CC1200_RXFIFO);

        /* Possible packet received, RX -> IDLE (0.1 us) */
        dev->cc1200_statistic.packets_in++;
    }
    //DEBUG("%s:%s:%u Packet Size: %u\n", RIOT_FILE_RELATIVE, __func__, __LINE__, pkt_buf->packet.length);
    //DEBUG("%s:%s:%u fifo: %u\n", RIOT_FILE_RELATIVE, __func__, __LINE__, fifo);

    //printf("%s:%u\n", __func__, __LINE__);
    int left = pkt_buf->packet.length+1 - pkt_buf->pos;

    /* if the fifo doesn't contain the rest of the packet,
     * leav at least one byte as per spec sheet. */
    int to_read = (fifo < left) ? (fifo-1) : fifo;
    if (to_read > left) {
        to_read = left;
    }
    //to_read = fifo;

    //DEBUG("%s:%s:%u to_read: %u\n", RIOT_FILE_RELATIVE, __func__, __LINE__, to_read);
    //printf("%s:%u\n", __func__, __LINE__);
    if (to_read) {
        /*
        //DEBUG("%s:%u to read:%u\n", __func__, __LINE__, to_read);
        uint8_t buffer[to_read];
        memset(buffer, 0x00, to_read);

        cc1200_readburst_reg(dev, CC1200_RXFIFO,
                (char*)buffer, to_read);

        DEBUG("%s:%u Received:\n", __func__, __LINE__);
        for(int i = 0; i < to_read; i++){
            DEBUG("0x%x ", buffer[i]);
        }
        DEBUG("\n");
        memcpy(((char*)&(pkt_buf->packet))+(pkt_buf->pos), buffer, to_read);
        */
        cc1200_readburst_reg(dev, CC1200_RXFIFO,
                ((char*)&(pkt_buf->packet))+(pkt_buf->pos), to_read);
        pkt_buf->pos += to_read;
    }

    //printf("%s:%u\n", __func__, __LINE__);
    //DEBUG("%s:%s:%u to_read: %u\n", RIOT_FILE_RELATIVE, __func__, __LINE__, to_read);
    if (to_read == left) {
        uint8_t status[2];
        /* full packet received. */
        /* Read the 2 appended status bytes (status[0] = RSSI, status[1] = LQI) */
        cc1200_readburst_reg(dev, CC1200_RXFIFO, (char *)status, 2);
        /*
        DEBUG("%s:%u status0: 0x%x\n", __func__, __LINE__, status[0]);
        DEBUG("%s:%u status1: 0x%x\n", __func__, __LINE__, status[1]);
        */


        /* Store RSSI value of packet */
        pkt_buf->rssi = status[I_RSSI];

        /* Bit 0-6 of LQI indicates the link quality (LQI) */
        pkt_buf->lqi = status[I_LQI] & LQI_EST;

        /* MSB of LQI is the CRC_OK bit */
        
#ifdef MODULE_CC1200
        /*We are using the CRC autoflush config, so when we get here, the CRC check already suceeded */
        int crc_ok = 1;
#else
        int crc_ok = (status[I_LQI] & CRC_OK) >> 7;
#endif
        if (crc_ok) {
                    DEBUG("cc1200: received packet from=%u to=%u payload "
                            "len=%u\n",
                    (unsigned)pkt_buf->packet.phy_src,
                    (unsigned)pkt_buf->packet.address,
                    pkt_buf->packet.length-3);
            /*Printing package */
            /*
            DEBUG("Printing received package after crc:\n");
            for(int i = 0; i < pkt_buf->packet.length+1; i++){
                DEBUG("0x%x ", *(((char*)&(pkt_buf->packet))+(i)));
            }
            DEBUG("\n");*/
            /* let someone know that we've got a packet */
            callback(arg);

            DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
    //printf("%s:%u\n", __func__, __LINE__);
            cc1200_switch_to_rx(dev);
        }
        else {
            //DEBUG("%s:%s:%u crc-error\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
            dev->cc1200_statistic.packets_in_crc_fail++;
            _rx_abort(dev);
        }
    }else{
        printf("%s:%u FIFO: %u\n", __func__, __LINE__, fifo);
        printf("%s:%u FIFO: %u\n", __func__, __LINE__, fifo);
    }
    //DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
}

static void _rx_continue(cc1200_t *dev, void(*callback)(void*), void*arg)
{
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
    printf("%s:%u\n", __func__, __LINE__);


    if (dev->radio_state != RADIO_RX_BUSY) {
        DEBUG("%s:%s:%u _rx_continue in invalid state\n", RIOT_FILE_RELATIVE,
                __func__, __LINE__);
        _rx_abort(dev);
        return;
    }

    gpio_irq_disable(dev->params.gdo2);
    cc1200_write_reg(dev, CC1200_IOCFG2, 0x01);
    do {
        _rx_read_data(dev, callback, arg);
    }
    while (gpio_read(dev->params.gdo2));
}

static void _rx_start(cc1200_t *dev)
{
    dev->radio_state = RADIO_RX_BUSY;

    cc1200_pkt_buf_t *pkt_buf = &dev->pkt_buf;
    pkt_buf->pos = 0;

    gpio_irq_disable(dev->params.gdo2);
    cc1200_write_reg(dev, CC1200_IOCFG2, 0x01);
    //cc1200_write_reg(dev, CC1200_IOCFG2, 0x07);
    if(gpio_read(dev->params.gdo2)){
        DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
        _rx_continue(dev, dev->isr_cb, dev->isr_cb_arg);
    }
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
    gpio_irq_enable(dev->params.gdo2);
}

static void _tx_abort(cc1200_t *dev)
{
    cc1200_switch_to_rx(dev);
}

static void _tx_continue(cc1200_t *dev)
{
    //DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
    printf("%s:%u\n", __func__, __LINE__);
    gpio_irq_disable(dev->params.gdo2);

    cc1200_pkt_t *pkt = &dev->pkt_buf.packet;
    //DEBUG("%s:%s:%u pkt->pos=%u, pkt->length=%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__, dev->pkt_buf.pos, pkt->length);
    /*
    DEBUG("%s:%s:%u printing package:\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
    for (int i = 0; i < pkt->length + 1; i++)
    {
        DEBUG("0x%x ", *(((char *)pkt) + i));
    }
    DEBUG("\n");*/

    int size = pkt->length + 1;

    int left = size - dev->pkt_buf.pos;

    if (!left)
    {
        dev->cc1200_statistic.raw_packets_out++;

        //LOG_DEBUG("cc1200: packet successfully sent.\n");

        cc1200_switch_to_rx(dev);
        return;
    }

//int fifo = 64 - cc1200_get_reg_robust(dev, 0xfa);
#if MODULE_CC1200
    int fifo = CC1200_PACKET_LENGTH - cc1200_get_reg_robust(dev, CC1200_NUM_TXBYTES);
#else
    int fifo = 64 - cc1200_get_reg_robust(dev, CC1200_NUM_TXBYTES);

    if (fifo & 0x80)
    {
        //DEBUG("%s:%s:%u tx underflow!\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
        _tx_abort(dev);
        return;
    }

    if (!fifo)
    {
        //DEBUG("%s:%s:%u fifo full!?\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
        _tx_abort(dev);
        return;
    }

#endif

    int status = cc1200_strobe(dev, CC1200_SNOP);
    if((status & 0x70) == 0x70){
        //DEBUG("%s:%s:%u fifo over/underflow!?\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
        _tx_abort(dev);
        return;
    }
    //DEBUG("%s:%s:%u fifo: %u\n", RIOT_FILE_RELATIVE, __func__, __LINE__, fifo);
    int to_send = left > fifo ? fifo : left;

    /* Write packet into TX FIFO */
    //DEBUG("%s:%s:%u pkt->pos=%u to_send=%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__, dev->pkt_buf.pos, to_send);

    /* Flush TX */
    cc1200_strobe(dev, CC1200_SFTX);
    cc1200_strobe(dev, CC1200_SFSTXON);
    //cc1200_write_reg(dev, CC1200_TXFIFO, pkt->length+1);
    //cc1200_writeburst_reg(dev, CC1200_TXFIFO, ((char *)pkt)+dev->pkt_buf.pos+1, to_send-1);
    cc1200_writeburst_reg(dev, CC1200_TXFIFO, ((char *)pkt) + dev->pkt_buf.pos, to_send);
    //cc1200_write_reg(dev, CC1200_TXFIFO, 0x00);
    dev->pkt_buf.pos += to_send;
    /*
    DEBUG("%s:%s:%u pkt->pos=%u to_send=%u, left=%u "
          "size=%u\n",
          RIOT_FILE_RELATIVE, __func__, __LINE__, dev->pkt_buf.pos, to_send, left, size);
*/
    if (left == size)
    {
        /* Switch to TX mode */
        cc1200_strobe(dev, CC1200_STX);
        xtimer_usleep(200);
    }

    if (to_send < left)
    {
        /* set GDO2 to 0x2 -> will deassert at TX FIFO below threshold */
        cc1200_write_reg(dev, CC1200_IOCFG2, 0x02);
        gpio_irq_enable(dev->params.gdo2);
    }
    else
    {
/* set GDO2 to 0x6 -> will deassert at packet end */
#ifdef MODULE_CC1200
        cc1200_write_reg(dev, CC1200_IOCFG2, 0x06);
#else
        cc1200_write_reg(dev, CC1200_IOCFG2, 0x06);
#endif
        gpio_irq_enable(dev->params.gdo2);
    }
}

void cc1200_isr_handler(cc1200_t *dev, void(*callback)(void*), void*arg)
{
//DEBUG("%s:%u\n", __func__, __LINE__);
uint8_t rxbytes = cc1200_read_reg(dev, CC1200_NUM_RXBYTES);
//xtimer_spin((xtimer_ticks33_t)100);
//xtimer_spin(xtimer_ticks_from_usec(700));
//DEBUG("%s:%u RX_BYTES: %u\n", __func__, __LINE__, rxbytes);
//printf("RX_BYTES %s\n", __func__);
//printf("test %u\n", rxbytes);

/*
#if ENABLE_DEBUG
uint8_t txbytes = cc1200_read_reg(dev, CC1200_NUM_TXBYTES);
DEBUG("%s:%u TX_BYTES: %u\n", __func__, __LINE__, txbytes);
#endif
*/

//if(gpio_read(dev->params.gdo0)) DEBUG("gd0 on\n");

    switch (dev->radio_state) {
        case RADIO_RX:
            DEBUG("radio rx\n");
            if (gpio_read(dev->params.gdo2) | (rxbytes > 0)) {
            //if (gpio_read(dev->params.gdo0) | gpio_read(dev->params.gdo2) | (rxbytes > 0)) {
            //if (gpio_read(dev->params.gdo2)) {
                DEBUG("cc1200_isr_handler((): starting RX\n");
                dev->isr_cb = callback;
                dev->isr_cb_arg = arg; 
                _rx_start(dev);
            }
            //#ifndef MODULE_CC1200
            else {
                DEBUG("cc1200_isr_handler((): isr handled too slow or falling edge trigger\n");
                _rx_abort(dev);
            }
            //#endif
            break;
        case RADIO_RX_BUSY:
    printf("%s:%u\n", __func__, __LINE__);
            DEBUG("radio rx busy\n");
            _rx_continue(dev, callback, arg);
            break;
        case RADIO_TX_BUSY:
            DEBUG("radio tx busy\n");
            if (!gpio_read(dev->params.gdo2)) {
                //DEBUG("cc1200_isr_handler() RADIO_TX_BUSY + GDO2 NOT!\n");
                _tx_continue(dev);
            }
            else {
                DEBUG("cc1200_isr_handler() RADIO_TX_BUSY + GDO2\n");
            }
            break;
        default:
            DEBUG("%s:%s:%u: unhandled mode\n", RIOT_FILE_RELATIVE,
                    __func__, __LINE__);
            break;
    }
}

int cc1200_send(cc1200_t *dev, cc1200_pkt_t *packet)
{
    //DEBUG("%s:%u\n", __func__, __LINE__);
    printf("%s:%u\n", __func__, __LINE__);
    //DEBUG("TEST\n");
    /*
    DEBUG("cc1200: snd pkt to %u payload_length=%u\n",
            (unsigned)packet->address, (unsigned)packet->length);
    for(int i = 0; i< packet->length+1; i++)DEBUG("0x%x ", *(((char* )packet)+i));
    DEBUG("\n");
    */

    uint8_t size;
    switch (dev->radio_state) {
        case RADIO_RX_BUSY:
            DEBUG("%s:%u\n", __func__, __LINE__);
            uint8_t rxbytes = cc1200_read_reg(dev, CC1200_NUM_RXBYTES);
            printf("%s:%u rx fifo: %u\n", __func__, __LINE__, rxbytes);
        case RADIO_TX_BUSY:
            /*
            DEBUG("cc1200: invalid state for sending: %s\n",
                    cc1200_state_to_text(dev->radio_state));
                    */
            DEBUG("%s:%u\n", __func__, __LINE__);
    printf("%s:%u\n", __func__, __LINE__);
            return -EAGAIN;
    }

    /*
     * Number of bytes to send is:
     * length of phy payload (packet->length)
     * + size of length field (1 byte)
     */
    size = packet->length + 1;

    if (size > CC1200_PACKET_LENGTH) {
        //DEBUG("%s:%s:%u trying to send oversized packet\n",
        //        RIOT_FILE_RELATIVE, __func__, __LINE__);
        return -ENOSPC;
    }

#ifndef MODULE_CC1200
    /* set source address */
    packet->phy_src = dev->radio_address;
#endif /* MODULE_CC1200 */

    /* Disable RX interrupt */
    gpio_irq_disable(dev->params.gdo2);
    dev->radio_state = RADIO_TX_BUSY;

#ifdef MODULE_CC1200_HOOKS
    cc1200_hook_tx();
#endif

    cc1200_write_reg(dev, CC1200_IOCFG2, 0x02);

    /* Put CC110x in IDLE mode to flush the FIFO */
    cc1200_strobe(dev, CC1200_SIDLE);
    /* Flush TX FIFO to be sure it is empty */
    cc1200_strobe(dev, CC1200_SFTX);

    memcpy((char*)&dev->pkt_buf.packet, packet, size);
    dev->pkt_buf.pos = 0;

    _tx_continue(dev);


    return size;
}
