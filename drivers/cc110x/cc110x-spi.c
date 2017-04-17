/*
 * Copyright (C) 2014 Freie Universität Berlin
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_cc110x
 * @{
 *
 * @file
 * @brief       TI Chipcon CC110x spi driver
 *
 * @author      Thomas Hillebrandt <hillebra@inf.fu-berlin.de>
 * @author      Heiko Will <hwill@inf.fu-berlin.de>
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 * @author      Joakim Gebart <joakim.gebart@eistec.se>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @}
 */

#include <stdio.h>

#include "cc110x.h"
#include "cc110x-spi.h"
#include "cc110x-internal.h"
#include "cc110x-defines.h"

#include "periph/gpio.h"
#include "periph/spi.h"

#include "xtimer.h"
#include "irq.h"

#define ENABLE_DEBUG 1
#include "debug.h"

#define SPI_CLK         SPI_CLK_5MHZ
#define SPI_MODE        SPI_MODE_0

/**********************************************************************
 *                      CC110x spi access
 **********************************************************************/

static inline void lock(cc110x_t *dev)
{
    spi_acquire(dev->params.spi, dev->params.cs, SPI_MODE, SPI_CLK);
}

void cc110x_cs(cc110x_t *dev)
{
#ifdef MODULE_CC1200
    //gpio_clear(dev->params.cs);
    return;
#endif 
    volatile int retry_count = 0;
    /* Switch MISO/GDO1 to GPIO input mode */
#ifndef GPIO_READS_SPI_PINS
    gpio_init(dev->params.gdo1, GPIO_IN);
#endif
    /* CS to low */
    gpio_clear(dev->params.cs);
    /* Wait for SO to go low (voltage regulator
     * has stabilized and the crystal is running) */
    while (gpio_read(dev->params.gdo1)) {
        /* Wait ~500us and try again */
        xtimer_usleep(CS_SO_WAIT_TIME);

        if (gpio_read(dev->params.gdo1)) {
            retry_count++;

            if (retry_count > CC110X_GDO1_LOW_RETRY) {
                puts("[CC110X spi] fatal error\n");
                break;
            }

            gpio_set(dev->params.cs);
            gpio_clear(dev->params.cs);
        }
    }
    /* Switch MISO/GDO1 to spi mode */
#ifndef GPIO_READS_SPI_PINS
    DEBUG("%s:%u\n", __func__, __LINE__);
    spi_init_pins(dev->params.spi);
#endif
}

#ifdef MODULE_CC1200
void cc110x_writeburst_reg(cc110x_t *dev, uint16_t addr, const char *src, uint8_t count)
#else
void cc110x_writeburst_reg(cc110x_t *dev, uint8_t addr, const char *src, uint8_t count)
#endif
{
    unsigned int cpsr;
    lock(dev);
    cpsr = irq_disable();
    cc110x_cs(dev);
    if((addr >> 8) == 0x2F){
        spi_transfer_byte(dev->params.spi, dev->params.cs, true, 0x2F | CC110X_WRITE_BURST);
    }
#ifndef MODULE_CC1200
    spi_transfer_regs(dev->params.spi, SPI_CS_UNDEF,
                      (addr | CC110X_WRITE_BURST), src, NULL, count);
    gpio_set(dev->params.cs);
#else
    spi_transfer_regs(dev->params.spi, dev->params.cs,
                      (addr | CC110X_WRITE_BURST), src, NULL, count);
#endif
    irq_restore(cpsr);
    spi_release(dev->params.spi);
}

#ifdef MODULE_CC1200
void cc110x_readburst_reg(cc110x_t *dev, uint16_t addr, char *buffer, uint8_t count)
#else
void cc110x_readburst_reg(cc110x_t *dev, uint8_t addr, char *buffer, uint8_t count)
#endif
{
    int i = 0;
    unsigned int cpsr;
    lock(dev);
    cpsr = irq_disable();
    cc110x_cs(dev);
    if((addr >> 8) == 0x2F){
        spi_transfer_byte(dev->params.spi, dev->params.cs, true, 0x2F | CC110X_READ_BURST);
    }
#ifndef MODULE_CC1200
    spi_transfer_byte(dev->params.spi, SPI_CS_UNDEF, false,
                      (addr | CC110X_READ_BURST));
    while (i < count) {
        buffer[i] = (char)spi_transfer_byte(dev->params.spi, SPI_CS_UNDEF,
                                            false, CC110X_NOBYTE);
        i++;
    }
    gpio_set(dev->params.cs);
#else
    spi_transfer_byte(dev->params.spi, dev->params.cs, false,
                      (addr | CC110X_READ_BURST));
    while (i < count) {
        buffer[i] = (char)spi_transfer_byte(dev->params.spi, dev->params.cs,
                                            false, CC110X_NOBYTE);
        i++;
    }
#endif
    irq_restore(cpsr);
    spi_release(dev->params.spi);
}

#ifdef MODULE_CC1200
void cc110x_write_reg(cc110x_t *dev, uint16_t addr, uint8_t value)
#else
void cc110x_write_reg(cc110x_t *dev, uint8_t addr, uint8_t value)
#endif
{
    unsigned int cpsr;
    lock(dev);
    cpsr = irq_disable();
    cc110x_cs(dev);
    if((addr >> 8) == 0x2F){
        spi_transfer_byte(dev->params.spi, dev->params.cs, true, 0x2F);
    }
#ifndef MODULE_CC1200
    spi_transfer_reg(dev->params.spi, SPI_CS_UNDEF, addr, value);
    gpio_set(dev->params.cs);
#else
    spi_transfer_reg(dev->params.spi, dev->params.cs, addr, value);
#endif
    irq_restore(cpsr);
    spi_release(dev->params.spi);
}

#ifdef MODULE_CC1200
uint8_t cc110x_read_reg(cc110x_t *dev, uint16_t addr)
#else
uint8_t cc110x_read_reg(cc110x_t *dev, uint8_t addr)
#endif
{
    uint8_t result;
    unsigned int cpsr;
    lock(dev);
    cpsr = irq_disable();
    cc110x_cs(dev);
    if((addr >> 8) == 0x2F){
        spi_transfer_byte(dev->params.spi, dev->params.cs, true, 0x2F | CC110X_READ_SINGLE);
    }
#ifndef MODULE_CC1200
    result = spi_transfer_reg(dev->params.spi, SPI_CS_UNDEF,
                              (addr | CC110X_READ_SINGLE), CC110X_NOBYTE);
    gpio_set(dev->params.cs);
#else
    result = spi_transfer_reg(dev->params.spi, dev->params.cs,
                              (addr | CC110X_READ_SINGLE), CC110X_NOBYTE);
#endif
    irq_restore(cpsr);
    spi_release(dev->params.spi);
    return result;
}

#ifdef MODULE_CC1200
uint8_t cc110x_read_status(cc110x_t *dev, uint16_t addr)
#else
uint8_t cc110x_read_status(cc110x_t *dev, uint8_t addr)
#endif
{
    uint8_t result;
    unsigned int cpsr;
    lock(dev);
    cpsr = irq_disable();
    cc110x_cs(dev);
    if((addr >> 8) == 0x2F){
        spi_transfer_byte(dev->params.spi, dev->params.cs, true, 0x2F | CC110X_READ_BURST);
    }
#ifndef MODULE_CC1200
    result = spi_transfer_reg(dev->params.spi, SPI_CS_UNDEF,
                              (addr | CC110X_READ_BURST), CC110X_NOBYTE);
    gpio_set(dev->params.cs);
#else
    result = spi_transfer_reg(dev->params.spi, dev->params.cs,
                              (addr | CC110X_READ_BURST), CC110X_NOBYTE);
#endif
    irq_restore(cpsr);
    spi_release(dev->params.spi);
    return (uint8_t) result;
}

#ifdef MODULE_CC1200
uint8_t cc110x_get_reg_robust(cc110x_t *dev, uint16_t addr)
#else
uint8_t cc110x_get_reg_robust(cc110x_t *dev, uint8_t addr)
#endif
{
    uint8_t res1, res2;
    unsigned int cpsr;
    lock(dev);
    cpsr = irq_disable();
    cc110x_cs(dev);
    if((addr >> 8) == 0x2F){
        spi_transfer_byte(dev->params.spi, dev->params.cs, true, 0x2F | CC110X_READ_BURST);
    }
#ifndef MODULE_CC1200
    do {
        res1 = spi_transfer_reg(dev->params.spi, SPI_CS_UNDEF,
                                (addr | CC110X_READ_BURST), CC110X_NOBYTE);
        res2 = spi_transfer_reg(dev->params.spi, SPI_CS_UNDEF,
                                (addr | CC110X_READ_BURST), CC110X_NOBYTE);
    } while (res1 != res2);
    gpio_set(dev->params.cs);
#else
    do {
        res1 = spi_transfer_reg(dev->params.spi, dev->params.cs,
                                (addr | CC110X_READ_BURST), CC110X_NOBYTE);
        res2 = spi_transfer_reg(dev->params.spi, dev->params.cs,
                                (addr | CC110X_READ_BURST), CC110X_NOBYTE);
    } while (res1 != res2);
#endif
    irq_restore(cpsr);
    spi_release(dev->params.spi);
    return res1;
}

uint8_t cc110x_strobe(cc110x_t *dev, uint8_t c)
{
    DEBUG("%s:%u strobe: 0x%x\n", __func__, __LINE__, (unsigned int)c);
#ifdef CC110X_DONT_RESET
    if (c == CC110X_SRES) {
        return 0;
    }
#endif

    uint8_t result;
    unsigned int cpsr;
    lock(dev);
    cpsr = irq_disable();
    cc110x_cs(dev);
    result = spi_transfer_byte(dev->params.spi, dev->params.cs, false,  c);
    if(c == CC110X_SRES){
        gpio_clear(dev->params.cs);
        result = spi_transfer_byte(dev->params.spi, SPI_CS_UNDEF, false,  c);
        xtimer_spin(xtimer_ticks_from_usec(RESET_WAIT_TIME));
        gpio_set(dev->params.cs);
    }else{
        result = spi_transfer_byte(dev->params.spi, dev->params.cs, false,  c);
    }
    irq_restore(cpsr);
    spi_release(dev->params.spi);
    return result;
}
