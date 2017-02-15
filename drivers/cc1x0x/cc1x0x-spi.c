/*
 * Copyright (C) 2014 Freie Universität Berlin
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
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

#include "cc1x0x.h"
#include "cc1x0x-spi.h"
#include "cc1x0x-internal.h"
#include "cc1x0x-defines.h"

#include "periph/gpio.h"
#include "periph/spi.h"

#include "xtimer.h"
#include "irq.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"
#include "panic.h"

/**********************************************************************
 *                      CC110x spi access
 **********************************************************************/

void cc1x0x_cs(cc1x0x_t *dev)
{
    unsigned int cpsr = irq_disable();
    volatile int retry_count = 0;
    /* Switch MISO/GDO1 to GPIO input mode */
    //DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
#ifndef GPIO_READS_SPI_PINS
    //DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
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

            if (retry_count > CC1X0X_GDO1_LOW_RETRY) {
                puts("[CC1X0X spi] fatal error\n");
                break;
            }

            gpio_set(dev->params.cs);
            gpio_clear(dev->params.cs);
        }
    }
    /* Switch MISO/GDO1 to spi mode */
#ifndef GPIO_READS_SPI_PINS
    spi_conf_pins(dev->params.spi);
#endif
    irq_restore(cpsr);
}

void cc1x0x_writeburst_reg(cc1x0x_t *dev, uint16_t addr, const char *src, uint8_t count)
{
    unsigned int cpsr;
    spi_acquire(dev->params.spi);
    cpsr = irq_disable();
    //cc1x0x_cs(dev);
    gpio_clear(dev->params.cs);
    spi_transfer_regs(dev->params.spi, addr | CC1X0X_WRITE_BURST, (char *)src, 0, count);
    gpio_set(dev->params.cs);
    irq_restore(cpsr);
    spi_release(dev->params.spi);
}

void cc1x0x_readburst_reg(cc1x0x_t *dev, uint16_t addr, char *buffer, uint8_t count)
{
    int i = 0;
    unsigned int cpsr;
    spi_acquire(dev->params.spi);
    cpsr = irq_disable();
    //cc1x0x_cs(dev);
    gpio_clear(dev->params.cs);
    spi_transfer_byte(dev->params.spi, addr | CC1X0X_READ_BURST, 0);
    while (i < count) {
        spi_transfer_byte(dev->params.spi, CC1X0X_NOBYTE, &buffer[i]);
        i++;
    }
    gpio_set(dev->params.cs);
    irq_restore(cpsr);
    spi_release(dev->params.spi);
}

void cc1x0x_write_reg(cc1x0x_t *dev, uint16_t addr, uint8_t value)
{
    unsigned int cpsr;
    spi_acquire(dev->params.spi);
    cpsr = irq_disable();
    //cc1x0x_cs(dev);
    gpio_clear(dev->params.cs);
    spi_transfer_reg(dev->params.spi, addr, value, 0);
    gpio_set(dev->params.cs);
    irq_restore(cpsr);
    spi_release(dev->params.spi);
}

uint8_t cc1x0x_read_reg(cc1x0x_t *dev, uint16_t addr)
{
    char result;
    unsigned int cpsr;
    spi_acquire(dev->params.spi);
    cpsr = irq_disable();
    //cc1x0x_cs(dev);
    gpio_clear(dev->params.cs);
    spi_transfer_reg(dev->params.spi, addr | CC1X0X_READ_SINGLE, CC1X0X_NOBYTE, &result);
    gpio_set(dev->params.cs);
    irq_restore(cpsr);
    spi_release(dev->params.spi);
    return (uint8_t) result;
}

uint8_t cc1x0x_read_status(cc1x0x_t *dev, uint16_t addr)
{
    char result;
    unsigned int cpsr;
    spi_acquire(dev->params.spi);
    cpsr = irq_disable();
    //cc1x0x_cs(dev);
    gpio_clear(dev->params.cs);
    spi_transfer_reg(dev->params.spi, addr | CC1X0X_READ_BURST, CC1X0X_NOBYTE, &result);
    gpio_set(dev->params.cs);
    irq_restore(cpsr);
    spi_release(dev->params.spi);
    return (uint8_t) result;
}

uint8_t cc1x0x_get_reg_robust(cc1x0x_t *dev, uint16_t addr)
{
    char result, result2;
    unsigned int cpsr;
    spi_acquire(dev->params.spi);
    cpsr = irq_disable();
    //cc1x0x_cs(dev);
    gpio_clear(dev->params.cs);
    do {
        spi_transfer_reg(dev->params.spi, addr | CC1X0X_READ_BURST, CC1X0X_NOBYTE, &result);
        spi_transfer_reg(dev->params.spi, addr | CC1X0X_READ_BURST, CC1X0X_NOBYTE, &result2);
    } while (result != result2);
    gpio_set(dev->params.cs);
    irq_restore(cpsr);
    spi_release(dev->params.spi);
    return (uint8_t) result;
}

uint8_t cc1x0x_strobe(cc1x0x_t *dev, uint8_t c)
{
#ifdef CC1X0X_DONT_RESET
    if (c == CC1X0X_SRES) {
        return 0;
    }
#endif

    char result;
    unsigned int cpsr;
    spi_acquire(dev->params.spi);
    cpsr = irq_disable();
    //cc1x0x_cs(dev);
    gpio_clear(dev->params.cs);
    spi_transfer_byte(dev->params.spi, c, &result);
    /* When calling SRES wait for SO to go low before
       toggling CS */
    if (c == CC1X0X_SRES) {
        DEBUG("%s:%s:%u waiting for sres\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
#ifndef GPIO_READS_SPI_PINS
        gpio_init(dev->params.gdo1, GPIO_IN);
#endif
         /*Wait for SO to go low  */
        while (gpio_read(dev->params.gdo1)) {};
         /*Switch MISO/GDO1 to spi mode */
#ifndef GPIO_READS_SPI_PINS
        spi_conf_pins(dev->params.spi);
#endif
    }
    gpio_set(dev->params.cs);
    spi_release(dev->params.spi);
    irq_restore(cpsr);
    return (uint8_t) result;
}
