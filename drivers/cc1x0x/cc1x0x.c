/*
 * Copyright (C) 2014 Freie Universität Berlin
 * Copyright (C) 2013 INRIA
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_cc1x0x
 * @{
 * @file
 * @brief       Basic functionality of cc1x0x driver
 *
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @}
 */

#include "board.h"
#include "periph/cpuid.h"
#include "periph/gpio.h"
#include "periph/spi.h"
#include "xtimer.h"
#include "cpu.h"
#include "log.h"

#include "cc1x0x.h"
#include "cc1x0x-defaultsettings.h"
#include "cc1x0x-defines.h"
#include "cc1x0x-interface.h"
#include "cc1x0x-internal.h"
#include "cc1x0x-spi.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

/* Internal function prototypes */
#ifndef CC1X0X_DONT_RESET
static void _reset(cc1x0x_t *dev);
static void _power_up_reset(cc1x0x_t *dev);
#endif

int cc1x0x_setup(cc1x0x_t *dev, const cc1x0x_params_t *params)
{
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);

#ifdef MODULE_CC1X0X_HOOKS
    cc1x0x_hooks_init();
#endif

    dev->params = *params;

    /* Configure chip-select */
    int res = gpio_init(dev->params.cs, GPIO_OUT);
    DEBUG("%s:%s:%u res=%i\n", RIOT_FILE_RELATIVE, __func__, __LINE__, res);
    gpio_set(dev->params.cs);

    /* Configure GDO1 */
    //gpio_init(dev->params.gdo1, GPIO_IN);

    /* Configure SPI */
    spi_acquire(dev->params.spi);
    spi_init_master(dev->params.spi, SPI_CONF_FIRST_RISING, SPI_SPEED_5MHZ);
    spi_release(dev->params.spi);

#ifndef CC1X0X_DONT_RESET
    /* reset device*/
    _power_up_reset(dev);
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
#endif

    /* set default state */
    dev->radio_state = RADIO_IDLE;

    /* Write configuration to configuration registers */
    cc1x0x_writeburst_reg(dev, 0x00, cc1x0x_default_conf, cc1x0x_default_conf_size);

#ifdef MODULE_CC1200
    /* Write extended config for CC1200 */
    cc1x0x_write_reg(dev, CC1X0X_IF_MIX_CFG, 0x18);
    cc1x0x_write_reg(dev, CC1X0X_TOC_CFG, 0x03);
    cc1x0x_write_reg(dev, CC1X0X_MDMCFG2, 0x02);
    cc1x0x_write_reg(dev, CC1X0X_FREQ2, 0x56);
    cc1x0x_write_reg(dev, CC1X0X_FREQ1, 0xCC);
    cc1x0x_write_reg(dev, CC1X0X_FREQ0, 0xCC);
    cc1x0x_write_reg(dev, CC1X0X_IF_ADC1, 0xEE);
    cc1x0x_write_reg(dev, CC1X0X_IF_ADC0, 0x10);
    cc1x0x_write_reg(dev, CC1X0X_FS_DIG1, 0x18);
    cc1x0x_write_reg(dev, CC1X0X_FS_DIG0, 0x50);
    cc1x0x_write_reg(dev, CC1X0X_FS_CAL1, 0x04);
    cc1x0x_write_reg(dev, CC1X0X_FS_CAL0, 0x0E);
    cc1x0x_write_reg(dev, CC1X0X_FS_DIVTWO, 0x03);
    cc1x0x_write_reg(dev, CC1X0X_FS_DSM0, 0x33);
    cc1x0x_write_reg(dev, CC1X0X_FS_DVC1, 0xF7);
    cc1x0x_write_reg(dev, CC1X0X_FS_DVC0, 0x0F);
    cc1x0x_write_reg(dev, CC1X0X_FS_PFD, 0x00);
    cc1x0x_write_reg(dev, CC1X0X_FS_PRE, 0x6E);
    cc1x0x_write_reg(dev, CC1X0X_FS_REG_DIV_CML, 0x1C);
    cc1x0x_write_reg(dev, CC1X0X_FS_SPARE, 0xAC);
    cc1x0x_write_reg(dev, CC1X0X_FS_VCO0, 0xB5);
    cc1x0x_write_reg(dev, CC1X0X_IFAMP, 0x05);
    cc1x0x_write_reg(dev, CC1X0X_XOSC5, 0x0E);
    cc1x0x_write_reg(dev, CC1X0X_XOSC1, 0x03);
    cc1x0x_write_reg(dev, CC1X0X_AGC_GAIN_ADJUST, CC1X0X_RF_CFG_RSSI_OFFSET);
    cc1x0x_write_reg(dev, CC1X0X_TXFIRST, 0);
    cc1x0x_write_reg(dev, CC1X0X_TXLAST, 0xFF);
    //cc1x0x_write_reg(dev, CC1X0X_PKT_CFG2, 0x02);
#else
    /* Write PATABLE (power settings) */
    cc1x0x_writeburst_reg(dev, CC1X0X_PATABLE, CC1X0X_DEFAULT_PATABLE, 8);

    /* set base frequency */
    cc1x0x_set_base_freq_raw(dev, CC1X0X_DEFAULT_FREQ);

    /* Set default channel number */
    cc1x0x_set_channel(dev, CC1X0X_DEFAULT_CHANNEL);
#endif

    /* set default node id */
#if CPUID_LEN
    char cpuid[CPUID_LEN];
    cpuid_get(cpuid);
    for (int i = 1; i < CPUID_LEN; i++) {
        cpuid[0] ^= cpuid[i];
    }
    cc1x0x_set_address(dev, (uint8_t) cpuid[0]);
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
#endif

    LOG_INFO("cc1x0x: initialized with address=%u and channel=%i\n",
            (unsigned)dev->radio_address,
            dev->radio_channel);


    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);

    return 0;
}

uint8_t cc1x0x_set_address(cc1x0x_t *dev, uint8_t address)
{
    DEBUG("%s:%s:%u setting address %u\n", RIOT_FILE_RELATIVE, __func__,
            __LINE__, (unsigned)address);
    if (!(address < MIN_UID) || (address > MAX_UID)) {
        if (dev->radio_state != RADIO_UNKNOWN) {
#ifndef MODULE_CC1200
            cc1x0x_write_register(dev, CC1X0X_ADDR, address);
#else
            cc1x0x_write_register(dev, CC1X0X_DEV_ADDR, address);
#endif
            dev->radio_address = address;
            return address;
        }
    }

    return 0;
}

void cc1x0x_set_base_freq_raw(cc1x0x_t *dev, const char* freq_array)
{
#if ENABLE_DEBUG == 1
    uint8_t _tmp[] = { freq_array[2], freq_array[1], freq_array[0], 0x00};
    uint32_t *FREQ = (uint32_t*) _tmp;

    DEBUG("cc1x0x_set_base_freq_raw(): setting base frequency to %uHz\n",
            (26000000>>16) * (unsigned)(*FREQ));
#endif
    cc1x0x_writeburst_reg(dev, CC1X0X_FREQ2, freq_array, 3);
}

void cc1x0x_set_monitor(cc1x0x_t *dev, uint8_t mode)
{
#ifndef MODULE_CC1200
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);

    cc1x0x_write_register(dev, CC1X0X_PKTCTRL1, mode ? 0x04 : 0x06);
#endif
}

void cc1x0x_setup_rx_mode(cc1x0x_t *dev)
{
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);

    /* Stay in RX mode until end of packet */
#ifndef MODULE_CC1200
    cc1x0x_write_reg(dev, CC1X0X_MCSM2, 0x07);
#endif
    cc1x0x_switch_to_rx(dev);
}

void cc1x0x_switch_to_rx(cc1x0x_t *dev)
{
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);

#ifdef MODULE_CC1X0X_HOOKS
    cc1x0x_hook_rx();
#endif

    gpio_irq_disable(dev->params.gdo2);

    /* flush RX fifo */
    cc1x0x_strobe(dev, CC1X0X_SIDLE);
    cc1x0x_strobe(dev, CC1X0X_SFRX);

    dev->radio_state = RADIO_RX;

    cc1x0x_write_reg(dev, CC1X0X_IOCFG2, 0x6);
    cc1x0x_strobe(dev, CC1X0X_SRX);

    gpio_irq_enable(dev->params.gdo2);
    //DEBUG("%s:%s:%u dev->radio_state=%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__, dev->radio_state);
}

void cc1x0x_wakeup_from_rx(cc1x0x_t *dev)
{
    if (dev->radio_state != RADIO_RX) {
        return;
    }

    LOG_DEBUG("cc1x0x: switching to idle mode\n");

    cc1x0x_strobe(dev, CC1X0X_SIDLE);
    dev->radio_state = RADIO_IDLE;
}

void cc1x0x_switch_to_pwd(cc1x0x_t *dev)
{
    LOG_DEBUG("cc1x0x: switching to powerdown mode\n");
    cc1x0x_wakeup_from_rx(dev);
    cc1x0x_strobe(dev, CC1X0X_SPWD);
    dev->radio_state = RADIO_PWD;

#ifdef MODULE_CC1X0X_HOOKS
     cc1x0x_hook_off();
#endif
}

#ifndef MODULE_CC1X0X_HOOKS
int16_t cc1x0x_set_channel(cc1x0x_t *dev, uint8_t channr)
{
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);

    if (channr > MAX_CHANNR) {
        return -1;
    }

    DEBUG("%s:%s:%u channel = %u\n", RIOT_FILE_RELATIVE, __func__, __LINE__, channr);
#ifndef MODULE_CC1200
    cc1x0x_write_register(dev, CC1X0X_CHANNR, channr * 10);
#else
    uint32_t freq;
    freq = CC1X0X_RF_CFG_CHAN_CENTER_F0 + (channr * CC1X0X_RF_CFG_CHAN_SPACING) / 1000 /* /1000 because chan_spacing is in Hz */;
    freq *= 4096; //Frequency Multiplier
    freq /= 625; //Frequency Divider
    cc1x0x_write_reg(dev, CC1X0X_FREQ2, ((uint8_t *)&freq)[2]);
    cc1x0x_write_reg(dev, CC1X0X_FREQ1, ((uint8_t *)&freq)[1]);
    cc1x0x_write_reg(dev, CC1X0X_FREQ0, ((uint8_t *)&freq)[0]);
#endif
    dev->radio_channel = channr;
    DEBUG("%s:%s:%u channel = %u\n", RIOT_FILE_RELATIVE, __func__, __LINE__, dev->radio_channel);

    return channr;
}
#endif

#ifndef CC1X0X_DONT_RESET
static void _reset(cc1x0x_t *dev)
{
    cc1x0x_wakeup_from_rx(dev);
    cc1x0x_cs(dev);
    cc1x0x_strobe(dev, CC1X0X_SRES);
    xtimer_usleep(100);
}

static void _power_up_reset(cc1x0x_t *dev)
{
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
    gpio_set(dev->params.cs);
    gpio_clear(dev->params.cs);
    gpio_set(dev->params.cs);
    xtimer_usleep(RESET_WAIT_TIME);
    _reset(dev);
}
#endif

void cc1x0x_write_register(cc1x0x_t *dev, uint8_t r, uint8_t value)
{
    /* Save old radio state */
    uint8_t old_state = dev->radio_state;

    /* Wake up from RX (no effect if in other mode) */
    cc1x0x_wakeup_from_rx(dev);
    cc1x0x_write_reg(dev, r, value);

    /* Have to put radio back to RX if old radio state
     * was RX, otherwise no action is necessary */
    if (old_state == RADIO_RX) {
        cc1x0x_switch_to_rx(dev);
    }
}

int cc1x0x_rd_set_mode(cc1x0x_t *dev, int mode)
{
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);

    int result;

    /* Get current radio mode */
    if ((dev->radio_state == RADIO_UNKNOWN) || (dev->radio_state == RADIO_PWD)) {
        result = RADIO_MODE_OFF;
    }
    else {
        result = RADIO_MODE_ON;
    }

    switch(mode) {
        case RADIO_MODE_ON:
            LOG_DEBUG("cc1x0x: switching to RX mode\n");
            cc1x0x_setup_rx_mode(dev);          /* Set chip to desired mode */
            break;

        case RADIO_MODE_OFF:
            gpio_irq_disable(dev->params.gdo2); /* Disable interrupts */
            cc1x0x_switch_to_pwd(dev);          /* Set chip to power down mode */
            break;

        case RADIO_MODE_GET:
            /* do nothing, just return current mode */
        default:
            /* do nothing */
            break;
    }

    /* Return previous mode */
    return result;
}
