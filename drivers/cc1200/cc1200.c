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
 * @ingroup     drivers_cc1200
 * @{
 * @file
 * @brief       Basic functionality of cc1200 driver
 *
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @}
 */

#include "luid.h"
#include "board.h"
#include "periph/gpio.h"
#include "periph/spi.h"
#include "xtimer.h"
#include "cpu.h"
#include "log.h"

#include "cc1200.h"
#include "cc1200-defaultsettings.h"
#include "cc1200-defines.h"
#include "cc1200-interface.h"
#include "cc1200-internal.h"
#include "cc1200-spi.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

/* Internal function prototypes */
#ifndef CC1200_DONT_RESET
static void _reset(cc1200_t *dev);
static void _power_up_reset(cc1200_t *dev);
#endif

int cc1200_setup(cc1200_t *dev, const cc1200_params_t *params)
{
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);

#ifdef MODULE_CC1200_HOOKS
    cc1200_hooks_init();
#endif


    dev->params = *params;

    /* Configure chip-select */
    spi_init(dev->params.spi);
    int spi_return = spi_init_cs(dev->params.spi, dev->params.cs);
    if(spi_return != SPI_OK){
        DEBUG("%s:%s:%u spi not ok\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
    }


#ifndef MODULE_CC1200
    /* Configure GDO1 */
    gpio_init(dev->params.gdo1, GPIO_IN);
#endif

#ifndef CC1200_DONT_RESET
    /* reset device*/
    _power_up_reset(dev);
#endif

    /* set default state */
    dev->radio_state = RADIO_IDLE;

    /* Write configuration to configuration registers */
    cc1200_writeburst_reg(dev, 0x00, cc1200_default_conf, cc1200_default_conf_size);

#ifdef MODULE_CC1200
    /* Write extended config for CC1200 */
    cc1200_write_reg(dev, CC1200_IF_MIX_CFG, 0x18);
    cc1200_write_reg(dev, CC1200_TOC_CFG, 0x03);
    cc1200_write_reg(dev, CC1200_MDMCFG2, 0x02);
    cc1200_write_reg(dev, CC1200_FREQ2, 0x56);
    cc1200_write_reg(dev, CC1200_FREQ1, 0xCC);
    cc1200_write_reg(dev, CC1200_FREQ0, 0xCC);
    cc1200_write_reg(dev, CC1200_IF_ADC1, 0xEE);
    cc1200_write_reg(dev, CC1200_IF_ADC0, 0x10);
    cc1200_write_reg(dev, CC1200_FS_DIG1, 0x04);
    cc1200_write_reg(dev, CC1200_FS_DIG0, 0x50);
    cc1200_write_reg(dev, CC1200_FS_CAL1, 0x40);
    cc1200_write_reg(dev, CC1200_FS_CAL0, 0x0E);
    cc1200_write_reg(dev, CC1200_FS_DIVTWO, 0x03);
    cc1200_write_reg(dev, CC1200_FS_DSM0, 0x33);
    cc1200_write_reg(dev, CC1200_FS_DVC1, 0xF7);
    cc1200_write_reg(dev, CC1200_FS_DVC0, 0x0F);
    cc1200_write_reg(dev, CC1200_FS_PFD, 0x00);
    cc1200_write_reg(dev, CC1200_FS_PRE, 0x6E);
    cc1200_write_reg(dev, CC1200_FS_REG_DIV_CML, 0x1C);
    cc1200_write_reg(dev, CC1200_FS_SPARE, 0xAC);
    cc1200_write_reg(dev, CC1200_FS_VCO0, 0xB5);
    cc1200_write_reg(dev, CC1200_IFAMP, 0x05);
    cc1200_write_reg(dev, CC1200_XOSC5, 0x0E);
    cc1200_write_reg(dev, CC1200_XOSC1, 0x03);
    cc1200_write_reg(dev, CC1200_AGC_CS_THR, (uint8_t) -91);
    cc1200_write_reg(dev, CC1200_AGC_GAIN_ADJUST, (int8_t)CC1200_RF_CFG_RSSI_OFFSET);
    cc1200_set_channel(dev, 26);
    DEBUG("READING CC1200_PKT_CFG2: 0x%x\n", cc1200_read_reg(dev, CC1200_PKT_CFG2));
    DEBUG("READING CC1200_MDMCFG1: 0x%x\n", cc1200_read_reg(dev, CC1200_MDMCFG1));

    /* set default node id */
    uint64_t addr;
    luid_get(&addr, 8);
    cc1200_set_address_long(dev, addr);
    cc1200_set_address_short(dev, (uint16_t)addr & 0xFFFF);
    cc1200_set_address(dev, (uint8_t) addr & 0xFF);
#else
    /* Write PATABLE (power settings) */
    cc1200_writeburst_reg(dev, CC1200_PATABLE, CC1200_DEFAULT_PATABLE, 8);

    /* set base frequency */
    cc1200_set_base_freq_raw(dev, CC1200_DEFAULT_FREQ);

    /* Set default channel number */
    cc1200_set_channel(dev, CC1200_DEFAULT_CHANNEL);

    /* set default node id */
    uint8_t addr;
    luid_get(&addr, 1);
    cc1200_set_address(dev, addr);
#endif


    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
    LOG_INFO("cc1200: initialized with address=%u and channel=%i\n",
            (unsigned)dev->radio_address,
            dev->radio_channel);
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
    return 0;
}

uint8_t cc1200_set_address(cc1200_t *dev, uint8_t address)
{
    DEBUG("%s:%s:%u setting address %u\n", RIOT_FILE_RELATIVE, __func__,
            __LINE__, (unsigned)address);
    if (!(address < MIN_UID) || (address > MAX_UID)) {
        if (dev->radio_state != RADIO_UNKNOWN) {
#ifndef MODULE_CC1200
            cc1200_write_register(dev, CC1200_ADDR, address);
#else
            cc1200_write_register(dev, CC1200_DEV_ADDR, address);
#endif           
            dev->radio_address = address;
            return address;
        }
    }

    return 0;
}

#ifdef MODULE_CC1200
uint64_t cc1200_set_address_long(cc1200_t *dev, uint64_t address)
{
    DEBUG("%s:%s:%u setting address long 0x%x\n", RIOT_FILE_RELATIVE, __func__,
            __LINE__, (unsigned)address);
    if (!(address < MIN_UID) || (address > 0xFFFFFFFFFFFFFFFF)) {
        if (dev->radio_state != RADIO_UNKNOWN) {
      
            dev->radio_address_long = address;
            return address;
        }
    }

    return 0;
}

uint16_t cc1200_set_address_short(cc1200_t *dev, uint16_t address)
{
    DEBUG("%s:%s:%u setting address short 0x%x\n", RIOT_FILE_RELATIVE, __func__,
            __LINE__, (unsigned)address);
    if (!(address < MIN_UID) || (address > 0xFFFF)) {
        if (dev->radio_state != RADIO_UNKNOWN) {
      
            dev->radio_address_short = address;
            return address;
        }
    }

    return 0;
}

uint16_t cc1200_set_pan_id(cc1200_t *dev, uint16_t pan_id)
{
    DEBUG("%s:%s:%u setting pan id 0x%x\n", RIOT_FILE_RELATIVE, __func__,
            __LINE__, (unsigned)pan_id);
    if (!(pan_id < MIN_UID) || (pan_id > 0xFF)) {
        if (dev->radio_state != RADIO_UNKNOWN) {
      
            dev->pan_id = pan_id;
            return pan_id;
        }
    }

    return 0;
}
#endif /* MODULE_CC1200 */

void cc1200_set_base_freq_raw(cc1200_t *dev, const char* freq_array)
{
#if ENABLE_DEBUG == 1
    uint8_t _tmp[] = { freq_array[2], freq_array[1], freq_array[0], 0x00};
    uint32_t *FREQ = (uint32_t*) _tmp;

    DEBUG("cc1200_set_base_freq_raw(): setting base frequency to %uHz\n",
            (26000000>>16) * (unsigned)(*FREQ));
#endif
    cc1200_writeburst_reg(dev, CC1200_FREQ2, freq_array, 3);
}

void cc1200_set_monitor(cc1200_t *dev, uint8_t mode)
{
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);

#ifndef MODULE_CC1200
    cc1200_write_register(dev, CC1200_PKTCTRL1, mode ? 0x04 : 0x06);
#else
    uint8_t stat = cc1200_read_reg(dev, CC1200_PKT_CFG1);
    stat &= 0xE6;
    if(mode){
        stat |= 0x1;
    }else{
        stat |= 0x11;
    }
    cc1200_write_register(dev, CC1200_PKT_CFG1, stat);
#endif


}

void cc1200_setup_rx_mode(cc1200_t *dev)
{
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);

    /* Stay in RX mode until end of packet */
#ifndef MODULE_CC1200
    cc1200_write_reg(dev, CC1200_MCSM2, 0x07);
#else
    uint8_t stat = cc1200_read_reg(dev, CC1200_RFEND_CFG1);
    DEBUG("%s:%s:%u CC1200_RFEND_CFG1: %u\n", RIOT_FILE_RELATIVE, __func__, __LINE__, stat);
    stat |= 0xE;
    cc1200_write_reg(dev, CC1200_RFEND_CFG1, 0xE);
#endif
    cc1200_switch_to_rx(dev);
}

void cc1200_switch_to_rx(cc1200_t *dev)
{
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);

#ifdef MODULE_CC1200_HOOKS
    cc1200_hook_rx();
#endif

    gpio_irq_disable(dev->params.gdo2);

    /* flush RX fifo */
    cc1200_strobe(dev, CC1200_SIDLE);
    cc1200_strobe(dev, CC1200_SFRX);

    dev->radio_state = RADIO_RX;

#ifdef MODULE_CC1200
    cc1200_write_reg(dev, CC1200_IOCFG2, 0x6);
#else
    cc1200_write_reg(dev, CC1200_IOCFG2, 0x6);
#endif
    cc1200_strobe(dev, CC1200_SRX);

    gpio_irq_enable(dev->params.gdo2);
}

void cc1200_wakeup_from_rx(cc1200_t *dev)
{
    if (dev->radio_state != RADIO_RX) {
        return;
    }

    LOG_DEBUG("cc1200: switching to idle mode\n");

    cc1200_strobe(dev, CC1200_SIDLE);
    dev->radio_state = RADIO_IDLE;
}

void cc1200_switch_to_pwd(cc1200_t *dev)
{
    LOG_DEBUG("cc1200: switching to powerdown mode\n");
    cc1200_wakeup_from_rx(dev);
    cc1200_strobe(dev, CC1200_SPWD);
    dev->radio_state = RADIO_PWD;

#ifdef MODULE_CC1200_HOOKS
     cc1200_hook_off();
#endif
}

#ifndef MODULE_CC1200_HOOKS
int16_t cc1200_set_channel(cc1200_t *dev, uint8_t channr)
{
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);

    if (channr > MAX_CHANNR) {
        return -1;
    }
#ifndef MODULE_CC1200
    cc1200_write_register(dev, CC1200_CHANNR, channr * 10);
#else
    uint32_t freq;
    freq = CC1200_RF_CFG_CHAN_CENTER_F0 + (channr * CC1200_RF_CFG_CHAN_SPACING) / 1000 /* /1000 because chan_spacing is in Hz */;
    freq *= 4096; //Frequency Multiplier
    freq /= 625; //Frequency Divider
    cc1200_write_reg(dev, CC1200_FREQ2, ((uint8_t *)&freq)[2]);
    cc1200_write_reg(dev, CC1200_FREQ1, ((uint8_t *)&freq)[1]);
    cc1200_write_reg(dev, CC1200_FREQ0, ((uint8_t *)&freq)[0]);
#endif
    dev->radio_channel = channr;
    cc1200_switch_to_rx(dev);

    return channr;
}
#endif

#ifndef CC1200_DONT_RESET
static void _reset(cc1200_t *dev)
{
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
    cc1200_wakeup_from_rx(dev);
    cc1200_cs(dev);
    cc1200_strobe(dev, CC1200_SRES);
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
    xtimer_usleep(RESET_WAIT_TIME);
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
}

static void _power_up_reset(cc1200_t *dev)
{
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
    gpio_set(dev->params.cs);
    gpio_clear(dev->params.cs);
    gpio_set(dev->params.cs);
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
    xtimer_usleep(RESET_WAIT_TIME);
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
    _reset(dev);
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
}
#endif

#ifdef MODULE_CC1200
void cc1200_write_register(cc1200_t *dev, uint16_t r, uint8_t value)
#else
void cc1200_write_register(cc1200_t *dev, uint8_t r, uint8_t value)
#endif
{
    /* Save old radio state */
    uint8_t old_state = dev->radio_state;

    /* Wake up from RX (no effect if in other mode) */
    cc1200_wakeup_from_rx(dev);
    cc1200_write_reg(dev, r, value);

    /* Have to put radio back to RX if old radio state
     * was RX, otherwise no action is necessary */
    if (old_state == RADIO_RX) {
        cc1200_switch_to_rx(dev);
    }
}

int cc1200_rd_set_mode(cc1200_t *dev, int mode)
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
            LOG_DEBUG("cc1200: switching to RX mode\n");
            cc1200_setup_rx_mode(dev);          /* Set chip to desired mode */
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
            break;

        case RADIO_MODE_OFF:
            gpio_irq_disable(dev->params.gdo2); /* Disable interrupts */
            cc1200_switch_to_pwd(dev);          /* Set chip to power down mode */
            break;

        case RADIO_MODE_GET:
            /* do nothing, just return current mode */
        default:
            /* do nothing */
            break;
    }

    /* Return previous mode */
    DEBUG("%s:%s:%u\n", RIOT_FILE_RELATIVE, __func__, __LINE__);
    return result;
}
