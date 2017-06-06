/*
 * Copyright (C) 2013 INRIA
 *               2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup   drivers_cc110x
 * @{
 *
 * @file
 * @brief   TI Chipcon CC110x default settings
 *
 * @author    Thomas Hillebrandt <hillebra@inf.fu-berlin.de>
 * @author    Heiko Will <hwill@inf.fu-berlin.de>
 * @author    Oliver Hahm <oliver.hahm@inria.fr>
 * @author    Kaspar Schleiser <kaspar@schleiser.de>
 * @}
 */

#include "board.h"
#include "cc110x.h"

/**
 * @brief   PATABLE with available output powers
 * @note    If changed in size, adjust MAX_OUTPUT_POWER definition
 *          in CC110x interface
*/
const char cc110x_default_pa_table[8] = {
    0x00,   /*< -52 dBm */
    0x0D,   /*< -20 dBm */
    0x34,   /*< -10 dBm */
    0x57,   /*< - 5 dBm */
    0x8E,   /*<   0 dBm */
    0x85,   /*< + 5 dBm */
    0xCC,   /*< + 7 dBm */
    0xC3    /*< +10 dBm */
};


#ifndef MODULE_CC1200

const char cc110x_default_base_freq[3] = { 0x21, 0x71, 0x7F };

/**
 * @brief cc110x default settings
 */
const char cc110x_default_conf[] = {
    0x06, /* IOCFG2 */
    0x2E, /* IOCFG1 */
    /* some boards use cc110x' GDO0 as clock source, so for those, we allow
     * overriding of the corresponding setting, e.g., in board.h */
#ifdef CC110X_IOCONF0_VAL
    CC110X_IOCONF0_VAL,
#else
    0x0E, /* IOCFG0 */
#endif
    0x07, /* FIFOTHR */
    0x9B, /* SYNC1 */
    0xAD, /* SYNC0 */
    0xFF, /* PKTLEN */
    0x06, /* PKTCTRL1 */
    0x45, /* PKTCTRL0 (variable packet length) */
    0xFF, /* ADDR */
    0x00, /* CHANNR */
    0x0F, /* FSCTRL1 */
    0x00, /* FSCTRL0 */
    0x21, /* FREQ2 */
    0x71, /* FREQ1 */
    0x7A, /* FREQ0 */
    0x7C, /* MDMCFG4 */
    0x7A, /* MDMCFG3 */
    0x06, /* MDMCFG2 */
    0xC0, /* MDMCFG1 */
    0xF8, /* MDMCFG0 */
    0x44, /* DEVIATN */
    0x07, /* MCSM2 */
    0x03, /* MCSM1 */
    0x18, /* MCSM0 */
    0x16, /* FOCCFG */
    0x6C, /* BSCFG */
    0x45, /* AGCCTRL2 */
    0x40, /* AGCCTRL1 */
    0x91, /* AGCCTRL0 */
    0x87, /* WOREVT1 */
    0x6B, /* WOREVT0 */
    0xF8, /* WORCTRL */
    0x56, /* FREND1 */
    0x17, /* FREND0 */
    0xEA, /* FSCAL3 */
    0x2A, /* FSCAL2 */
    0x00, /* FSCAL1 */
    0x1F, /* FSCAL0 */
    0x00  /* padding to 4 bytes */
};

#else

const char cc110x_default_base_freq[3] = { 0xCC, 0xCC, 0x56 };

/**
 * @brief cc1200 default settings
 */
const char cc110x_default_conf[] = {
    0x30, /* IOCFG3 */
    0x06, /* IOCFG2 */
    0x30, /* IOCFG1 */
    0x11, /* IOCFG0 */
    0x6E, /* SYNC3 */
    0x4E, /* SYNC2 */
    0x90, /* SYNC1 */
    0x4E, /* SYNC0 */
    //0xE5, /* SYNC CFG1 */
    0xAA, /* SYNC CFG1 */
    0x23, /* SYNC CFG0 */
    0x47, /* DEVIATION M */
    0x0B, /* MODCFG DEV E */
    0x56, /* DCFILT CFG */
    0x19, /* PREAMBLE CFG1 */
    0xBA, /* PREAMBLE CFG0 */
    0xC8, /* IQIC */
    0x84, /* CHAN BW */
    0x42, /* MDMCFG1 */
    0x05, /* MDMCFG0 */
    0x94, /* SYMBOL RATE2 */
    0x7A, /* SYMBOL RATE1 */
    0xE1, /* SYMBOL RATE0 */
    0x27, /* AGC REF */
    0xF1, /* AGC CS THR */
    0x00, /* AGC GAIN ADJUST */
    0x00, /* AGC CFG3 */
    0x00, /* AGC CFG2 */
    0x11, /* AGC CFG1 */
    0x90, /* AGC CFG0 */
    //0x00, /* FIFO CFG */
    0x80, /* FIFO CFG */
    0x00, /* DEV ADDR */
    0x0B, /* SETTLING CFG */
    0x12, /* FS CFG */
    0x04, /* WOR CFG1 */
    0x21, /* WOR CFG0 */
    0x00, /* WOR EVENT0 MSB */
    0x00, /* WOR EVENT0 LSB */
    0x00, /* RXDCM TIME */
    0x24, /* PKT CFG2 */
    //0x04, /* PKT CFG2 */
    0x05, /* PKT CFG1 */
    0x20, /* PKT CFG0 */
    //0x0F, /* RFEND CFG1 */
    0x0E, /* RFEND CFG1 */
    0x30, /* RFEND CFG0 */
    0x7F, /* PA CFG1 */
    0x56, /* PA CFG0 */
    0x0F, /* ASK CFG */
    0xFF /* PKT LEN */
    };
#endif

/**
 * @brief The size of the configuration array for CC110X in bytes
 * */
const uint8_t cc110x_default_conf_size = sizeof(cc110x_default_conf);
