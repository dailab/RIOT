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
 *
 * @file
 * @brief       CC1200 SPI functions
 *
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 */

#ifndef CC1200_SPI_H
#define CC1200_SPI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Write a set of bytes using burst mode (if available)
 *
 * @param dev       Device to work on
 * @param addr      Destination register
 * @param buffer    Data to be written
 * @param count     Size of data
 */
#ifdef MODULE_CC1200
void cc1200_writeburst_reg(cc1200_t *dev, uint16_t addr, const char *buffer, uint8_t count);
#else
void cc1200_writeburst_reg(cc1200_t *dev, uint8_t addr, const char *buffer, uint8_t count);
#endif

/**
 * @brief Read a set of bytes using burst mode (if available)
 *
 * @param dev       Device to work on
 * @param addr      Source register
 * @param buffer    Buffer to store read data
 * @param count     Size of data to be read
 */
#ifdef MODULE_CC1200
void cc1200_readburst_reg(cc1200_t *dev, uint16_t addr, char *buffer, uint8_t count);
#else
void cc1200_readburst_reg(cc1200_t *dev, uint8_t addr, char *buffer, uint8_t count);
#endif

/**
 * @brief Write one byte to a register
 *
 * @param dev       Device to work on
 * @param addr      Destinatoin register
 * @param value     New value
 */
#ifdef MODULE_CC1200
void cc1200_write_reg(cc1200_t *dev, uint16_t addr, uint8_t value);
#else
void cc1200_write_reg(cc1200_t *dev, uint8_t addr, uint8_t value);
#endif

/**
 * @brief Read a byte from register
 *
 * @param dev       Device to work on
 * @param addr  Source register
 *
 * @return Read state and value of register
 */
#ifdef MODULE_CC1200
uint8_t cc1200_read_reg(cc1200_t *dev, uint16_t addr);
#else
uint8_t cc1200_read_reg(cc1200_t *dev, uint8_t addr);
#endif

/**
 * @brief Read a byte from register, robust version
 *
 * Datasheet states some registered should be read twice until
 * it returns the same value.
 *
 * @param dev       Device to work on
 * @param addr      Source register
 *
 * @return Read state and value of register
 */
#ifdef MODULE_CC1200
uint8_t cc1200_get_reg_robust(cc1200_t *dev, uint16_t addr);
#else
uint8_t cc1200_get_reg_robust(cc1200_t *dev, uint8_t addr);
#endif

/**
 * @brief Read state of a register
 *
 * @param dev       Device to work on
 * @param addr      Source register
 *
 * @return State of register
 */
#ifdef MODULE_CC1200
uint8_t cc1200_read_status(cc1200_t *dev, uint16_t addr);
#else
uint8_t cc1200_read_status(cc1200_t *dev, uint8_t addr);
#endif

/**
 * @brief Sends a command strobe
 *
 * @param dev       Device to work on
 * @param c         Command code
 *
 * @return Command response
 */
uint8_t cc1200_strobe(cc1200_t *dev, uint8_t c);

/**
 * @brief Pull CS to low and wait for CC110x stabilization
 *
 * @param dev       Device to work on
 */
void cc1200_cs(cc1200_t *dev);

#ifdef __cplusplus
}
#endif

/** @} */
#endif /* CC1200_SPI_H */
