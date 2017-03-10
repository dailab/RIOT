/*
 * Copyright (C) 2015-2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup         cpu_cc2538
 * @{
 *
 * @file
 * @brief           CPU specific definitions for internal peripheral handling
 *
 * @author          Hauke Petersen <hauke.petersen@fu-berlin.de>
 */

#ifndef PERIPH_CPU_H
#define PERIPH_CPU_H

#include <stdint.h>

#include "cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Length of the CPU_ID in octets
 */
#define CPUID_LEN           (8U)

/**
 * @brief   Define a custom type for GPIO pins
 * @{
 */
#define HAVE_GPIO_T
typedef uint32_t gpio_t;
/** @} */

/**
 * @brief   Define a custom GPIO_PIN macro
 *
 * For the CC2538, we use OR the gpio ports base register address with the
 * actual pin number.
 */
#define GPIO_PIN(port, pin) (gpio_t)(((uint32_t)GPIO_A + (port << 12)) | pin)
//#define GPIO_PIN(port, pin) (gpio_t)(((uint32_t)GPIO_A + (port << 3)) | pin)

/** @name Unique names for each GPIO port/pin combination
 * @{
 */
//enum {
//    GPIO_PA0 = GPIO_PIN(PORT_A, 0),          /**< PA0 */
//    GPIO_PA1 = GPIO_PIN(PORT_A, 1),          /**< PA1 */
//    GPIO_PA2 = GPIO_PIN(PORT_A, 2),          /**< PA2 */
//    GPIO_PA3 = GPIO_PIN(PORT_A, 3),          /**< PA3 */
//    GPIO_PA4 = GPIO_PIN(PORT_A, 4),          /**< PA4 */
//    GPIO_PA5 = GPIO_PIN(PORT_A, 5),          /**< PA5 */
//    GPIO_PA6 = GPIO_PIN(PORT_A, 6),          /**< PA6 */
//    GPIO_PA7 = GPIO_PIN(PORT_A, 7),          /**< PA7 */
//    GPIO_PB0 = GPIO_PIN(PORT_B, 0),          /**< PB0 */
//    GPIO_PB1 = GPIO_PIN(PORT_B, 1),          /**< PB1 */
//    GPIO_PB2 = GPIO_PIN(PORT_B, 2),          /**< PB2 */
//    GPIO_PB3 = GPIO_PIN(PORT_B, 3),          /**< PB3 */
//    GPIO_PB4 = GPIO_PIN(PORT_B, 4),          /**< PB4 */
//    GPIO_PB5 = GPIO_PIN(PORT_B, 5),          /**< PB5 */
//    GPIO_PB6 = GPIO_PIN(PORT_B, 6),          /**< PB6 */
//    GPIO_PB7 = GPIO_PIN(PORT_B, 7),          /**< PB7 */
//    GPIO_PC0 = GPIO_PIN(PORT_C, 0),          /**< PC0 */
//    GPIO_PC1 = GPIO_PIN(PORT_C, 1),          /**< PC1 */
//    GPIO_PC2 = GPIO_PIN(PORT_C, 2),          /**< PC2 */
//    GPIO_PC3 = GPIO_PIN(PORT_C, 3),          /**< PC3 */
//    GPIO_PC4 = GPIO_PIN(PORT_C, 4),          /**< PC4 */
//    GPIO_PC5 = GPIO_PIN(PORT_C, 5),          /**< PC5 */
//    GPIO_PC6 = GPIO_PIN(PORT_C, 6),          /**< PC6 */
//    GPIO_PC7 = GPIO_PIN(PORT_C, 7),          /**< PC7 */
//    GPIO_PD0 = GPIO_PIN(PORT_D, 0),          /**< PD0 */
//    GPIO_PD1 = GPIO_PIN(PORT_D, 1),          /**< PD1 */
//    GPIO_PD2 = GPIO_PIN(PORT_D, 2),          /**< PD2 */
//    GPIO_PD3 = GPIO_PIN(PORT_D, 3),          /**< PD3 */
//    GPIO_PD4 = GPIO_PIN(PORT_D, 4),          /**< PD4 */
//    GPIO_PD5 = GPIO_PIN(PORT_D, 5),          /**< PD5 */
//    GPIO_PD6 = GPIO_PIN(PORT_D, 6),          /**< PD6 */
//    GPIO_PD7 = GPIO_PIN(PORT_D, 7),          /**< PD7 */
//};
    /** @} */


/**
 * @brief   Define a custom GPIO_UNDEF value
 */
#define GPIO_UNDEF 99

/**
 * @brief   I2C configuration options
 */
typedef struct {
    gpio_t scl_pin;         /**< pin used for SCL */
    gpio_t sda_pin;         /**< pin used for SDA */
} i2c_conf_t;

/**
 * @brief declare needed generic SPI functions
 * @{
 */
#define PERIPH_SPI_NEEDS_INIT_CS
#define PERIPH_SPI_NEEDS_TRANSFER_BYTE
#define PERIPH_SPI_NEEDS_TRANSFER_REG
#define PERIPH_SPI_NEEDS_TRANSFER_REGS
/** @} */

/**
 * @brief   Override the default GPIO mode settings
 * @{
 */
#define HAVE_GPIO_MODE_T
typedef enum {
    GPIO_IN    = ((uint8_t)0x00),               /**< input, no pull */
    GPIO_IN_PD = ((uint8_t)IOC_OVERRIDE_PDE),   /**< input, pull-down */
    GPIO_IN_PU = ((uint8_t)IOC_OVERRIDE_PUE),   /**< input, pull-up */
    GPIO_OUT   = ((uint8_t)IOC_OVERRIDE_OE),    /**< output */
    GPIO_OD    = (0xff),                        /**< not supported */
    GPIO_OD_PU = (0xff)                         /**< not supported */
} gpio_mode_t;
/** @} */

/**
 * @brief   Override SPI mode settings
 * @{
 */
#define HAVE_SPI_MODE_T
typedef enum {
    SPI_MODE_0 = 0,                             /**< CPOL=0, CPHA=0 */
    SPI_MODE_1 = (SSI_CR0_SPH),                 /**< CPOL=0, CPHA=1 */
    SPI_MODE_2 = (SSI_CR0_SPO),                 /**< CPOL=1, CPHA=0 */
    SPI_MODE_3 = (SSI_CR0_SPO | SSI_CR0_SPH)    /**< CPOL=1, CPHA=1 */
} spi_mode_t;
/** @ */

/**
 * @brief   Override SPI clock settings
 * @{
 */
#define HAVE_SPI_CLK_T
typedef enum {
    SPI_CLK_100KHZ = 0,     /**< drive the SPI bus with 100KHz */
    SPI_CLK_400KHZ = 1,     /**< drive the SPI bus with 400KHz */
    SPI_CLK_1MHZ   = 2,     /**< drive the SPI bus with 1MHz */
    SPI_CLK_5MHZ   = 3,     /**< drive the SPI bus with 5MHz */
    SPI_CLK_10MHZ  = 4      /**< drive the SPI bus with 10MHz */
} spi_clk_t;
/** @} */

/**
 * @brief   Datafields for static SPI clock configuration values
 */
typedef struct {
    uint8_t cpsr;           /**< CPSR clock divider */
    uint8_t scr;            /**< SCR clock divider */
} spi_clk_conf_t;

/**
 * @brief   SPI configuration data structure
 * @{
 */
typedef struct {
    cc2538_ssi_t *dev;      /**< SSI device */
    gpio_t mosi_pin;        /**< pin used for MOSI */
    gpio_t miso_pin;        /**< pin used for MISO */
    gpio_t sck_pin;         /**< pin used for SCK */
    gpio_t cs_pin;          /**< pin used for CS */
} spi_conf_t;
/** @} */

/**
 * @brief   Timer configuration data
 */
typedef struct {
    cc2538_gptimer_t *dev;  /**< timer device */
    uint_fast8_t channels;  /**< number of channels */
    uint_fast8_t cfg;       /**< timer config word */
} timer_conf_t;

#ifdef __cplusplus
}
#endif

#include "periph/dev_enums.h"

#endif /* PERIPH_CPU_H */
/** @} */
