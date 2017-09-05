/*
 * Copyright (C) 2017 DAI Labor TU Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    boards_firefly Firefly Board
 * @ingroup     boards
 * @brief       Support for the Firefly board
 * @{
 *
 * @file
 * @brief       Board specific definitions for the Firefly board
 *
 * @author      Anon Mall <anon.mall@gt-arc.com>
 */

#ifndef BOARD_H
#define BOARD_H

#include "cpu.h"
#include "board_common.h"
#include "periph_cpu.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @name LED pin definitions
 * @{
 */
#define LED0_PIN        GPIO_PIN(3, 5)
#define LED1_PIN        GPIO_PIN(3, 4)
#define LED2_PIN        GPIO_PIN(3, 3)

#define LED0_MASK       (1 << 5)
#define LED1_MASK       (1 << 4)
#define LED2_MASK       (1 << 3)

#define LED0_ON         (GPIO_D->DATA |=  LED0_MASK)
#define LED0_OFF        (GPIO_D->DATA &= ~LED0_MASK)
#define LED0_TOGGLE     (GPIO_D->DATA ^=  LED0_MASK)

#define LED1_ON         (GPIO_D->DATA |=  LED1_MASK)
#define LED1_OFF        (GPIO_D->DATA &= ~LED1_MASK)
#define LED1_TOGGLE     (GPIO_D->DATA ^=  LED1_MASK)

#define LED2_ON         (GPIO_D->DATA |=  LED2_MASK)
#define LED2_OFF        (GPIO_D->DATA &= ~LED2_MASK)
#define LED2_TOGGLE     (GPIO_D->DATA ^=  LED2_MASK)
/** @} */

/**
 * @name User button pin definition
 * @{
 */
#define BTN0_PIN        GPIO_PIN(0, 3)
#define BTN0_MODE       GPIO_IN_PU
/** @} */

/**
 * @name CC1200 SPI and pins definitions
 * @{
 */
#define CC1200_SPI_DEV      SSI0
#define CC1200_MOSI_GPIO    GPIO_PIN(1, 1)
#define CC1200_MISO_GPIO    GPIO_PIN(1, 3)
#define CC1200_SCLK_GPIO    GPIO_PIN(1, 2)
#define CC1200_CSN_GPIO     GPIO_PIN(1, 5)
#define CC1200_RESET_GPIO   GPIO_PIN(2, 7)
#define CC1200_GPD0_GPIO    GPIO_PIN(1, 4)
#define CC1200_GPD2_GPIO    GPIO_PIN(1, 0)
/** @} */

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
} /* end extern "C" */
#endif
#endif /* BOARD_H */
/** @} */
