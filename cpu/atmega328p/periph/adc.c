/*
 * Copyright (C) 2017 Technische Universität Berlin

 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_atmega328p
 * @ingroup     drivers_periph_adc
 * @{
 *
 * @file
 * @brief       Low-level ADC driver implementation
 *
 * @author      Thomas Geithner <thomas.geithner@dai-labor.de>
 * @}
 */

#include <stdio.h>

#include "cpu.h"
#include "periph/adc.h"
#include "periph_conf.h"
#include "xtimer.h"
#include "thread.h"

#include "arduino_pinmap.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

typedef uint8_t atmega_reg_t;

typedef union {
    atmega_reg_t admux;
    struct{
        atmega_reg_t mux   :4; /* analog channel selection bits */
        atmega_reg_t res   :1; /* reserved */
        atmega_reg_t adlar :1; /* ADC left adjust result */
        atmega_reg_t ref   :2; /* reference selection bits */
    } bits;

} atmega328p_admux_t;

typedef union {
    atmega_reg_t adcsra;
    struct {
        atmega_reg_t adps  :3; /* ADC prescaler selection */
        atmega_reg_t adie  :1; /* ADC interrupt enable */
        atmega_reg_t adif  :1; /* ADC interrupt flag */
        atmega_reg_t adate :1; /* ADC auto trigger enable */
        atmega_reg_t adsc  :1; /* ADC start conversion */
        atmega_reg_t aden  :1; /* ADC enable */
    } bits;

} atmega328p_adcsra_t;

atmega328p_admux_t * const atmega_admux = (atmega328p_admux_t*) &ADMUX;
atmega328p_adcsra_t * const atmega_adcsra = (atmega328p_adcsra_t*) &ADCSRA;


static mutex_t adc_mtx = MUTEX_INIT;

int adc_init(adc_t line)
{
    if (line >= ADC_NUMOF) {
        DEBUG("adc_init: invalid ADC line (%d)!\n", line);
        return -1;
    }

    return 0;
}

int adc_sample(adc_t line, adc_res_t res)
{
    uint16_t value;

    if (line >= ADC_NUMOF) {
        DEBUG("adc_sample: invalid ADC line (%d)!\n", line);
        return -1;
    }

    mutex_lock(&adc_mtx);

    atmega_admux->bits.mux = line;
    atmega_admux->bits.adlar = 1;    /* left adjust */
    atmega_admux->bits.ref = 1;      /* using Vcc as reference voltage */

    atmega_adcsra->adcsra = 0;
    atmega_adcsra->bits.adps = 0x07; /* prescaler = 128 */
    atmega_adcsra->bits.aden = 1;    /* ADC enabled */
    atmega_adcsra->bits.adsc = 1;    /* start ADC conversion */

    /* waiting for ADC... */
    while(atmega_adcsra->bits.adsc){
        thread_yield();
    }

    /* reading result */
    switch(res){
    case ADC_RES_6BIT:
        value = ADCH >> 2;
        break;
    case ADC_RES_8BIT:
        value = ADCH;
        break;
    case ADC_RES_10BIT:
        value = (ADCL >> 6) | (ADCH << 2) ;
        break;
    default:
        mutex_unlock(&adc_mtx);
        return -1;
        break;
    }

    atmega_adcsra->bits.aden = 0;    /* ADC disabled */
    mutex_unlock(&adc_mtx);
    return value;
}
