/*
 * Copyright (c) 2015, Zolertia <http://www.zolertia.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup remote-rtcc
 * @{
 *
 * Driver for the RE-Mote RTCC (Real Time Clock Calendar)
 * @{
 *
 * \file
 * Driver for the RE-Mote RF Real Time Clock Calendar (RTCC)
 *
 * \author
 *
 * Antonio Lignan <alinan@zolertia.com>
 * Aitor Mejias <amejias@zolertia.com>
 * Toni Lozano <tlozano@zolertia.com>
 */
/*---------------------------------------------------------------------------*/
//#include "contiki.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "periph/rtc.h"
#include "thread.h"
#include "ab8005.h"
#include "ab8005-config.h"
//#include "dev/leds.h"
#include "xtimer.h"

#include <string.h>
#include <stdio.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/* Callback pointers when interrupt occurs */
rtc_alarm_cb_t rtcc_int1_callback;

/*---------------------------------------------------------------------------*/
kernel_pid_t ab8005_pid;
char ab8005_stack[THREAD_STACKSIZE_DEFAULT];
/* -------------------------------------------------------------------------- */
#if 1
static const char *ab080x_td_register_name[] =
{
  "Mseconds",
  "Seconds",
  "Minutes",
  "Hours",
  "Days",
  "Months",
  "Years",
  "Weekdays",
};
#endif
/* -------------------------------------------------------------------------- */
#if 1
static const char *ab080x_config_register_name[] =
{
  "STATUS",
  "CTRL1",
  "CTRL2",
  "INTMASK",
  "SQW",
  "CAL_XT",
  "CAL_RCU",
  "CAL_RCL",
  "INTPOL",
  "TIMER_CTRL",
  "TIMER_CDOWN",
  "TIMER_INIT",
  "WDT",
  "OSC_CTRL",
  "OSC_STAT",
  "CONF_KEY",
  "TRICKLE",
  "BREF",
};
#endif
/*---------------------------------------------------------------------------*/
static uint8_t
bcd_to_dec(uint8_t val)
{
  return (uint8_t)(((val >> 4) * 10) + (val % 16));
}
/*---------------------------------------------------------------------------*/
static uint8_t
dec_to_bcd(uint8_t val)
{
  return (uint8_t)(((val / 10) << 4) + (val % 10));
}
/*---------------------------------------------------------------------------*/
static uint16_t
ab08xx_read_reg(uint8_t reg, uint8_t *buf, uint8_t regnum)
{
  i2c_acquire(RTC_I2C_DEV);

  if(i2c_read_regs(RTC_I2C_DEV, AB0805_ADDR, reg, buf, regnum) == regnum){
      i2c_release(RTC_I2C_DEV);
      return AB08_SUCCESS;
  }

  i2c_release(RTC_0_DEV);
  return AB08_ERROR;
}
/*---------------------------------------------------------------------------*/
static int8_t
ab08xx_write_reg(uint8_t reg, uint8_t *buf, uint8_t regnum)
{
  i2c_acquire(RTC_I2C_DEV);

  if(i2c_write_regs(RTC_I2C_DEV, AB0805_ADDR, reg, buf, regnum) == regnum){
    i2c_release(RTC_I2C_DEV);
    return AB08_SUCCESS;
  }

  i2c_release(RTC_I2C_DEV);
  return AB08_ERROR;
}
/*---------------------------------------------------------------------------*/
static void
write_default_config(void)
{
  const ab080x_register_config_t *settings;
  settings = ab080x_default_setting;
  uint8_t i, len = (sizeof(ab080x_default_setting) / sizeof(ab080x_register_config_t));

  for(i = 0; i < len; i++) {
    ab08xx_write_reg(settings[i].reg, (uint8_t *)&settings[i].val, 1);
  }
}
/*---------------------------------------------------------------------------*/
static int8_t
ab08_key_reg(uint8_t unlock)
{
  if((unlock != RTCC_CONFKEY_OSCONTROL) && (unlock != RTCC_CONFKEY_SWRESET) &&
     (unlock != RTCC_CONFKEY_DEFREGS)) {
    printf("RTC: invalid confkey values\n");
    return AB08_ERROR;
  }

  if(ab08xx_write_reg((CONFIG_MAP_OFFSET + CONF_KEY_ADDR), &unlock, 1)) {
    printf("RTC: failed to write to confkey register\n");
    return AB08_ERROR;
  }

  return AB08_SUCCESS;
}
/*---------------------------------------------------------------------------*/
static int8_t
ab08_read_status(uint8_t *buf)
{
  return ab08xx_read_reg((STATUS_ADDR + CONFIG_MAP_OFFSET), buf, 1);
}
/*---------------------------------------------------------------------------*/
static int8_t
ab08_ctrl1_config(uint8_t cmd)
{
  uint8_t ctrl1 = 0;

  if(cmd >= RTCC_CMD_MAX) {
    return AB08_ERROR;
  }

  if(ab08xx_read_reg((CONFIG_MAP_OFFSET + CTRL_1_ADDR), &ctrl1, 1)) {
    printf("RTC: failed to retrieve CTRL1 register\n");
    return AB08_ERROR;
  }

  switch(cmd) {
  case RTCC_CMD_LOCK:
    ctrl1 &= ~CTRL1_WRTC;
    break;
  case RTCC_CMD_UNLOCK:
    ctrl1 |= CTRL1_WRTC;
    break;
  case RTCC_CMD_ENABLE:
    ctrl1 &= ~CTRL1_STOP;
    break;
  case RTCC_CMD_STOP:
    ctrl1 |= CTRL1_STOP;
    break;
  default:
    return AB08_ERROR;
  }

  if(ab08xx_write_reg((CONFIG_MAP_OFFSET + CTRL_1_ADDR),
                    &ctrl1, 1) == AB08_ERROR) {
    printf("RTC: failed to write to the CTRL1 register\n");
    return AB08_ERROR;
  }

  return AB08_SUCCESS;
}
/*---------------------------------------------------------------------------*/
void*
ab8005_thread(void* arg)
{
  static uint8_t buf;
  //PROCESS_EXITHANDLER();
  //PROCESS_BEGIN();
  while(1) {

    //PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    thread_sleep();

    if(ab08_read_status(&buf) == AB08_ERROR) {
      printf("RTC: failed to retrieve ARST value\n");
      //PROCESS_EXIT();
      return NULL; // thread done.
    }

    /* We only handle the AIE (alarm interrupt) only */
    if((buf & STATUS_ALM) && (rtcc_int1_callback != NULL)) {
#if RTCC_CLEAR_INT_MANUALLY
      buf &= ~STATUS_ALM;
      if(ab08xx_write_reg((STATUS_ADDR + CONFIG_MAP_OFFSET),
                        &buf, 1) == AB08_ERROR) {
        printf("RTC: failed to clear the alarm\n");
        //return AB08_ERROR;
        return NULL;
      }
#endif
      rtcc_int1_callback(0);
    }
  }
  //PROCESS_END();

  return NULL;
}
/*---------------------------------------------------------------------------*/
#if 1
int8_t
ab8005_print(uint8_t value)
{
  uint8_t i, len, reg;
  char **name;
  uint8_t rtc_buffer[RTCC_CONFIG_MAP_SIZE];

  if(value >= RTCC_PRINT_MAX) {
    return AB08_ERROR;
  }

  switch(value) {
  case RTCC_PRINT_CONFIG:
    len = (RTCC_CONFIG_MAP_SIZE - 1);
    reg = STATUS_ADDR + CONFIG_MAP_OFFSET;
    name = (char **)ab080x_config_register_name;
    break;
  case RTCC_PRINT_ALARM:
  case RTCC_PRINT_ALARM_DEC:
    len = RTCC_ALARM_MAP_SIZE;
    reg = HUNDREDTHS_ALARM_ADDR + ALARM_MAP_OFFSET;
    name = (char **)ab080x_td_register_name;
    break;
  case RTCC_PRINT_DATE:
  case RTCC_PRINT_DATE_DEC:
    len = RTCC_TD_MAP_SIZE;
    reg = HUNDREDTHS_ADDR;
    name = (char **)ab080x_td_register_name;
    break;
  default:
    return AB08_ERROR;
  }

  if(ab08xx_read_reg(reg, rtc_buffer, len) == AB08_ERROR) {
    printf("RTC: failed to retrieve values to print\n");
    return AB08_ERROR;
  }

  if(value == RTCC_PRINT_ALARM_DEC) {
    printf("%02u/%02u (%02u) %02u:%02u:%02u/%02u\n",
           bcd_to_dec(rtc_buffer[MONTH_ALARM_ADDR]),
           bcd_to_dec(rtc_buffer[DAY_ALARM_ADDR]),
           bcd_to_dec(rtc_buffer[WEEKDAY_ALARM_ADDR]),
           bcd_to_dec(rtc_buffer[HOUR_ALARM_ADDR]),
           bcd_to_dec(rtc_buffer[MIN_ALARM_ADDR]),
           bcd_to_dec(rtc_buffer[SEC_ALARM_ADDR]),
           bcd_to_dec(rtc_buffer[HUNDREDTHS_ALARM_ADDR]));
    return AB08_SUCCESS;
  }

  if(value == RTCC_PRINT_DATE_DEC) {
    printf("%02u/%02u/%02u (%02u) %02u:%02u:%02u/%02u\n",
           bcd_to_dec(rtc_buffer[YEAR_ADDR]),
           bcd_to_dec(rtc_buffer[MONTH_ADDR]),
           bcd_to_dec(rtc_buffer[DAY_ADDR]),
           bcd_to_dec(rtc_buffer[WEEKDAY_ADDR]),
           bcd_to_dec(rtc_buffer[HOUR_ADDR]),
           bcd_to_dec(rtc_buffer[MIN_ADDR]),
           bcd_to_dec(rtc_buffer[SEC_ADDR]),
           bcd_to_dec(rtc_buffer[HUNDREDTHS_ADDR]));
    return AB08_SUCCESS;
  }

  for(i = 0; i < len; i++) {
    printf("0x%02X <- %s\n", rtc_buffer[i], name[i]);
  }

  return AB08_SUCCESS;
}
#endif
/*---------------------------------------------------------------------------*/
static void
ab8005_isr(void* arg)
{
  thread_wakeup(ab8005_pid);
}
/*---------------------------------------------------------------------------*/
int8_t
rtcc_set_autocalibration(uint8_t period)
{
  uint8_t aux;

  if(period > RTCC_AUTOCAL_9_MIN) {
    printf("RTC: invalid autocal value\n");
    return AB08_ERROR;
  }

  if(ab08xx_read_reg((OSC_CONTROL_ADDR + CONFIG_MAP_OFFSET),
                   &aux, 1) == AB08_ERROR) {
    printf("RTC: failed to read oscillator registers\n");
    return AB08_ERROR;
  }

  /* Clear ACAL */
  aux &= ~OSCONTROL_ACAL_9_MIN;

  /* Unlock the key register */
  ab08_key_reg(RTCC_CONFKEY_OSCONTROL);

  switch(period) {
  case RTCC_AUTOCAL_DISABLE:
    break;
  case RTCC_AUTOCAL_ONCE:
  case RTCC_AUTOCAL_17_MIN:
    aux |= OSCONTROL_ACAL_17_MIN;
    break;
  case RTCC_AUTOCAL_9_MIN:
    aux |= OSCONTROL_ACAL_9_MIN;
    break;
  default:
    return AB08_ERROR;
  }

  if(ab08xx_write_reg((OSC_CONTROL_ADDR + CONFIG_MAP_OFFSET),
                    &aux, 1) == AB08_ERROR) {
    printf("RTC: failed to clear the autocalibration\n");
    return AB08_ERROR;
  }

  if(period == RTCC_AUTOCAL_ONCE) {
    //clock_delay_usec(10000);
    xtimer_usleep(10000);
    ab08_key_reg(RTCC_CONFKEY_OSCONTROL);
    aux &= ~OSCONTROL_ACAL_9_MIN;
    if(ab08xx_write_reg((OSC_CONTROL_ADDR + CONFIG_MAP_OFFSET),
                      &aux, 1) == AB08_ERROR) {
      printf("RTC: failed to clear the autocalibration\n");
      return AB08_ERROR;
    }
  }

  return AB08_SUCCESS;
}
/*---------------------------------------------------------------------------*/
int8_t
rtcc_set_calibration(uint8_t mode, int32_t adjust)
{
  int32_t adjint;
  uint8_t adjreg[2];
  uint8_t xtcal;

  if(mode > RTCC_CAL_RC_OSC) {
    printf("RTC: invalid calibration mode\n");
    return AB08_ERROR;
  }

  /* Fixed values dependant on the oscillator source (Application Manual) */
  if((mode == RTCC_CAL_XT_OSC) && ((adjust <= -610) || (adjust >= 242))) {
    printf("RTC: invalid adjust value for XT oscillator\n");
    return AB08_ERROR;
  }

  if((mode == RTCC_CAL_RC_OSC) && ((adjust <= -65536) || (adjust >= 65520))) {
    printf("RTC: invalid adjust value for XT oscillator\n");
    return AB08_ERROR;
  }

  /* Calibration routine taken from the Application manual */
  if(adjust < 0) {
    adjint = ((adjust) * 1000 - 953);
  } else {
    adjint = ((adjust) * 1000 + 953);
  }

  adjint = adjint / 1907;

  if(mode == RTCC_CAL_XT_OSC) {
    if(adjint > 63) {
      xtcal = 0;
      /* CMDX = 1 */
      adjreg[0] = ((adjint >> 1) & 0x3F) | 0x80;
    } else if(adjint > -65) {
      xtcal = 0;
      adjreg[0] = (adjint & 0x7F);
    } else if(adjint > -129) {
      xtcal = 1;
      adjreg[0] = ((adjint + 64) & 0x7F);
    } else if(adjint > -193) {
      xtcal = 2;
      adjreg[0] = ((adjint + 128) & 0x7F);
    } else if(adjint > -257) {
      xtcal = 3;
      adjreg[0] = ((adjint + 192) & 0x7F);
    } else {
      xtcal = 3;
      adjreg[0] = ((adjint + 192) >> 1) & 0xFF;
    }

    if(ab08xx_write_reg((CAL_XT_ADDR + CONFIG_MAP_OFFSET),
                      &adjreg[0], 1) == AB08_ERROR) {
      printf("RTC: failed to clear the autocalibration\n");
      return AB08_ERROR;
    }

    if(ab08xx_read_reg((OSC_STATUS_ADDR + CONFIG_MAP_OFFSET),
                     &adjreg[0], 1) == AB08_ERROR) {
      printf("RTC: failed to read oscillator registers\n");
      return AB08_ERROR;
    }

    /* Clear XTCAL and write new value */
    adjreg[0] &= 0x3F;
    adjreg[0] |= (xtcal << 6);

    if(ab08xx_write_reg((OSC_STATUS_ADDR + CONFIG_MAP_OFFSET),
                      &adjreg[0], 1) == AB08_ERROR) {
      printf("RTC: failed to clear the autocalibration\n");
      return AB08_ERROR;
    }
  } else if(mode == RTCC_CAL_RC_OSC) {
    if(adjint > 32767) {
      adjreg[1] = ((adjint >> 3) & 0xFF);
      adjreg[0] = ((adjint >> 11) | 0xC0);
    } else if(adjint > 16383) {
      adjreg[1] = ((adjint >> 2) & 0xFF);
      adjreg[0] = ((adjint >> 10) | 0x80);
    } else if(adjint > 8191) {
      adjreg[1] = ((adjint >> 1) & 0xFF);
      adjreg[0] = ((adjint >> 9) | 0x40);
    } else if(adjint >= 0) {
      adjreg[1] = ((adjint) & 0xFF);
      adjreg[0] = (adjint >> 8);
    } else if(adjint > -8193) {
      adjreg[1] = ((adjint) & 0xFF);
      adjreg[0] = (adjint >> 8) & 0x3F;
    } else if(adjint > -16385) {
      adjreg[1] = ((adjint >> 1) & 0xFF);
      adjreg[0] = (adjint >> 9) & 0x7F;
    } else if(adjint > -32769) {
      adjreg[1] = ((adjint >> 2) & 0xFF);
      adjreg[0] = (adjint >> 10) & 0xBF;
    } else {
      adjreg[1] = ((adjint >> 3) & 0xFF);
      adjreg[0] = (adjint >> 11) & 0xFF;
    }

    if(ab08xx_write_reg((CAL_RC_HI_ADDR + CONFIG_MAP_OFFSET),
                      adjreg, 2) == AB08_ERROR) {
      printf("RTC: failed to set the RC calibration\n");
      return AB08_ERROR;
    }

    /* This should not happen */
  } else {
    return AB08_ERROR;
  }

  return AB08_SUCCESS;
}
/*---------------------------------------------------------------------------*/
/*
int8_t
rtcc_init(void)
*/
void
rtc_init(void)
{
  //printf("starting ab8005 initialization...\n");
  /*
  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN,
           I2C_SCL_NORMAL_BUS_SPEED);
  */
  i2c_init_master(RTC_I2C_DEV, I2C_SPEED_NORMAL);

#if RTCC_SET_DEFAULT_CONFIG
  write_default_config();
#endif

#if RTCC_SET_AUTOCAL
  rtcc_set_autocalibration(RTCC_AUTOCAL_17_MIN);
#endif

  /* Initialize interrupts handlers */
  rtcc_int1_callback = NULL;

  /* Configure the interrupts pins */
  //gpio_init(RTC_IRQ_PIN, GPIO_IN_PU);
  if( gpio_init_int(RTC_IRQ_PIN, GPIO_IN_PU, GPIO_FALLING, ab8005_isr, NULL) ==
      -1){
      printf("failed to initialize RTC GPIO Pin\n");
  }
  /*
  GPIO_SOFTWARE_CONTROL(RTC_INT1_PORT_BASE, RTC_INT1_PIN_MASK);
  GPIO_SET_INPUT(RTC_INT1_PORT_BASE, RTC_INT1_PIN_MASK);
   */

  /* Pull-up resistor, detect falling edge */
  //GPIO_DETECT_EDGE(RTC_INT1_PORT_BASE, RTC_INT1_PIN_MASK);
  //GPIO_TRIGGER_SINGLE_EDGE(RTC_INT1_PORT_BASE, RTC_INT1_PIN_MASK);
  //GPIO_DETECT_FALLING(RTC_INT1_PORT_BASE, RTC_INT1_PIN_MASK);
  gpio_irq_enable(RTC_IRQ_PIN);
  //gpio_register_callback(rtcc_interrupt_handler, RTC_INT1_PORT, RTC_INT1_PIN);

  /* Spin process until an interrupt is received */
  ab8005_pid = thread_create(
      ab8005_stack,
      sizeof(ab8005_stack),
      AB8005_PRIO,
      THREAD_CREATE_STACKTEST,
      ab8005_thread,
      NULL,
      "ab8005 RTC"
      );

  //printf("ab8005 initialization done.\n");
  //return AB08_SUCCESS;
}

int rtc_set_time(struct tm *time)
{
  uint8_t aux = 0;
  uint8_t rtc_buffer[RTCC_TD_MAP_SIZE];
  memset(rtc_buffer, 0, sizeof(rtc_buffer));

#if 0
  if(ab08_check_td_format(data, 0) == AB08_ERROR) {
    printf("RTC: Invalid time/date values\n");
    return AB08_ERROR;
  }
#endif

  if(ab08xx_read_reg((CTRL_1_ADDR + CONFIG_MAP_OFFSET),
                   &aux, 1) == AB08_ERROR) {
    printf("RTC: failed to retrieve CONTROL1 register\n");
    return -1;
  }

  rtc_buffer[WEEKDAY_ADDR] = dec_to_bcd(time->tm_wday);
  rtc_buffer[YEAR_ADDR] = dec_to_bcd(time->tm_year % 100);
  rtc_buffer[MONTH_ADDR] = dec_to_bcd(time->tm_mon);
  rtc_buffer[DAY_ADDR] = dec_to_bcd(time->tm_mday);
  rtc_buffer[HOUR_ADDR] = dec_to_bcd(time->tm_hour);
  rtc_buffer[MIN_ADDR] = dec_to_bcd(time->tm_min);
  rtc_buffer[SEC_ADDR] = dec_to_bcd(time->tm_sec);
  rtc_buffer[HUNDREDTHS_ADDR] = dec_to_bcd(0);

  // set to 24h format
  aux &= ~CTRL1_1224;

  /* Write the 12h/24h config */
  if(ab08xx_write_reg((CTRL_1_ADDR + CONFIG_MAP_OFFSET),
                    &aux, 1) == AB08_ERROR) {
    printf("RTC: failed to write 12h/24h configuration\n");
    return -1;
  }

  /* Reading the STATUS register with the CONTROL1.ARST set will clear the
   * interrupt flags, we write directly to the register without caring its
   * actual status and let the interrupt handler take care of any pending flag
   */

  if(ab08xx_read_reg((STATUS_ADDR + CONFIG_MAP_OFFSET), &aux, 1) == AB08_ERROR) {
    printf("RTC: failed to retrieve STATUS register\n");
    return -1;
  }

  uint8_t century = (time->tm_year / 100) + 19;

  //if(data->century == RTCC_CENTURY_20XX) {
  if(century == 20) {
    aux |= STATUS_CB;
  //} else if(data->century == RTCC_CENTURY_19XX_21XX) {
  } else if((century == 19) || (century == 21)) {
    aux |= ~STATUS_CB;
  } else {
    printf("RTC: invalid century value: %d\n", century);
    return -1;
  }

  if(ab08xx_write_reg((STATUS_ADDR + CONFIG_MAP_OFFSET), &aux, 1) == AB08_ERROR) {
    printf("RTC: failed to write century to STATUS register\n");
    return -1;
  }

  /* Set the WRTC bit to enable writting to the counters */
  if(ab08_ctrl1_config(RTCC_CMD_UNLOCK) == AB08_ERROR) {
    printf("RTC: could not unlock\n");
    return -1;
  }

  /* Write the buffers but the mode and century fields (used only for config) */
  if(ab08xx_write_reg(HUNDREDTHS_ADDR, rtc_buffer,
                    RTCC_TD_MAP_SIZE) == AB08_ERROR) {
    printf("RTC: failed to write date configuration\n");
    return -1;
  }

  /* Lock the RTCC and return */
  if(ab08_ctrl1_config(RTCC_CMD_LOCK) == AB08_ERROR) {
    printf("RTC: could not lock\n");
    return -1;
  }

  return 0;
}

int rtc_get_time(struct tm *time)
{
  uint8_t len, reg;
  uint8_t status_val;
  //char **name;
  uint8_t rtc_buffer[RTCC_CONFIG_MAP_SIZE];
  memset(rtc_buffer, 0, sizeof(rtc_buffer));

  /*
  printf("%s: TODO\n", __func__);
  ab8005_print(RTCC_PRINT_DATE);
  */

  len = RTCC_TD_MAP_SIZE;
  reg = HUNDREDTHS_ADDR;
  //name = (char **)ab080x_td_register_name;

  if(ab08xx_read_reg(reg, rtc_buffer, len) == AB08_ERROR) {
    printf("RTC: failed to retrieve values to print\n");
    return -1;
  }

  if(ab08xx_read_reg((STATUS_ADDR + CONFIG_MAP_OFFSET), &status_val, 1)
      == AB08_ERROR) {
    printf("RTC: failed to retrieve values to print\n");
    return -1;
  }

  memset(time, 0, sizeof(struct tm));
  //bzero(time, sizeof(struct tm));

  time->tm_year = bcd_to_dec(rtc_buffer[YEAR_ADDR]);
  if(status_val | STATUS_CB){
      // century flag is set
      time->tm_year += 100;
  }
  time->tm_mon = bcd_to_dec(rtc_buffer[MONTH_ADDR]);
  time->tm_mday = bcd_to_dec(rtc_buffer[DAY_ADDR]);
  time->tm_wday = bcd_to_dec(rtc_buffer[WEEKDAY_ADDR]);
  time->tm_hour = bcd_to_dec(rtc_buffer[HOUR_ADDR]);
  time->tm_min = bcd_to_dec(rtc_buffer[MIN_ADDR]);
  time->tm_sec = bcd_to_dec(rtc_buffer[SEC_ADDR]);



  return 0;
}

int rtc_set_alarm(struct tm *time, rtc_alarm_cb_t cb, void *arg)
{
  uint8_t aux[4], buf[RTCC_ALARM_MAP_SIZE];

  /* Stop the RTCC */
  ab08_ctrl1_config(RTCC_CMD_STOP);

  buf[WEEKDAY_ALARM_ADDR] = dec_to_bcd(time->tm_wday);
  buf[MONTH_ALARM_ADDR] = dec_to_bcd(time->tm_mon);
  buf[DAY_ALARM_ADDR] = dec_to_bcd(time->tm_mday);
  buf[HOUR_ALARM_ADDR] = dec_to_bcd(time->tm_hour);
  buf[MIN_ALARM_ADDR] = dec_to_bcd(time->tm_min);
  buf[SEC_ALARM_ADDR] = dec_to_bcd(time->tm_sec);
  buf[HUNDREDTHS_ALARM_ADDR] = dec_to_bcd(0);

  /* Check if the 12h/24h match the current configuration */
  if(ab08xx_read_reg((CTRL_1_ADDR + CONFIG_MAP_OFFSET),
                   &aux[0], 1) == AB08_ERROR) {
    printf("RTC: failed to retrieve CONTROL1 register\n");
    return -1;
  }

  /* Clear the RPT field */
  if(ab08xx_read_reg((TIMER_CONTROL_ADDR + CONFIG_MAP_OFFSET),
                   &aux[0], 1) == AB08_ERROR) {
    printf("RTC: failed to retrieve TIMER CTRL register\n");
    return -1;
  }

  aux[0] &= ~COUNTDOWN_TIMER_RPT_SECOND;


  /* We are using as default the level interrupt instead of pulses */
  /* FIXME: make this selectable */
  aux[0] |= COUNTDOWN_TIMER_TM;
  //aux[0] &= ~COUNTDOWN_TIMER_TM;
  aux[0] &= ~COUNTDOWN_TIMER_TRPT;

  // enable timer
  aux[0] |= COUNTDOWN_TIMER_RPT_YEAR;
  //aux[0] |= COUNTDOWN_TIMER_RPT_SECOND;
  aux[0] |= COUNTDOWN_TIMER_TE;

  if(ab08xx_write_reg((TIMER_CONTROL_ADDR + CONFIG_MAP_OFFSET),
                    &aux[0], 1) == AB08_ERROR) {
    printf("RTC: failed to clear the alarm config\n");
    return -1;
  }

  if(ab08xx_read_reg((STATUS_ADDR + CONFIG_MAP_OFFSET),
                   aux, 4) == AB08_ERROR) {
    printf("RTC: failed to read configuration registers\n");
    return -1;
  }

  /* Clear ALM field if any */
  aux[STATUS_ADDR] &= ~STATUS_ALM;

#if RTCC_CLEAR_INT_MANUALLY
  aux[CTRL_1_ADDR] &= ~CTRL1_ARST;
#endif

  /* Clear the AIE alarm bit */
  aux[INT_MASK_ADDR] &= ~INTMASK_AIE;

  // set callback
  rtcc_int1_callback = cb;

  // initializing INT1
  aux[CTRL_2_ADDR] |= CTRL2_OUT1S_NIRQ_NAIRQ_OUT;
  gpio_irq_enable(RTC_IRQ_PIN);

  // disable repeat
  aux[INT_MASK_ADDR] |= INTMASK_IM_LOW;

  if(ab08xx_write_reg((STATUS_ADDR + CONFIG_MAP_OFFSET), aux, 4) == AB08_ERROR) {
    printf("RTC: failed to clear alarm config\n");
    return -1;
  }

  /* Write to the alarm counters */
  if(ab08xx_write_reg((HUNDREDTHS_ALARM_ADDR + ALARM_MAP_OFFSET), buf,
                    RTCC_ALARM_MAP_SIZE) == AB08_ERROR) {
    printf("RTC: failed to set the alarm\n");
    return -1;
  }

  /* And finally enable the AIE bit */
  aux[INT_MASK_ADDR] |= INTMASK_AIE;
  if(ab08xx_write_reg((INT_MASK_ADDR + CONFIG_MAP_OFFSET),
                    &aux[INT_MASK_ADDR], 1) == AB08_ERROR) {
    printf("RTC: failed to enable the alarm\n");
    return -1;
  }

  /* Enable back the RTCC */
  ab08_ctrl1_config(RTCC_CMD_ENABLE);
  return 0;
}

int rtc_get_alarm(struct tm *time)
{
  uint8_t len;
  uint8_t buf[RTCC_ALARM_MAP_SIZE];

  len = RTCC_ALARM_MAP_SIZE;

  /* Read the alarm counters */
  if(ab08xx_read_reg((HUNDREDTHS_ALARM_ADDR + ALARM_MAP_OFFSET), buf,
                    len) == AB08_ERROR) {
    printf("RTC: failed to set the alarm\n");
    return -1;
  }

  memset(time, 0, sizeof(struct tm));

  time->tm_mon = bcd_to_dec(buf[MONTH_ALARM_ADDR]);
  time->tm_mday = bcd_to_dec(buf[DAY_ALARM_ADDR]);
  time->tm_wday = bcd_to_dec(buf[WEEKDAY_ALARM_ADDR]);
  time->tm_hour = bcd_to_dec(buf[HOUR_ALARM_ADDR]);
  time->tm_min = bcd_to_dec(buf[MIN_ALARM_ADDR]);
  time->tm_sec = bcd_to_dec(buf[SEC_ALARM_ADDR]);

  return 0;
}

void rtc_clear_alarm(void)
{
  uint8_t aux[4], buf[RTCC_ALARM_MAP_SIZE];


  /* Stop the RTCC */
  ab08_ctrl1_config(RTCC_CMD_STOP);

  memset(buf, 0, sizeof(buf));

  /* Clear the RPT field */
  if(ab08xx_read_reg((TIMER_CONTROL_ADDR + CONFIG_MAP_OFFSET),
                   &aux[0], 1) == AB08_ERROR) {
    printf("RTC: failed to retrieve TIMER CTRL register\n");
    return;
  }

  aux[0] &= ~COUNTDOWN_TIMER_RPT_SECOND;


  if(ab08xx_read_reg((STATUS_ADDR + CONFIG_MAP_OFFSET),
                   aux, 4) == AB08_ERROR) {
    printf("RTC: failed to read configuration registers\n");
    return;
  }

  /* Clear ALM field if any */
  aux[STATUS_ADDR] &= ~STATUS_ALM;

  /* Clear the AIE alarm bit */
  aux[INT_MASK_ADDR] &= ~INTMASK_AIE;

  // initializing INT1
  aux[CTRL_2_ADDR] |= CTRL2_OUT1S_NIRQ_NAIRQ_OUT;
  gpio_irq_disable(RTC_IRQ_PIN);

  // disable repeat
  aux[INT_MASK_ADDR] |= INTMASK_IM_LOW;

  if(ab08xx_write_reg((STATUS_ADDR + CONFIG_MAP_OFFSET), aux, 4) == AB08_ERROR) {
    printf("RTC: failed to clear alarm config\n");
    return;
  }

  /* Write to the alarm counters */
  if(ab08xx_write_reg((HUNDREDTHS_ALARM_ADDR + ALARM_MAP_OFFSET), buf,
                    RTCC_ALARM_MAP_SIZE) == AB08_ERROR) {
    printf("RTC: failed to set the alarm\n");
    return;
  }

  /* And finally enable the AIE bit */
  aux[INT_MASK_ADDR] |= INTMASK_AIE;
  if(ab08xx_write_reg((INT_MASK_ADDR + CONFIG_MAP_OFFSET),
                    &aux[INT_MASK_ADDR], 1) == AB08_ERROR) {
    printf("RTC: failed to enable the alarm\n");
    return;
  }

  /* Enable back the RTCC */
  ab08_ctrl1_config(RTCC_CMD_ENABLE);
}

void rtc_poweron(void)
{
  printf("%s: TODO\n", __func__);
}

void rtc_poweroff(void)
{
  printf("%s: TODO\n", __func__);
}

/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
