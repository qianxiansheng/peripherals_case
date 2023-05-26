#ifndef __PCH_H__
#define __PCH_H__

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "platform_api.h"
#include "ingsoc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define CASE_GPIO   0
#define CASE_UART   0
#define CASE_PWM    0
#define CASE_EFLASH 0
#define CASE_ADC    0
#define CASE_SPI    1
#define CASE_RTC    0
#define CASE_TIMER  0
#define CASE_WDT    1
#define CASE_I2C    0

#if CASE_ADC
#include "case_adc.h"
#endif

#if CASE_EFLASH
#include "case_eflash.h"
#endif

#if CASE_GPIO
#include "case_gpio.h"
#endif

#if CASE_I2C
#include "case_iic.h"
#endif

#if CASE_PWM
#include "case_pwm.h"
#endif

#if CASE_RTC
#include "case_rtc.h"
#endif

#if CASE_SPI
#include "case_spi.h"
#endif

#if CASE_TIMER
#include "case_timer.h"
#endif

#if CASE_UART
#include "case_uart.h"
#endif

#if CASE_WDT
#include "case_wdt.h"
#endif

#endif