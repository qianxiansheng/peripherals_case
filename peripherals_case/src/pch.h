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

//#define CASE_GPIO
//#define CASE_UART
//#define CASE_PWM
//#define CASE_EFLASH
//#define CASE_ADC
//#define CASE_SPI
//#define CASE_RTC
#define CASE_TIMER
#define CASE_WDT
//#define CASE_I2C

#ifdef CASE_ADC
#include "case_adc.h"
#endif

#ifdef CASE_EFLASH
#include "case_eflash.h"
#endif

#ifdef CASE_GPIO
#include "case_gpio.h"
#endif

#ifdef CASE_I2C
#include "case_iic.h"
#endif

#ifdef CASE_PWM
#include "case_pwm.h"
#endif

#ifdef CASE_RTC
#include "case_rtc.h"
#endif

#ifdef CASE_SPI
#include "case_spi.h"
#endif

#ifdef CASE_TIMER
#include "case_timer.h"
#endif

#ifdef CASE_UART
#include "case_uart.h"
#endif

#ifdef CASE_WDT
#include "case_wdt.h"
#endif

#endif