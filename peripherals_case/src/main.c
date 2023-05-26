/* $(license) */

/* 
 * 916 Peripheral Developer Handbook:
 *     https://ingchips.github.io/drafts/pg_ing916
 * power saving：
 *     https://ingchips.github.io/application-notes/pg_power_saving_en/ch-framework.html
 * ING91680X datasheet:
 *     https://vsite.xincache.cn/100085_2204195046/ING91680C_Datasheet_for_Ingchips_BLE5.3_SoC.pdf
 * ING91682X datasheet:
 *     https://vsite.xincache.cn/100085_2204195046/ING91682C_Datasheet_for_Ingchips_BLE5.3_SoC.pdf
 * Cube
 */


#include "pch.h"

#define PRINT_PORT    APB_UART0

uint32_t cb_putc(char *c, void *dummy)
{
    while (apUART_Check_TXFIFO_FULL(PRINT_PORT) == 1);
    UART_SendData(PRINT_PORT, (uint8_t)*c);
    return 0;
}

int fputc(int ch, FILE *f)
{
    cb_putc((char *)&ch, NULL);
    return ch;
}

void config_uart(uint32_t freq, uint32_t baud)
{
    UART_sStateStruct config;

    config.word_length       = UART_WLEN_8_BITS;
    config.parity            = UART_PARITY_NOT_CHECK;
    config.fifo_enable       = 1;
    config.two_stop_bits     = 0;
    config.receive_en        = 1;
    config.transmit_en       = 1;
    config.UART_en           = 1;
    config.cts_en            = 0;
    config.rts_en            = 0;
    config.rxfifo_waterlevel = 1;
    config.txfifo_waterlevel = 1;
    config.ClockFrequency    = freq;
    config.BaudRate          = baud;

    apUART_Initialize(PRINT_PORT, &config, 0);
}


void setup_peripherals(void)
{
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_SysCtrl)
                            | (1 << SYSCTRL_ITEM_APB_PinCtrl)
                            | (1 << SYSCTRL_ITEM_APB_UART0));
    config_uart(OSC_CLK_FREQ, 115200);
    
    // GPIO
#if CASE_GPIO
    gpio_test_init();
#endif
    
    // UART
#if CASE_UART
    uart_test_init();
#endif
    
    // PWM
#if CASE_PWM
    pwm_test_init();
#endif
    
    // ADC
#if CASE_ADC
    adc_test_init();
#endif
    
    // SPI
#if CASE_SPI
    spi_test_init();
#endif

    // EFLASH
#if CASE_EFLASH
    eflash_test_init();
#endif

    // RTC
#if CASE_RTC
    rtc_test_init();
#endif
    
    // TIMER
#if CASE_TIMER
    timer_test_init();
#endif

    // Watch Dog
#if CASE_WDT
    wdt_test_init();
#endif
    
    // I2C
#if CASE_I2C
    i2c_test_init();
#endif
    
}

uint32_t setup_profile(void *data, void *user_data)
{
    platform_printf("setup profile\n");
    // GPIO
#if CASE_GPIO
    gpio_test();
#endif
    
    // UART
#if CASE_UART
    uart_test();
#endif
    
    // PWM
#if CASE_PWM
    pwm_test();
#endif
    
    // ADC
#if CASE_ADC
    adc_test();
#endif
    
    // SPI
#if CASE_SPI
    spi_test();
#endif

    // EFLASH
#if CASE_EFLASH
    eflash_test();
#endif

    // RTC
#if CASE_RTC
    rtc_test();
#endif
    
    // TIMER
#if CASE_TIMER
    timer_test();
#endif

    // Watch Dog
#if CASE_WDT
    wdt_test();
#endif

    // I2C
#if CASE_I2C
    i2c_test();
#endif
    return 0;
}


int app_main()
{
    platform_32k_rc_auto_tune();
    
    platform_set_evt_callback(PLATFORM_CB_EVT_PROFILE_INIT, setup_profile, NULL);
    
    platform_set_evt_callback(PLATFORM_CB_EVT_PUTC, (f_platform_evt_cb)cb_putc, NULL);

#if CASE_TIMER
    // SysTick 增强
    platform_config(PLATFORM_CFG_RTOS_ENH_TICK, PLATFORM_CFG_ENABLE);
#endif
    
    platform_config(PLATFORM_CFG_24M_OSC_TUNE, 0x2C);
    
    setup_peripherals();

    return 0;
}

