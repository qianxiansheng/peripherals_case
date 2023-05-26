/* $(license) */

/* 
 * more detail：
 *     https://ingchips.github.io/drafts/pg_ing916/ch-pinctrl.html
 *     https://ingchips.github.io/drafts/pg_ing916/ch-uart.html
 *     datasheet
 * 一般需要恢复PinCtrl和UART的时钟（即消除门控）
 * 
 * 通过UART_PORT->DataRead 从RX_FIFO中读出数据
 * 通过UART_SendData(UART_PORT, c); 把数据写入TX_FIFO
 * 
 * 下面的代码演示 
 * UART1初始化
 * UART1接收的数据从UART0发送出去
 * UART1每秒发送26个字母(timer task)
 *
 * DMA: TODO
 * 
 */
 
#include "case_uart.h"
// UART
#if CASE_UART
#define UART_PORT      APB_UART1

#define USER_UART_IO_TX 9
#define USER_UART_IO_RX 10
#define USER_UART_IO_RTS 11
#define USER_UART_IO_CTS 12


void uart_test_config_uart(uint32_t freq, uint32_t baud)
{
    UART_sStateStruct config;

    config.word_length       = UART_WLEN_8_BITS;
    config.parity            = UART_PARITY_NOT_CHECK;
    config.fifo_enable       = 1;
    config.two_stop_bits     = 0;
    config.receive_en        = 1;
    config.transmit_en       = 1;
    config.UART_en           = 1;
    config.cts_en            = 1;
    config.rts_en            = 1;
    config.rxfifo_waterlevel = 1;
    config.txfifo_waterlevel = 1;
    config.ClockFrequency    = freq;
    config.BaudRate          = baud;

    apUART_Initialize(UART_PORT, &config, (1 << bsUART_RECEIVE_INTENAB));   //初始化并使能接收中断
}

uint32_t uart_isr(void *user_data)
{
    uint32_t status;

    while(1)
    {
        status = apUART_Get_all_raw_int_stat(UART_PORT);    // 获取中断状态
        if (status == 0)
            break;

        UART_PORT->IntClear = status;                       // 清除中断状态

        // rx int
        if (status & (1 << bsUART_RECEIVE_INTENAB))         // 判断中断为接收中断
        {
            while (apUART_Check_RXFIFO_EMPTY(UART_PORT) != 1)
            {
                platform_printf("%c", UART_PORT->DataRead);
            }
        }
    }
    
    return 0;
}

void uart_test_init()
{
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_SysCtrl)
                            | (1 << SYSCTRL_ITEM_APB_PinCtrl)
                            | (1 << SYSCTRL_ITEM_APB_UART1));
    
    PINCTRL_SetPadMux(USER_UART_IO_RX, IO_SOURCE_GENERAL);
    PINCTRL_SetPadMux(USER_UART_IO_TX, IO_SOURCE_UART1_TXD);
    PINCTRL_SetPadMux(USER_UART_IO_RTS, IO_SOURCE_UART1_RTS);
    
    // RX 上拉
    PINCTRL_Pull(USER_UART_IO_RX, PINCTRL_PULL_UP);
    PINCTRL_SelUartRxdIn(UART_PORT_1, USER_UART_IO_RX);
    
    PINCTRL_Pull(USER_UART_IO_CTS, PINCTRL_PULL_DOWN);
    PINCTRL_SelUartCtsIn(UART_PORT_1, USER_UART_IO_CTS);
    
    
    uart_test_config_uart(OSC_CLK_FREQ, 115200);
    
    platform_set_irq_callback(PLATFORM_CB_IRQ_UART1, uart_isr, NULL);
}

void uart_test_timer_task(TimerHandle_t xTimer)
{
    int i = 0;   
    for (i = 'a'; i < 'z'; ++i)
    {
        while (apUART_Check_RXFIFO_FULL(UART_PORT) != 0){}
        UART_SendData(UART_PORT, i);
    }
}

void uart_test()
{
    TimerHandle_t handle = xTimerCreate("uart",               //pcTimerName
                                        pdMS_TO_TICKS(1000),  //xTimerPeriodInTicks
                                        pdTRUE,               //uxAutoReload
                                        NULL,                 //pvTimerID
                                        uart_test_timer_task);//pxCallbackFunction
    xTimerStart(handle, portMAX_DELAY);
}

#endif


