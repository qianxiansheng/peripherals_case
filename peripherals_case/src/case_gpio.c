/* $(license) */

/* 
 * more detail：
 *     https://ingchips.github.io/drafts/pg_ing916/ch-pinctrl.html
 *     https://ingchips.github.io/drafts/pg_ing916/ch-gpio.html
 *     datasheet
 *
 * 一般需要恢复PinCtrl和GPIO的时钟（即消除门控）
 * 输出模式：
 *     通过GIO_WriteValue(pin, 1或0) 来拉高或拉低电平
 *     通过GIO_ReadOutputValue 读取当前电平
 * 输入模式：
 *     可配置中断，水平触发或边沿触发
 *     通过GIO_ReadValue(pin) 读取当前电平
 */

#include "case_gpio.h"

// GPIO
#if CASE_GPIO
 
uint32_t gpio_isr(void *user_data)
{
    
    if (0 != GIO_GetIntStatus(GIO_GPIO_4))
    {
        platform_printf("GPIO4 %d\n", GIO_ReadValue(GIO_GPIO_4));
    }
    
    if (0 != GIO_GetIntStatus(GIO_GPIO_5))
    {
        platform_printf("GPIO5 %d\n", GIO_ReadValue(GIO_GPIO_4));
    }
    
    if (0 != GIO_GetIntStatus(GIO_GPIO_6))
    {
        platform_printf("6 \n");
    }
    
    GIO_ClearAllIntStatus();  // 清除中断触发状态
    return 0;
}

void gpio_test_init()
{
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_SysCtrl)
                            | (1 << SYSCTRL_ITEM_APB_PinCtrl)
                            | (1 << SYSCTRL_ITEM_APB_GPIO0));
    
    // GPIO3 output
    PINCTRL_SetPadMux(GIO_GPIO_3, IO_SOURCE_GPIO);
    GIO_SetDirection(GIO_GPIO_3, GIO_DIR_OUTPUT);
    
    
    // GPIO4 input 上升沿触发中断
    PINCTRL_SetPadMux(GIO_GPIO_4, IO_SOURCE_GPIO);
    PINCTRL_Pull(GIO_GPIO_4, PINCTRL_PULL_DOWN);    //下拉
    GIO_SetDirection(GIO_GPIO_4, GIO_DIR_INPUT);
    GIO_ConfigIntSource(GIO_GPIO_4, GIO_INT_EN_LOGIC_HIGH_OR_RISING_EDGE, GIO_INT_EDGE);
    
    // GPIO5 input 双沿触发中断
    PINCTRL_SetPadMux(GIO_GPIO_5, IO_SOURCE_GPIO);
    PINCTRL_Pull(GIO_GPIO_5, PINCTRL_PULL_DOWN);
    GIO_SetDirection(GIO_GPIO_5, GIO_DIR_INPUT);
    GIO_ConfigIntSource(GIO_GPIO_5, 
                        GIO_INT_EN_LOGIC_LOW_OR_FALLING_EDGE | GIO_INT_EN_LOGIC_HIGH_OR_RISING_EDGE, 
                        GIO_INT_EDGE);
    
    // GPIO6 input 高电平触发
    PINCTRL_SetPadMux(GIO_GPIO_6, IO_SOURCE_GPIO);
    PINCTRL_Pull(GIO_GPIO_6, PINCTRL_PULL_DOWN);
    GIO_SetDirection(GIO_GPIO_6, GIO_DIR_INPUT);
    GIO_ConfigIntSource(GIO_GPIO_6, GIO_INT_EN_LOGIC_HIGH_OR_RISING_EDGE, GIO_INT_LOGIC);
    
    // 设置GPIO中断回调函数
    platform_set_irq_callback(PLATFORM_CB_IRQ_GPIO0, gpio_isr, NULL);
}


void gpio_test_timer_task(TimerHandle_t xTimer)
{
    if (0 == GIO_ReadOutputValue(GIO_GPIO_3))
        GIO_WriteValue(GIO_GPIO_3, 1);
    else
        GIO_WriteValue(GIO_GPIO_3, 0);
}

void gpio_test()
{
    // 1s周期的TimerTask
    TimerHandle_t handle = xTimerCreate("gpio",               //pcTimerName
                                        pdMS_TO_TICKS(1000),  //xTimerPeriodInTicks
                                        pdTRUE,               //uxAutoReload
                                        NULL,                 //pvTimerID
                                        gpio_test_timer_task);//pxCallbackFunction
    xTimerStart(handle, portMAX_DELAY);
}
#endif
