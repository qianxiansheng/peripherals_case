/* $(license) */

/* 
 * more detail：
 *     https://ingchips.github.io/drafts/pg_ing916/ch-timer.html
 *     datasheet
 */
 
#include "case_timer.h"

// TIMER
#ifdef CASE_TIMER

#define TMR_CLK_EXTERNAL_FREQ   6000000
#define TMR_CLK_APB_FREQ        112000000

uint32_t tmr_100us_cnt = 0;
uint32_t tmr_50ms_cnt = 0;
uint32_t tmr_20ms_cnt = 0;

uint32_t timer0_irq_cb(void *user_data)
{
    // TMR0 channel 0 每100us触发 计数加1
    uint8_t stat = TMR_IntHappened(APB_TMR0, 0);
    if (stat & 0x01)
    {
        TMR_IntClr(APB_TMR0, 0, 0x01);
        
        tmr_100us_cnt++;
    }
    
    // TMR0 channel 1 每1s触发 打印计数并清零
    stat = TMR_IntHappened(APB_TMR0, 1);
    if (stat & 0x01)
    {
        TMR_IntClr(APB_TMR0, 1, 0x01);
        
        platform_printf("100us:%d 50ms:%d, 20ms:%d", tmr_100us_cnt, tmr_50ms_cnt, tmr_20ms_cnt);
        tmr_100us_cnt = tmr_50ms_cnt = tmr_20ms_cnt = 0;
    }
    
    return 0;
}

uint32_t timer1_irq_cb(void *user_data)
{
    uint8_t stat = TMR_IntHappened(APB_TMR1, 0);
    if (stat != 0)
    {
        // TMR1 channel 0 16bit timer0  每50ms触发计数加1
        if (stat & 0x01) // BIT(0)
        {
            TMR_IntClr(APB_TMR1, 0, 0x01);
            
            tmr_50ms_cnt++;
        }
        // TMR1 channel 0 16bit timer1  每20ms触发计数加1
        if (stat & 0x02) // BIT(1)
        {
            TMR_IntClr(APB_TMR1, 0, 0x02);
            
            tmr_20ms_cnt++;
        }
    }
    return 0;
}

void timer_test_init()
{
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_TMR0)
                            | (1 << SYSCTRL_ITEM_APB_TMR1)
                            | (1 << SYSCTRL_ITEM_APB_TMR2));
    
    // TMR0 channel 0 每100us触发 计数加1
    // TMR0 channel 1 每1s触发 打印计数并清零
    // 当硬件定时器以一定频率产生中断时，会对SysTick产生影响
    // 具体可见 https://ingchips.github.io/blog/2021-09-29-sdk-6.5-new/ SysTick 增强
    {
        uint32_t freq = 10000;   // 100us
        
        TMR_SetOpMode(APB_TMR0, 0, TMR_CTL_OP_MODE_32BIT_TIMER_x1, TMR_CLK_MODE_APB, 0);
        TMR_SetReload(APB_TMR0, 0, TMR_GetClk(APB_TMR0, 0) / 10000 - 1);
        TMR_IntEnable(APB_TMR0, 0, 0x01);
        TMR_Enable   (APB_TMR0, 0, 0x01);
        
        TMR_SetOpMode(APB_TMR0, 1, TMR_CTL_OP_MODE_32BIT_TIMER_x1, TMR_CLK_MODE_EXTERNAL, 0);
        TMR_SetReload(APB_TMR0, 1, TMR_GetClk(APB_TMR0, 1) - 1);
        TMR_IntEnable(APB_TMR0, 1, 0x01);
        TMR_Enable   (APB_TMR0, 1, 0x01);
    }
    
    // 使用TMR实现PWM可见CASE_PWM中的实现
    
    // TMR1 channel 0 16bit timer0  每50ms触发计数加1
    // TMR1 channel 0 16bit timer1  每20ms触发计数加1
    {
        uint16_t freq1 = 200;   // 50ms
        uint16_t freq2 = 500;   // 20ms
        
        TMR_SetOpMode(APB_TMR1, 0, TMR_CTL_OP_MODE_16BIT_TIMER_x2, TMR_CLK_MODE_EXTERNAL, 0);
        uint16_t tmr1_channel_0_timer0_cmp = TMR_GetClk(APB_TMR1, 0) / freq1 - 1;
        uint16_t tmr1_channel_0_timer1_cmp = TMR_GetClk(APB_TMR1, 0) / freq2 - 1;
        TMR_SetReload(APB_TMR1, 0, (tmr1_channel_0_timer1_cmp << 16) | tmr1_channel_0_timer0_cmp);
        TMR_IntEnable(APB_TMR1, 0, 0x03);   // BIT(0) + BIT(1): Ch0Int0   Ch0Int1
        TMR_Enable   (APB_TMR1, 0, 0x03);   // BIT(0) + BIT(1): Ch0TMR0En Ch0TMR1En
        
        platform_printf("%d %d\n", tmr1_channel_0_timer0_cmp, tmr1_channel_0_timer1_cmp);
        platform_printf("%d\n", (tmr1_channel_0_timer0_cmp << 16) | tmr1_channel_0_timer1_cmp);
    }
    
    platform_set_irq_callback(PLATFORM_CB_IRQ_TIMER0, timer0_irq_cb, NULL);
    platform_set_irq_callback(PLATFORM_CB_IRQ_TIMER1, timer1_irq_cb, NULL);
}

void timer_test()
{

}

#endif


