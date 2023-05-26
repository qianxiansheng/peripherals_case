/* $(license) */

/* 
 * more detail：
 *     https://ingchips.github.io/drafts/pg_ing916/ch-rtc.html
 *     datasheet
 */
 
 
#include "case_rtc.h"
 
// RTC
#if CASE_RTC
typedef struct
{
    uint8_t hour;
    uint8_t minute;
    uint8_t secend;
} timer_t;

uint32_t rtc_test_irq_cb(void *user_data)
{
    uint32_t stat = RTC_GetIntState();
    
    if (stat & RTC_IRQ_SECOND)
    {
        RTC_ClearIntState(RTC_IRQ_SECOND);
        
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
        uint16_t day = RTC_GetTime(&hour, &minute, &second);
        
        platform_printf("%d %d %d %d\n", day, hour, minute, second);
    }
    
    if (stat & RTC_IRQ_ALARM)
    {
        RTC_ClearIntState(RTC_IRQ_ALARM);
        
        platform_printf("alarm\n");
    }
    return 0;
}


void rtc_test_init()
{
    RTC_ModifyTime(15, 18, 57, 00);
    
    //RTC_ConfigAlarm(18, 57, 30);
    RTC_EnableIRQ((0 << 2)  // 使能alarm中断
                 |(0 << 3)  // 使能day中断
                 |(0 << 4)  // 使能hour中断
                 |(0 << 5)  // 使能minute中断
                 |(1 << 6)  // 使能second中断
                 |(0 << 7));// 使能half-second中断
    RTC_Enable(1);
    
    platform_set_irq_callback(PLATFORM_CB_IRQ_RTC, rtc_test_irq_cb, NULL);
}

void rtc_test()
{
    
}

#endif

