/* $(license) */

/* 
 * more detail：
 *     https://ingchips.github.io/drafts/pg_ing916/ch-watchdog.html
 *     datasheet
 */
 
#include "case_wdt.h"
// Watch Dog
#ifdef CASE_WDT
uint32_t wdt_test_irq_cb(void *user_data)
{
    TMR_WatchDogClearInt();
    
    platform_printf("watch dog reset");
    
    return 0;
}

void wdt_test_init()
{
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_WDT));
    
    TMR_WatchDogEnable3(WDT_INTTIME_INTERVAL_16S, WDT_RSTTIME_INTERVAL_500MS, 1);
    
    platform_set_irq_callback(PLATFORM_CB_IRQ_WDT, wdt_test_irq_cb, NULL);
}

void wdt_test_timer_task(TimerHandle_t xTimer)
{
    //feed wtd
    TMR_WatchDogRestart();
}

void wdt_test()
{
    TimerHandle_t handle = xTimerCreate("wdt",               //pcTimerName
                                        pdMS_TO_TICKS(15000),//xTimerPeriodInTicks
                                        pdTRUE,              //uxAutoReload
                                        NULL,                //pvTimerID
                                        wdt_test_timer_task);//pxCallbackFunction
    xTimerStart(handle, portMAX_DELAY);
}
#endif

