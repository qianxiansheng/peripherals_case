/* $(license) */

/* 
 * more detail：
 *     https://ingchips.github.io/drafts/pg_ing916/ch-pinctrl.html
 *     https://ingchips.github.io/drafts/pg_ing916/ch-adc.html
 *     datasheet
 * ADC量程等信息需要在datasheet中查看
 * 
 * (1) 单次校准模式，单端输入
 * (2) TODO 单次校准模式，差分输入
 * (3) TODO 单次转换模式
 * (4) TODO 循环校准模式，单端输入
 */
 
#include "case_adc.h"

// ADC
#ifdef CASE_ADC

#define SAMPLERATE  100
#define ADC_CH_NUM  1
#define ADC_CLK_MHZ 6
#define ADC_CHANNEL ADC_CH_1
#define AVE_NUM 5
#define LOOP_DELAY(c, s, ch)      ((((c) * (1000000)) / (s)) - (((16) * (ch)) + (5)))

static uint32_t ADC_cb_isr(void *user_data)
{
    uint32_t data = ADC_PopFifoData();
    SADC_channelId channel = ADC_GetDataChannel(data);
    if (channel == ADC_CHANNEL) {
        uint16_t sample = ADC_GetData(data);
        // do something with 'sample'
        float voltage = ADC_GetVol(sample);
        platform_printf("S:%d, V:%f", sample, voltage);
    }
    return 0;
}

void adc_test_init()
{
    SYSCTRL_ClearClkGate(SYSCTRL_ClkGate_APB_ADC);
    SYSCTRL_SetAdcClkDiv(24 / ADC_CLK_MHZ);
    SYSCTRL_ReleaseBlock(SYSCTRL_ClkGate_APB_ADC);
    ADC_Reset();
    ADC_ftInit();                           //ADC 精度初始化 顺序不影响采样
    ADC_Calibration(SINGLE_END_MODE);       //ADC 精度校准 顺序不影响采样
    ADC_VrefCalibration();                  //内部参考电压校准 必须先进行ADC精度初始化
    
    ADC_ConvCfg(SINGLE_MODE, PGA_GAIN_2, 1, (SADC_channelId)ADC_CHANNEL, 1, 0, SINGLE_END_MODE, 0);
    platform_set_irq_callback(PLATFORM_CB_IRQ_SADC, ADC_cb_isr, 0);
    ADC_Start(0);
}

void adc_test_timer_task(TimerHandle_t xTimer)
{
    ADC_Start(0);
    ADC_Start(1);
}

void adc_test()
{
    TimerHandle_t handle = xTimerCreate("adc",               //pcTimerName
                                        pdMS_TO_TICKS(1000),    //xTimerPeriodInTicks
                                        pdTRUE,                 //uxAutoReload
                                        NULL,                   //pvTimerID
                                        adc_test_timer_task);//pxCallbackFunction
    xTimerStart(handle, portMAX_DELAY);
}

#endif

