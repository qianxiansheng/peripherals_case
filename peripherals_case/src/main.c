#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "platform_api.h"
#include "ingsoc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "eflash.h"
#include "iic.h"

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

//#define CASE_GPIO
//#define CASE_UART
//#define CASE_PWM
//#define CASE_EFLASH
//#define CASE_ADC
#define CASE_SPI
//#define CASE_RTC
//#define CASE_TIMER
//#define CASE_WDT
//#define CASE_I2C

// GPIO
#ifdef CASE_GPIO
uint32_t gpio_isr(void *user_data)
{
    GIO_ClearAllIntStatus();  // 清除中断触发状态
    
    platform_printf("-");
    
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
    PINCTRL_Pull(GIO_GPIO_4, PINCTRL_PULL_DOWN);
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
    TimerHandle_t handle = xTimerCreate("gpio",               //pcTimerName
                                        pdMS_TO_TICKS(1000),    //xTimerPeriodInTicks
                                        pdTRUE,                 //uxAutoReload
                                        NULL,                   //pvTimerID
                                        gpio_test_timer_task);//pxCallbackFunction
    xTimerStart(handle, portMAX_DELAY);
}
#endif

// UART
#ifdef CASE_UART

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

    apUART_Initialize(UART_PORT, &config, 1 << bsUART_RECEIVE_INTENAB);
}

uint32_t uart_isr(void *user_data)
{
    uint32_t status;

    while(1)
    {
        status = apUART_Get_all_raw_int_stat(UART_PORT);
        if (status == 0)
            break;

        UART_PORT->IntClear = status;

        // rx int
        if (status & (1 << bsUART_RECEIVE_INTENAB))
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
                                        pdMS_TO_TICKS(1000),    //xTimerPeriodInTicks
                                        pdTRUE,                 //uxAutoReload
                                        NULL,                   //pvTimerID
                                        uart_test_timer_task);//pxCallbackFunction
    xTimerStart(handle, portMAX_DELAY);
}

#endif


// PWM
#ifdef CASE_PWM

//#define CASE_PWM_MAX_OUTPUT             // 利用Timer完成9路独立PWM输出
#define CASE_PWM_COMPLEMENT_OUTPUT      // 互补输出


#define PIN_LED1_A GIO_GPIO_4
#define PIN_LED2_A GIO_GPIO_6
#define PIN_LED3_A GIO_GPIO_8
#define PIN_LED4_A GIO_GPIO_10
#define PIN_LED5_B GIO_GPIO_3
#define PIN_LED6_B GIO_GPIO_5
#define PIN_LED7_B GIO_GPIO_7
#define PIN_LED8_B GIO_GPIO_9
#define PIN_LED9_B GIO_GPIO_15

void TMR_PWM_SetupSimple(TMR_TypeDef *pTMR, uint8_t ch_id, uint32_t freq_clk, uint32_t frequency, uint32_t on_duty)
{
    uint32_t pera = freq_clk / frequency;
    uint32_t high = pera > 1000 ? pera / 100 * on_duty : pera * on_duty / 100;
    uint32_t low = pera > 1000 ? pera / 100 * (100 - on_duty) : pera * (100 - on_duty) / 100;
    TMR_SetReload(pTMR, ch_id, (high << 16) | low);
}

void pwm_test_init()
{
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_SysCtrl)
                            | (1 << SYSCTRL_ITEM_APB_PinCtrl)
                            | (1 << SYSCTRL_ITEM_APB_PWM)
                            | (1 << SYSCTRL_ITEM_APB_TMR0)
                            | (1 << SYSCTRL_ITEM_APB_TMR1)
                            | (1 << SYSCTRL_ITEM_APB_TMR2));
    
#ifdef CASE_PWM_COMPLEMENT_OUTPUT
    PINCTRL_SetPadMux(PIN_LED1_A, IO_SOURCE_PWM0_A);
    PINCTRL_SetPadMux(PIN_LED5_B, IO_SOURCE_PWM0_B);
    PWM_SetupSimple(0, 100, 20);
    PWM_Enable(0, 1);
#endif
    
#ifdef CASE_PWM_MAX_OUTPUT
    //配置PWM
    PINCTRL_SetPadMux(PIN_LED1_A, IO_SOURCE_PWM0_A);
    PWM_SetupSimple(0, 100, 50);
    PWM_Enable(0, 1);
    
    PINCTRL_SetPadMux(PIN_LED2_A, IO_SOURCE_PWM1_A);
    PWM_SetupSimple(1, 100, 20);
    PWM_Enable(1, 1);
    
    PINCTRL_SetPadMux(PIN_LED3_A, IO_SOURCE_PWM2_A);
    PWM_SetupSimple(2, 100, 80);
    PWM_Enable(2, 1);


    //PWM Timer impl
    SYSCTRL_SelectTimerClk(TMR_PORT_1, SYSCTRL_CLK_32k);
    TMR_SetOpMode      (APB_TMR1, 0, TMR_CTL_OP_MODE_16BIT_PWM, TMR_CLK_MODE_EXTERNAL, 0);
    TMR_PWM_SetupSimple(APB_TMR1, 0, 32000, 10, 50);
    TMR_Enable         (APB_TMR1, 0, (1 << 3));
    PINCTRL_SetPadMux(PIN_LED4_A, IO_SOURCE_TIMER1_PWM0_A);
    
    SYSCTRL_SelectTimerClk(TMR_PORT_1, SYSCTRL_CLK_32k);
    TMR_SetOpMode      (APB_TMR1, 1, TMR_CTL_OP_MODE_16BIT_PWM, TMR_CLK_MODE_EXTERNAL, 0);
    TMR_PWM_SetupSimple(APB_TMR1, 1, 32000, 10, 10);
    TMR_Enable         (APB_TMR1, 1, (1 << 3));
    PINCTRL_SetPadMux(PIN_LED5_B, IO_SOURCE_TIMER1_PWM1_B);
    
    SYSCTRL_SelectTimerClk(TMR_PORT_2, SYSCTRL_CLK_32k);
    TMR_SetOpMode      (APB_TMR2, 0, TMR_CTL_OP_MODE_16BIT_PWM, TMR_CLK_MODE_EXTERNAL, 0);
    TMR_PWM_SetupSimple(APB_TMR2, 0, 32000, 10, 20);
    TMR_Enable         (APB_TMR2, 0, (1 << 3));
    PINCTRL_SetPadMux(PIN_LED6_B, IO_SOURCE_TIMER2_PWM0_B);
    
    SYSCTRL_SelectTimerClk(TMR_PORT_2, SYSCTRL_CLK_32k);
    TMR_SetOpMode      (APB_TMR2, 1, TMR_CTL_OP_MODE_16BIT_PWM, TMR_CLK_MODE_EXTERNAL, 0);
    TMR_PWM_SetupSimple(APB_TMR2, 1, 32000, 10, 30);
    TMR_Enable         (APB_TMR2, 1, (1 << 3));
    PINCTRL_SetPadMux(PIN_LED7_B, IO_SOURCE_TIMER2_PWM1_B);
    
    SYSCTRL_SelectTimerClk(TMR_PORT_0, SYSCTRL_CLK_32k);
    TMR_SetOpMode      (APB_TMR0, 0, TMR_CTL_OP_MODE_16BIT_PWM, TMR_CLK_MODE_EXTERNAL, 0);
    TMR_PWM_SetupSimple(APB_TMR0, 0, 32000, 10, 40);
    TMR_Enable         (APB_TMR0, 0, (1 << 3));
    PINCTRL_SetPadMux(PIN_LED8_B, IO_SOURCE_TIMER0_PWM0_B);
    
    SYSCTRL_SelectTimerClk(TMR_PORT_0, SYSCTRL_CLK_32k);
    TMR_SetOpMode      (APB_TMR0, 1, TMR_CTL_OP_MODE_16BIT_PWM, TMR_CLK_MODE_EXTERNAL, 0);
    TMR_PWM_SetupSimple(APB_TMR0, 1, 32000, 10, 50);
    TMR_Enable         (APB_TMR0, 1, (1 << 3));
    PINCTRL_SetPadMux(PIN_LED9_B, IO_SOURCE_TIMER0_PWM1_B);
#endif
}
void pwm_test()
{

}

#endif

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

// SPI
#ifdef CASE_SPI

/* 
 * (1) normal SPI. TODO
 * (2) SPI DMA.
 * (3) high speed SPI (clock frequnce > 20Mbps).
 * 
 */
 
#define HIGH_SPEED
#define USE_SPI0

#ifdef USE_SPI0

//SYSCTRL_CLK_PLL_DIV_4 means 336M / 4 = 84M
//"eSclkDiv == 1"       means 84M/(2*(1+1)) = 21M(spi clk speed)
#define SPI_INTERFACETIMINGSCLKDIV_SPI1_21M    (1)


#define SSP_Ptr             AHB_SSP0
#define SPI_DMA_DST         SYSCTRL_DMA_SPI0_TX

#define SPI_MIC_CLK         GIO_GPIO_19
#define SPI_MIC_CS          GIO_GPIO_18
#define SPI_MIC_MISO        GIO_GPIO_27
#define SPI_MIC_MOSI        GIO_GPIO_28
#define SPI_MIC_HOLD        IO_NOT_A_PIN    //HS: IO20
#define SPI_MIC_WP          IO_NOT_A_PIN    //HS: IO26

#else

#define SSP_Ptr             APB_SSP1
#define SPI_DMA_DST         SYSCTRL_DMA_SPI1_TX

#define SPI_MIC_CLK         GIO_GPIO_3
#define SPI_MIC_CS          GIO_GPIO_4
#define SPI_MIC_MISO        GIO_GPIO_5
#define SPI_MIC_MOSI        GIO_GPIO_6
#define SPI_MIC_HOLD        IO_NOT_A_PIN 
#define SPI_MIC_WP          IO_NOT_A_PIN 


#endif

#define DATA_LEN_CNT_MAX_FOR_SPI (512)

#define SPI_DMA_TX_CHANNEL (0)


void spi_set_data_size(uint8_t dataSize)
{
    apSSP_SetTransferFormat(SSP_Ptr, (dataSize - 1), bsSPI_TRANSFMT_DATALEN, bwSPI_TRANSFMT_DATALEN);
}
void spi_set_trans_cnt(uint32_t transCnt)
{
    apSSP_SetTransferControlWrTranCnt(SSP_Ptr, transCnt);
}

typedef struct 
{
    SemaphoreHandle_t sem_over;
    uint32_t data_cnt;
} spi_dma_tx_t;

spi_dma_tx_t spi_dma_context = {.sem_over = NULL,.data_cnt = 0};

static uint32_t peripherals_spi_isr(void* user_data)
{
    uint32_t stat = apSSP_GetIntRawStatus(SSP_Ptr);
    
    if (stat & (1 << bsSPI_INTREN_ENDINTEN))
    {
        apSSP_ClearIntStatus(SSP_Ptr, 1 << bsSPI_INTREN_ENDINTEN);
        
        if (spi_dma_context.data_cnt > DATA_LEN_CNT_MAX_FOR_SPI)
        {
            spi_dma_context.data_cnt -= DATA_LEN_CNT_MAX_FOR_SPI;
            apSSP_WriteCmd(SSP_Ptr, 0x00, 0x00);   // trigger transfer
        }
        else if (spi_dma_context.data_cnt > 0)      // last package
        {
            spi_set_trans_cnt(spi_dma_context.data_cnt);
            spi_dma_context.data_cnt = 0;
            
            apSSP_WriteCmd(SSP_Ptr, 0x00, 0x00);   // trigger transfer
        }
        else                                        // transfer all over
        {
            apSSP_ResetTxFifo(SSP_Ptr);
            apSSP_SetTxDmaEn(SSP_Ptr,0);
            
            BaseType_t xHigherPriorityTaskWoke = pdFALSE;
            xSemaphoreGiveFromISR(spi_dma_context.sem_over, &xHigherPriorityTaskWoke);
        }
    }
    return 0;
}
void spi_config()
{
#ifdef USE_SPI0
    
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_AHB_SPI0));
    
    #ifdef HIGH_SPEED
    SYSCTRL_SelectSpiClk(SPI_PORT_0, SYSCTRL_CLK_PLL_DIV_4);  //PPL 336M / 4 = 84M
    #endif
    
    PINCTRL_SelSpiIn(SPI_PORT_0, SPI_MIC_CLK, SPI_MIC_CS, SPI_MIC_HOLD, SPI_MIC_WP, SPI_MIC_MISO, SPI_MIC_MOSI);
    PINCTRL_SetPadMux(SPI_MIC_CLK, IO_SOURCE_SPI0_CLK_OUT);
    PINCTRL_SetPadMux(SPI_MIC_CS, IO_SOURCE_SPI0_CSN_OUT);
    PINCTRL_SetPadMux(SPI_MIC_MOSI, IO_SOURCE_SPI0_MOSI_OUT);
    platform_set_irq_callback(PLATFORM_CB_IRQ_AHPSPI, peripherals_spi_isr, NULL);
#else
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_SPI1));
    
    PINCTRL_SelSpiIn(SPI_PORT_1, SPI_MIC_CLK, SPI_MIC_CS, SPI_MIC_HOLD, SPI_MIC_WP, SPI_MIC_MISO, SPI_MIC_MOSI);
    PINCTRL_SetPadMux(SPI_MIC_CLK, IO_SOURCE_SPI1_CLK_OUT);
    PINCTRL_SetPadMux(SPI_MIC_CS, IO_SOURCE_SPI1_CSN_OUT);
    PINCTRL_SetPadMux(SPI_MIC_MOSI, IO_SOURCE_SPI1_MOSI_OUT);
    platform_set_irq_callback(PLATFORM_CB_IRQ_APBSPI, peripherals_spi_isr, NULL);
#endif
    
    apSSP_sDeviceControlBlock param;
#ifdef HIGH_SPEED
    param.eSclkDiv         = (  SPI_InterfaceTimingSclkDiv )SPI_INTERFACETIMINGSCLKDIV_SPI1_21M;    //"eSclkDiv == 1" means 84M/(2*(1+1)) = 21M(spi clk speed)
#else
    param.eSclkDiv         = (  SPI_InterfaceTimingSclkDiv )SPI_INTERFACETIMINGSCLKDIV_DEFAULT_2M;
#endif
    param.eSCLKPhase       = (  SPI_TransFmt_CPHA_e        )SPI_CPHA_ODD_SCLK_EDGES;
    param.eSCLKPolarity    = (  SPI_TransFmt_CPOL_e        )SPI_CPOL_SCLK_LOW_IN_IDLE_STATES;
    param.eLsbMsbOrder     = (  SPI_TransFmt_LSB_e         )SPI_LSB_MOST_SIGNIFICANT_BIT_FIRST;
    param.eDataSize        = (  SPI_TransFmt_DataLen_e     )SPI_DATALEN_8_BITS;
    param.eMasterSlaveMode = (  SPI_TransFmt_SlvMode_e     )SPI_SLVMODE_MASTER_MODE;
    param.eReadWriteMode   = (  SPI_TransCtrl_TransMode_e  )SPI_TRANSMODE_WRITE_ONLY;
    param.eQuadMode        = (  SPI_TransCtrl_DualQuad_e   )SPI_DUALQUAD_REGULAR_MODE;
    param.eWriteTransCnt   = (  SPI_TransCtrl_TransCnt     )1;
    param.eReadTransCnt    = (  SPI_TransCtrl_TransCnt     )1;
    param.eAddrEn          = (  SPI_TransCtrl_AddrEn_e     )SPI_ADDREN_DISABLE;
    param.eCmdEn           = (  SPI_TransCtrl_CmdEn_e      )SPI_CMDEN_DISABLE;
    param.eInterruptMask   = (  SPI_InterruptEnableMask    )(1 << bsSPI_INTREN_ENDINTEN);
    param.TxThres          = (  SPI_ControlTxThres         )1;
    param.RxThres          = (  SPI_ControlRxThres         )1;
    param.SlaveDataOnly    = (  SPI_TransCtrl_SlvDataOnly_e)SPI_SLVDATAONLY_ENABLE;
    param.eAddrLen         = (  SPI_TransFmt_AddrLen_e     )SPI_ADDRLEN_1_BYTE;
    
    apSSP_DeviceParametersSet(SSP_Ptr, &param);
}

void dma_config()
{
    SYSCTRL_ClearClkGateMulti(1 << SYSCTRL_ClkGate_APB_DMA);
    DMA_Reset(1);
    DMA_Reset(0);
}

void spi_test_init()
{
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_SysCtrl)
                             |(1 << SYSCTRL_ITEM_APB_PinCtrl));
    
    spi_config();
    dma_config();
    
    spi_dma_context.sem_over = xSemaphoreCreateBinary();
}

void peripherals_spi_dma_to_txfifo(int channel_id, void *src, int size)
{
    DMA_Descriptor descriptor __attribute__((aligned (8)));
    descriptor.Next = NULL;
    DMA_PrepareMem2Peripheral(&descriptor, SPI_DMA_DST, src, size, DMA_ADDRESS_INC, 0);

    DMA_EnableChannel(channel_id, &descriptor);
}

void peripherals_spi_send_data(uint8_t *data, int size)
{
    apSSP_SetTxDmaEn(SSP_Ptr,1);
    peripherals_spi_dma_to_txfifo(SPI_DMA_TX_CHANNEL, data, size);

    apSSP_WriteCmd(SSP_Ptr, 0x00, 0x00);   //trigger transfer
}

/*************************************************************
 * 
 * @brief   SPI sends data of any length using DMA
 *          blocking function
 * 
 * @Param[in]   data    data pointer
 * @Param[in]   size    data size
 * 
 */
void spi_send_data_dma_sync(uint8_t *data, uint16_t size)
{
    spi_set_data_size(8);
    spi_set_trans_cnt(DATA_LEN_CNT_MAX_FOR_SPI);
    
    if (size > DATA_LEN_CNT_MAX_FOR_SPI)
        spi_dma_context.data_cnt = size - DATA_LEN_CNT_MAX_FOR_SPI;
    else
        spi_dma_context.data_cnt = 0;
    
    peripherals_spi_send_data(data, size);
    
    BaseType_t r = xSemaphoreTake(spi_dma_context.sem_over,  portMAX_DELAY);
    if (r != pdTRUE)
    {
        // exception!
    }
}

void spi_test_timer_task(TimerHandle_t xTimer)
{
    static uint8_t data[10000] = {0};
    for (uint16_t i = 0; i < 10000; ++i)
        data[i] = i;
    
    spi_send_data_dma_sync((uint8_t *)data, sizeof(data));   // 10000
    
    platform_printf("step1.\n");
    
    spi_send_data_dma_sync((uint8_t *)data, sizeof(data));   // 10000
    
    platform_printf("step2.\n");
    
    spi_send_data_dma_sync((uint8_t *)data, sizeof(data));   // 10000
    
    platform_printf("step3.\n");
}

void spi_test()
{
    TimerHandle_t handle = xTimerCreate("spi",               //pcTimerName
                                        pdMS_TO_TICKS(1000), //xTimerPeriodInTicks
                                        pdFALSE,             //uxAutoReload
                                        NULL,                //pvTimerID
                                        spi_test_timer_task);//pxCallbackFunction
    xTimerStart(handle, portMAX_DELAY);
}

#endif
// EFLASH
#ifdef CASE_EFLASH

// EFLASH_SECTOR_SIZE = 0x1000
#define CASE_EFLASH_LOAD_ADDR 0x2141000
#define CASE_EFLASH_AUTO_ERASE true

struct power_off_save_data_t 
{
    uint16_t init_flag;
    uint16_t cnt;
} power_off_save_data = 
{
    .init_flag = 0xABCD,
    .cnt = 0x0000,
};


void eflash_save_data()
{
    uint32_t size = sizeof(power_off_save_data);
    
    // round up to a multiple of 4
    size = (size + 0x3) & ~0x3; 
    
    if (true == CASE_EFLASH_AUTO_ERASE)
    {
        // auto erase and write data
        program_flash(CASE_EFLASH_LOAD_ADDR, (uint8_t *)&power_off_save_data, size);
    }
    else
    {
        // step.1 erase
        uint32_t i = CASE_EFLASH_LOAD_ADDR;
        uint32_t b = CASE_EFLASH_LOAD_ADDR + size;
        while (i < b)
        {
            erase_flash_sector(i);
            
            i += EFLASH_SECTOR_SIZE;
        }
        
        // step.2 write
        write_flash(CASE_EFLASH_LOAD_ADDR, (uint8_t *)&power_off_save_data, size);
    }
}

void eflash_load_data()
{
    uint32_t size = sizeof(power_off_save_data);
    memcpy((uint8_t *)&power_off_save_data, (uint8_t *)CASE_EFLASH_LOAD_ADDR, size);
}

void eflash_test_init()
{
    if ((*(struct power_off_save_data_t *)CASE_EFLASH_LOAD_ADDR).init_flag == 0xABCD)
        eflash_load_data();
}

void eflash_test_timer_task(TimerHandle_t xTimer)
{
    power_off_save_data.cnt++;
    platform_printf("cnt = %d\n", power_off_save_data.cnt);
    
    // Save every 10 times
    if (power_off_save_data.cnt % 10 == 0)
    {
        platform_printf("save data to flash\n");
        eflash_save_data();
    }
}

void eflash_test()
{
    TimerHandle_t handle = xTimerCreate("eflash",               //pcTimerName
                                        pdMS_TO_TICKS(1000),    //xTimerPeriodInTicks
                                        pdTRUE,                 //uxAutoReload
                                        NULL,                   //pvTimerID
                                        eflash_test_timer_task);//pxCallbackFunction
    xTimerStart(handle, portMAX_DELAY);
}



#endif


// RTC
#ifdef CASE_RTC

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

// TIMER
#ifdef CASE_TIMER


#define TMR_CLK_EXTERNAL_FREQ   6000000
#define TMR_CLK_APB_FREQ        112000000

uint32_t tmr_100us_cnt = 0;
uint32_t tmr_50ms_cnt = 0;
uint32_t tmr_20ms_cnt = 0;

uint32_t timer_case_irq_cb(void *user_data)
{
    // TMR0 channel 0 每100us触发 计数加1
    uint8_t stat = TMR_IntHappened(APB_TMR0, 0);
    if (stat & 0x01)
    {
        TMR_IntClr(APB_TMR0, 0, 0x01);
        
        tmr_100us_cnt++;
    }
    
    stat = TMR_IntHappened(APB_TMR1, 0);
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
    
    platform_set_irq_callback(PLATFORM_CB_IRQ_TIMER0, timer_case_irq_cb, NULL);
}

void timer_test()
{

}

#endif


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

// I2C
#ifdef CASE_I2C

/*
 * (1) master read slave data
 * (2) master send data to slave
 * (3) master read slave data with dma
 * (4) master send data to slave with dma
 * (5) master read slave data using blocking mode
 */
//#define MASTER_SEND_TO_SLAVE
//#define MASTER_READ_FROM_SLAVE
//#define MASTER_SEND_TO_SLAVE_WITH_DMA
//#define MASTER_READ_FROM_SLAVE_WITH_DMA
#define MASTER_READ_PERIPHERALS_BLOCKING_MODE

// Undefine ROLE_MASTER is ROLE_SLAVE
#define ROLE_MASTER

#define CASE_I2C_PIN_SCL GIO_GPIO_9
#define CASE_I2C_PIN_SDA GIO_GPIO_10

#define ADDRESS (0x71)
#define DATA_CNT (99)

// The sender uses TX channel, while the receiver uses RX channel
#define I2C_DMA_RX_CHANNEL (0)  // DMA channel 0
#define I2C_DMA_TX_CHANNEL (0)  // DMA channel 0

uint8_t read_data[DATA_CNT] = {0};
uint8_t read_data_cnt = 0;

uint8_t write_data[DATA_CNT] = {
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
    0x01, 0x23, 0x45,
};
uint8_t write_data_cnt = 0;

#ifdef MASTER_SEND_TO_SLAVE
/*
 * master:
 * Trigger transmission, set R/W to write, specify the number of data bytes to write.
 * When TX FIFO is empty, fill in the data.
 *
 * slave:
 * Enter the state of receiving data when there are transmission instructions detected in the bus 
 *   and the address matches itself.
 * Receiving data status: When RX FIFO is full, extract data.
 * Exit data receiving status when data transmission is completed.
 */

#ifdef ROLE_MASTER
uint32_t i2c_test_irq_cb(void *user_data)
{
    uint8_t i = 0;
    uint32_t status = I2C_GetIntState(APB_I2C0);
    
    if (status & (1 << I2C_STATUS_FIFO_EMPTY))
    {
        for (; write_data_cnt < DATA_CNT; write_data_cnt++) 
        {
            if (I2C_FifoFull(APB_I2C0))
                break;
            I2C_DataWrite(APB_I2C0, write_data[write_data_cnt]);
        }
    }
    
    if (status & (1 << I2C_STATUS_CMPL))
    {
        I2C_ClearIntState(APB_I2C0, (1 << I2C_STATUS_CMPL));
        
        platform_printf("transfer over: %02X %02X ... %02X, total: %d\n", 
                        write_data[0], write_data[1], write_data[DATA_CNT - 1], write_data_cnt);
    }
    
    return 0;
}
#else
uint32_t i2c_test_irq_cb(void *user_data)
{
    uint8_t i = 0;
    uint32_t status = I2C_GetIntState(APB_I2C0);
    
    static uint8_t dir = 2; //init invalid value
    
    if (status & (1 << I2C_INT_ADDR_HIT))
    {
        dir = I2C_GetTransactionDir(APB_I2C0);
        if (dir == I2C_TRANSACTION_MASTER2SLAVE)
            I2C_IntEnable(APB_I2C0, (1 << I2C_INT_FIFO_FULL));
        else if (dir == I2C_TRANSACTION_SLAVE2MASTER)
            I2C_IntEnable(APB_I2C0, (1 << I2C_INT_FIFO_EMPTY));
        
        I2C_ClearIntState(APB_I2C0, (1 << I2C_INT_ADDR_HIT));
    }
    
    if (status & (1 << I2C_INT_FIFO_FULL))
    {
        for (; read_data_cnt < DATA_CNT; ++read_data_cnt) 
        {
            if (I2C_FifoEmpty(APB_I2C0))
                break;
            read_data[read_data_cnt] = I2C_DataRead(APB_I2C0);
        }
    }
    
    if (status & (1 << I2C_STATUS_CMPL))
    {
        I2C_ClearIntState(APB_I2C0, (1 << I2C_STATUS_CMPL));
        
        if (dir == I2C_TRANSACTION_MASTER2SLAVE)
            I2C_IntDisable(APB_I2C0, (1 << I2C_INT_FIFO_FULL));
        else
            I2C_IntDisable(APB_I2C0, (1 << I2C_INT_FIFO_EMPTY));
        
        for (; read_data_cnt < DATA_CNT; ++read_data_cnt) 
        {
            if (I2C_FifoEmpty(APB_I2C0))
                break;
            read_data[read_data_cnt] = I2C_DataRead(APB_I2C0);
        }
        platform_printf("transfer over: %02X %02X ... %02X, total: %d\n", 
                        read_data[0], read_data[1], read_data[DATA_CNT - 1], read_data_cnt);
    }
    
    return 0;
}
#endif 

void i2c_test_init()
{
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_I2C0));
    
    PINCTRL_SelI2cIn(I2C_PORT_0,CASE_I2C_PIN_SCL,CASE_I2C_PIN_SDA);
    
#ifdef ROLE_MASTER
    I2C_Config(APB_I2C0, I2C_ROLE_MASTER, I2C_ADDRESSING_MODE_07BIT, ADDRESS);
    I2C_ConfigClkFrequency(APB_I2C0, I2C_CLOCKFREQUENY_STANDARD);
    I2C_Enable(APB_I2C0, 1);
    I2C_IntEnable(APB_I2C0, (1 << I2C_INT_CMPL) | (1 << I2C_INT_FIFO_EMPTY));
#else
    I2C_Config(APB_I2C0, I2C_ROLE_SLAVE, I2C_ADDRESSING_MODE_07BIT, ADDRESS);
    I2C_Enable(APB_I2C0, 1);
    I2C_IntEnable(APB_I2C0, (1 << I2C_INT_ADDR_HIT) | (1 << I2C_INT_CMPL));
#endif 

    platform_set_irq_callback(PLATFORM_CB_IRQ_I2C0, i2c_test_irq_cb, NULL);
}

void peripheral_i2c_send_data()
{
    // 设置传输方向
    I2C_CtrlUpdateDirection(APB_I2C0, I2C_TRANSACTION_MASTER2SLAVE);
    
    // 设置传输数据
    I2C_CtrlUpdateDataCnt(APB_I2C0, DATA_CNT);
    
    // 数据传输指令
    I2C_CommandWrite(APB_I2C0, I2C_COMMAND_ISSUE_DATA_TRANSACTION);
}

void i2c_test()
{
#ifdef ROLE_MASTER
    peripheral_i2c_send_data();
#endif
}
#endif

#ifdef MASTER_READ_FROM_SLAVE

/*
 * master:
 * Trigger transmission, set R/W to read, specify the number of data bytes to read.
 * Extract data when FIFO is full to receive subsequent data.
 *
 * slave:
 * Enter the state of sending data when there are transmission instructions detected in the bus 
 *   and the address matches itself.
 * Sending data status: When TX FIFO is empty, fill in the data.
 * Exit data sending status when data transmission is completed.
 */

#ifdef ROLE_MASTER
uint32_t i2c_test_irq_cb(void *user_data)
{
    uint8_t i = 0;
    uint32_t status = I2C_GetIntState(APB_I2C0);
    
    if (status & (1 << I2C_STATUS_FIFO_FULL))
    {
        for (; read_data_cnt < DATA_CNT; read_data_cnt++) 
        {
            if (I2C_FifoEmpty(APB_I2C0))
                break;
            read_data[read_data_cnt] = I2C_DataRead(APB_I2C0);
        }
    }
    
    // Received data of specified length 
    // master response NAK and terminate transaction
    if (status & (1 << I2C_STATUS_CMPL))
    {
        for (; read_data_cnt < DATA_CNT; ++read_data_cnt) 
        {
            read_data[read_data_cnt] = I2C_DataRead(APB_I2C0);
        }
        I2C_ClearIntState(APB_I2C0, (1 << I2C_STATUS_CMPL));
        
        platform_printf("transfer over: %02X %02X ... %02X, total: %d\n", 
                        read_data[0], read_data[1], read_data[DATA_CNT - 1], read_data_cnt);
    }
    
    return 0;
}
#else
uint32_t i2c_test_irq_cb(void *user_data)
{
    uint8_t i = 0;
    uint32_t status = I2C_GetIntState(APB_I2C0);
    
    static uint8_t dir = 2; //init as invalid value
    
    if (status & (1 << I2C_INT_ADDR_HIT))
    {
        dir = I2C_GetTransactionDir(APB_I2C0);
        if (dir == I2C_TRANSACTION_MASTER2SLAVE)
            I2C_IntEnable(APB_I2C0, (1 << I2C_INT_FIFO_FULL));
        else if (dir == I2C_TRANSACTION_SLAVE2MASTER)
            I2C_IntEnable(APB_I2C0, (1 << I2C_INT_FIFO_EMPTY));
        
        I2C_ClearIntState(APB_I2C0, (1 << I2C_INT_ADDR_HIT));
    }
    
    if (status & (1 << I2C_INT_FIFO_EMPTY))
    {
        for (; write_data_cnt < DATA_CNT; ++write_data_cnt) 
        {
            if (I2C_FifoFull(APB_I2C0))
                break;
            I2C_DataWrite(APB_I2C0, write_data[write_data_cnt]);
        }
    }
    
    // MASTER terminate transaction
    if (status & (1 << I2C_STATUS_CMPL))
    {
        I2C_ClearIntState(APB_I2C0, (1 << I2C_STATUS_CMPL));
        
        if (dir == I2C_TRANSACTION_MASTER2SLAVE)
            I2C_IntDisable(APB_I2C0, (1 << I2C_INT_FIFO_FULL));
        else
            I2C_IntDisable(APB_I2C0, (1 << I2C_INT_FIFO_EMPTY));
    }
    
    return 0;
}
#endif 

void i2c_test_init()
{
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_I2C0));
    
    PINCTRL_SelI2cIn(I2C_PORT_0,CASE_I2C_PIN_SCL,CASE_I2C_PIN_SDA);
    
#ifdef ROLE_MASTER
    I2C_Config(APB_I2C0, I2C_ROLE_MASTER, I2C_ADDRESSING_MODE_07BIT, ADDRESS);
    I2C_ConfigClkFrequency(APB_I2C0, I2C_CLOCKFREQUENY_STANDARD);
    I2C_Enable(APB_I2C0, 1);
    I2C_IntEnable(APB_I2C0, (1 << I2C_INT_CMPL) | (1 << I2C_INT_FIFO_FULL));
#else
    I2C_Config(APB_I2C0, I2C_ROLE_SLAVE, I2C_ADDRESSING_MODE_07BIT, ADDRESS);
    I2C_Enable(APB_I2C0, 1);
    I2C_IntEnable(APB_I2C0, (1 << I2C_INT_ADDR_HIT) | (1 << I2C_INT_CMPL));
#endif 

    platform_set_irq_callback(PLATFORM_CB_IRQ_I2C0, i2c_test_irq_cb, NULL);
}

void peripheral_i2c_send_data()
{
    // 设置传输方向
    I2C_CtrlUpdateDirection(APB_I2C0, I2C_TRANSACTION_SLAVE2MASTER);
    
    // 设置传输数据
    I2C_CtrlUpdateDataCnt(APB_I2C0, DATA_CNT);
    
    // 数据传输指令
    I2C_CommandWrite(APB_I2C0, I2C_COMMAND_ISSUE_DATA_TRANSACTION);
}

void i2c_test()
{
#ifdef ROLE_MASTER
    peripheral_i2c_send_data();
#endif
}
#endif


#ifdef MASTER_SEND_TO_SLAVE_WITH_DMA
/*
 * master:
 * Trigger transmission, set R/W to write, specify the number of data bytes to write.
 * Using DMA to move memory data to peripheral.
 *
 * slave:
 * Enter the state of receiving data when there are transmission instructions detected in the bus 
 *   and the address matches itself.
 * Using DMA to move peripheral data to memory.
 * Exit data receiving status when data transmission is completed.
 */
 
/*
 * @brief   I2C Slave DMA setting, peripherals -> memory
 */
void peripherals_i2c_rxfifo_to_dma(int channel_id, void *dst, int size)
{
    DMA_Descriptor descriptor __attribute__((aligned (8)));
    
    descriptor.Next = (DMA_Descriptor *)NULL;
    DMA_PreparePeripheral2Mem(&descriptor, dst, SYSCTRL_DMA_I2C0, size, DMA_ADDRESS_INC, 0);
    
    DMA_EnableChannel(channel_id, &descriptor);
}

/*
 * @brief   I2C Master DMA setting, memory -> peripherals
 */
void peripherals_i2c_dma_to_txfifo(int channel_id, void *src, int size)
{
    DMA_Descriptor descriptor __attribute__((aligned(8)));
    
    descriptor.Next = (DMA_Descriptor *)NULL;
    DMA_PrepareMem2Peripheral(&descriptor, SYSCTRL_DMA_I2C0, src, size, DMA_ADDRESS_INC, 0);
    
    DMA_EnableChannel(channel_id, &descriptor);
}

#ifdef ROLE_MASTER
uint32_t i2c_test_irq_cb(void *user_data)
{
    uint8_t i = 0;
    uint32_t status = I2C_GetIntState(APB_I2C0);
    
    // Received data of specified length 
    // master response NAK and terminate transaction
    if (status & (1 << I2C_STATUS_CMPL))
    {
        I2C_ClearIntState(APB_I2C0, (1 << I2C_STATUS_CMPL));
        
        platform_printf("transfer over: %02X %02X ... %02X, total: %d\n", 
                        write_data[0], write_data[1], write_data[DATA_CNT - 1],
                        APB_DMA->Channels[I2C_DMA_TX_CHANNEL].Descriptor.SrcAddr - (uint32_t)write_data);
    }
    
    return 0;
}
#else

/* 
 * @brief   set DMA description and enable I2C DMA
 */
void peripherals_i2c_read_data_dma_setup(void)
{
    peripherals_i2c_rxfifo_to_dma(I2C_DMA_RX_CHANNEL, read_data, sizeof(read_data));
    I2C_DmaEnable(APB_I2C0, 1);
}

uint32_t i2c_test_irq_cb(void *user_data)
{
    uint8_t i = 0;
    uint32_t status = I2C_GetIntState(APB_I2C0);
    
    static uint8_t dir = 2; //init as invalid value
    
    if (status & (1 << I2C_INT_ADDR_HIT))
    {
        dir = I2C_GetTransactionDir(APB_I2C0);
        
        if (dir == I2C_TRANSACTION_MASTER2SLAVE)
            peripherals_i2c_read_data_dma_setup(); //trigger DMA transfer
        
        I2C_ClearIntState(APB_I2C0, (1 << I2C_INT_ADDR_HIT));
    }
    
    // MASTER terminate transaction
    if (status & (1 << I2C_STATUS_CMPL))
    {
        I2C_DmaEnable(APB_I2C0,0);  //disable DMA
        I2C_ClearIntState(APB_I2C0, (1 << I2C_STATUS_CMPL));
        
        platform_printf("transfer over: %02X %02X ... %02X, total: %d\n", 
                        read_data[0], read_data[1], read_data[DATA_CNT - 1],
                        APB_DMA->Channels[I2C_DMA_RX_CHANNEL].Descriptor.DstAddr - (uint32_t)read_data);
        
    }
    
    return 0;
}
#endif 


/*
 * @brief   Configuring I2C
 */
static void setup_peripherals_i2c_module(void)
{
#ifdef ROLE_MASTER
    I2C_Config(APB_I2C0, I2C_ROLE_MASTER, I2C_ADDRESSING_MODE_07BIT, ADDRESS);
    I2C_ConfigClkFrequency(APB_I2C0, I2C_CLOCKFREQUENY_STANDARD);
    I2C_Enable(APB_I2C0, 1);
    I2C_IntEnable(APB_I2C0, (1 << I2C_INT_CMPL));       // DMA, no need to open FIFO interrupt
#else
    I2C_Config(APB_I2C0, I2C_ROLE_SLAVE, I2C_ADDRESSING_MODE_07BIT, ADDRESS);
    I2C_Enable(APB_I2C0, 1);
    I2C_IntEnable(APB_I2C0, (1 << I2C_INT_ADDR_HIT) | (1 << I2C_INT_CMPL));
#endif
}

/*
 * @brief   Configuring DMA
 */
static void setup_peripherals_dma_module(void)
{
    SYSCTRL_ClearClkGateMulti(1 << SYSCTRL_ClkGate_APB_DMA);
    DMA_Reset(1);
    DMA_Reset(0);
}

void i2c_test_init()
{
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_I2C0));
    
    PINCTRL_SelI2cIn(I2C_PORT_0,CASE_I2C_PIN_SCL,CASE_I2C_PIN_SDA);
    
    setup_peripherals_i2c_module();
    setup_peripherals_dma_module();

    platform_set_irq_callback(PLATFORM_CB_IRQ_I2C0, i2c_test_irq_cb, NULL);
}

/*
 * @brief   Master trigger transfer
 */
void peripheral_i2c_send_data()
{
    peripherals_i2c_dma_to_txfifo(I2C_DMA_RX_CHANNEL, write_data, sizeof(write_data));
    
    I2C_DmaEnable(APB_I2C0, 1);
    I2C_CtrlUpdateDirection(APB_I2C0, I2C_TRANSACTION_MASTER2SLAVE);
    I2C_CtrlUpdateDataCnt(APB_I2C0, DATA_CNT);
    
    I2C_CommandWrite(APB_I2C0, I2C_COMMAND_ISSUE_DATA_TRANSACTION);
}

void i2cdma_test_timer_task(TimerHandle_t xTimer)
{
#ifdef ROLE_MASTER
    //platform_printf("recv data: %02X %02X ... %02X, total: %d\n", 
    //                read_data[0], read_data[1], read_data[DATA_CNT - 1],
    //                APB_DMA->Channels[I2C_DMA_RX_CHANNEL].Descriptor.DstAddr - (uint32_t)read_data);
#else
    //platform_printf("total: %d\n", 
    //                APB_DMA->Channels[I2C_DMA_TX_CHANNEL].Descriptor.SrcAddr - (uint32_t)write_data);
#endif
}

void i2c_test()
{
#ifdef ROLE_MASTER
    peripheral_i2c_send_data();
#else
#endif
    TimerHandle_t handle = xTimerCreate("i2cdma",               //pcTimerName
                                        pdMS_TO_TICKS(1000),//xTimerPeriodInTicks
                                        pdTRUE,              //uxAutoReload
                                        NULL,                //pvTimerID
                                        i2cdma_test_timer_task);//pxCallbackFunction
    //xTimerStart(handle, portMAX_DELAY);
}
#endif


#ifdef MASTER_READ_FROM_SLAVE_WITH_DMA

/*
 * master:
 * Trigger transmission, set R/W to read, specify the number of data bytes to read.
 * Using DMA to move peripheral data to memory.
 *
 * slave:
 * Enter the state of sending data when there are transmission instructions detected in the bus 
 *   and the address matches itself.
 * Using DMA to move memory data to peripheral.
 * Exit data sending status when data transmission is completed.
 */

/*
 * @brief   I2C Master DMA setting, peripherals -> memory
 */
void peripherals_i2c_rxfifo_to_dma(int channel_id, void *dst, int size)
{
    DMA_Descriptor descriptor __attribute__((aligned (8)));
    
    descriptor.Next = (DMA_Descriptor *)NULL;
    DMA_PreparePeripheral2Mem(&descriptor, dst, SYSCTRL_DMA_I2C0, size, DMA_ADDRESS_INC, 0);
    
    DMA_EnableChannel(channel_id, &descriptor);
}

/*
 * @brief   I2C Slave DMA setting, memory -> peripherals
 */
void peripherals_i2c_dma_to_txfifo(int channel_id, void *src, int size)
{
    DMA_Descriptor descriptor __attribute__((aligned(8)));
    
    descriptor.Next = (DMA_Descriptor *)NULL;
    DMA_PrepareMem2Peripheral(&descriptor, SYSCTRL_DMA_I2C0, src, size, DMA_ADDRESS_INC, 0);
    
    DMA_EnableChannel(channel_id, &descriptor);
}

#ifdef ROLE_MASTER
uint32_t i2c_test_irq_cb(void *user_data)
{
    uint8_t i = 0;
    uint32_t status = I2C_GetIntState(APB_I2C0);
    
    // Received data of specified length 
    // master response NAK and terminate transaction
    if (status & (1 << I2C_STATUS_CMPL))
    {
        I2C_ClearIntState(APB_I2C0, (1 << I2C_STATUS_CMPL));
        
        platform_printf("transfer over: %02X %02X ... %02X, total: %d\n", 
                        read_data[0], read_data[1], read_data[DATA_CNT - 1],
                        APB_DMA->Channels[I2C_DMA_RX_CHANNEL].Descriptor.DstAddr - (uint32_t)read_data);
    }
    
    return 0;
}
#else


/* 
 * @brief   set DMA description and enable I2C DMA
 */
void peripherals_i2c_write_data_dma_setup(void)
{
    peripherals_i2c_dma_to_txfifo(I2C_DMA_TX_CHANNEL, write_data, sizeof(write_data));
    I2C_CtrlUpdateDataCnt(APB_I2C0, DATA_CNT);
    I2C_DmaEnable(APB_I2C0, 1);
    
}

uint32_t i2c_test_irq_cb(void *user_data)
{
    uint8_t i = 0;
    uint32_t status = I2C_GetIntState(APB_I2C0);
    
    static uint8_t dir = 2; //init as invalid value
    
    if (status & (1 << I2C_INT_ADDR_HIT))
    {
        dir = I2C_GetTransactionDir(APB_I2C0);
        
        if (dir == I2C_TRANSACTION_SLAVE2MASTER)
            peripherals_i2c_write_data_dma_setup(); //trigger DMA transfer
        
        I2C_ClearIntState(APB_I2C0, (1 << I2C_INT_ADDR_HIT));
    }
    
    // MASTER terminate transaction
    if (status & (1 << I2C_STATUS_CMPL))
    {
        I2C_DmaEnable(APB_I2C0,0);  //disable DMA
        I2C_ClearIntState(APB_I2C0, (1 << I2C_STATUS_CMPL));
        
        platform_printf("transfer over: %02X %02X ... %02X, total: %d\n", 
                        write_data[0], write_data[1], write_data[DATA_CNT - 1],
                        APB_DMA->Channels[I2C_DMA_TX_CHANNEL].Descriptor.SrcAddr - (uint32_t)write_data);
        
    }
    
    return 0;
}
#endif 


/*
 * @brief   Configuring I2C
 */
static void setup_peripherals_i2c_module(void)
{
#ifdef ROLE_MASTER
    I2C_Config(APB_I2C0, I2C_ROLE_MASTER, I2C_ADDRESSING_MODE_07BIT, ADDRESS);
    I2C_ConfigClkFrequency(APB_I2C0, I2C_CLOCKFREQUENY_STANDARD);
    I2C_Enable(APB_I2C0, 1);
    I2C_IntEnable(APB_I2C0, (1 << I2C_INT_CMPL));       // DMA, no need to open FIFO interrupt
#else
    I2C_Config(APB_I2C0, I2C_ROLE_SLAVE, I2C_ADDRESSING_MODE_07BIT, ADDRESS);
    I2C_Enable(APB_I2C0, 1);
    I2C_IntEnable(APB_I2C0, (1 << I2C_INT_ADDR_HIT) | (1 << I2C_INT_CMPL));
#endif
}

/*
 * @brief   Configuring DMA
 */
static void setup_peripherals_dma_module(void)
{
    SYSCTRL_ClearClkGateMulti(1 << SYSCTRL_ClkGate_APB_DMA);
    DMA_Reset(1);
    DMA_Reset(0);
}

void i2c_test_init()
{
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_I2C0));
    
    PINCTRL_SelI2cIn(I2C_PORT_0,CASE_I2C_PIN_SCL,CASE_I2C_PIN_SDA);
    
    setup_peripherals_i2c_module();
    setup_peripherals_dma_module();

    platform_set_irq_callback(PLATFORM_CB_IRQ_I2C0, i2c_test_irq_cb, NULL);
}

/*
 * @brief   Master trigger transfer
 */
void peripheral_i2c_send_data()
{
    peripherals_i2c_rxfifo_to_dma(I2C_DMA_RX_CHANNEL, read_data, sizeof(read_data));
    
    I2C_DmaEnable(APB_I2C0, 1);
    I2C_CtrlUpdateDirection(APB_I2C0, I2C_TRANSACTION_SLAVE2MASTER);
    I2C_CtrlUpdateDataCnt(APB_I2C0, DATA_CNT);
    
    I2C_CommandWrite(APB_I2C0, I2C_COMMAND_ISSUE_DATA_TRANSACTION);
}

void i2cdma_test_timer_task(TimerHandle_t xTimer)
{
#ifdef ROLE_MASTER
    //platform_printf("recv data: %02X %02X ... %02X, total: %d\n", 
    //                read_data[0], read_data[1], read_data[DATA_CNT - 1],
    //                APB_DMA->Channels[I2C_DMA_RX_CHANNEL].Descriptor.DstAddr - (uint32_t)read_data);
#else
    //platform_printf("total: %d\n", 
    //                APB_DMA->Channels[I2C_DMA_TX_CHANNEL].Descriptor.SrcAddr - (uint32_t)write_data);
#endif
}

void i2c_test()
{
#ifdef ROLE_MASTER
    peripheral_i2c_send_data();
#else
#endif
    TimerHandle_t handle = xTimerCreate("i2cdma",               //pcTimerName
                                        pdMS_TO_TICKS(1000),//xTimerPeriodInTicks
                                        pdTRUE,              //uxAutoReload
                                        NULL,                //pvTimerID
                                        i2cdma_test_timer_task);//pxCallbackFunction
    //xTimerStart(handle, portMAX_DELAY);
}
#endif
#ifdef MASTER_READ_PERIPHERALS_BLOCKING_MODE

#ifdef I2C_DMA_TX_CHANNEL
#undef I2C_DMA_TX_CHANNEL
#define I2C_DMA_TX_CHANNEL (1)  // DMA channel 1
#endif
/*
 * master:
 * Trigger transmission, set R/W to read, specify the number of data bytes to read.
 * Using blocking mode to read slave data
 *
 * slave:
 * Enter the state of sending data when there are transmission instructions detected in the bus 
 *   and the address matches itself.
 * master read from slave : Using DMA to move memory data to peripheral.
 * master send to slave   : Using DMA to move peripheral data to memory.
 * Exit data sending status when data transmission is completed.
 */

#ifdef ROLE_MASTER
#else
 
/*
 * @brief   I2C Slave DMA receive setting, peripherals -> memory
 */
void peripherals_i2c_rxfifo_to_dma(int channel_id, void *dst, int size)
{
    DMA_Descriptor descriptor __attribute__((aligned (8)));
    
    descriptor.Next = (DMA_Descriptor *)NULL;
    DMA_PreparePeripheral2Mem(&descriptor, dst, SYSCTRL_DMA_I2C0, size, DMA_ADDRESS_INC, 0);
    
    DMA_EnableChannel(channel_id, &descriptor);
}

/*
 * @brief   I2C Slave DMA send setting, memory -> peripherals
 */
void peripherals_i2c_dma_to_txfifo(int channel_id, void *src, int size)
{
    DMA_Descriptor descriptor __attribute__((aligned(8)));
    
    descriptor.Next = (DMA_Descriptor *)NULL;
    DMA_PrepareMem2Peripheral(&descriptor, SYSCTRL_DMA_I2C0, src, size, DMA_ADDRESS_INC, 0);
    
    DMA_EnableChannel(channel_id, &descriptor);
}

/* 
 * @brief   set DMA description and enable I2C DMA
 */
void peripherals_i2c_write_data_dma_setup(void)
{
    peripherals_i2c_dma_to_txfifo(I2C_DMA_TX_CHANNEL, write_data, sizeof(write_data));
    I2C_CtrlUpdateDataCnt(APB_I2C0, DATA_CNT);
    I2C_DmaEnable(APB_I2C0, 1);
}

/* 
 * @brief   set DMA description and enable I2C DMA
 */
void peripherals_i2c_read_data_dma_setup(void)
{
    peripherals_i2c_rxfifo_to_dma(I2C_DMA_RX_CHANNEL, read_data, sizeof(read_data));
    I2C_DmaEnable(APB_I2C0, 1);
    
}

uint32_t i2c_test_irq_cb(void *user_data)
{
    uint8_t i = 0;
    uint32_t status = I2C_GetIntState(APB_I2C0);
    
    static uint8_t dir = 2; //init as invalid value
    
    if (status & (1 << I2C_INT_ADDR_HIT))
    {
        dir = I2C_GetTransactionDir(APB_I2C0);
        
        if (dir == I2C_TRANSACTION_MASTER2SLAVE)
            peripherals_i2c_read_data_dma_setup();
        else if (dir == I2C_TRANSACTION_SLAVE2MASTER)
            peripherals_i2c_write_data_dma_setup(); //trigger DMA transfer
        
        I2C_ClearIntState(APB_I2C0, (1 << I2C_INT_ADDR_HIT));
    }
    
    // MASTER terminate transaction
    if (status & (1 << I2C_STATUS_CMPL))
    {
        I2C_DmaEnable(APB_I2C0,0);  //disable DMA
        
        I2C_ClearIntState(APB_I2C0, (1 << I2C_STATUS_CMPL));
        
        // print result
        if (dir == I2C_TRANSACTION_MASTER2SLAVE)
        {
            platform_printf("MASTER2SLAVE transfer over:");
            
            for (uint32_t i = 0; i < DATA_CNT; ++i) 
            {
                platform_printf(" %02X", read_data[i]);
            }
            platform_printf(", total: %d\n", APB_DMA->Channels[I2C_DMA_RX_CHANNEL].Descriptor.DstAddr - (uint32_t)read_data);
        
            memset(read_data, 0, sizeof(read_data));
        }
        else if (dir == I2C_TRANSACTION_SLAVE2MASTER)
        {
            platform_printf("SLAVE2MASTER transfer over:");
            
            for (uint32_t i = 0; i < DATA_CNT; ++i) 
            {
                platform_printf(" %02X", write_data[i]);
            }
            platform_printf(", total: %d\n", APB_DMA->Channels[I2C_DMA_TX_CHANNEL].Descriptor.SrcAddr - (uint32_t)write_data);
        }
        
    }
    
    return 0;
}
#endif 


/*
 * @brief   Configuring DMA
 */
static void setup_peripherals_dma_module(void)
{
    SYSCTRL_ClearClkGateMulti(1 << SYSCTRL_ClkGate_APB_DMA);
    DMA_Reset(1);
    DMA_Reset(0);
}

void i2c_test_init()
{
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_I2C0));
    
    PINCTRL_SelI2cIn(I2C_PORT_0,CASE_I2C_PIN_SCL,CASE_I2C_PIN_SDA);
    
#ifdef ROLE_MASTER
    i2c_init(I2C_PORT_0);
#else
    I2C_Config(APB_I2C0, I2C_ROLE_SLAVE, I2C_ADDRESSING_MODE_07BIT, ADDRESS);
    I2C_Enable(APB_I2C0, 1);
    I2C_IntEnable(APB_I2C0, (1 << I2C_INT_ADDR_HIT) | (1 << I2C_INT_CMPL));
    
    setup_peripherals_dma_module();
    
    platform_set_irq_callback(PLATFORM_CB_IRQ_I2C0, i2c_test_irq_cb, NULL);
#endif 
}


void i2c_test()
{
#ifdef ROLE_MASTER
    
    i2c_write(I2C_PORT_0, ADDRESS, write_data, DATA_CNT);
    
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    
    i2c_read(I2C_PORT_0, ADDRESS, NULL, 0, read_data, DATA_CNT);
    
    
    platform_printf("SLAVE2MASTER transfer over:");
    
    for (uint32_t i = 0; i < DATA_CNT; ++i) 
    {
        platform_printf(" %02X", read_data[i]);
    }
    platform_printf(", total: %d\n", DATA_CNT);

#endif
}


#endif

#endif



void setup_peripherals(void)
{
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_SysCtrl)
                            | (1 << SYSCTRL_ITEM_APB_PinCtrl)
                            | (1 << SYSCTRL_ITEM_APB_UART0));
    config_uart(OSC_CLK_FREQ, 115200);
    
    // GPIO
#ifdef CASE_GPIO
    gpio_test_init();
#endif
    
    // UART
#ifdef CASE_UART
    uart_test_init();
#endif
    
    // PWM
#ifdef CASE_PWM
    pwm_test_init();
#endif
    
    // ADC
#ifdef CASE_ADC
    adc_test_init();
#endif
    
    // SPI
#ifdef CASE_SPI
    spi_test_init();
#endif

    // EFLASH
#ifdef CASE_EFLASH
    eflash_test_init();
#endif

    // RTC
#ifdef CASE_RTC
    rtc_test_init();
#endif
    
    // TIMER
#ifdef CASE_TIMER
    timer_test_init();
#endif

    // Watch Dog
#ifdef CASE_WDT
    wdt_test_init();
#endif
    
    // I2C
#ifdef CASE_I2C
    i2c_test_init();
#endif
    
}

uint32_t setup_profile(void *data, void *user_data)
{
    platform_printf("setup profile\n");
    // GPIO
#ifdef CASE_GPIO
    gpio_test();
#endif
    
    // UART
#ifdef CASE_UART
    uart_test();
#endif
    
    // PWM
#ifdef CASE_PWM
    pwm_test();
#endif
    
    // ADC
#ifdef CASE_ADC
    adc_test();
#endif
    
    // SPI
#ifdef CASE_SPI
    spi_test();
#endif

    // EFLASH
#ifdef CASE_EFLASH
    eflash_test();
#endif

    // RTC
#ifdef CASE_RTC
    rtc_test();
#endif
    
    // TIMER
#ifdef CASE_TIMER
    timer_test();
#endif

    // Watch Dog
#ifdef CASE_WDT
    wdt_test();
#endif

    // I2C
#ifdef CASE_I2C
    i2c_test();
#endif
    return 0;
}


int app_main()
{
    platform_32k_rc_auto_tune();
    
    platform_set_evt_callback(PLATFORM_CB_EVT_PROFILE_INIT, setup_profile, NULL);
    
    platform_set_evt_callback(PLATFORM_CB_EVT_PUTC, (f_platform_evt_cb)cb_putc, NULL);

#ifdef CASE_TIMER
    // SysTick 增强
    platform_config(PLATFORM_CFG_RTOS_ENH_TICK, PLATFORM_CFG_ENABLE);
#endif
    
    setup_peripherals();

    return 0;
}

