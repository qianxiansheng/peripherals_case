/* $(license) */

/* 
 * more detail：
 *     https://ingchips.github.io/drafts/pg_ing916/ch-sysctrl.html
 *     https://ingchips.github.io/drafts/pg_ing916/ch-pinctrl.html
 *     https://ingchips.github.io/drafts/pg_ing916/ch-spi.html
 *     datasheet
 *
 * (1) normal SPI. TODO
 * (2) SPI DMA.
 * (3) high speed SPI (clock frequnce > 20Mbps).
 *     need to use SYSCtrl to switch spi clock source first
 * 
 */
 
#include "case_spi.h"

#include "semphr.h"
 
// SPI
#ifdef CASE_SPI
 
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
