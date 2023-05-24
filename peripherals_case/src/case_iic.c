/* $(license) */

/*
 * more detail：
 *     https://ingchips.github.io/drafts/pg_ing916/ch-pinctrl.html
 *     https://ingchips.github.io/drafts/pg_ing916/ch-iic.html
 *     datasheet
 *
 * (1) master read slave data
 * (2) master send data to slave
 * (3) master read slave data with dma
 * (4) master send data to slave with dma
 * (5) master read slave data using blocking mode
 */
 
#include "case_iic.h"
#include "iic.h"
 
// I2C
#ifdef CASE_I2C
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



