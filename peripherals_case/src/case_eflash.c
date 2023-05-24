/* $(license) */

/* 
 * more detail：
 *     https://ingchips.github.io/drafts/pg_ing916/ch-eflash.html
 *     https://ingchips.github.io/application-notes/pg_ble_stack_cn/ch-misc.html#键值存储
 *     https://ingchips.github.io/application-notes/pg_ble_stack_cn/ch-misc.html#ch98-le-dev-db
 *     datasheet
 * 
 * API:
 * program_flash
 * erase_flash_sector
 * write_flash
 * 
 */
#include "case_eflash.h"

#include "eflash.h"

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


