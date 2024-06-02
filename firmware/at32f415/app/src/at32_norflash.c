#include <stdint.h>
#include "at32f415.h"
#include "board.h"

void norflash_cfgPins(void)
{
    WR_NOR_HIGH;
    RD_NOR_HIGH;

    GPIOA->scr = 0xFF;  // data bus = 0xFF
    GPIOB->odt = 0;     // Address 0
    LSB_NOR_LOW;

    GPIOA->cfglr = 0x44444444;
    GPIOB->cfglr = 0x22222222;
    GPIOB->cfghr = 0x22222222;

    GPIOC->cfghr = 0x22244444;
    GPIOF->cfglr = 0x24444444;   
}

/**
 * @brief Initialyzes GPIO peripherals and 
 * configures necessary pins
 * 
 * @return flash_res_t 
 */
flash_res_t norflash_init(void)
{
    crm_periph_clock_enable (CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable (CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable (CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable (CRM_IOMUX_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable (CRM_GPIOF_PERIPH_CLOCK, TRUE);

    gpio_pin_remap_config(SWJTAG_GMUX_010, TRUE);

    norflash_cfgPins();

    return FLASH_OK;
}

/**
 * @brief 
 * 
 */
void norflash_deinit(void)
{
    GPIOA->cfglr = 0x44444444;
    GPIOB->cfglr = 0x44444444;
    GPIOB->cfghr = 0x44444444;
}

/**
 * @brief Reads byte from nor flash
 *  In HW revA address bus is shifted left one bit.
 *  Due to this error rom_read only accesses half of flash.
 *  When issuing commands to flash, addresses must be trnaslated 
 *  by shifting them right by one and use LSb with setA_1(x)
 * 
 * @param addr 
 * @return uint8_t 
 */
uint8_t norflash_readbyte(uint16_t addr)
{
    uint8_t data;
    WR_NOR_HIGH;
    RD_NOR_HIGH;
    GPIOB->odt = addr >> 1;
    setA_1(addr & 1);
    delay_us(1);
    RD_NOR_LOW;
    delay_us(1);
    data = (uint8_t)GPIOA->idt;
    RD_NOR_HIGH;
    delay_us(1);

    return data;
}

/**
 * @brief Write byte to flash nor
 * 
 * @param addr 
 * @param data 
 */
void norflash_writebyte(uint16_t addr, uint8_t data)
{    
    RD_NOR_HIGH;
    WR_NOR_HIGH;
    GPIOA->cfglr = 0x22222222;  // output
    GPIOB->odt = addr >> 1;
    setA_1(addr & 1);
    GPIOA->odt = (GPIOA->odt & 0xFF00) | data;
    delay_us(10);
    WR_NOR_LOW;
    delay_us(10);
    WR_NOR_HIGH;
    
    GPIOA->cfglr = 0x44444444;  // input
}

/**
 * @brief Generic read
 * 
 * @param buf 
 * @param addr 
 * @param len 
 * @return flash_res_t 
 */
flash_res_t norflash_read(uint8_t *buf, uint32_t addr, uint16_t len)
{
    WR_NOR_HIGH;
    RD_NOR_HIGH;

    while(len--){
        GPIOB->odt = addr >> 1;
        setA_1(addr & 1);
        delay_us(1);
        RD_NOR_LOW;
        delay_us(1);
        *buf++ = (uint8_t)GPIOA->idt;
        RD_NOR_HIGH;
        delay_us(1);
        addr++;
    }

    return FLASH_OK;
}

/**
 * @brief Generic program
 * 
 * @param buf 
 * @param addr 
 * @param len 
 * @return flash_res_t 
 */
flash_res_t norflash_write(const uint8_t *buf, uint32_t addr, uint16_t len)
{
     while(len--){
        norflash_writebyte(0xAAA, 0xAA);
        norflash_writebyte(0x555, 0x55);
        norflash_writebyte(0xAAA, 0xA0);
        norflash_writebyte(addr++, *buf);

        if(norflash_polling(*buf++) != FLASH_OK){
            return FLASH_ERROR;
        }
    }

    return FLASH_OK;
}

/**
 * @brief perfroms a read cycle on bus
 * 
 * @return uint8_t 
 */
uint8_t norflash_readbus(void)
{
    RD_NOR_LOW;
    delay_us(1);
    uint8_t data = (uint8_t)GPIOA->idt;
    delay_us(1);
    RD_NOR_HIGH;
    return data;
}

/**
 * @brief generic bit6 toggle polling
 * 
 * @param expecteddata 
 * @return flash_res_t 
 */
flash_res_t norflash_polling(uint8_t expecteddata)
{
    uint8_t last_status = norflash_readbus();
    uint8_t toggle = 2;
    uint32_t time = get_tick();
    
    do{
        uint8_t status = norflash_readbus();

        if(status == expecteddata){
            return FLASH_OK;
        }

        // mask bit 6
        status &= ~(1<<6);

        // if same state as before decrement count
        if(status == last_status){
            toggle--;
        }

        // save status
        last_status = status;

        // check for timeout, valid only if bit6 is not stable
        if((get_tick() - time) > 30000 && toggle){
            return FLASH_ERROR_TIMEOUT;
        }
    }while(toggle);
    
    return FLASH_OK; // bit6 has stop toggling
}