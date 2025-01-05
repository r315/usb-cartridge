#include <stdint.h>
#include "at32f415.h"
#include "board.h"

/**
 * @brief
 *
 *  In HW revA address bus is shifted left one bit.
 *  Due to this error rom_read only accesses half of flash.
 *  When issuing commands to flash, addresses must be trnaslated
 *  by shifting them right by one and use LSb with setA_1(x)
 *
 * PF7    -> A-1
 * PB14:0 -> A13:0
 * MBC1   -> A17:14
 * PA7:0 <-> D7:0
 * PC15   -> RDn
 * PC14   -> WRn
 *
 *
 *  For solo programmer pinout follows
 *
 * PB7:0 -> A7:0
 * PA7:0 -> A15:8
 * TBD   -> A18:A16
 * PB15:8 <-> D7:0  // These pins are 5V tolerant
 * PC15   -> RDn
 * PC14   -> WRn
 *
 */
static void norflash_cfgPins(void)
{
    // Set bus states before enable them
    WR_NOR_HIGH;
    RD_NOR_HIGH;
    NOR_DATA_WRITE(0xFF);
    NOR_ADDRESS_SET(0);             // A15:0, output
    NOR_CTRL_OUTPUT;                // WRn, RDn, output
    NOR_DATA_INPUT;
    NOR_ADDRESS_OUTPUT;
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
    NOR_DATA_INPUT;
    NOR_ADDRESS_INPUT;
    NOR_CTRL_INPUT;
}

/**
 * @brief Reads byte from nor flash
 *
 * @param addr
 * @return uint8_t
 */
uint8_t norflash_byte_read(uint16_t addr)
{
    uint8_t data;
    NOR_ADDRESS_SET(addr);
    delay_us(1);
    data = norflash_bus_read();
    delay_us(1);
    return data;
}

/**
 * @brief Write byte to flash nor
 *
 * @param addr
 * @param data
 */
void norflash_byte_write(uint16_t addr, uint8_t data)
{
    NOR_DATA_OUTPUT;
    NOR_ADDRESS_SET(addr);
    norflash_bus_write(data);
    NOR_DATA_INPUT;
}

/**
 * @brief perfroms a read cycle on bus
 *
 * @return uint8_t
 */
uint8_t norflash_bus_read(void)
{
    RD_NOR_LOW;
    delay_us(1);
    uint8_t data = NOR_DATA_READ;
    delay_us(1);
    RD_NOR_HIGH;
    return data;
}

void norflash_bus_write(uint8_t data)
{
    NOR_DATA_WRITE(data);
    delay_us(10);
    WR_NOR_LOW;
    delay_us(10);
    WR_NOR_HIGH;
}
