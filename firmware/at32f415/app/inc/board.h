#ifndef _board_h_
#define _board_h_

#include "at32_norflash.h"
#include "at32_spiflash.h"
#include "at32f415.h"

#define LED1_OFF        GPIOA->scr = (1 << 8)
#define LED1_ON         GPIOA->clr = (1 << 8)
#define LED1_TOGGLE     GPIOA->odt = GPIOA->idt ^ (1 << 8)
#define LED1_INIT \
        CRM->apb2en_bit.gpioaen = 1; \
        GPIOA->cfghr |= (6 << 0)

enum mem_bus {
    MEM_BUS_NONE = 0,
    MEM_BUS_SPI,
    MEM_BUS_NOR,
};

void board_init(void);
void system_clock_config(void);
void system_tick_init(void);
void cycle_count_init(void);
void delay_init(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
uint32_t get_tick(void);
void usb_clock48m_select(usb_clk48_s clk_s);
void usb_config(void);
void usb_unplug(void);
uint8_t usb_isConnected(void);
void disconnectUSB(void);
void connectUSB(void);
void sw_reset(void);
uint32_t serial_read(uint8_t *data, uint32_t len);
uint32_t serial_write(const uint8_t *buf, uint32_t len);
uint32_t serial_available(void);
uint8_t isInserted(void);
void insertDetection_init(void);
flash_res_t rom_read(uint8_t *data, uint32_t address, uint32_t len);
void rom_setBank(uint8_t bank);
uint8_t rom_byte_read(uint32_t address);
flash_res_t rom_program(const uint8_t *data, uint32_t addr, uint32_t len);
void mem_bus_configure(uint8_t bus);
#endif