#ifndef _at32_nor_h_
#define _at32_nor_h_

#include <stdint.h>
#include "flash.h"

#define WR_PAL_LOW      GPIOC->clr = (1 << 13)
#define WR_PAL_HIGH     GPIOC->scr = (1 << 13)
#define WR_NOR_LOW      GPIOC->clr = (1 << 14)
#define WR_NOR_HIGH     GPIOC->scr = (1 << 14)
#define RD_NOR_LOW      GPIOC->clr = (1 << 15)
#define RD_NOR_HIGH     GPIOC->scr = (1 << 15)

#define setA_1(S)       GPIOF->scr = (1 << 23) | ( (S) << 7) // A-1 (Address line -1)
#define LSB_NOR_LOW     setA_1(0)
#define LSB_NOR_HIGH    setA_1(1)

#ifdef MBC1_REVA
#define NOR_DATA_INPUT      GPIOA->cfglr = 0x44444444
#define NOR_DATA_OUTPUT     GPIOA->cfglr = 0x22222222
#define NOR_DATA_READ       (uint8_t)GPIOA->idt;
#define NOR_DATA_WRITE(data) GPIOA->scr =  0x00FF0000 | (data); // set overrides clear
#define NOR_ADDRESS_OUTPUT  \
        GPIOB->cfglr = 0x22222222; \
        GPIOB->cfghr = 0x22222222

#define NOR_ADDRESS_INPUT  \
        GPIOB->cfglr = 0x44444444; \
        GPIOB->cfghr = 0x44444444

#define NOR_ADDRESS_SET(addr)  \
        GPIOB->odt = (addr) >> 1; \
        setA_1((addr) & 1);

// RDn, RWEn, WRn
#define NOR_CTRL_OUTPUT     GPIOC->cfghr = 0x22244444
#define NOR_CTRL_INPUT      GPIOC->cfghr = 0x44444444
#define NOR_WR_OUTPUT       GPIOC->cfghr = 0x42444444
#else
#define NOR_DATA_INPUT      GPIOB->cfghr = 0x44444444
#define NOR_DATA_OUTPUT     GPIOB->cfghr = 0x22222222
#define NOR_DATA_READ       *((uint8_t*)&GPIOB->idt + 1)
#define NOR_DATA_WRITE(data) GPIOB->scr =  0xFF000000 | ((data) << 8);
#define NOR_ADDRESS_OUTPUT  \
        GPIOB->cfglr = 0x22222222; \
        GPIOA->cfglr = 0x22222222

#define NOR_ADDRESS_INPUT  \
        GPIOB->cfglr = 0x44444444; \
        GPIOA->cfglr = 0x44444444

#define NOR_ADDRESS_SET(addr)  \
        GPIOB->scr =  0x00FF0000 | ((addr) & 255); \
        GPIOA->scr =  0x00FF0000 | ((addr) >> 8)
// RDn, WRn
#define NOR_CTRL_OUTPUT     GPIOC->cfghr = 0x22444444
#define NOR_CTRL_INPUT      GPIOC->cfghr = 0x44444444
#define NOR_WR_OUTPUT       GPIOC->cfghr = 0x42444444
#endif



flash_res_t norflash_init(void);
void norflash_deinit(void);
uint8_t norflash_byte_read(uint16_t addr);
void norflash_byte_write(uint16_t addr, uint8_t data);
uint8_t norflash_bus_read(void);
void norflash_bus_write(uint8_t data);
#endif