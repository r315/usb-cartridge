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

#define setA_1(S)  GPIOF->scr = (1 << 23) | ( (S) << 7) // A-1 (Address line -1)
#define LSB_NOR_LOW     setA_1(0)
#define LSB_NOR_HIGH    setA_1(1)

flash_res_t norflash_init(void);
void norflash_deinit(void);
uint8_t norflash_readbyte(uint16_t addr);
void norflash_writebyte(uint16_t addr, uint8_t data);
flash_res_t norflash_write(const uint8_t* pbuffer, uint32_t writeaddr, uint16_t numbytetowrite);
flash_res_t norflash_read(uint8_t* pbuffer, uint32_t readaddr, uint16_t numbytetoread);
flash_res_t norflash_polling(uint8_t expecteddata);
uint8_t norflash_readbus(void);
#endif