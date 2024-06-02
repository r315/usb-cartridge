#include <stdint.h>
#include <stddef.h>
#include "board.h"
#include "flash.h"
#include "flashnor.h"

extern const flash_t en29lv320;
static const flash_t *romflash;

uint32_t flashnor_read_id(void)
{
    uint32_t id;

    norflash_writebyte(FLASH_CMD_SEQ_ADDR1, FLASH_CMD_SEQ_DATA1);
    norflash_writebyte(FLASH_CMD_SEQ_ADDR2, FLASH_CMD_SEQ_DATA2);
    norflash_writebyte(FLASH_CMD_SEQ_ADDR1, FLASH_CMD_RMID);

    id = norflash_readbyte(FLASH_ADDR_RDMID_1) << 8;
    id |= norflash_readbyte(FLASH_ADDR_RDMID_0);

    norflash_writebyte(FLASH_CMD_SEQ_ADDR1, FLASH_CMD_RESET);

    return id;
}

/**
 * @brief return data structure with 
 * detected flash information.
 * Must be called after flashnor_init
 * 
 * @return const flash_t* 
 */
const flash_t *flashnor_get(void){
    return romflash;
}

uint32_t flashnor_getCFI(uint8_t *buf)
{
    norflash_writebyte(FLASH_CMD_CFI_ADDR, FLASH_CMD_CFI_ENTER);
   
    for(uint8_t offset = 0; offset < 64; offset++ )
        *buf++ = norflash_readbyte(FLASH_CMD_CFI_BASE_ADDR + (offset << 1));
   
    norflash_writebyte(FLASH_CMD_CFI_ADDR, FLASH_CMD_RESET);

    return 64;
}

flash_res_t flashnor_init(void)
{
    norflash_init();

    norflash_writebyte(FLASH_CMD_CFI_ADDR, FLASH_CMD_RESET);

    uint32_t id = flashnor_read_id();

    if(id == en29lv320.mid){
        romflash = &en29lv320;
    }else{
        romflash = NULL;
    }    

    if(romflash){
        return romflash->init();
    }

    return FLASH_ERROR;
}

void flashnor_deinit(void)
{
    norflash_deinit();
}

flash_res_t flashnor_read(uint8_t *data, uint32_t address, uint32_t len)
{
    return romflash->read(data, address, len);
}

flash_res_t flashnor_write(uint8_t *data, uint32_t address, uint32_t len)
{
    return romflash->write(data, address, len);
}

