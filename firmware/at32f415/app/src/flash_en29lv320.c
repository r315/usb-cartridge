#include "board.h"
#include "flashnor.h"

#define EN29LV320_M_ID          0x1C7F
#define EN29LV320_SECTOR_SIZE   0x2000 /* 8kB */
#define EN29LV320_PAGE_SIZE     0
#define EN29LV320_SIZE          0x00400000 /* 4MB */
#define EN29LV320_CHIP_ERASE_TIME    70000UL


static flash_res_t en29lv320_init(void)
{
    return FLASH_OK;
}

/**
 * @brief Chip erase
 * 
 * @return flash_res_t 
 */
static flash_res_t en29lv320_erase(uint32_t sector)
{
    norflash_writebyte(FLASH_CMD_SEQ_ADDR1, FLASH_CMD_SEQ_DATA1);
    norflash_writebyte(FLASH_CMD_SEQ_ADDR2, FLASH_CMD_SEQ_DATA2);
    norflash_writebyte(FLASH_CMD_SEQ_ADDR1, 0x80);
    norflash_writebyte(FLASH_CMD_SEQ_ADDR1, 0xAA);
    norflash_writebyte(FLASH_CMD_SEQ_ADDR2, 0x55);

    if(sector == FLASH_CHIP_ERASE){
        norflash_writebyte(FLASH_CMD_SEQ_ADDR1, 0x10);
        //delay_ms(EN29LV320_CHIP_ERASE_TIME);
    }else{
        norflash_writebyte(sector * EN29LV320_SECTOR_SIZE, 0x30);
    }

    return norflash_polling(0xFF);
}

/**
 * @brief Write buffer to flash
 *        This flash seams to only support single by write
 *        thus making write quite slow
 * 
 * @param data 
 * @param address 
 * @param len 
 * @return flash_res_t 
 */
static flash_res_t en29lv320_write(const uint8_t *data, uint32_t address, uint16_t len)
{
    while(len--){
        norflash_writebyte(FLASH_CMD_SEQ_ADDR1, FLASH_CMD_SEQ_DATA1);
        norflash_writebyte(FLASH_CMD_SEQ_ADDR2, FLASH_CMD_SEQ_DATA2);
        norflash_writebyte(FLASH_CMD_SEQ_ADDR1, 0xA0);
        norflash_writebyte(address++, *data);

        if(norflash_polling(*data) != FLASH_OK){
            return FLASH_ERROR;
        }
    }

    return FLASH_OK;
}

const flash_t en29lv320 = {
    .name = "EN29LV320",
    .itf = FLASH_ITF_NOR,
    .mid = EN29LV320_M_ID,
    .size = EN29LV320_SIZE,
    .pagesize = EN29LV320_PAGE_SIZE,
    .sectorsize = EN29LV320_SECTOR_SIZE,
    .init = en29lv320_init,
    .erase = en29lv320_erase,
    .read = norflash_read,
    .write = en29lv320_write
};