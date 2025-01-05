#include "board.h"
#include "flashnor.h"

//Fujitsu MBM29F040C-70
#define MBM29F040_M_ID            0xA404
#define MBM29F040_SECTOR_SIZE     0x10000    /* 64k */
#define MBM29F040_PAGE_SIZE       1          /* 1 byte program */
#define MBM29F040_SIZE            0x00080000 /* 512kB */
#define MBM29F040_CHIP_ERASE_TIME 70000UL


static flash_res_t mbm29f040_init(void)
{
    return FLASH_OK;
}

/**
 * @brief Chip erase
 *
 * @return flash_res_t
 */
static flash_res_t mbm29f040_erase(uint32_t sector)
{
    norflash_byte_write(FLASH_CMD_SEQ_ADDR1, FLASH_CMD_SEQ_DATA1);
    norflash_byte_write(FLASH_CMD_SEQ_ADDR2, FLASH_CMD_SEQ_DATA2);
    norflash_byte_write(FLASH_CMD_SEQ_ADDR1, FLASH_CMD_ERASE);
    norflash_byte_write(FLASH_CMD_SEQ_ADDR1, FLASH_CMD_SEQ_DATA1);
    norflash_byte_write(FLASH_CMD_SEQ_ADDR2, FLASH_CMD_SEQ_DATA2);

    if(sector == FLASH_CHIP_ERASE){
        norflash_byte_write(FLASH_CMD_SEQ_ADDR1, FLASH_CMD_ERASE_CHIP);
        //delay_ms(f29f040_CHIP_ERASE_TIME);
    }else{
        norflash_byte_write(sector * MBM29F040_SECTOR_SIZE, FLASH_CMD_ERASE_SECTOR);
    }

    return flashnor_polling(0xFF);
}


const flash_t mbm29f040 = {
    .name = "29f040",
    .opt = FLASH_OPT_ITF_NOR,
    .mid = MBM29F040_M_ID,
    .size = MBM29F040_SIZE,
    .pagesize = MBM29F040_PAGE_SIZE,
    .sectorsize = MBM29F040_SECTOR_SIZE,
    .init = mbm29f040_init,
    .erase = mbm29f040_erase,
    .read = flashnor_read,
    .write = flashnor_write
};