#include "board.h"
#include "flashnor.h"

#define EN29LV320_M_ID              0x1C7F
#define EN29LV320_SECTOR_SIZE       0x2000      /* 8kB */
#define EN29LV320_PAGE_SIZE         1           /* Single byte program */
#define EN29LV320_SIZE              0x00400000  /* 4MB */
#define EN29LV320_CHIP_ERASE_TIME   70000UL

#if EN29LV320_MODE_BYTE
#define EN29LV320_CMD_SEQ_ADDR1     0x0AAA
#define EN29LV320_CMD_SEQ_ADDR2     0x0555
#else
#define EN29LV320_CMD_SEQ_ADDR1     0x0555
#define EN29LV320_CMD_SEQ_ADDR2     0x02AA
#endif

#define EN29LV320_ADDR_RDMID_0      0x0000
#define EN29LV320_ADDR_RDMID_1      0x0200

static flash_res_t en29lv320_init(void)
{
    return FLASH_OK;
}

uint32_t en29lv320_id_read(void)
{
    uint32_t id;

    norflash_byte_write(FLASH_CMD_CFI_ADDR, FLASH_CMD_RESET);

    norflash_byte_write(EN29LV320_CMD_SEQ_ADDR1, FLASH_CMD_SEQ_DATA1);
    norflash_byte_write(EN29LV320_CMD_SEQ_ADDR2, FLASH_CMD_SEQ_DATA2);
    norflash_byte_write(EN29LV320_CMD_SEQ_ADDR1, FLASH_CMD_RMID);

    id = (norflash_byte_read(EN29LV320_ADDR_RDMID_1) << 8) |
          norflash_byte_read(EN29LV320_ADDR_RDMID_0);

    norflash_byte_write(EN29LV320_CMD_SEQ_ADDR1, FLASH_CMD_RESET);

    return id;
}

/**
 * @brief Chip erase
 *
 * @return flash_res_t
 */
static flash_res_t en29lv320_erase(uint32_t sector)
{
    norflash_byte_write(EN29LV320_CMD_SEQ_ADDR1, FLASH_CMD_SEQ_DATA1);
    norflash_byte_write(EN29LV320_CMD_SEQ_ADDR2, FLASH_CMD_SEQ_DATA2);
    norflash_byte_write(EN29LV320_CMD_SEQ_ADDR1, FLASH_CMD_ERASE);
    norflash_byte_write(EN29LV320_CMD_SEQ_ADDR1, FLASH_CMD_SEQ_DATA1);
    norflash_byte_write(EN29LV320_CMD_SEQ_ADDR2, FLASH_CMD_SEQ_DATA2);

    if(sector == FLASH_CHIP_ERASE){
        norflash_byte_write(EN29LV320_CMD_SEQ_ADDR1, FLASH_CMD_ERASE_CHIP);
        //delay_ms(EN29LV320_CHIP_ERASE_TIME);
    }else{
        norflash_byte_write(sector * EN29LV320_SECTOR_SIZE, FLASH_CMD_ERASE_SECTOR);
    }

    return flashnor_polling(0xFF);
}

flash_res_t en29lv320_write(const uint8_t *data, uint32_t address, uint32_t len)
{
    while(len--){
        norflash_byte_write(EN29LV320_CMD_SEQ_ADDR1, FLASH_CMD_SEQ_DATA1);
        norflash_byte_write(EN29LV320_CMD_SEQ_ADDR2, FLASH_CMD_SEQ_DATA2);
        norflash_byte_write(EN29LV320_CMD_SEQ_ADDR1, FLASH_CMD_PROGRAM);
        norflash_byte_write(address++, *data);

        if(flashnor_polling(*data) != FLASH_OK){
            return FLASH_ERROR;
        }
        data++;
    }

    return FLASH_OK;
}

const flash_t en29lv320 = {
    .name = "EN29LV320",
    .opt = FLASH_OPT_ITF_NOR | FLASH_OPT_CFI,
    .mid = EN29LV320_M_ID,
    .size = EN29LV320_SIZE,
    .pagesize = EN29LV320_PAGE_SIZE,
    .sectorsize = EN29LV320_SECTOR_SIZE,
    .init = en29lv320_init,
    .erase = en29lv320_erase,
    .read = flashnor_read,
    .write = en29lv320_write,
    .id_read = en29lv320_id_read
};