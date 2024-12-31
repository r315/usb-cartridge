#include <stdint.h>
#include <stddef.h>
#include "board.h"
#include "flash.h"
#include "flashnor.h"

#define FLASH_DEVICES_COUNT sizeof(flashnor_devices) / sizeof(flash_t *)

extern const flash_t en29lv320;
extern const flash_t mbm29f040;

static const flash_t *flashnor;
static const flash_t *flashnor_devices[] = {
    &en29lv320,
    &mbm29f040};

uint32_t flashnor_read_id(void)
{
    uint32_t id;
#ifdef MBC1_REVA
    // MBC1 Flash supports word access, thus id sequence is different
    norflash_writebyte(FLASH_CMD_CFI_ADDR, FLASH_CMD_RESET);
    norflash_writebyte(FLASH_CMD_SEQ_ADDR1, FLASH_CMD_SEQ_DATA1);
    norflash_writebyte(FLASH_CMD_SEQ_ADDR2, FLASH_CMD_SEQ_DATA2);
    norflash_writebyte(FLASH_CMD_SEQ_ADDR1, FLASH_CMD_RMID);
    id = norflash_readbyte(FLASH_ADDR_RDMID_0);
    id = norflash_readbyte(FLASH_ADDR_RDMID_1) << 8 | id;
#else
    norflash_writebyte(0x555, 0xAA);
    norflash_writebyte(0x2AA, 0x55);
    norflash_writebyte(0x555, FLASH_CMD_RMID);
    id = (norflash_readbyte(1) << 8) | norflash_readbyte(0);
#endif

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
const flash_t *flashnor_get(void)
{
    return flashnor;
}

uint32_t flashnor_getCFI(uint8_t *buf)
{
    norflash_writebyte(FLASH_CMD_CFI_ADDR, FLASH_CMD_CFI_ENTER);

    for (uint8_t offset = 0; offset < 64; offset++)
        *buf++ = norflash_readbyte(FLASH_CMD_CFI_BASE_ADDR + (offset << 1));

    norflash_writebyte(FLASH_CMD_CFI_ADDR, FLASH_CMD_RESET);

    return 64;
}

flash_res_t flashnor_init(void)
{
    norflash_init();

    uint32_t device_id = flashnor_read_id();

    flashnor = NULL;

    for (uint8_t i = 0; i < FLASH_DEVICES_COUNT; i++)
    {
        if (flashnor_devices[i]->mid == device_id)
        {
            flashnor = flashnor_devices[i];
            break;
        }
    }

    if (flashnor)
    {
        return flashnor->init();
    }

    return FLASH_ERROR;
}

void flashnor_deinit(void)
{
    norflash_deinit();
}

flash_res_t flashnor_read(uint8_t *data, uint32_t address, uint32_t len)
{
    return flashnor->read(data, address, len);
}

flash_res_t flashnor_write(uint8_t *data, uint32_t address, uint32_t len)
{
    return flashnor->write(data, address, len);
}
