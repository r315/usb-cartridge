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

uint32_t flashnor_id_read(void)
{
    uint32_t id;

    norflash_byte_write(FLASH_CMD_CFI_ADDR, FLASH_CMD_RESET);

    norflash_byte_write(FLASH_CMD_SEQ_ADDR1, FLASH_CMD_SEQ_DATA1);
    norflash_byte_write(FLASH_CMD_SEQ_ADDR2, FLASH_CMD_SEQ_DATA2);
    norflash_byte_write(FLASH_CMD_SEQ_ADDR1, FLASH_CMD_RMID);

    id = (norflash_byte_read(FLASH_ADDR_RDMID_1) << 8) |
          norflash_byte_read(FLASH_ADDR_RDMID_0);

    norflash_byte_write(FLASH_CMD_SEQ_ADDR1, FLASH_CMD_RESET);

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

uint32_t flashnor_cfi_read(uint8_t *buf)
{
    norflash_byte_write(FLASH_CMD_CFI_ADDR, FLASH_CMD_CFI_ENTER);

    for (uint8_t offset = 0; offset < 64; offset++)
        *buf++ = norflash_byte_read(FLASH_CMD_CFI_BASE_ADDR + (offset << 1));

    norflash_byte_write(FLASH_CMD_CFI_ADDR, FLASH_CMD_RESET);

    return 64;
}

flash_res_t flashnor_init(void)
{
    norflash_init();

    for (uint8_t i = 0; i < FLASH_DEVICES_COUNT; i++)
    {
        flashnor = flashnor_devices[i];
        if (flashnor->mid == flashnor->id_read())
        {
            return flashnor->init();
        }
    }

    flashnor = NULL;

    return FLASH_ERROR;
}

void flashnor_deinit(void)
{
    norflash_deinit();
}

flash_res_t flashnor_read(uint8_t *data, uint32_t address, uint32_t len)
{
    while(len--){
        NOR_ADDRESS_SET(address);
        delay_us(1);
        *data++ = norflash_bus_read();
        delay_us(1);
        address++;
    }

    return FLASH_OK;
}

flash_res_t flashnor_write(const uint8_t *data, uint32_t address, uint32_t len)
{
    while(len--){
        norflash_byte_write(FLASH_CMD_SEQ_ADDR1, FLASH_CMD_SEQ_DATA1);
        norflash_byte_write(FLASH_CMD_SEQ_ADDR2, FLASH_CMD_SEQ_DATA2);
        norflash_byte_write(FLASH_CMD_SEQ_ADDR1, FLASH_CMD_PROGRAM);
        norflash_byte_write(address++, *data);

        if(flashnor_polling(*data) != FLASH_OK){
            return FLASH_ERROR;
        }
        data++;
    }

    return FLASH_OK;
}

/**
 * @brief generic bit6 toggle polling
 *
 * @param expecteddata
 * @return flash_res_t
 */
flash_res_t flashnor_polling(uint8_t expecteddata)
{
    uint8_t last_status = norflash_bus_read();
    uint8_t toggle = 2;
    uint32_t time = get_tick();

    do{
        uint8_t status = norflash_bus_read();

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