#include <stdio.h>
#include "board.h"
#include "flashspi.h"

#define W25Q64_M_ID             0xEF16
#define W25Q64_DEV_ID           0xEF4017  // Single

#define W25Q128_M_ID            0xEF17

#define W25X32_M_ID             0xEF15
#define W25X32_DEV_ID           0xEF3016   // Manufacturer and device id

#define W25X32_PAGE_SIZE        256
#define W25X32_SECTOR_SIZE      0x1000     /* 4kB */
#define W25X32_SIZE             0x00400000 /* 4MB */

#define WINBOND_tCE             100000UL    // 100s

#define W25X32_CMD_WRSR         0x01
#define W25X32_CMD_WRITE_DIS    0x04
#define W25X32_CMD_READ_FAST    0x0B
#define W25X32_CMD_READ_FAST_DUAL  0x3B
#define W25X32_CMD_BLOCK_ERASE  0xD8
#define W25X32_CMD_CHIP_ERASE   0xC7
#define W25X32_CMD_POWER_DOWN   0xB9
#define W25X32_CMD_POWER_UP_ID  0xAB

#define WINDOND_SR_BUSY         (1<<0)

static flash_res_t w25q_init(void)
{
    return FLASH_OK;
}

static flash_res_t w25x32_init(void)
{
    return (flashspi_read_id_jedec() == W25X32_DEV_ID) ? FLASH_OK: FLASH_ERROR_ID;
}

static flash_res_t winbond_erase(uint32_t sector)
{
    uint32_t time;
    flashspi_writeenable();

    spiflash_cs (CS_LOW);
    spiflash_sendbyte (W25X32_CMD_CHIP_ERASE);
    spiflash_cs (CS_HIGH);
    time = get_tick();

    do{
        delay_ms(1);
        uint8_t sr = flashspi_read_status();
        if(!(sr & WINDOND_SR_BUSY)){
            return FLASH_OK;
        }
    }while(get_tick() - time < WINBOND_tCE);

    return FLASH_ERROR_TIMEOUT;
}

const flash_t w25x32 = 
{
    .name = "W25X32",
    .mid = W25X32_M_ID,
    .itf = FLASH_ITF_SPI,
    .size = W25X32_SIZE,
    .pagesize = W25X32_PAGE_SIZE,
    .sectorsize = W25X32_SECTOR_SIZE,  
    .init = w25x32_init,
    .erase = winbond_erase,
    .read = flashspi_read,
    .write = flashspi_write
};

const flash_t w25q64 = 
{
    .name = "W25Q64",
    .mid = W25Q64_M_ID,
    .itf = FLASH_ITF_SPI,
    .size = 0x00800000, /*8m byte*/
    .pagesize = 256,
    .sectorsize = 0x1000,
    .init = w25q_init,
    .erase = winbond_erase,
    .read = flashspi_read,
    .write = flashspi_write
};

const flash_t w25q128 = 
{
    .name = "W25Q128",
    .mid = W25Q128_M_ID,
    .itf = FLASH_ITF_SPI,
    .size = 0x01000000, /*16m byte*/
    .pagesize = 256,
    .sectorsize = 0x1000,
    .init = w25q_init,
    .erase = winbond_erase,
    .read = flashspi_read,
    .write = flashspi_write
};


