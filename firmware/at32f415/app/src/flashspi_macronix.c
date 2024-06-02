#include <stdio.h>
#include "board.h"
#include "flashspi.h"

#define MX25L1635_M_ID          0xC224
#define MX25L1635_DEV_ID        0xC22415

#define MX25L1635_PAGE_SIZE        256
#define MX25L1635_SECTOR_SIZE      0x1000     /* 4kB */
#define MX25L1635_SIZE             0x00200000 /* 2MB */

#define MACRONIX_tCE               30000UL  //30s

#define MACRONIX_CMD_WRSR          0x01

#define MACRONIX_SR_QE             (1<<6)
#define MACRONIX_SR_WIP            (1<<0)

static flash_res_t macronix_wait_wip()
{
    uint32_t time;

    time = get_tick();
    
    do{
        delay_ms(1);
        uint8_t sr = flashspi_read_status();
        if(!(sr & MACRONIX_SR_WIP)){
            return FLASH_OK;
        }
    }while(get_tick() - time < MACRONIX_tCE);

    return FLASH_ERROR_TIMEOUT;
}

static void macronix_wrsr(uint8_t sr)
{
    flashspi_writeenable();

    spiflash_cs (CS_LOW);
    spiflash_sendbyte (MACRONIX_CMD_WRSR);
    spiflash_sendbyte (sr);
    spiflash_cs (CS_HIGH);
}

static flash_res_t mx25l1635_init(void)
{
    uint8_t sr;

    if(flashspi_read_id_jedec() != MX25L1635_DEV_ID)
        return FLASH_ERROR_ID;

    sr = flashspi_read_status();

    if(sr & MACRONIX_SR_QE)
    {
        macronix_wrsr(sr & ~MACRONIX_SR_QE);
        return macronix_wait_wip();
    }

    return FLASH_OK;
}

static flash_res_t macronix_erase(uint32_t sector)
{
    flashspi_writeenable();

    spiflash_cs (CS_LOW);
    spiflash_sendbyte (FLASH_SPI_CMD_CE);
    spiflash_cs (CS_HIGH);

    return macronix_wait_wip();
}

const flash_t mx25l1635 = 
{
    .name = "MX25L1635",
    .mid = MX25L1635_M_ID,
    .itf = FLASH_ITF_SPI,
    .size = MX25L1635_SIZE,
    .pagesize = MX25L1635_PAGE_SIZE,
    .sectorsize = MX25L1635_SECTOR_SIZE,  
    .init = mx25l1635_init,
    .erase = macronix_erase,
    .read = flashspi_read,
    .write = flashspi_write
};
