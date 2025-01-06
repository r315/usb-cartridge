#include <stddef.h>
#include "board.h"
#include "flashspi.h"

#define FLASH_DEVICES_COUNT sizeof (flashspi_devices) / sizeof (flash_t *)
#define FLASH_SPI_TIMEOUT       30000UL

static uint32_t flashspi_read_id (void);
static void flashspi_writepage (const uint8_t *pbuffer, uint32_t writeaddr, uint16_t numbytetowrite);
static void flashspi_writebuffer (const uint8_t *pbuffer, uint32_t writeaddr, uint16_t numbytetowrite);

extern const flash_t gd25lq16;
extern const flash_t w25q64;
extern const flash_t w25q128;
extern const flash_t w25x32;
extern const flash_t mx25l1635;
extern const flash_t at25sf321b;

static const flash_t *spiflash;
static const flash_t *flashspi_devices[] = {
    &gd25lq16,
    &w25q64,
    &w25q128,
    &w25x32,
    &mx25l1635,
    &at25sf321b
};

/**
 * @brief Calls SOC low level spi initialization and select flash
 *        according read id.
 * @param  None
 * @retval None
 */
flash_res_t flashspi_init (void)
{
   spiflash_init ();

   uint32_t device_id = flashspi_read_id ();

   spiflash = NULL;

   for (uint8_t i = 0; i < FLASH_DEVICES_COUNT; i++)
   {
      if (flashspi_devices[i]->mid == device_id)
      {
         spiflash = flashspi_devices[i];
         break;
      }
   }

   if(spiflash){
      return spiflash->init();
   }

   return FLASH_ERROR;
}

/**
 * @brief  Get current active spi flash device.
 * @param  None
 * @retval
 */
const flash_t *flashspi_get (void)
{
   return spiflash;
}

/**
 * @brief  Get FLASH SIZE.
 * @param  None
 * @retval FLASH identification
 */
uint32_t flashspi_getsize (void)
{
   return (spiflash) ? spiflash->size : 0;
}

/**
 * @brief  Get FLASH Sector SIZE.
 * @param  None
 * @retval FLASH identification
 */
uint32_t flashspi_getsectorsize (void)
{
   return (spiflash) ? spiflash->sectorsize : 0;
}

/**
 * @brief  Get FLASH Page SIZE.
 * @param  None
 * @retval FLASH identification
 */
uint32_t flashspi_getpagesize (void)
{
   return (spiflash) ? spiflash->pagesize : 0;
}

/**
 * @brief  Get FLASH Page SIZE.
 * @param  None
 * @retval FLASH identification
 */
const char* flashspi_getname (void)
{
   return (spiflash) ? spiflash->name : "";
}

/**
 * @brief  Reads a block of data from the FLASH.
 * @param  pBuffer: pointer to the buffer that receives the data read from the
 * FLASH.
 * @param  ReadAddr: FLASH's internal address to read from.
 * @param  NumByteToRead: number of bytes to read from the FLASH.
 * @retval None
 */
flash_res_t flashspi_read (uint8_t *pbuffer, uint32_t readaddr,
                     uint32_t numbytetoread)
{
   /*!< select the flash: chip select low */
   spiflash_cs (CS_LOW);

   /*!< send "read from memory " instruction */
   spiflash_sendbyte (FLASH_SPI_CMD_READ);
   /*!< send readaddr high nibble address byte to read from */
   spiflash_sendbyte ((readaddr & 0xff0000) >> 16);
   /*!< send readaddr medium nibble address byte to read from */
   spiflash_sendbyte ((readaddr & 0xff00) >> 8);
   /*!< send readaddr low nibble address byte to read from */
   spiflash_sendbyte (readaddr & 0xff);

   while (numbytetoread--) /*!< while there is data to be read */
   {
      /*!< read a byte from the flash */
      *pbuffer = spiflash_sendbyte (FLASH_DUMMY_BYTE);
      /*!< point to the next location where the byte read will be saved */
      pbuffer++;
   }

   /*!< deselect the flash: chip select high */
   spiflash_cs (CS_HIGH);

   return FLASH_OK;
}

/**
 * @brief  Writes block of data to the FLASH. In this function, the number of
 *         WRITE cycles are reduced, using Page WRITE sequence.
 * @param  pBuffer: pointer to the buffer  containing the data to be written
 *         to the FLASH.
 * @param  WriteAddr: FLASH's internal address to write to.
 * @param  NumByteToWrite: number of bytes to write to the FLASH.
 * @retval None
 */
flash_res_t flashspi_write (const uint8_t *pbuffer, uint32_t writeaddr,
                      uint32_t numbytetowrite)
{
   uint32_t cnt = 0;
   uint32_t sectornum, sectoroffset, sectorremain;
   uint8_t gtmpbuff[spiflash->sectorsize];

   sectornum    = writeaddr / spiflash->sectorsize;
   sectoroffset = writeaddr % spiflash->sectorsize;
   sectorremain = spiflash->sectorsize - sectoroffset;

   if (numbytetowrite <= sectorremain)
   {
      sectorremain = numbytetowrite;
   }

   while (1)
   {
      /*read all sector data*/
      flashspi_read (gtmpbuff, sectornum * spiflash->sectorsize,
                      spiflash->sectorsize);

      /*check data is 0xff ?*/
      for (cnt = 0; cnt < sectorremain; cnt++)
      {
         if (gtmpbuff[sectoroffset + cnt] != 0xff)
            break;
      }

      if (cnt < sectorremain) /*need sector erase*/
      {
         flashspi_erasesector (sectornum);
         for (cnt = 0; cnt < sectorremain; cnt++)
            gtmpbuff[cnt + sectoroffset] = pbuffer[cnt];
         flashspi_writebuffer (gtmpbuff, sectornum * spiflash->sectorsize,
                                spiflash->sectorsize);
      }
      else
      {
         flashspi_writebuffer (pbuffer, writeaddr, sectorremain);
      }

      if (sectorremain == numbytetowrite)
         break;

      else
      {
         sectornum++;
         sectoroffset = 0;
         pbuffer += sectorremain;
         writeaddr += sectorremain;
         numbytetowrite -= sectorremain;
         if (numbytetowrite > spiflash->sectorsize)
         {
            sectorremain = spiflash->sectorsize;
         }
         else
         {
            sectorremain = numbytetowrite;
         }
      }
   }

   return FLASH_OK;
}

/**
 * @brief Performs a chip erase
 *
 * @return flash_res_t
 */
flash_res_t flashspi_erase(void)
{
    if(!spiflash){
        return FLASH_ERROR;
    }

    return spiflash->erase(FLASH_CHIP_ERASE);
}

/**
 * @brief  Reads FLASH identification.
 * @param  None
 * @retval FLASH identification
 */
static uint32_t flashspi_read_id (void)
{
   uint16_t flash_id = 0;
   spiflash_cs (CS_LOW);
   spiflash_sendbyte (FLASH_SPI_CMD_REMS);
   spiflash_sendbyte (0x00);
   spiflash_sendbyte (0x00);
   spiflash_sendbyte (0x00);
   flash_id |= spiflash_readbyte () << 8;
   flash_id |= spiflash_readbyte ();
   spiflash_cs (CS_HIGH);
   return flash_id;
}

uint32_t flashspi_read_id_jedec()
{
    uint32_t device_id = 0;
    spiflash_cs (CS_LOW);
    spiflash_sendbyte (FLASH_SPI_CMD_RDID);
    device_id |= spiflash_readbyte () << 16; // M7-M0
    device_id |= spiflash_readbyte () << 8;  // ID15-ID8
    device_id |= spiflash_readbyte () << 0;  // ID7-ID0
    spiflash_cs (CS_HIGH);
    return device_id;
}

uint8_t flashspi_read_status(void)
{
    uint8_t sr_data;

    spiflash_cs (CS_LOW);
    spiflash_sendbyte (FLASH_SPI_CMD_RDSR);
    sr_data = spiflash_sendbyte (FLASH_DUMMY_BYTE);
    spiflash_cs (CS_HIGH);

    return sr_data;
}

/**
 * @brief  Writes block of data to the FLASH. In this function, the number of
 *         WRITE cycles are reduced, using Page WRITE sequence.
 * @param  pBuffer: pointer to the buffer  containing the data to be written
 *         to the FLASH.
 * @param  WriteAddr: FLASH's internal address to write to.
 * @param  NumByteToWrite: number of bytes to write to the FLASH.
 * @retval None
 */
static void flashspi_writebuffer (const uint8_t *pbuffer, uint32_t writeaddr,
                            uint16_t numbytetowrite)
{
   uint16_t pager;
   pager = spiflash->pagesize - (writeaddr % spiflash->pagesize);
   if (numbytetowrite <= pager)
   {
      pager = numbytetowrite;
   }

   while (1)
   {
      flashspi_writepage (pbuffer, writeaddr, pager);
      if (pager == numbytetowrite)
         break;
      else
      {
         pbuffer += pager;
         writeaddr += pager;
         numbytetowrite -= pager;
         if (numbytetowrite > spiflash->pagesize)
            pager = spiflash->pagesize;
         else
            pager = numbytetowrite;
      }
   }
}

/**
 * @brief  Writes more than one byte to the FLASH with a single WRITE cycle
 *         (Page WRITE sequence).
 * @note   The number of byte can't exceed the FLASH page size.
 * @param  pBuffer: pointer to the buffer  containing the data to be written
 *         to the FLASH.
 * @param  WriteAddr: FLASH's internal address to write to.
 * @param  NumByteToWrite: number of bytes to write to the FLASH, must be equal
 *         or less than "sFLASH_PAGESIZE" value.
 * @retval None
 */
static void flashspi_writepage (const uint8_t *pbuffer, uint32_t writeaddr,
                                 uint16_t numbytetowrite)
{
   /*!< enable the write access to the flash */
   flashspi_writeenable ();

   /*!< select the flash: chip select low */
   spiflash_cs (CS_LOW);
   /*!< send "write to memory " instruction */
   spiflash_sendbyte (FLASH_SPI_CMD_PP);
   /*!< send writeaddr high nibble address byte to write to */
   spiflash_sendbyte ((writeaddr & 0xff0000) >> 16);
   /*!< send writeaddr medium nibble address byte to write to */
   spiflash_sendbyte ((writeaddr & 0xff00) >> 8);
   /*!< send writeaddr low nibble address byte to write to */
   spiflash_sendbyte (writeaddr & 0xff);

   /*!< while there is data to be written on the flash */
   while (numbytetowrite--)
   {
      /*!< send the current byte */
      spiflash_sendbyte (*pbuffer);
      /*!< point on the next byte to be written */
      pbuffer++;
   }

   /*!< deselect the flash: chip select high */
   spiflash_cs (CS_HIGH);

   /*!< wait the end of flash writing */
   flashspi_write_end_wait (FLASH_SPI_TIMEOUT);
}

/**
 * @brief  Enables the write access to the FLASH.
 * @param  None
 * @retval None
 */
void flashspi_writeenable (void)
{
   /*!< Select the FLASH: Chip Select low */
   spiflash_cs (CS_LOW);

   /*!< Send "Write Enable" instruction */
   spiflash_sendbyte (FLASH_SPI_CMD_WREN);

   /*!< Deselect the FLASH: Chip Select high */
   spiflash_cs (CS_HIGH);
}

/**
 * @brief  Polls the status of the Write In Progress (WIP) flag in the FLASH's
 *         status register and loop until write opertaion has completed.
 * @param  None
 * @retval None
 */
flash_res_t flashspi_write_end_wait (uint32_t timeout)
{
    uint32_t time;
    flash_res_t res = FLASH_ERROR_TIMEOUT;

    time = get_tick();

    spiflash_cs (CS_LOW);

    spiflash_sendbyte (FLASH_SPI_CMD_RDSR);

    /*!< Loop as long as the memory is busy with a write cycle */
    do{
        uint8_t flashstatus = spiflash_sendbyte (FLASH_DUMMY_BYTE);
        if(!(flashstatus & FLASH_SPI_SR_BSY)){
            res = FLASH_OK;
            break;
        }
   } while (get_tick() - time < timeout); /* Write in progress */

   spiflash_cs (CS_HIGH);

   return res;
}

/**
 * @brief  Erases the specified FLASH sector.
 * @param  SectorAddr: address of the sector to erase.
 * @retval None
 */
void flashspi_erasesector (uint32_t sector)
{
   // Calculate sector address
   sector = sector * spiflash->sectorsize;

   /*!< send write enable instruction */
   flashspi_writeenable ();

   /*!< sector erase */
   /*!< select the flash: chip select low */
   spiflash_cs (CS_LOW);
   /*!< send sector erase instruction */
   spiflash_sendbyte (FLASH_SPI_CMD_SE);
   /*!< send sectoraddr high nibble address byte */
   spiflash_sendbyte ((sector & 0xff0000) >> 16);
   /*!< send sectoraddr medium nibble address byte */
   spiflash_sendbyte ((sector & 0xff00) >> 8);
   /*!< send sectoraddr low nibble address byte */
   spiflash_sendbyte (sector & 0xff);
   /*!< deselect the flash: chip select high */
   spiflash_cs (CS_HIGH);

   /*!< wait the end of flash writing */
   flashspi_write_end_wait (FLASH_SPI_TIMEOUT);
}
