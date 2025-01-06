#ifndef FLASHSPI_H
#define FLASHSPI_H

#include <stdint.h>
#include "flash.h"

#define FLASH_SPI_CMD_PP        0x02  /*!< Page Program */
#define FLASH_SPI_CMD_READ      0x03  /*!< Read Page Register instruction */
#define FLASH_SPI_CMD_RDSR	    0x05  /*!< Read Status Register instruction */
#define FLASH_SPI_CMD_WREN      0x06  /*!< Write enable instruction */
#define FLASH_SPI_CMD_SE        0x20  /*!< Sector Erase instruction */
#define FLASH_SPI_CMD_REMS      0x90  /*!< Read Electronic Manufacturer Signature */
#define FLASH_SPI_CMD_RDID      0x9F  /*!< Read ID (JEDEC Manufacturer ID and JEDEC CFI) */
#define FLASH_SPI_CMD_CE        0xC7  /*!< Chip Erase */

#define FLASH_SPI_SR_BSY        1

flash_res_t flashspi_init(void);
flash_res_t flashspi_write(const uint8_t* pbuffer, uint32_t writeaddr, uint32_t numbytetowrite);
flash_res_t flashspi_read(uint8_t* pbuffer, uint32_t readaddr, uint32_t numbytetoread);
void flashspi_writeenable(void);
uint32_t flashspi_getsize(void);
uint32_t flashspi_getsectorsize(void);
uint32_t flashspi_getpagesize(void);
const char* flashspi_getname (void);
const flash_t *flashspi_get(void);
flash_res_t flashspi_erase(void);
void flashspi_erasesector (uint32_t sector);
uint32_t flashspi_read_id_jedec(void);
uint8_t flashspi_read_status(void);
flash_res_t flashspi_write_end_wait (uint32_t);
#endif