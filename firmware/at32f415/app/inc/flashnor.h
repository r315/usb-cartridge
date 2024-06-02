#ifndef _flashnor_h_
#define _flashnor_h_

#include <stdint.h>
#include "flash.h"

#define FLASH_CMD_CFI_ADDR      0x00AA
#define FLASH_CMD_CFI_ENTER     0x98
#define FLASH_CMD_CFI_BASE_ADDR 0x0020

#define FLASH_CMD_SEQ_ADDR1     0x0AAA
#define FLASH_CMD_SEQ_ADDR2     0x0555
#define FLASH_CMD_SEQ_DATA1     0xAA
#define FLASH_CMD_SEQ_DATA2     0x55

#define FLASH_CMD_RMID          0x90
#define FLASH_CMD_RESET         0xF0

#define FLASH_ADDR_RDMID_0      0x0000
#define FLASH_ADDR_RDMID_1      0x0200


flash_res_t flashnor_init(void);
void flashnor_deinit(void);
flash_res_t flashnor_read(uint8_t *data, uint32_t address, uint32_t len);
flash_res_t flashnor_write(uint8_t *data, uint32_t address, uint32_t len);
uint32_t flashnor_getCFI(uint8_t *buf);
const flash_t *flashnor_get(void);

#endif