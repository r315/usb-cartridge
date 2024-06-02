#ifndef _flash_h_
#define _flash_h_

#include <stdint.h>

#define FLASH_CHIP_ERASE    0xFFFFFFFF

typedef enum flash_res_e{
    FLASH_OK = 0,            /* (0) Success */
    FLASH_ERROR,             /* (1) Generic error */
    FLASH_ERROR_ID,          /* (2) ID mismatch */
    FLASH_ERROR_TIMEOUT,     /* (3) Operation timedout */
}flash_res_t;

typedef enum flash_itf_e{
    FLASH_ITF_SPI = 0,
    FLASH_ITF_NOR,
    FLASH_ITF_NAND
}flash_itf_t;

typedef struct {
    const char *name;
    const uint32_t size;
    const uint32_t sectorsize;
    const uint32_t pagesize;
    const uint32_t mid;             // Manufacturer/id
    const flash_itf_t itf;
    flash_res_t (*init)(void);
    flash_res_t (*erase)(uint32_t sector);
    flash_res_t (*read)(uint8_t *data, uint32_t addr, uint16_t len);
    flash_res_t (*write)(const uint8_t *data, uint32_t addr, uint16_t len);
}flash_t;

#endif