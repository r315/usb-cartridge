/**
  **************************************************************************
  * @file     msc_diskio.h
  * @version  v2.0.9
  * @date     2022-06-28
  * @brief    usb mass storage disk interface header file
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MSC_DISKIO_H
#define __MSC_DISKIO_H

#include <stdint.h>
#include "usb_std.h"
#include "diskio.h"

/** @addtogroup AT32F415_periph_examples
  * @{
  */

/** @addtogroup 415_USB_device_msc
  * @{
  */
 

#define SPI_FLASH_LUN                    0
#define SPI_FLASH_DISK                   SPI_FLASH_LUN

DSTATUS msc_init (uint8_t lun);
uint8_t *get_inquiry(uint8_t lun);
DSTATUS msc_disk_read(uint8_t lun, uint32_t addr, uint8_t *read_buf, uint32_t len);
DSTATUS msc_disk_write(uint8_t lun, uint32_t addr, uint8_t *buf, uint32_t len);
DSTATUS msc_disk_capacity(uint8_t lun, uint32_t *blk_nbr, uint32_t *blk_size);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif


