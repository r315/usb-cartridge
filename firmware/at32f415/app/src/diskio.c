/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various existing       */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "msc_diskio.h" /* Declarations of disk functions */
#include "cdc_msc_class.h"
#include "usb_std.h"
#include "flashspi.h"
#include "board.h"

uint8_t scsi_inquiry[MSC_SUPPORT_MAX_LUN][SCSI_INQUIRY_DATA_LENGTH] = {
    /* lun = 0 */
    {
#ifdef MSC_CDROM
        0x05,
        0x08,
        0x02,
        0x02,
#else   // mass storage
        0x00, /* peripheral device type (direct-access device) */
        0x80, /* removable media bit */
        0x00, /* ansi version, ecma version, iso version */
        0x01, /* respond data format */
#endif

        SCSI_INQUIRY_DATA_LENGTH - 5, /* additional length */
        0x00,
        0x00,
        0x00, /* reserved */
        'M',  /* vendor information */
        'B',
        'C',
        '!',
        ' ',
        ' ',
        ' ',
        ' ', 
        'F', /* Product identification */
        'l',
        'a',
        's',
        'h',
        ' ',
        'd',
        'i',
        's',
        'k',
        ' ',
        ' ',
        ' ',
        ' ',
        ' ',
        ' ', 
        '1', /* product revision level */
        '.',
        '0',
        '0' 
    }
};


/**
 * @brief
 * @param
 * @retval
 */
DSTATUS msc_init (uint8_t lun)
{
    if(lun == SPI_FLASH_LUN){
        return flashspi_init ();
    }

    return USB_ERROR;
}

/**
 * @brief  get disk inquiry
 * @param  lun: logical units number
 * @retval inquiry string
 */
uint8_t *get_inquiry (uint8_t lun)
{
   if (lun < MSC_SUPPORT_MAX_LUN)
      return (uint8_t *) scsi_inquiry[lun];
   else
      return NULL;
}

/**
 * @brief  Read data block from disk
 * 
 * @param  lun:   logical units number
 * @param  addr:  logical address
 * @param  buf:   pointer to read buffer
 * @param  len:   read length
 * @retval status of usb_sts_type
 */
DSTATUS msc_disk_read (uint8_t lun, uint32_t addr, uint8_t *buf, uint32_t len)
{
    if(lun == SPI_FLASH_LUN){
        return (usb_sts_type) flashspi_read (buf, addr, len);
    }

    return USB_NOT_SUPPORT;
}

/**
 * @brief  Write data block to disk
 * 
 * @param  lun:  logical units number
 * @param  addr: logical address
 * @param  buf:  pointer to write buffer
 * @param  len:  write length
 * @retval status of usb_sts_type
 */
DSTATUS msc_disk_write (uint8_t lun, uint32_t addr, uint8_t *buf, uint32_t len)
{
   if(lun == SPI_FLASH_LUN){
        return (usb_sts_type) flashspi_write (buf, addr, len);
    }

    return USB_NOT_SUPPORT;
}

/**
 * @brief  Responds with disk capacity in blocks and block size
 * 
 * @param  lun: logical units number
 * @param  blk_nbr: pointer to number of block
 * @param  blk_size: pointer to block size
 * @retval status of usb_sts_type
 */
DSTATUS msc_disk_capacity (uint8_t lun, uint32_t *blk_nbr, uint32_t *blk_size)
{
    if(lun == SPI_FLASH_LUN){
        *blk_size = 512;
        *blk_nbr  = flashspi_getsize () / 512;
        return USB_OK;
    }
    
    return USB_ERROR;
}

/*-----------------------------------------------------------------*/
/* Disk access API for FF                                          */
/*-----------------------------------------------------------------*/

/**
 * @brief Check if requested drive is available
 * 
 * @param pdrv Physical drive number to identify the drive
 * @return DSTATUS 
 */
DSTATUS disk_status (BYTE pdrv)
{
   DSTATUS status = STA_NOINIT;

    if(pdrv == SPI_FLASH_DISK){
        status &= ~STA_NOINIT;
    }

   return status;
}

/**
 * @brief Initialyses disk
 * 
 * @param pdrv Physical drive number to identify the drive
 * @return DSTATUS 
 */
DSTATUS disk_initialize (BYTE pdrv)
{
   DSTATUS status = STA_NOINIT;

    if(pdrv == SPI_FLASH_DISK){
        if ((DRESULT) flashspi_init () == RES_OK){
            status &= ~STA_NOINIT;
        }
    }

    return status;   
}

/**
 * @brief Read sectors from disk
 * 
 * @param pdrv   Physical drive number to identify the drive
 * @param buff   Data buffer to store read data
 * @param sector Start sector in LBA
 * @param count  Number of sectors to read
 */
DRESULT disk_read (BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    if(pdrv == SPI_FLASH_DISK){
        return (DRESULT) flashspi_read (buff, sector * 512, count * 512);
    }

   return RES_PARERR;
}

#if FF_FS_READONLY == 0
/**
 * @brief Write sectors to disk
 * 
 * @param pdrv   Physical drive number to identify the drive
 * @param buff   Data to be written
 * @param sector Start sector in LBA
 * @param count  Number of sectors to written
 * @return DRESULT 
 */
DRESULT disk_write (BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    if(pdrv == SPI_FLASH_DISK){
        return (DRESULT) flashspi_write (buff, sector * 512, count * 512);
    }

    return RES_PARERR;
}
#endif



/**
 * @brief 
 * 
 * @param pdrv  Physical drive number to identify the drive
 * @param cmd   Control code
 * @param buff  Buffer to send/receive control data
 * @return DRESULT 
 */
DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void *buff)
{
    DRESULT status = RES_PARERR;
      
    if(pdrv == SPI_FLASH_DISK){
        switch (cmd){
            case CTRL_SYNC:
                status = RES_OK;
                break;
            case GET_SECTOR_SIZE:
                *(DWORD *) buff = 512;
                status          = RES_OK;
                break;
            case GET_SECTOR_COUNT:
                *(DWORD *) buff =  flashspi_getsize () / 512;
                status          = RES_OK;
                break;
            case GET_BLOCK_SIZE:
                *(DWORD *) buff =  flashspi_getsize ();
                status          = RES_OK;
                break;
            default:
                status = RES_PARERR;
                break;
        }            
   }
   
   return status;
}
