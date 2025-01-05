/**
  **************************************************************************
  * @file     main.c
  * @version  v2.0.8
  * @date     2022-04-25
  * @brief    main program
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "cli_simple.h"
#include "ff.h"
#include "msc_diskio.h"
#include "flashspi.h"
#include "flashnor.h"
#include "at32_norflash.h"
#include "at32_spiflash.h"

#define GAME_HEADER_SIZE    0x50
#define GAME_HEADER_ADDR    0x100
#define PROG_BLOCK_SIZE     0x400


typedef struct disksize_s{
	uint32_t totalsize;
	uint32_t freesize;
}disksize_t;

static FATFS fatfs;
const flash_t *pfls;
static uint8_t mounted;

static void printBuf(const uint8_t *buf, uint32_t len){
    for(int i = 0; i < len; i ++){
        if( (i & 15) == 0)
            printf("\n%02X: ", i & 0xF0);
        printf("%02X ", buf[i]);
    }
    putchar('\n');
}

static void printGameTitle(const uint8_t* title)
{
    for(uint8_t i = 0; i < 16; i++, title++){
        if(*title == '\0')
            break;
        putchar(*title);
    }

    putchar('\n');
}

static void printGameHeader(const uint8_t* header)
{
    // Dump logo
    printf("\nLogo:\n");
    printBuf(header + 4, 0x30);
    printGameTitle(header + 0x34);
    printf("Color:%s\n", (header[0x43] & 0x80) ? "Yes" : "No");
    printf("License: %02X%02X\n", header[0x44], header[0x45]);
    printf("GB/SGB: %02X\n", header[0x46]);
    printf("Cartridge type: %02X\n", header[0x47]);
    printf("Rom size: %02X\n", header[0x48]);
    printf("Ram size: %02X\n", header[0x49]);
    printf("Jp: %02X\n", header[0x4A]);
    printf("License (old): %02X\n", header[0x4B]);
    printf("Mask rom version: %02X\n", header[0x4C]);
    printf("Complement check: %02X\n", header[0x4D]);
    printf("Checksum: %02X%02X\n", header[0x4F], header[0x4E]);
}

FRESULT getDiskSize(TCHAR *path, disksize_t *disk_size)
{
	FATFS *pfs;
    FRESULT res;
	DWORD fre_clust;

	res = f_getfree(path, &fre_clust, &pfs);

	if(res == FR_OK){
		disk_size->totalsize = (pfs->n_fatent - 2) * pfs->csize / 2;
		disk_size->freesize = fre_clust * pfs->csize / 2;
	}

	return res;
}

FRESULT mount(TCHAR *path, uint8_t m)
{
    FRESULT r;
    disksize_t disk_size = {0};

    if(m)
    {
        if(mounted){
            printf("FS already mounted\n");
            return FR_OK;
        }

        r = f_mount(&fatfs, path, 0);  // 0 = mount later option

        printf("Mount ");
        if(r != FR_OK){
            printf("fail: %d\n", r);
        }else{
            r = getDiskSize(path, &disk_size);
            if(r == FR_OK){
                printf("ok\n"
                       "\tDisk size: %luk\n", disk_size.totalsize);
                printf("\t     Free: %luk\n", disk_size.freesize);
            }else{
                printf("FRESULT Error: %d\n", r);
            }
            mounted = 1;
        }
    }else{
        r = f_mount(NULL, "0:", 0);

        mounted = 0;

        printf("Unmounted\n");
    }

    return r;
}

static int readBlockFromFile(uint8_t *data, uint16_t len, const char *filename, uint32_t offset)
{
    FIL fil;
    FRESULT fr;
    UINT br = 0;

    if(!mounted){
        printf("No fs mounted\n");
        return -FR_NO_FILESYSTEM;
    }

    fr = f_open(&fil, filename, FA_READ);

    if(fr){
        printf("Error open: %d\n", fr);
        return -(int)fr;
    }

    fr = f_lseek(&fil, offset);

    if(fr){
        printf("Error seeking: %d", fr);
        goto err;
    }

    fr = f_read(&fil, data, len, &br);

    if(fr){
        printf("Error reading: %d", fr);
    }

err:
    f_close(&fil);

    return br;
}

static void selectFlashSPI(void){
    spiflash_init();
    pfls = flashspi_get();
}

static void selectFlashNOR(void){
    flashnor_init();
    pfls = flashnor_get();
}

static int listCmd(int argc, char **argv)
{
    FRESULT res;
    FILINFO fileInfo;
    uint32_t totalFiles = 0;
    uint32_t totalDirs = 0;
    DIR dir;

    res = f_opendir(&dir, "/");

    if(res != FR_OK) {
        printf("f_opendir() failed, res = %d\r\n", res);
        return CLI_OK;
    }

    printf("--------\r\nRoot directory:\r\n");

    for(;;) {
        res = f_readdir(&dir, &fileInfo);
        if((res != FR_OK) || (fileInfo.fname[0] == '\0')) {
            break;
        }

        if(fileInfo.fattrib & AM_DIR) {
            printf("  DIR  %s\r\n", fileInfo.fname);
            totalDirs++;
        } else {
            printf("  FILE %s\r\n", fileInfo.fname);
            totalFiles++;
        }
    }

    printf("(total: %lu dirs, %lu files)\r\n--------\r\n", totalDirs, totalFiles);

    res = f_closedir(&dir);

    return CLI_OK;
}

static int mountCmd(int argc, char **argv)
{
    if(argc < 2){
        printf("usage: mount <0|1>\n");
        return CLI_BAD_PARAM;
    }

	mount("0:", argv[1][0] == '1');
    return CLI_OK;
}

static int resetCmd(int argc, char **argv)
{
    sw_reset();
    return CLI_OK;
}

static int flashCmd(int argc, char **argv)
{
    flash_res_t res;

    if(argc < 2){
        printf("usage: flash <select|init|info|erase|read>\n");
        printf("\tinit\n");
        printf("\tinfo\n");
        printf("\terase\n");
        printf("\tformat, applies only to spi flash\n");
        printf("\tread <addr> <len>\n");
        printf("\tselect <nor|spi>\n");
        return CLI_OK_LF;
    }

    if(!strcmp(argv[1], "select")){
        if(!strcmp(argv[2], "nor")){
            selectFlashNOR();
        }else{
           selectFlashSPI();
        }
        return CLI_OK;
    }

    if(!pfls){
        printf("Flash not detected\n");
        return CLI_OK;
    }

    if(!strcmp(argv[1], "init")){
        pfls->init();
        return CLI_OK;
    }

    if(!strcmp(argv[1], "info")){
        printf("Name: %s\n", pfls->name);
        printf("Total size: %u (0x%08X) bytes\n", (unsigned int)pfls->size, (unsigned int)pfls->size);
        printf("Sector size: %u (0x%04X) bytes\n",  (unsigned int)pfls->sectorsize,  (unsigned int)pfls->sectorsize);
        printf("Page size: %u (0x%02X) bytes\n",  (unsigned int)pfls->pagesize,  (unsigned int)pfls->pagesize);

        if(pfls->opt & FLASH_OPT_CFI){
            uint8_t size = flashnor_getCFI((uint8_t*)argv[0]);
            printf("CFI bytes:\n");
            printBuf((const uint8_t*)argv[0], size);
        }
        return CLI_OK;
    }

    if(!strcmp(argv[1], "read")) {
        uint32_t addr;
        int32_t len;
        CLI_Ha2i(argv[2], &addr);
        CLI_Ia2i(argv[3], &len);
        if(addr < pfls->size && len < 512){
            uint8_t *buf = (uint8_t*)malloc(len);
            if(buf){
                pfls->read((uint8_t*)buf, addr, len);
                printBuf((const uint8_t*)buf, len);
                free(buf);
                return CLI_OK_LF;
            }
        }
    }

    if(!strcmp(argv[1], "write")) {
        pfls->write((const uint8_t*)"This is test data", 0 , 18);
        return CLI_OK_LF;
    }

    if(!strcmp(argv[1], "erase")) {
        int32_t sector;

        if(argc == 2){
            res = pfls->erase(FLASH_CHIP_ERASE);
        }else{
            CLI_Ia2i(argv[2], &sector);
            res = pfls->erase(sector);
        }

        if(res != FLASH_OK){
            printf("Error %d\n", res);
        }

        return CLI_OK;
    }

#if FF_USE_MKFS
    if(!strcmp(argv[1], "format")) {
        if((pfls->opt & FLASH_OPT_ITF_MASK) == FLASH_OPT_ITF_SPI){
            printf("Formating spi flash... ");
            res = f_mkfs("0:", NULL, fatfs.win, sizeof(fatfs.win));
            printf("%d\n", res);
            return CLI_OK;
        }
    }
#endif

    return CLI_BAD_PARAM;
}

static int romCmd(int argc, char **argv)
{
    if(argc < 2){
        printf("usage: rom <init|info|erase|program|bank|read>\n");
        printf("\tinfo [file],     prints game header stored on nor flash or from file\n");
        printf("\terase,           Erase NOR flash, same as 'flash erase' command\n");
        printf("\tread <addr> <n>, read n data from rom using cartride bus\n");
        printf("\treadb <addr>,    read byte from rom using cartride bus\n");
        printf("\tprogram <file>,  programs binary file into rom\n");
        return CLI_OK_LF;
    }

    if(!pfls){
        selectFlashNOR();
    }

    if(!strcmp(argv[1], "info")){
        uint8_t rom_header[GAME_HEADER_SIZE];
        if(argc == 2){
            selectFlashNOR();
            rom_read(rom_header, GAME_HEADER_ADDR, sizeof(rom_header));
        }else{
            selectFlashSPI();
            if(!mounted){
                if(mount("0:", 1) != FR_OK){
                    return CLI_ERROR;
                }
            }
            readBlockFromFile(rom_header, sizeof(rom_header) , argv[2], GAME_HEADER_ADDR);
        }
        printGameHeader(rom_header);
        return CLI_OK_LF;
    }

    if(!strcmp(argv[1], "erase")) {
        selectFlashNOR();
        if(pfls->erase(FLASH_CHIP_ERASE) != FLASH_OK){
            printf("Chip erase failed\n");
        }
        return CLI_OK;
    }

    if(!strcmp(argv[1], "program")) {
        uint8_t *block = (uint8_t*)malloc(PROG_BLOCK_SIZE);
        uint32_t addr = 0;
        uint32_t progress = 0;
        uint8_t fail = 0;

        if(!block){
            printf("Fail allocation\n");
            return CLI_OK;
        }

        mem_bus_configure(MEM_BUS_SPI);

        if(!mounted)
            mount("0:", 1);

        int res = readBlockFromFile(block, PROG_BLOCK_SIZE, argv[2], addr);

        if(res <= 0){
            free(block);
            return CLI_BAD_PARAM;
        }

        printGameHeader(block + GAME_HEADER_ADDR);

        selectFlashNOR();

        printf("\nErasing..\n");
        printf("%s\n", (pfls->erase(FLASH_CHIP_ERASE) == FLASH_OK) ? "ok" : "fail");

        while(1){
            if(res > 0){
                mem_bus_configure(MEM_BUS_NOR);

                if(rom_program(block, addr, PROG_BLOCK_SIZE) != FLASH_OK){
                    printf("\nProgram failed at block: %lx\n", addr);
                    break;
                }

                for(int i = 0; i < PROG_BLOCK_SIZE; i++){
                    if(rom_byte_read(addr + i) != block[i]){
                        printf("\nFailed at address: %lx\n", addr + i);
                        fail = 1;
                        break;
                    }
                }

                if(fail){
                    break;
                }

                addr += PROG_BLOCK_SIZE;

                if((progress & 0x1f) == 0){
                    putchar('\n');
                }

                putchar('#');
                progress++;

            }else{
                printf("\nDone\n");
                break;
            }

            mem_bus_configure(MEM_BUS_SPI);
            res = readBlockFromFile(block, PROG_BLOCK_SIZE, argv[2], addr);
        }

        free(block);
        return CLI_OK;
    }

    if(!strcmp(argv[1], "bank")) {
        uint32_t bank;
        CLI_Ia2i(argv[2], (int32_t*)&bank);

        if(bank < 32)
            rom_setBank(bank);

        return CLI_OK;
    }

    if(!strcmp(argv[1], "read")){
        uint32_t addr;
        uint32_t len;

        CLI_Ha2i(argv[2], &addr);
        CLI_Ia2i(argv[3], (int32_t*)&len);

        if(addr < pfls->size && len < 512){
            uint8_t *buf = (uint8_t*)malloc(len);
            if(buf){
                rom_read((uint8_t*)buf, addr, len);
                printBuf((const uint8_t*)buf, len);
                free(buf);
                return CLI_OK_LF;
            }
        }
    }

    if(!strcmp(argv[1], "readb")){
        uint32_t addr;

        CLI_Ha2i(argv[2], &addr);
        if(addr < pfls->size){
            uint8_t data = rom_byte_read(addr);
            printf("%02X\n", data);
            return CLI_OK;
        }
    }

    return CLI_BAD_PARAM;
}

cli_command_t cli_cmds [] = {
    {"help", ((int (*)(int, char**))CLI_Commands)},
    {"reset", resetCmd},
    {"rom", romCmd},
    {"mount", mountCmd},
    {"list", listCmd},
    {"flash", flashCmd}
};

/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
int main(void)
{
    uint8_t inserted;

    board_init();

    CLI_Init("cart >");
    CLI_RegisterCommand(cli_cmds, sizeof(cli_cmds) / sizeof(cli_command_t));
#ifdef MBC1_REVA
    insertDetection_init();
#else
    selectFlashNOR();
    mem_bus_configure(MEM_BUS_SPI);
#endif
    mounted = 0;

    delay_ms(500);  // some delay before checking if is inserted

    inserted = isInserted();

    if(!inserted){
        connectUSB();
    }

	while(1){
        if(inserted){
            if(isInserted()){
                LED1_OFF;
                delay_ms(100);
            }else{
                connectUSB();
                inserted = 0;
            }
        }else{
            if(isInserted()){
                disconnectUSB();
                inserted = 1;
            }else{
                if(usb_isConnected()){
                    if(CLI_ReadLine()){
                        CLI_HandleLine();
                    }
                    LED1_ON;
                    delay_ms(20);
                }
            }
	    }
    }
}