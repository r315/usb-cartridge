#include <string.h>
#include "board.h"
#include "cdc_msc_class.h"
#include "cdc_msc_desc.h"
#include "usbd_int.h"
#include "msc_diskio.h"
#include "syscalls.h"

#define ROM_ADDR_BANK_SELL      0x2000
#define ROM_ADDR_BANK_SELH      0x4000
#define ROM_ADDR_MODE_SEL       0x6000

static otg_core_type otg_core_struct;
static uint8_t serial_buf[USBD_CDC_MSC_OUT_MAXPACKET_SIZE];
static uint32_t serial_queued = 0;
static uint8_t *serial_currentPos;

static void serial_init(void){}

static stdout_ops_t stdout_ops_serial = {
    .init = serial_init,
    .available = serial_available,
    .read = serial_read,
    .write = serial_write
};

void sw_reset(void){
    NVIC_SystemReset();
}

void board_init(void)
{
#ifdef MBC1_REVA
    // HW revA requires A-1 low (Correction made: PF7 -> Q15/A-1)
    // and must be set low as soon as possible
    // other pins are configured when required
    crm_periph_clock_enable(CRM_GPIOF_PERIPH_CLOCK, TRUE);
    LSB_NOR_LOW;                // Set pin A-1 to low
    GPIOF->cfglr = 0x24444444;  // PF7 as output push-pull
#endif
    LED1_INIT;
    SystemInit();
    system_clock_config();
    system_tick_init();
    cycle_count_init();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

    redirect_stdout(&stdout_ops_serial);
}

void mem_bus_configure(uint8_t bus)
{
    switch(bus){
        case MEM_BUS_NONE:
            GPIOA->cfglr = 0x44444444; // D7:0, input
            GPIOB->cfglr = 0x44444444; // A7:0, input
            GPIOB->cfghr = 0x44444444; // A15:8, input
            GPIOC->cfghr = 0x44444444; // WRn, RWEn, RDn, input
        #ifdef MBC1_REVA
            setA_1(0);
            GPIOF->cfghr = 0x24444444; // A-1, output low
        #endif
            break;
        case MEM_BUS_SPI:
            #ifdef AT32_SPI2
            GPIOB->cfghr = (GPIOB->cfghr &0x000FFFFF) | 0x99900000; // MISO, MOSI, SCK
            #else
            GPIOA->cfglr = (GPIOA->cfglr &0x000FFFFF) | 0x99900000; // MISO, MOSI, SCK
            #endif
            GPIOA->cfghr = (GPIOA->cfghr &0xFFFFF0FF) | 0x00000100; // CS
            break;
        case MEM_BUS_NOR:
            GPIOA->cfglr = 0x44444444; // D7:0, input
            GPIOB->cfglr = 0x22222222; // A7:0, output
            GPIOB->cfghr = 0x22222222; // A15:0, output
            GPIOC->cfghr = 0x22244444; // WRn, RWEn, RDn, output
            GPIOF->cfglr = (GPIOF->cfglr &0x0FFFFFFF) | 0x20000000; // A-1, output
            break;
    }
}

/**
  * @brief  this function config gpio.
  * @param  none
  * @retval none
  */
void usb_gpio_config(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);

  // disconnected state
  gpio_bits_write(GPIOA, GPIO_PINS_11, FALSE); // D- Low
  gpio_bits_write(GPIOA, GPIO_PINS_12, FALSE); // D+ Low

  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_11 | GPIO_PINS_12;
  gpio_init(GPIOA, &gpio_init_struct);
}

/**
 * @brief puts usb gpio pins in reset state
 * (input float)
 *
 */
void usb_gpio_deinit(void)
{
    gpio_init_type gpio_init_struct;

    gpio_default_para_init(&gpio_init_struct);

    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_11 | GPIO_PINS_12;
    gpio_init(GPIOA, &gpio_init_struct);
}

/**
 * @brief Configures usb peripheral,
 * usb gpio pins and make usb peripheral
 * ready to be plugged.
 *
 */
void usb_config(void)
{
    #ifdef USB_LOW_POWER_WAKUP
    usb_low_power_wakeup_config();
    #endif

    /* enable otgfs clock */
    crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, TRUE);

    crm_periph_reset(CRM_OTGFS1_PERIPH_RESET, TRUE);

    /* select usb 48m clock source */
    //usb_clock48m_select(USB_CLK_HICK);
    /* Using internal oscillator */
    crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_HICK);

    crm_periph_reset(CRM_OTGFS1_PERIPH_RESET, FALSE);

    /* enable otgfs irq with high priority*/
    nvic_irq_enable(OTGFS1_IRQn, 1, 0);

    usb_gpio_config();

    /* init usb */
    usbd_init(&otg_core_struct,
            USB_FULL_SPEED_CORE_ID,
            USB_ID,
            &cdc_msc_class_handler,
            &cdc_msc_desc_handler
        );
}

/**
 * @brief Issues a usb disconnect to
 * usb peripheral disables interrupt
 * and de initialyses usb gpio pins
 *
 */
void usb_unplug(void)
{
    usb_disconnect(otg_core_struct.usb_reg);
    delay_ms(100);
    nvic_irq_disable(OTGFS1_IRQn);
    crm_periph_reset(CRM_OTGFS1_PERIPH_RESET, TRUE);
    usb_gpio_deinit();

    otg_core_struct.usb_reg = NULL;
}

/**
 * @brief Check if a connection between usb peripheral and
 * usb host is established
 *
 * @return uint8_t
 */
uint8_t usb_isConnected(void)
{
    return otg_core_struct.dev.conn_state == USB_CONN_STATE_CONFIGURED;
}

/**
 * @brief Initialyses spi flash memory
 * and if success initialyses usb peripheral
 *
 */
void connectUSB(void)
{
    if(msc_init(SPI_FLASH_LUN) == RES_OK){
        usb_config();
    }
}

/**
 * @brief Discconnects usb and place all
 * gpio pins connected to nor flash in
 * input float state
 */
void disconnectUSB(void)
{
    usb_unplug();
    mem_bus_configure(MEM_BUS_NONE);
}

/**
 * @brief Reads data from serial in blocking
 * manner
 *
 * @param data
 * @param len
 * @return uint32_t
 */
uint32_t serial_read(uint8_t *data, uint32_t len)
{
    uint32_t count = len;

	while(count){
        while(serial_available() == 0);
        *data++ = *serial_currentPos++;
        count--;
        serial_queued--;
    }

    return len;
}

/**
 * @brief Writes data to serial port.
 * not thread safe, do not call from
 * interrupts
 *
 * @param buf
 * @param len
 * @return uint32_t
 */
uint32_t serial_write(const uint8_t *buf, uint32_t len)
{
    uint32_t time = get_tick();
    do{
        if(usb_vcp_send_data(&otg_core_struct.dev, (uint8_t*)buf, len) == SUCCESS){
            return len;
        }
    }while(get_tick() - time < 1000);

    return 0;
}

/**
 * @brief Checks if data serial data has been received
 *
 * @return uint32_t
 */
uint32_t serial_available(void)
{
    if(!serial_queued){
        serial_queued = usb_vcp_get_rxdata(&otg_core_struct.dev, serial_buf);
        serial_currentPos = serial_buf;
    }

    return serial_queued;
}

/**
 * @brief Initialyses interrupt to
 *        disable all io pins connected to
 *        nor flash memory
 *
 */
void insertDetection_init(void)
{
    crm_periph_clock_enable (CRM_GPIOA_PERIPH_CLOCK, TRUE);
    GPIOA->cfghr = (GPIOA->cfghr & 0x0FFFFFFF) | 0x40000000;
    IOMUX->exintc4_bit.exint15 = 0;    // Assing PA15 to EXINT15
    EXINT->polcfg1 = GPIO_PINS_15;     // Rising edge
    EXINT->inten = GPIO_PINS_15;       // Enable EXINT interrupt request

    nvic_irq_enable(EXINT15_10_IRQn, 0, 0); // Highiest priority
}

/**
 * @brief Checks if card is inserted on device
 *
 * @return uint8_t
 */
uint8_t isInserted(void)
{
#ifndef MBC1_REVA
    return 0;
#else
    return !!(GPIOA->idt & GPIO_PINS_15);
#endif
}

/**
 * @brief Change bank for rom addresses 4000:7FFF
 *        Specification assigns 5 + 2 bits for bank selection
 *        however this hw only supports 4bits
 *
 * @param bank
 */
void rom_setBank(uint8_t bank)
{
    WR_NOR_HIGH;
    RD_NOR_HIGH;
    GPIOA->cfglr = 0x22222222;  // output
    GPIOB->odt = ROM_ADDR_BANK_SELL;
    GPIOA->odt = (GPIOA->odt & 0xFF00) | (bank & 0x0F);
    delay_us(10);
    WR_PAL_LOW;
    delay_us(10);
    WR_PAL_HIGH;
    GPIOA->cfglr = 0x44444444;  // input
}

/**
 * @brief Read a single byte from rom
 *
 * @param address
 * @return uint8_t
 */
uint8_t rom_readbyte(uint32_t address)
{
    uint8_t data;

    WR_NOR_HIGH;
    RD_NOR_HIGH;
    LSB_NOR_LOW;

    // Select bank
    if(address < 0x4000){
        rom_setBank(0);
    }else{
        // Accessing rom bank, use A[17:14] bits as bank select
        rom_setBank(address >> 14);
        // The 16kB of rom bank are accessed with A14 high and A[13:0] bits
        address = (0x4000 | address) & 0x7FFF;
    }

    GPIOB->odt = address;
    delay_us(1);
    RD_NOR_LOW;
    delay_us(1);
    data = (uint8_t)GPIOA->idt;
    RD_NOR_HIGH;
    delay_us(1);

    return data;
}

/**
 * @brief Reads data from rom.
 *
 * @param data
 * @param address
 * @param len
 *
 * @return flash_res_t
 */
flash_res_t rom_read(uint8_t *data, uint32_t address, uint32_t len)
{
    uint32_t romaddr;

    RD_NOR_HIGH;
    WR_NOR_HIGH;
    LSB_NOR_LOW;

    while(len--){
        if(address < 0x4000){
            rom_setBank(0);
            romaddr = address;
        }else{
            rom_setBank(address >> 14);
            romaddr = (0x4000 | address) & 0x7FFF;
        }

        GPIOB->odt = romaddr;
        delay_us(1);
        RD_NOR_LOW;
        delay_us(1);
        *data++ = (uint8_t)GPIOA->idt;
        RD_NOR_HIGH;
        delay_us(1);

        address++;
    }

    return FLASH_OK;
}

/**
 * @brief Programs a block of data int to a rom address
 *
 * @param data
 * @param addr
 * @param len
 * @return flash_res_t
 */
flash_res_t rom_program(const uint8_t *data, uint32_t addr, uint32_t len)
{
    flash_res_t res = FLASH_OK;
    uint32_t romaddr;
    WR_NOR_HIGH;
    RD_NOR_HIGH;
    LSB_NOR_LOW;

    while(len--){
        if(addr < 0x4000){
            rom_setBank(0);
            romaddr = addr;
        }else{
            rom_setBank(addr >> 14);
            romaddr = (0x4000 | addr) & 0x7FFF;
        }

        // Program command sequence requires A-1
        norflash_writebyte(0xAAA, 0xAA);
        norflash_writebyte(0x555, 0x55);
        norflash_writebyte(0xAAA, 0xA0);

        GPIOA->cfglr = 0x22222222;  // output
        GPIOB->odt = romaddr;       // set rom address
        LSB_NOR_LOW;                // Ignore address line A-1, only even addresses are writen
        GPIOA->odt = (GPIOA->odt & 0xFF00) | *data;
        delay_us(10);
        WR_NOR_LOW;
        delay_us(10);
        WR_NOR_HIGH;
        delay_us(10);

        if(norflash_polling(*data) != FLASH_OK){
            res = FLASH_ERROR;
            break;
        }

        addr++;
        data++;
    }

    GPIOA->cfglr = 0x44444444;  // input

    return res;
}

/**
  * @brief  Interrupt handler fo otgfs
  * @param  none
  * @retval none
  */
void OTGFS1_IRQHandler(void)
{
  usbd_irq_handler(&otg_core_struct);
}

/**
 * @brief Interrupt handler that
 * disables all pins
 *
 */
void EXINT15_10_IRQHandler(void)
{
    if(EXINT->intsts & GPIO_PINS_15){
        mem_bus_configure(MEM_BUS_NONE);
        EXINT->intsts = GPIO_PINS_15;
    }
}