######################################
# target
######################################
TARGET =usb-cartridge

#######################################
# Features
#######################################

FEATURE_ENABLE = \
_ENABLE_CLI=1 \
_ENABLE_FF=1 \

#######################################
# paths
#######################################
BUILD_DIR 		:=build
PROJECT_PATH 	=.$(CWD)

APP_PATH           =app
DRIVERS_PER_PATH   =libraries/drivers
DRIVERS_CMSIS_PATH =libraries/cmsis/cm4
MIDDLEWARES_PATH   =middlewares
BSP                =bsp

BUILD_PATH =$(PROJECT_PATH)/$(BUILD_DIR)
#######################################
# Includes
#######################################
C_INCLUDES = \
$(APP_PATH)/inc \
$(BSP) \
$(DRIVERS_PER_PATH)/inc \
$(DRIVERS_CMSIS_PATH)/device_support \
$(DRIVERS_CMSIS_PATH)/core_support \
$(MIDDLEWARES_PATH)/usb_drivers/inc \
$(MIDDLEWARES_PATH)/usbd_class/composite_cdc_msc \
$(MIDDLEWARES_PATH)/3rd_party/fatfs/source \
$(MIDDLEWARES_PATH)/3rd_party/cli_simple \
$(BSP) \
#$(MIDDLEWARES_PATH)/usbd_class/msc \

AS_INCLUDES =\

######################################
# Sources
######################################
CSRCS = \
$(DRIVERS_CMSIS_PATH)/device_support/startup/system_at32f415.c\
$(DRIVERS_CMSIS_PATH)/device_support/startup/startup_at32f415cbt7.c\
$(BSP)/gb_mbc1_board.c \
$(BSP)/at32f415_clock.c \
$(DRIVERS_PER_PATH)/src/at32f415_usb.c \
$(DRIVERS_PER_PATH)/src/at32f415_crm.c \
$(DRIVERS_PER_PATH)/src/at32f415_gpio.c \
$(DRIVERS_PER_PATH)/src/at32f415_dma.c \
$(DRIVERS_PER_PATH)/src/at32f415_misc.c \
$(DRIVERS_PER_PATH)/src/at32f415_spi.c \
$(MIDDLEWARES_PATH)/usb_drivers/src/usb_core.c \
$(MIDDLEWARES_PATH)/usb_drivers/src/usbd_core.c \
$(MIDDLEWARES_PATH)/usb_drivers/src/usbd_sdr.c \
$(MIDDLEWARES_PATH)/usb_drivers/src/usbd_int.c \
$(MIDDLEWARES_PATH)/usbd_class/composite_cdc_msc/cdc_msc_desc.c \
$(MIDDLEWARES_PATH)/usbd_class/composite_cdc_msc/cdc_msc_class.c \
$(MIDDLEWARES_PATH)/usbd_class/composite_cdc_msc/cdc_msc_bot_scsi.c \
$(MIDDLEWARES_PATH)/3rd_party/fatfs/source/ff.c \
$(MIDDLEWARES_PATH)/3rd_party/fatfs/source/ffunicode.c \
$(MIDDLEWARES_PATH)/3rd_party/cli_simple/cli_simple.c \
$(MIDDLEWARES_PATH)/3rd_party/cli_simple/syscalls.c \
$(APP_PATH)/src/main.c \
$(APP_PATH)/src/diskio.c \
$(APP_PATH)/src/at32_spiflash.c \
$(APP_PATH)/src/at32_norflash.c \
$(APP_PATH)/src/flashspi.c \
$(APP_PATH)/src/flashnor.c \
$(APP_PATH)/src/flashspi_gd25lq16.c \
$(APP_PATH)/src/flashspi_winbond.c \
$(APP_PATH)/src/flashspi_macronix.c \
$(APP_PATH)/src/flash_en29lv320.c \

CPPSRCS = \

ASRCS = \

LDSCRIPT =$(DRIVERS_CMSIS_PATH)/device_support/startup/linker/AT32F415xB_FLASH.ld
#######################################
# Misc
#######################################

OCD_CONFIG =$(PROJECT_PATH)/at32f415.cfg

C_DEFS =\
$(DEVICE) \
BOARD_415DK \
USE_STDPERIPH_DRIVER \
AT32_SPI2 \
MBC1_REVA \

C_DEFS +=$(FEATURE_ENABLE)

#######################################
# CFLAGS
#######################################

DEVICE =AT32F415CBT7
CPU =-mcpu=cortex-m4 -mthumb

# fpu
#at43f415 does not have FPU
#FPU =-mfloat-abi=hard -mfpu=fpv4-sp-d16
#FPU =-mfloat-abi=soft

# float-abi
FLOAT_ABI =#-u_printf_float

# mcu
MCU = $(CPU) $(FPU) $(FLOAT_ABI)

ifdef RELEASE
OPT   =-O2
else
OPT   =-O0 -g -gdwarf-2 -Wall #-Wextra
endif

ifndef V
VERBOSE =@
else
VERBOSE =
endif

ASFLAGS  =$(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS   =$(MCU) $(OPT) $(addprefix -D, $(C_DEFS)) $(addprefix -I, $(C_INCLUDES)) -std=gnu11 -fdata-sections -ffunction-sections #-fstack-usage
CPPFLAGS =$(CPU) $(OPT) $(addprefix -D, $(C_DEFS)) $(addprefix -I, $(C_INCLUDES)) -fdata-sections -ffunction-sections -fno-unwind-tables -fno-exceptions -fno-rtti
LDFLAGS  =$(MCU) $(SPECS) -Wl,-Map,$(BUILD_PATH)/$(TARGET).map,--cref,--gc-sections

# Generate dependency information
#CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -Wa,-a,-ad,-alms=$(BUILD_PATH)/$(notdir $(<:.c=.lst))

#######################################
# Libraries
#######################################

LIBDIR =
ifdef SEMIHOSTING
SPECS =--specs=rdimon.specs
LDLIBS =-nostartfiles -lc -lrdimon
else
SPECS=--specs=nosys.specs --specs=nano.specs
#-nostdlib -lstd++ -lnosys -lm
LIBS =-lstdc++
endif

#######################################
# Objects
#######################################

OBJECTS =  $(addprefix $(BUILD_PATH)/, $(CSRCS:.c=.o))
OBJECTS += $(addprefix $(BUILD_PATH)/, $(CPPSRCS:.c=.o))
OBJECTS += $(addprefix $(BUILD_PATH)/, $(ASRCS:.s=.o))

#######################################
# Tool binaries
#######################################
PREFIX = arm-none-eabi-
CC  =$(PREFIX)gcc
CPP =$(PREFIX)g++
AS  =$(PREFIX)gcc -x assembler-with-cpp
LD  =$(PREFIX)gcc
AR  =$(PREFIX)ar
SZ  =$(PREFIX)size
OBJCOPY =$(PREFIX)objcopy
OBJDUMP =$(PREFIX)objdump
HEX = $(OBJCOPY) -O ihex
BIN = $(OBJCOPY) -O binary -S

ifeq ($(shell uname -s), Linux)
PRG_DEP =$(BUILD_PATH)/$(TARGET).elf
PRG_CFG =$(OCD_CONFIG)
PRG_CMD =openocd -f $(PRG_CFG) -c "program $(PRG_DEP) verify reset exit"
#JLK ="/opt/SEGGER/JLink/JLinkExe"
#PRG_DEP =bin $(PRG_CFG)
#PRG_CFG =$(BUILD_DIR)/$(TARGET).jlink
#PRG_CMD =$(VERBOSE)$(JLK) -device $(DEVICE) -if SWD -speed auto -CommanderScript $(PRG_CFG)
else
JLK :=JLink
PRG_DEP =bin $(PRG_CFG)
PRG_CFG =$(BUILD_DIR)/$(TARGET).jlink
PRG_CMD =$(VERBOSE)$(JLK) -device $(DEVICE) -if SWD -speed auto -CommanderScript $(PRG_CFG)
#-device CORTEX-M4
endif

#######################################
# Rules
#######################################
default:
	@$(MAKE) $(TARGET)

$(TARGET): $(BUILD_PATH)/$(TARGET).elf
	@echo "--- Size ---"
	$(VERBOSE)$(SZ) -A -x $<
	$(VERBOSE)$(SZ) -B $<


bin: $(BUILD_PATH)/$(TARGET).bin

program: default $(PRG_DEP)
	$(PRG_CMD)

erase:
	$(ERASE_CMD)

$(BUILD_PATH)/$(TARGET).jlink: $(BUILD_DIR)/$(TARGET).bin
	@echo "Creating Jlink configuration file"
	@echo "loadfile $< 0x08000000" > $@
	@echo "r" >> $@
	@echo "q" >> $@

test:
	@$(foreach obj, $(C_DEFS), echo $(obj);)

#######################################
# build rules
#######################################
$(BUILD_PATH)/%.d: %.c
	$(VERBOSE)$(CC) $(CFLAGS) -MF"$@" -MG -MM -MP -MT"$@" -MT"$(<:.c=.o)" "$<"

$(BUILD_PATH)/%.o: %.c
	@echo "[CC]  $<"
	$(VERBOSE)$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_PATH)/%.obj: %.cpp
	@echo "[CP]  $<"
	$(VERBOSE)$(CPP) -c $(CPPFLAGS)  $< -o $@

$(BUILD_PATH)/%.o: %.s
	@echo "[AS]  $<"
	$(VERBOSE)$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_PATH)/$(TARGET).elf: $(BUILD_PATH) $(OBJECTS)
	@echo "[LD]  $@"
	$(VERBOSE)$(LD) $(LIBDIR) -T$(LDSCRIPT) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $@

$(BUILD_PATH)/%.hex: $(BUILD_PATH)/%.elf
	$(VERBOSE)$(HEX) $< $@

$(BUILD_PATH)/%.bin: $(BUILD_PATH)/%.elf
	$(VERBOSE)$(BIN) $< $@

$(BUILD_PATH):
	@$(foreach obj, $(OBJECTS), mkdir -p $(dir $(obj));)

#######################################
# clean up
#######################################
clean:
	$(VERBOSE)-rm -fR $(BUILD_PATH)

#######################################
# dependencies
#######################################
#-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
ifeq ($(GCC_COLORS), )
export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'
#unexport GCC_COLORS
endif


# *** EOF ***
