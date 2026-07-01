# GD32 build rules

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-


dirs-$(CONFIG_MACH_GD32F30X) += src/gd32 src/generic lib/gd32f30x
dirs-$(CONFIG_MACH_GD32E23X) += src/gd32 src/generic lib/gd32e23x

MCU := $(shell echo $(CONFIG_MCU))
MCU_UPPER := $(shell echo $(CONFIG_MCU) | tr a-z A-Z | tr X x)

CFLAGS-$(CONFIG_MACH_GD32F30X) += -DUSE_STDPERIPH_DRIVER
CFLAGS-$(CONFIG_MACH_GD32F303XB) += -DGD32F30X_HD
CFLAGS-$(CONFIG_MACH_GD32F303XC) += -DGD32F30X_HD
CFLAGS-$(CONFIG_MACH_GD32F303XE) += -DGD32F30X_HD
CFLAGS-$(CONFIG_MACH_GD32F303XG) += -DGD32F30X_XD
CFLAGS-$(CONFIG_MACH_GD32F303XI) += -DGD32F30X_XD
CFLAGS-$(CONFIG_MACH_GD32F303XK) += -DGD32F30X_XD
CFLAGS-$(CONFIG_MACH_GD32F305XX) += -DGD32F30X_CL
CFLAGS-$(CONFIG_MACH_GD32F307XX) += -DGD32F30X_CL
CFLAGS-$(CONFIG_MACH_GD32E23X) += -DGD32E230

CFLAGS-$(CONFIG_MACH_GD32E23X) += -mcpu=cortex-m23 -Ilib/gd32e23x
CFLAGS-$(CONFIG_MACH_GD32F30X) += -mcpu=cortex-m4 -Ilib/gd32f30x
CFLAGS += $(CFLAGS-y) -D$(MCU_UPPER) -mthumb -Ilib/cmsis-core

# Creality-specific on-board loader header (for the factory MCU update path)
ifeq ($(CONFIG_BOARD_INFO_CONFIGURE),y)
ifeq ($(CONFIG_MAIN_MCU_BOARD),y)
board_type := mcu
else ifeq ($(CONFIG_NOZZLE_MCU_BOARD),y)
board_type := noz
else ifeq ($(CONFIG_BED_MCU_BOARD),y)
board_type := bed
endif
mcu_menu := $(patsubst "%",%,$(CONFIG_MCU_MENU))
mcu_type := $(patsubst "%",%,$(CONFIG_MCU_TYPE))
board_hw_version := $(board_type)$(CONFIG_MCU_BOARD_ID)_$(CONFIG_MCU_BOARD_HW_VER)_$(mcu_menu)$(mcu_type)
board_fw_version := $(board_type)$(CONFIG_MCU_BOARD_ID)_$(CONFIG_MCU_BOARD_FW_VER)_$(CONFIG_MCU_BOARD_FW_RESERVED)
CFLAGS += -DBOARD_FW_VERSION=\"$(board_fw_version)\"
endif

CFLAGS_klipper.elf += --specs=nano.specs --specs=nosys.specs
CFLAGS_klipper.elf += -T $(OUT)src/generic/armcm_link.ld
$(OUT)klipper.elf: $(OUT)src/generic/armcm_link.ld

# Add source files
src-y += generic/armcm_boot.c generic/armcm_irq.c
src-y += generic/armcm_reset.c generic/crc16_ccitt.c

src-$(CONFIG_MACH_GD32F30X) += gd32/gd32f30x.c gd32/gd32f30x_gpio.c
src-$(CONFIG_MACH_GD32F30X) += generic/armcm_timer.c

src-$(CONFIG_MACH_GD32E23X) += gd32/gd32e23x.c gd32/gd32e23x_gpio.c
src-$(CONFIG_MACH_GD32E23X) += gd32/gd32e23x_timer.c generic/timer_irq.c

src-$(CONFIG_HAVE_GPIO_ADC) += gd32/adc.c
src-$(CONFIG_HAVE_GPIO_I2C) += gd32/i2c.c
src-$(CONFIG_HAVE_GPIO_SPI) += gd32/spi.c
src-$(CONFIG_USBSERIAL) += gd32/usbserial.c
src-$(CONFIG_USBSERIAL) += generic/usb_cdc.c
src-$(CONFIG_SERIAL) += gd32/serial.c generic/serial_irq.c
src-$(CONFIG_HAVE_GPIO_HARD_PWM) += $(pwm-src-y)
src-$(CONFIG_BOARD_INFO_CONFIGURE) += gd32/board_info.c

# Binary output file rules
target-y += $(OUT)klipper.bin

$(OUT)klipper.bin: $(OUT)klipper.elf
	@echo "  Creating bin file $@"
	$(Q)$(OBJCOPY) -O binary $< $@
ifeq ($(CONFIG_BOARD_INFO_CONFIGURE),y)
	$(Q)$(PYTHON) ./scripts/gd32_board_crc16.py $@
endif

ifeq ($(CONFIG_BOARD_INFO_CONFIGURE),y)
# Emit a copy named like the Creality factory image so it can be flashed
# through the on-board MCU update path.
target-y += $(OUT)$(board_hw_version)-$(board_fw_version).bin
$(OUT)$(board_hw_version)-$(board_fw_version).bin: $(OUT)klipper.bin
	@echo "  Naming firmware $@"
	$(Q)cp $< $@
endif

# Flash rules
flash: $(OUT)klipper.bin
	@echo "  Flashing $< to $(FLASH_DEVICE)"
	$(Q)$(PYTHON) ./scripts/flash_usb.py -t $(CONFIG_MCU) -d "$(FLASH_DEVICE)" $(if $(NOSUDO),--no-sudo) $(OUT)klipper.bin
