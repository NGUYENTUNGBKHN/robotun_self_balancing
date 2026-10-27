TARGET_NAME:=boot
LINKER_NAME:=boot
BUILD_DIR:=_build
ROOT_PROJ:=./

BOOT_DIR:=$(ROOT_PROJ)/boot_process
COMMON_DIR:=$(ROOT_PROJ)/../Common
SDK_DIR:=$(ROOT_PROJ)/../SDK

# Toolchain
CC:=arm-none-eabi-gcc
OBJCOPY:=arm-none-eabi-objcopy
SIZE:=arm-none-eabi-size

# Flags
CPU=-mcpu=cortex-m3 -mthumb -mabi=aapcs
OPT=-O2 -g3
CCFLAGS+=$(CPU) $(OPT)
CCFLAGS+=-ffunction-sections -fdata-sections -fno-strict-aliasing
CCFLAGS+=-fno-builtin -fshort-enums
CCFLAGS+=-Wall -Wno-unused-variable
CCFLAGS+=-DSTM32F103xB

ASMFLAGS+=$(CPU) $(OPT)

LDFLAGS+=$(CPU) $(OPT)
LDFLAGS+=-Wl,--gc-sections
LDFLAGS+=--specs=nano.specs
LDFLAGS+=-T$(ROOT_PROJ)/$(LINKER_NAME).ld
LDFLAGS+=-Wl,-Map=$(BUILD_DIR)/$(TARGET_NAME).map

SRC_EXCLUDE=\

SRC_INCLUDE+=$(wildcard *.c)
SRC_INCLUDE+=$(wildcard $(BOOT_DIR)/*.c)
SRC_INCLUDE+=$(wildcard $(BOOT_DIR)/operation/*.c)
SRC_INCLUDE+=$(wildcard *.s)
SRC_INCLUDE+=$(wildcard *.S)

SRC_FILES:=$(filter-out $(SRC_EXCLUDE), $(SRC_INCLUDE))

INC_FOLDERS=\
	$(ROOT_PROJ)\
	$(COMMON_DIR)\
	$(SDK_DIR)/Core/Include\
	$(SDK_DIR)/Device/Inc\
	$(SDK_DIR)/Driver/hal\
	$(SDK_DIR)/Driver/hal/Inc\
	$(BOOT_DIR)\
	$(BOOT_DIR)/operation

LIBS+=$(SDK_DIR)/_build/stm32_lib.a

INC+=$(foreach inc, $(INC_FOLDERS), -I$(inc))

OBJ_FILES:=$(foreach file, $(SRC_FILES), $(BUILD_DIR)/obj/$(file).o)

.PHONY: all clean flash

all: $(BUILD_DIR)/$(TARGET_NAME).elf $(BUILD_DIR)/$(TARGET_NAME).bin $(BUILD_DIR)/$(TARGET_NAME).hex
	@$(SIZE) $(BUILD_DIR)/$(TARGET_NAME).elf

$(BUILD_DIR)/obj/%.c.o: %.c
	@echo "Compiling $(notdir $<)"
	@mkdir -p $(shell dirname $@)
	@$(CC) $(CCFLAGS) $(INC) -c $< -o $@

$(BUILD_DIR)/obj/%.S.o: %.S
	@echo "Assembling $(notdir $<)"
	@mkdir -p $(shell dirname $@)
	@$(CC) $(ASMFLAGS) $(INC) -c $< -o $@

$(BUILD_DIR)/obj/%.s.o: %.s
	@echo "Assembling $(notdir $<)"
	@mkdir -p $(shell dirname $@)
	@$(CC) $(ASMFLAGS) $(INC) -c $< -o $@

$(BUILD_DIR)/$(TARGET_NAME).elf: $(OBJ_FILES) $(LIBS)
	@echo "Linking $@"
	@mkdir -p $(BUILD_DIR)
	@$(CC) $(LDFLAGS) $(OBJ_FILES) $(LIBS) -o $@

$(BUILD_DIR)/$(TARGET_NAME).bin: $(BUILD_DIR)/$(TARGET_NAME).elf
	@$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/$(TARGET_NAME).hex: $(BUILD_DIR)/$(TARGET_NAME).elf
	@$(OBJCOPY) -O ihex $< $@

PROBE_PORT?=JLINK

flash: all
	STM32_Programmer_CLI.exe -c port=$(PROBE_PORT) -w $(BUILD_DIR)/$(TARGET_NAME).bin 0x08000000 -v -rst

clean:
	rm -rf $(BUILD_DIR)
