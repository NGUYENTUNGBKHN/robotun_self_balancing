TARGET_NAME=stm32_lib
BUILD_DIR=_build
OBJ_DIR=$(BUILD_DIR)/obj
LIB_OUTPUT_PATH = $(BUILD_DIR)/$(TARGET_NAME).a

DEVICE_DIR:=./Device
CORE_DIR:=./Core
DRIVER_DIR:=./Driver/hal

# TOOLCHAIN
CAR=arm-none-eabi-ar
CC=arm-none-eabi-gcc

# CFLAGS
CPU=-mcpu=cortex-m3 -mthumb -mabi=aapcs
OPT=-O0 -g
CFLAG+= $(CPU) $(OPT) -DSTM32F103xB

SRC_EXCLUDE=\
	$(wildcard $(DRIVER_DIR)/Src/*template*.c)

SRC_INCLUDE+=$(wildcard $(DEVICE_DIR)/Src/*.c)
SRC_INCLUDE+=$(wildcard $(DRIVER_DIR)/Src/*.c)
SRC_INCLUDE+=$(wildcard $(DRIVER_DIR)/Src/Legacy/*.c)

SRC_FILES:=$(filter-out $(SRC_EXCLUDE), $(SRC_INCLUDE))

INC_FOLDERS=\
	$(DEVICE_DIR)/Inc\
	$(DRIVER_DIR)/Inc\
	$(CORE_DIR)/Include\
	$(DRIVER_DIR)

OBJ_FILES:=$(foreach obj_file, $(SRC_FILES), $(OBJ_DIR)/$(obj_file).o)

INC+=$(foreach inc, $(INC_FOLDERS), -I$(inc))

.PHONY: all clean

all: $(LIB_OUTPUT_PATH)

$(OBJ_DIR)/%.c.o: %.c
	@echo "Compiling $(notdir $<)"
	@mkdir -p $(shell dirname $@)
	@$(CC) $(CFLAG) $(INC) -c $< -o $@

$(LIB_OUTPUT_PATH): $(OBJ_FILES)
	@echo "Archiving $(notdir $@)"
	@mkdir -p $(shell dirname $@)
	@$(CAR) -r $@ $(OBJ_FILES)

clean:
	rm -rf $(BUILD_DIR)
