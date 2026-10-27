TARGET_NAME=tx_m3
BUILD_DIR=_build
OBJ_DIR=$(BUILD_DIR)/obj
LIB_OUTPUT_PATH = $(BUILD_DIR)/$(TARGET_NAME).a
COMMON=./common/
CORE=./ports/cortex_m3/gnu/

# TOOLCHAIN
CAR=arm-none-eabi-ar
CC=arm-none-eabi-gcc

# CFLAGS
CPU=-mcpu=cortex-m3 -mthumb -mabi=aapcs
OPT=-O0 -g
CFLAG+= $(CPU) $(OPT) -DSTM32F103xB

SRC_EXCLUDE=\

SRC_INCLUDE+=$(wildcard $(COMMON)/src/*.c)
SRC_INCLUDE+=$(wildcard $(CORE)/src/*.c)
SRC_INCLUDE+=$(wildcard $(CORE)/src/*.S)

SRC_FILES:=$(filter-out $(SRC_EXCLUDE), $(SRC_INCLUDE))

INC_FOLDERS=\
	$(COMMON)/inc\
	$(CORE)/inc\

OBJ_FILES:=$(foreach obj_file, $(SRC_FILES), $(OBJ_DIR)/$(obj_file).o)

INC+=$(foreach inc, $(INC_FOLDERS), -I$(inc))

.PHONY: all clean

all: $(LIB_OUTPUT_PATH)

$(OBJ_DIR)/%.c.o: %.c
	@echo "Compiling $(notdir $<)"
	@mkdir -p $(shell dirname $@)
	@$(CC) $(CFLAG) $(INC) -c $< -o $@

$(OBJ_DIR)/%.S.o: %.S
	@echo "Compiling $(notdir $<)"
	@mkdir -p $(shell dirname $@)
	@$(CC) $(CFLAG) $(INC) -c $< -o $@

$(LIB_OUTPUT_PATH): $(OBJ_FILES)
	@echo "Archiving $(notdir $@)"
	@mkdir -p $(shell dirname $@)
	@$(CAR) -r $@ $(OBJ_FILES)

clean:
	rm -rf $(BUILD_DIR)
