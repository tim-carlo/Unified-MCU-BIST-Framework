MSP_GNU_ROOT := /opt/msp430-gcc
PREFIX := $(MSP_GNU_ROOT)/bin/msp430-elf-
SUPPORT_FILE_DIRECTORY := $(MSP_GNU_ROOT)/include

# Docker integration
# Docker is needed since the MSP430 toolchain is not available on MacOS with Apple Silicon.
DOCKER_IMAGE := msp430-buildenv
DOCKER_CMD := docker run --rm --platform linux/amd64 -v "$(CURDIR)/../..:/project" -w /project/targets/msp430 $(DOCKER_IMAGE)

ifndef MSP_GNU_ROOT
define GNU_ERROR_BODY


endef
$(error $(GNU_ERROR_BODY))
endif

# Project structure directories
PROJECT_ROOT := ../..
CORE_DIR := $(PROJECT_ROOT)/core
HAL_DIR := $(PROJECT_ROOT)/hal/msp430
COMMON_UTILS_DIR := $(PROJECT_ROOT)/common/utils
COMMON_INCLUDE_DIR := $(PROJECT_ROOT)/common/include

# Output and build directories
OUTPUT_DIR := build
BUILD_SUBDIRS := core hal common_utils

# Device configuration
DEVICE := MSP430FR5994

# Source files
CSRC_FILES := \
    $(wildcard $(CORE_DIR)/*.c) \
    $(wildcard $(HAL_DIR)/*.c) \
    $(wildcard $(COMMON_UTILS_DIR)/*.c)

ASRC_FILES := \
    $(wildcard $(CORE_DIR)/*.S) \
    $(wildcard $(HAL_DIR)/*.S) \
    $(wildcard $(COMMON_UTILS_DIR)/*.S)


# Object files with directory structure
OBJ_FILES := \
    $(patsubst $(PROJECT_ROOT)/%.c,$(OUTPUT_DIR)/%.o,$(CSRC_FILES)) \
    $(patsubst $(PROJECT_ROOT)/%.S,$(OUTPUT_DIR)/%.o,$(ASRC_FILES))

# Dependency files
DEP_FILES := $(OBJ_FILES:.o=.d)

# Include paths
INC_DIRS := \
    $(CORE_DIR) \
    $(HAL_DIR) \
    $(COMMON_INCLUDE_DIR) \
    $(SUPPORT_FILE_DIRECTORY)

INCLUDES := $(foreach dir,$(INC_DIRS),-I$(dir))

# Compiler flags
CFLAGS += -Os -g3 -flto -Wall -msmall -mmcu=$(DEVICE) $(INCLUDES)
CFLAGS += -MMD -MP  # Generate dependencies

# Linker flags
LDFLAGS += $(CFLAGS)
LDFLAGS += -L $(SUPPORT_FILE_DIRECTORY)
LDFLAGS += -Wl,-Map,$(OUTPUT_DIR)/build.map,--gc-sections
LDFLAGS += -Wl,--defsym=__stack_size=0x400

.PHONY: all docker-build clean flash debug

all: | $(OUTPUT_DIR) $(OUTPUT_DIR)/build.hex $(OUTPUT_DIR)/build.elf

# Docker build command
docker-build:
	@echo "Building in Docker container..."
	$(DOCKER_CMD) make

# Create output directories
$(OUTPUT_DIR):
	mkdir -p $(OUTPUT_DIR)
	@for dir in $(BUILD_SUBDIRS); do \
		mkdir -p $(OUTPUT_DIR)/$$dir; \
	done

# Compile C source to .o with dependency generation
$(OUTPUT_DIR)/%.o: $(PROJECT_ROOT)/%.c
	@mkdir -p $(@D)
	$(PREFIX)gcc $(CFLAGS) -c $< -o $@
	@echo "CC $<"

# Compile assembly source to .o
$(OUTPUT_DIR)/%.o: $(PROJECT_ROOT)/%.S
	@mkdir -p $(@D)
	$(PREFIX)gcc $(CFLAGS) -c $< -o $@
	@echo "AS $<"

# Link
$(OUTPUT_DIR)/build.elf: $(OBJ_FILES)
	@echo "Linking with objects:"
	@echo $(OBJ_FILES)
	$(PREFIX)gcc $(LDFLAGS) $^ -o $@
	@$(PREFIX)size $@
	@echo "Generated $@"

# Convert ELF to HEX
$(OUTPUT_DIR)/build.hex: $(OUTPUT_DIR)/build.elf
	@$(PREFIX)objcopy -O ihex $< $@
	@echo "Created HEX: $@"

# Flash using script
flash: $(OUTPUT_DIR)/build.hex
	@echo "Flashing..."
	$(PROJECT_ROOT)/tools/flash_launchpad.sh $(abspath $<)

# Clean build artifacts
clean:
	rm -rf $(OUTPUT_DIR)

# Include generated dependencies
-include $(DEP_FILES)