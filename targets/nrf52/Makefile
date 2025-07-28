CC       := arm-none-eabi-gcc
OBJCOPY  := arm-none-eabi-objcopy
SIZE     := arm-none-eabi-size
MKDIR    := mkdir -p

NRF_DEV_NUM     := 52840
LINKER_SCRIPT   := nrf52840_xxaa.ld
OUTPUT_DIR      := build/nrf52
ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)


SRC_DIRS := \
    $(ROOT)/core \
    $(ROOT)/common \
    $(ROOT)/hal/nrf52 \
    $(ROOT)/targets/nrf52

SRCS := $(foreach dir,$(SRC_DIRS),$(shell find $(dir) \( -name '*.c' -o -name '*.S' \)))

OBJS := $(patsubst $(ROOT)/%,$(OUTPUT_DIR)/%,$(SRCS:.c=.o))
OBJS := $(OBJS:.S=.o)

OBJS := $(patsubst $(ROOT)/%,$(OUTPUT_DIR)/%,$(SRCS:.c=.o))
OBJS := $(OBJS:.S=.o)

INC_DIRS := \
    $(ROOT) \
    Include \
    CMSIS_5/CMSIS/Core/Include \
    $(ROOT)/common/include \
    $(ROOT)/hal/nrf52

INCLUDES := $(addprefix -I,$(INC_DIRS))


CFLAGS := $(INCLUDES) -O3 -g3
CFLAGS += -DNRF${NRF_DEV_NUM}_XXAA -DARM_MATH_CM4 -DFLOAT_ABI_HARD
CFLAGS += -Wall -fno-builtin -mthumb -mcpu=cortex-m4
CFLAGS += -mabi=aapcs -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -fsingle-precision-constant -ffunction-sections -fdata-sections
CFLAGS += -Wa,-adhlns=$@.lst

ASMFLAGS := $(filter-out -std=gnu99,$(CFLAGS))


LDFLAGS += $(OPT)
LDFLAGS += -T$(LINKER_SCRIPT)
LDFLAGS += -Wl,--gc-sections,-Map=${OUTPUT_DIR}/build.map
LDFLAGS += --specs=nosys.specs --specs=nano.specs  # Beide specs hinzufügen
LDFLAGS += -Wl,--entry=Reset_Handler  # Expliziter Entry Point
LDFLAGS += -Wl,--no-warn-rwx-segments  # Unterdrückt Warnungen
LDFLAGS += -mthumb -mabi=aapcs -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

# Minimal required libraries
LIBRARIES := -lc_nano -lm -lnosys

# Debug: Show found source files
$(info === Found C source files ===)
$(foreach file,$(CSRC_FILES),$(info - $(file)))
$(info =============================)

.PHONY: all clean flash

all: $(OUTPUT_DIR)/build.hex

clean:
	rm -rf $(OUTPUT_DIR)

flash: all
	nrfjprog -f NRF52 --program $(OUTPUT_DIR)/build.hex --chiperase --verify
	nrfjprog -f NRF52 --pinresetenable  
	nrfjprog -f NRF52 --reset           


$(OUTPUT_DIR)/%.o: $(ROOT)/%.c | $(OUTPUT_DIR)
	@echo "CC $<"
	@$(MKDIR) $(@D)
	@$(CC) $(CFLAGS) -c $< -o $@

$(OUTPUT_DIR)/%.o: $(ROOT)/%.S | $(OUTPUT_DIR)
	@echo "AS $<"
	@$(MKDIR) $(@D)
	@$(CC) $(ASMFLAGS) -c $< -o $@

$(OUTPUT_DIR):
	@$(MKDIR) $@

$(OUTPUT_DIR)/build.elf: $(OBJS) | $(OUTPUT_DIR)
	@echo "Linking $@"
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBRARIES) -o $@
	@$(SIZE) $@

$(OUTPUT_DIR)/build.hex: $(OUTPUT_DIR)/build.elf
	@echo "Generating HEX"
	@$(OBJCOPY) -O ihex $< $@