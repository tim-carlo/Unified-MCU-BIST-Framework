# Toolchain definitions
CC := arm-none-eabi-gcc
CXX := arm-none-eabi-c++
OBJCOPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size

# Project configuration
NRF_DEV_NUM := 52840
LINKER_SCRIPT := nrf52840_xxaa.ld
OUTPUT_DIR := build

# Source files
SRCS = $(filter-out gcc_startup_nrf52840.S, $(wildcard *.c) $(wildcard *.S)) $(wildcard routines/*.c)
OBJS := $(patsubst %.c,$(OUTPUT_DIR)/%.o,$(SRCS)) $(patsubst %.S,$(OUTPUT_DIR)/%.o,$(wildcard *.S))

# Include directories
INC_DIRS += \
  ./ \
  ./Include \
  ./CMSIS_5/CMSIS/Core/Include \
  ./routines

INCLUDES = $(INC_DIRS:%=-I%)

# Compiler flags
OPT = -O3 -g3

CFLAGS = ${INCLUDES}
CFLAGS += $(OPT)
CFLAGS += -DNRF${NRF_DEV_NUM}_XXAA
CFLAGS += -DARM_MATH_CM4
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -Wall
CFLAGS += -fno-builtin
CFLAGS += -mthumb
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mabi=aapcs
CFLAGS += -mfloat-abi=hard
CFLAGS += -mfpu=fpv4-sp-d16
CFLAGS += -fsingle-precision-constant
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
CFLAGS += -Wa,-adhlns="$@.lst"

CPPFLAGS = ${CFLAGS} -fno-exceptions

ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF${NRF_DEV_NUM}_XXAA

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -T$(LINKER_SCRIPT)
LDFLAGS += -mthumb -mabi=aapcs
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS += -Wl,--gc-sections,-Map=${OUTPUT_DIR}/build.map
LDFLAGS += --specs=nosys.specs

ARFLAGS = -rcs

.PHONY: all clean app flash recover

all: app
	@echo "Build successful!"

app: ${OUTPUT_DIR}/build.hex

recover:
	nrfjprog -f NRF52 --recover

clear:
	rm -rf $(OUTPUT_DIR)/*

flash: app
	nrfjprog -f NRF52 --program ${OUTPUT_DIR}/build.hex --chiperase --verify
	nrfjprog -f NRF52 --pinresetenable
	nrfjprog -f NRF52 --reset

# Compilation rules
${OUTPUT_DIR}/%.o: %.c
	@mkdir -p $(@D)
	@echo "CC $<"
	$(CC) ${CFLAGS} -c $< -o $@

${OUTPUT_DIR}/%.o: %.S
	@mkdir -p $(@D)
	@echo "AS $<"
	$(CC) ${ASMFLAGS} -c $< -o $@

${OUTPUT_DIR}/%.o: %.cpp
	@mkdir -p $(@D)
	@echo "CXX $<"
	$(CXX) ${CPPFLAGS} -c $< -o $@

# Linking
${OUTPUT_DIR}/build.elf: $(OBJS)
	@echo "Linking $@"
	$(CXX) ${LDFLAGS} $(OBJS) -o $@ ${LIB_FILES}
	$(SIZE) $@

# Hex generation
${OUTPUT_DIR}/build.hex: ${OUTPUT_DIR}/build.elf
	@echo "Generating HEX"
	$(OBJCOPY) -O ihex $< $@