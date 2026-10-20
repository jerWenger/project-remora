# VCB pixhawk-pwm firmware - command-line build with arm-none-eabi-gcc.
# STM32CubeIDE builds the same tree from .project/.cproject; this Makefile is
# for CI-style verification and for people without the IDE.
#
#   make            -> build/VCB-pixhawk.elf/.bin/.hex/.map
#   make size
#   make clean
#   make TOOLCHAIN=/path/to/bin/   (prefix with trailing slash)

TARGET    := VCB-pixhawk
BUILD     := build
TOOLCHAIN ?=
CC   := $(TOOLCHAIN)arm-none-eabi-gcc
CXX  := $(TOOLCHAIN)arm-none-eabi-g++
OBJCOPY := $(TOOLCHAIN)arm-none-eabi-objcopy
SIZE := $(TOOLCHAIN)arm-none-eabi-size

MCU  := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
DEFS := -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_PWR_LDO_SUPPLY
INCS := -ICore/Inc \
        -IDrivers/STM32H7xx_HAL_Driver/Inc \
        -IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy \
        -IDrivers/CMSIS/Device/ST/STM32H7xx/Include \
        -IDrivers/CMSIS/Include

OPT      ?= -Os
COMMON   := $(MCU) $(DEFS) $(INCS) $(OPT) -g3 -Wall -Wextra -Wno-missing-field-initializers -Wno-unused-parameter -ffunction-sections -fdata-sections -fstack-usage -MMD -MP
CFLAGS   := $(COMMON) -std=gnu11
CXXFLAGS := $(COMMON) -std=gnu++17 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics
ASFLAGS  := $(MCU) -x assembler-with-cpp -g3

# newlib-nano if the toolchain has it (CubeIDE default), full newlib otherwise.
NANO_SPECS = $(shell $(CC) -print-file-name=nano.specs)
ifneq ($(NANO_SPECS),nano.specs)
  SPECS := --specs=nano.specs -u _printf_float
else
  SPECS :=
endif
LDFLAGS := $(MCU) -T STM32H723VGTX_FLASH.ld $(SPECS) -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map -static \
           -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group

C_SRCS   := $(wildcard Core/Src/*.c) $(wildcard Drivers/STM32H7xx_HAL_Driver/Src/*.c)
CXX_SRCS := $(wildcard Core/Src/*.cpp)
AS_SRCS  := $(wildcard Core/Startup/*.s)
OBJS := $(patsubst %.c,$(BUILD)/%.o,$(C_SRCS)) $(patsubst %.cpp,$(BUILD)/%.o,$(CXX_SRCS)) $(patsubst %.s,$(BUILD)/%.o,$(AS_SRCS))

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex size

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(ASFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS) STM32H723VGTX_FLASH.ld
	$(CXX) $(OBJS) $(LDFLAGS) -o $@

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/%.hex: $(BUILD)/%.elf
	$(OBJCOPY) -O ihex $< $@

size: $(BUILD)/$(TARGET).elf
	$(SIZE) $<

clean:
	rm -rf $(BUILD)

-include $(OBJS:.o=.d)
.PHONY: all size clean
