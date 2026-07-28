SDK_DIR ?= C:/ti/mspm0_sdk_2_10_00_04
TIARMCLANG ?= D:/DevTools/ti/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe
TIARMAR ?= D:/DevTools/ti/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmar.exe

CONFIG_DIR := ../MSPM0G3507_LineFollowing_Car
FREERTOS_DIR := $(SDK_DIR)/kernel/freertos/Source
PORT_DIR := $(FREERTOS_DIR)/portable/TI_ARM_CLANG/ARM_CM0
BUILD_DIR := Debug
LIBRARY := $(BUILD_DIR)/freertos_kernel_ticlang.lib

CPPFLAGS := -D__MSPM0G3507__ \
    -I$(CONFIG_DIR) \
    -I$(FREERTOS_DIR)/include \
    -I$(PORT_DIR) \
    -I$(SDK_DIR)/source/third_party/CMSIS/Core/Include \
    -I$(SDK_DIR)/source
CFLAGS := -mcpu=cortex-m0plus -march=thumbv6m -mfloat-abi=soft -mthumb -O2 -Wall

SOURCES := $(FREERTOS_DIR)/tasks.c $(FREERTOS_DIR)/list.c \
    $(PORT_DIR)/port.c portasm_diagnostic.c
OBJECTS := $(BUILD_DIR)/tasks.o $(BUILD_DIR)/list.o \
    $(BUILD_DIR)/port.o $(BUILD_DIR)/portasm.o

.PHONY: all clean

all: $(LIBRARY)

$(BUILD_DIR):
	@if not exist $(BUILD_DIR) mkdir $(BUILD_DIR)

$(BUILD_DIR)/tasks.o: $(FREERTOS_DIR)/tasks.c $(CONFIG_DIR)/FreeRTOSConfig.h | $(BUILD_DIR)
	$(TIARMCLANG) -c $(CPPFLAGS) $(CFLAGS) -o $@ $<

$(BUILD_DIR)/list.o: $(FREERTOS_DIR)/list.c $(CONFIG_DIR)/FreeRTOSConfig.h | $(BUILD_DIR)
	$(TIARMCLANG) -c $(CPPFLAGS) $(CFLAGS) -o $@ $<

$(BUILD_DIR)/port.o: $(PORT_DIR)/port.c $(CONFIG_DIR)/FreeRTOSConfig.h | $(BUILD_DIR)
	$(TIARMCLANG) -c $(CPPFLAGS) $(CFLAGS) -o $@ $<

$(BUILD_DIR)/portasm.o: portasm_diagnostic.c $(PORT_DIR)/portasm.c $(CONFIG_DIR)/FreeRTOSConfig.h | $(BUILD_DIR)
	$(TIARMCLANG) -c $(CPPFLAGS) $(CFLAGS) -o $@ $<

$(LIBRARY): $(OBJECTS)
	$(TIARMAR) r $@ $(OBJECTS)

clean:
	@if exist $(BUILD_DIR) rmdir /S /Q $(BUILD_DIR)
