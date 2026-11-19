MCU       ?= atmega328p
F_CPU     ?= 14745600
AVR_CC       ?= avr-gcc
AVR_OBJCOPY  ?= avr-objcopy
AVR_SIZE     ?= avr-size
BUILD_DIR := build

SOURCES := \
	src/main.c \
	src/Charging.c \
	src/Discharging.c \
	src/Storage.c \
	src/Capacity.c \
	src/twi.c \
	src/ina233.c \
	src/Encoder.c \
	src/Buzzer.c \
	src/OLED_Screen.c

HEADERS := \
	include/config.h \
	include/Charging.h \
	include/Discharging.h \
	include/Storage.h \
	include/Capacity.h \
	include/twi.h \
	include/ina233.h \
	include/Encoder.h \
	include/Buzzer.h \
	include/OLED_Screen.h

CFLAGS := -mmcu=$(MCU) -DF_CPU=$(F_CPU)UL -std=gnu99 -Os \
	-Wall -Wextra -Werror -Iinclude -ffunction-sections -fdata-sections
LDFLAGS := -mmcu=$(MCU) -Wl,--gc-sections -lm

.PHONY: all clean size test

all: $(BUILD_DIR)/firmware.hex

$(BUILD_DIR):
	if not exist $(BUILD_DIR) mkdir $(BUILD_DIR)

$(BUILD_DIR)/firmware.elf: $(SOURCES) $(HEADERS) | $(BUILD_DIR)
	$(AVR_CC) $(CFLAGS) $(SOURCES) $(LDFLAGS) -o $@
	$(AVR_SIZE) $@

$(BUILD_DIR)/firmware.hex: $(BUILD_DIR)/firmware.elf
	$(AVR_OBJCOPY) -O ihex -R .eeprom $< $@

size: $(BUILD_DIR)/firmware.elf
	$(AVR_SIZE) $<

test: $(SOURCES) $(HEADERS) tests/test_firmware.c | $(BUILD_DIR)
	$(AVR_CC) $(CFLAGS) tests/test_firmware.c src/Charging.c src/Discharging.c src/Storage.c src/Capacity.c src/twi.c src/ina233.c src/Encoder.c src/Buzzer.c $(LDFLAGS) -o $(BUILD_DIR)/test_firmware.elf
	avr-gdb -batch -ex "target sim" -ex "load" -ex "run" -ex "print (int)*(unsigned short*)&s_passed_tests" -ex "print (int)*(unsigned short*)&s_failed_tests" $(BUILD_DIR)/test_firmware.elf

clean:
	if exist $(BUILD_DIR) rmdir /s /q $(BUILD_DIR)
