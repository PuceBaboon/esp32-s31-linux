SHELL := /bin/bash

IDF_PATH ?= $(HOME)/.espressif/master/esp-idf
IDF_PY := $(IDF_PATH)/tools/idf.py
BUILD_DIR := $(CURDIR)/build
LP_OUTPUT := $(BUILD_DIR)/esp-idf/main/s31_lp_main/s31_lp_main
STAGE_DIR := $(CURDIR)/../buildroot-external/board/esp32-s31/overlay/lib/firmware/esp32s31

.PHONY: all stage clean

all:
	. $(IDF_PATH)/export.sh >/dev/null && \
		IDF_TARGET=esp32s31 $(IDF_PY) -B $(BUILD_DIR) build

stage: all
	install -D -m 0644 $(LP_OUTPUT).elf $(STAGE_DIR)/s31-lp-core.elf
	install -D -m 0644 $(LP_OUTPUT).bin $(STAGE_DIR)/s31-lp-core.bin

clean:
	rm -rf $(BUILD_DIR) sdkconfig sdkconfig.old
