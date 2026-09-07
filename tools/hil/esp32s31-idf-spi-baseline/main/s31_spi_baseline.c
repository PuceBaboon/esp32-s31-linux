// SPDX-License-Identifier: MIT

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/soc.h"
#include "soc/axi_dma_reg.h"
#include "soc/spi_struct.h"

#define BASELINE_HOST SPI2_HOST
#define PIN_SCLK 42
#define PIN_MOSI 43
#define PIN_CS   44
#define PIN_MISO 45

static uint32_t crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = ~0U;

    while (length--) {
        crc ^= *data++;
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xedb88320U & -(crc & 1U));
        }
    }
    return ~crc;
}

static void print_pin_registers(void)
{
    printf("PIN iomux42=%08" PRIx32 " iomux43=%08" PRIx32
           " iomux44=%08" PRIx32 " iomux45=%08" PRIx32
           " out42=%08" PRIx32 " out43=%08" PRIx32
           " out44=%08" PRIx32 " out45=%08" PRIx32 "\n",
           REG_READ(IO_MUX_GPIO42_REG), REG_READ(IO_MUX_GPIO43_REG),
           REG_READ(IO_MUX_GPIO44_REG), REG_READ(IO_MUX_GPIO45_REG),
           REG_READ(GPIO_FUNC42_OUT_SEL_CFG_REG),
           REG_READ(GPIO_FUNC43_OUT_SEL_CFG_REG),
           REG_READ(GPIO_FUNC44_OUT_SEL_CFG_REG),
           REG_READ(GPIO_FUNC45_OUT_SEL_CFG_REG));
}

static void print_dma_registers(void)
{
    for (unsigned channel = 0; channel < 3; ++channel) {
        uintptr_t rx = DR_REG_AXI_DMA_BASE + channel * 0x68;
        uintptr_t tx = DR_REG_AXI_DMA_BASE + 0x138 + channel * 0x68;

        printf("GDMA ch=%u rx-conf0=%08" PRIx32 " rx-conf1=%08" PRIx32
               " rx-pri=%08" PRIx32 " rx-peri=%08" PRIx32
               " tx-conf0=%08" PRIx32 " tx-conf1=%08" PRIx32
               " tx-pri=%08" PRIx32 " tx-peri=%08" PRIx32 "\n",
               channel, REG_READ(rx + 0x10), REG_READ(rx + 0x14),
               REG_READ(rx + 0x40), REG_READ(rx + 0x44),
               REG_READ(tx + 0x10), REG_READ(tx + 0x14),
               REG_READ(tx + 0x40), REG_READ(tx + 0x44));
    }
}

static esp_err_t run_transfer(unsigned mode, uint32_t speed, size_t length)
{
    uint8_t *tx = heap_caps_malloc(length, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    uint8_t *rx = heap_caps_calloc(length, 1, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (tx == NULL || rx == NULL) {
        free(tx);
        free(rx);
        return ESP_ERR_NO_MEM;
    }
    for (size_t i = 0; i < length; ++i) {
        tx[i] = (uint8_t)(i * 37U + 11U);
    }

    spi_device_interface_config_t device_config = {
        .clock_speed_hz = (int)speed,
        .mode = (uint8_t)mode,
	.spics_io_num = -1,
        .queue_size = 1,
    };
    spi_device_handle_t device = NULL;
    esp_err_t err = spi_bus_add_device(BASELINE_HOST, &device_config, &device);
    int actual_khz = 0;
    if (err == ESP_OK) {
        spi_device_get_actual_freq(device, &actual_khz);
        spi_transaction_t transaction = {
            .length = length * 8,
            .tx_buffer = tx,
            .rx_buffer = rx,
        };
		gpio_set_level(PIN_CS, 0);
		esp_rom_delay_us(2);
		err = spi_device_transmit(device, &transaction);
		esp_rom_delay_us(2);
		gpio_set_level(PIN_CS, 1);
    }
	print_dma_registers();
	printf("REG ctrl=%08" PRIx32 " clock=%08" PRIx32
	       " user=%08" PRIx32 " user1=%08" PRIx32
	       " user2=%08" PRIx32 " misc=%08" PRIx32
	       " dinmode=%08" PRIx32 " dinnum=%08" PRIx32
	       " dout=%08" PRIx32 " dma=%08" PRIx32
	       " gate=%08" PRIx32 "\n",
	       GPSPI2.ctrl.val, GPSPI2.clock.val, GPSPI2.user.val,
	       GPSPI2.user1.val, GPSPI2.user2.val, GPSPI2.misc.val,
	       GPSPI2.din_mode.val, GPSPI2.din_num.val,
	       GPSPI2.dout_mode.val, GPSPI2.dma_conf.val,
	       GPSPI2.clk_gate.val);

    size_t mismatch = length;
    if (err == ESP_OK) {
        for (size_t i = 0; i < length; ++i) {
            if (rx[i] != (uint8_t)(i ^ 0xa5U)) {
                mismatch = i;
                break;
            }
        }
    }
    printf("%s idf-spi mode=%u requested=%" PRIu32
           " effective=%d bytes=%u tx_crc32=%08" PRIx32
           " rx_crc32=%08" PRIx32,
           err == ESP_OK && mismatch == length ? "PASS" : "FAIL",
           mode, speed, actual_khz * 1000,
           (unsigned)length, crc32(tx, length), crc32(rx, length));
    if (mismatch != length) {
        printf(" mismatch=%u got=%02x expected=%02x", (unsigned)mismatch,
               rx[mismatch], (uint8_t)(mismatch ^ 0xa5U));
    }
    printf(" result=%s\n", esp_err_to_name(err));
    fflush(stdout);

    if (device != NULL) {
        spi_bus_remove_device(device);
    }
    free(tx);
    free(rx);
    return err == ESP_OK && mismatch == length ? ESP_OK : ESP_FAIL;
}

void app_main(void)
{
	gpio_config_t cs_config = {
		.pin_bit_mask = BIT64(PIN_CS),
		.mode = GPIO_MODE_OUTPUT,
	};
	gpio_config(&cs_config);
	gpio_set_level(PIN_CS, 1);

    spi_bus_config_t bus_config = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(BASELINE_HOST, &bus_config,
                                       SPI_DMA_CH_AUTO));
    print_pin_registers();
    printf("READY s31-idf-spi pins=sclk:%d,mosi:%d,miso:%d,cs:%d\n",
           PIN_SCLK, PIN_MOSI, PIN_MISO, PIN_CS);
    fflush(stdout);

    char line[96];
    size_t used = 0;
    while (true) {
        int character = getchar();
        if (character < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (character != '\r' && character != '\n') {
            if (used + 1 < sizeof(line)) {
                line[used++] = (char)character;
            }
            continue;
        }
        if (used == 0) {
            continue;
        }
        line[used] = '\0';
        used = 0;
        unsigned mode = 0;
        unsigned speed = 0;
        unsigned length = 0;

        if (sscanf(line, "spi %u %u %u", &mode, &speed, &length) == 3 &&
            mode <= 3 && speed > 0 && speed <= 20000000 &&
            length > 0 && length <= 4096) {
            run_transfer(mode, speed, length);
        } else if (strncmp(line, "hello", 5) == 0) {
            printf("READY s31-idf-spi\n");
            fflush(stdout);
        } else {
            printf("ERROR use: spi <mode:0-3> <hz:1-20000000> <bytes:1-4096>\n");
            fflush(stdout);
        }
    }
}
