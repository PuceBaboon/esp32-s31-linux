// SPDX-License-Identifier: Apache-2.0

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <netdb.h>

#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "driver/i2c_slave.h"
#include "driver/i2s_std.h"
#include "driver/pulse_cnt.h"
#include "driver/spi_slave.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_chip_info.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_hosted.h"
#include "esp_eth.h"
#include "esp_eth_mac_esp.h"
#include "esp_eth_netif_glue.h"
#include "esp_eth_phy.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal/spi_ll.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "lwip/ip_addr.h"
#include "lwip/sockets.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#define HIL_SPI_HOST SPI3_HOST
#include "nvs_flash.h"
#include "os/os_mbuf.h"

#define HIL_PROTOCOL_VERSION 1
#define HIL_ARM_WINDOW_US (10LL * 1000LL * 1000LL)
#define HIL_LANE_COUNT 4
#define BLE_SYNC_BIT BIT0
#define ETH_LINK_BIT BIT1
#define WIFI_IP_BIT BIT2
#define WIFI_DISCONNECTED_BIT BIT3
#define BLE_FOUND_BIT BIT8
#define BLE_CONNECTED_BIT BIT9
#define BLE_DISCONNECTED_BIT BIT10
#define BLE_SERVICE_BIT BIT11
#define BLE_CHARACTERISTIC_BIT BIT12
#define BLE_READ_BIT BIT13

/*
 * Logical fixture lanes follow the S31 order GPIO42..45.  The assembled
 * harness is physically reversed at the P4 header, so L0..L3 land on
 * GPIO23..20 respectively.  Protocol tests use lanes; raw GPIO commands
 * remain available for fixture diagnosis.
 */
static const gpio_num_t s_hil_lane_pins[HIL_LANE_COUNT] = {
    GPIO_NUM_23, GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_20,
};

static const gpio_num_t s_hil_pins[] = {
    GPIO_NUM_2, GPIO_NUM_3, GPIO_NUM_4, GPIO_NUM_5,
    GPIO_NUM_20, GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_23,
    GPIO_NUM_46, GPIO_NUM_47, GPIO_NUM_48,
};

static SemaphoreHandle_t s_output_lock;
static EventGroupHandle_t s_ble_events;
static bool s_wifi_ready;
static esp_netif_t *s_wifi_ap_netif;
static TaskHandle_t s_wifi_ap_echo_task;
static volatile bool s_wifi_ap_active;
static volatile uint32_t s_wifi_ap_packets;
static volatile uint32_t s_wifi_ap_bytes;
static volatile uint32_t s_wifi_ap_pattern_errors;
static volatile uint32_t s_wifi_ap_echo_errors;
static volatile uint32_t s_wifi_ap_connects;
static volatile uint32_t s_wifi_ap_disconnects;
static volatile uint32_t s_wifi_ap_last_disconnect_reason;
static bool s_ble_started;
static esp_netif_ip_info_t s_wifi_ip;
static ble_addr_t s_ble_target;
static uint16_t s_ble_conn_handle;
static uint16_t s_ble_service_start;
static uint16_t s_ble_service_end;
static uint16_t s_ble_value_handle;
static int s_ble_status;
static char s_ble_value[16];
static esp_eth_handle_t s_eth_handle;
static esp_netif_t *s_eth_netif;
static esp_eth_netif_glue_handle_t s_eth_glue;
static esp_eth_mac_t *s_eth_mac;
static esp_eth_phy_t *s_eth_phy;
static TaskHandle_t s_eth_echo_task;
static bool s_armed;
static int64_t s_arm_deadline_us;
static uint32_t s_arm_token;
static TaskHandle_t s_uart_peer_task;
static TaskHandle_t s_spi_peer_task;
static TaskHandle_t s_i2c_peer_task;
static TaskHandle_t s_gpio_wake_task;
static gpio_num_t s_gpio_wake_pin;
static int s_gpio_wake_inactive;
static uint32_t s_gpio_wake_delay_ms;
static uint32_t s_gpio_wake_hold_ms;
static QueueHandle_t s_i2c_events;
static i2c_slave_dev_handle_t s_i2c_slave;
static bool s_spi_active;
static bool s_spi_master_active;
static spi_device_handle_t s_spi_master_device;
static uint32_t s_uart_bytes;
static uint32_t s_uart_crc = ~0U;
static volatile uint32_t s_spi_bytes;
static volatile uint32_t s_spi_crc = ~0U;
static volatile uint32_t s_spi_completions;
static volatile size_t s_spi_first_bits;
static volatile size_t s_spi_last_bits;
static uint8_t s_spi_first_rx[16];
static size_t s_spi_length = 4096;
static uint8_t *s_spi_tx;
static uint8_t *s_spi_rx;
static spi_slave_transaction_t s_spi_transaction;
static pcnt_unit_handle_t s_spi_clock_unit;
static pcnt_channel_handle_t s_spi_clock_channel;
static pcnt_unit_handle_t s_spi_cs_unit;
static pcnt_channel_handle_t s_spi_cs_channel;
static uint8_t s_i2c_registers[256];
static uint8_t *s_i2c_rx;
static uint8_t *s_i2c_response;
static size_t s_i2c_response_length = 16;
static volatile size_t s_i2c_rx_length;
static volatile uint8_t s_i2c_register;
static uint32_t s_i2c_reads;
static uint32_t s_i2c_writes;
static volatile uint32_t s_pulse_rises;
static volatile uint32_t s_pulse_falls;
static volatile int64_t s_pulse_first_rise;
static volatile int64_t s_pulse_last_rise;
static volatile int64_t s_pulse_last_edge;
static volatile uint64_t s_pulse_high_us;
static volatile uint64_t s_pulse_low_us;
static volatile int s_pulse_level;
static bool s_pulse_monitor_active;
static i2s_chan_handle_t s_i2s_channel;
static TaskHandle_t s_i2s_peer_task;
static uint8_t *s_i2s_buffer;
static size_t s_i2s_length;
static size_t s_i2s_scan_limit;
static uint8_t s_i2s_raw_sample[32];
static size_t s_i2s_raw_length;
static volatile size_t s_i2s_bytes;
static volatile size_t s_i2s_discarded;
static volatile int s_i2s_mismatch = -1;
static volatile bool s_i2s_ready;
static volatile bool s_i2s_stop_requested;
static volatile esp_err_t s_i2s_error;

enum hil_i2c_event {
    HIL_I2C_RECEIVE,
    HIL_I2C_REQUEST,
};

static void hil_pulse_monitor_stop(void);

static void hil_i2s_peer_stop(void)
{
    s_i2s_stop_requested = true;
    if (s_i2s_channel != NULL) {
        i2s_channel_disable(s_i2s_channel);
    }
    if (s_i2s_peer_task != NULL) {
        for (unsigned wait = 0; wait < 50 && s_i2s_peer_task != NULL; ++wait) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (s_i2s_peer_task != NULL) {
            vTaskDelete(s_i2s_peer_task);
            s_i2s_peer_task = NULL;
        }
    }
    if (s_i2s_channel != NULL) {
        i2s_del_channel(s_i2s_channel);
        s_i2s_channel = NULL;
    }
    free(s_i2s_buffer);
    s_i2s_buffer = NULL;
    s_i2s_bytes = 0;
}

static void hil_emit(const char *status, const char *test, const char *level,
                     const char *detail)
{
    if (s_output_lock != NULL) {
        xSemaphoreTake(s_output_lock, portMAX_DELAY);
    }
    printf("HIL1 {\"board\":\"esp32-p4-wifi6-dev-kit\","
           "\"status\":\"%s\",\"test\":\"%s\",\"level\":\"%s\","
           "\"detail\":\"%s\"}\n",
           status, test, level, detail);
    fflush(stdout);
    if (s_output_lock != NULL) {
        xSemaphoreGive(s_output_lock);
    }
}

static bool hil_pin_allowed(int pin)
{
    for (size_t i = 0; i < sizeof(s_hil_pins) / sizeof(s_hil_pins[0]); ++i) {
        if ((int)s_hil_pins[i] == pin) {
            return true;
        }
    }
    return false;
}

static uint32_t hil_crc32_update(uint32_t crc, const uint8_t *data, size_t length)
{
    while (length-- != 0) {
        crc ^= *data++;
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xedb88320U & -(crc & 1U));
        }
    }
    return crc;
}

static void hil_peer_stop(void)
{
    if (s_gpio_wake_task != NULL &&
        s_gpio_wake_task != xTaskGetCurrentTaskHandle()) {
        vTaskDelete(s_gpio_wake_task);
        s_gpio_wake_task = NULL;
    }
    hil_pulse_monitor_stop();
    hil_i2s_peer_stop();
    if (s_uart_peer_task != NULL) {
        vTaskDelete(s_uart_peer_task);
        s_uart_peer_task = NULL;
    }
    if (uart_is_driver_installed(UART_NUM_1)) {
        uart_driver_delete(UART_NUM_1);
    }
    if (s_spi_peer_task != NULL) {
        vTaskDelete(s_spi_peer_task);
        s_spi_peer_task = NULL;
    }
    if (s_spi_active) {
        spi_slave_free(HIL_SPI_HOST);
        s_spi_active = false;
    }
    if (s_spi_master_device != NULL) {
        spi_bus_remove_device(s_spi_master_device);
        s_spi_master_device = NULL;
    }
    if (s_spi_master_active) {
        spi_bus_free(HIL_SPI_HOST);
        s_spi_master_active = false;
    }
    if (s_spi_clock_unit != NULL) {
        pcnt_unit_stop(s_spi_clock_unit);
        pcnt_unit_disable(s_spi_clock_unit);
    }
    if (s_spi_clock_channel != NULL) {
        pcnt_del_channel(s_spi_clock_channel);
        s_spi_clock_channel = NULL;
    }
    if (s_spi_clock_unit != NULL) {
        pcnt_del_unit(s_spi_clock_unit);
        s_spi_clock_unit = NULL;
    }
    if (s_spi_cs_unit != NULL) {
        pcnt_unit_stop(s_spi_cs_unit);
        pcnt_unit_disable(s_spi_cs_unit);
    }
    if (s_spi_cs_channel != NULL) {
        pcnt_del_channel(s_spi_cs_channel);
        s_spi_cs_channel = NULL;
    }
    if (s_spi_cs_unit != NULL) {
        pcnt_del_unit(s_spi_cs_unit);
        s_spi_cs_unit = NULL;
    }
    free(s_spi_tx);
    free(s_spi_rx);
    s_spi_tx = NULL;
    s_spi_rx = NULL;
    if (s_i2c_peer_task != NULL) {
        vTaskDelete(s_i2c_peer_task);
        s_i2c_peer_task = NULL;
    }
    if (s_i2c_slave != NULL) {
        i2c_del_slave_device(s_i2c_slave);
        s_i2c_slave = NULL;
    }
    if (s_i2c_events != NULL) {
        vQueueDelete(s_i2c_events);
        s_i2c_events = NULL;
    }
    free(s_i2c_rx);
    free(s_i2c_response);
    s_i2c_rx = NULL;
    s_i2c_response = NULL;
    s_i2c_rx_length = 0;
}

static void hil_gpio_wake_task(void *argument)
{
    (void)argument;
    gpio_num_t pin = s_gpio_wake_pin;
    int inactive = s_gpio_wake_inactive;
    uint32_t delay_ms = s_gpio_wake_delay_ms;
    uint32_t hold_ms = s_gpio_wake_hold_ms;

    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    gpio_set_level(pin, !inactive);
    vTaskDelay(pdMS_TO_TICKS(hold_ms));
    gpio_set_level(pin, inactive);
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(pin, GPIO_FLOATING);
    s_gpio_wake_task = NULL;

    char detail[128];
    snprintf(detail, sizeof(detail),
             "gpio=%d active=%d delay_ms=%" PRIu32 " hold_ms=%" PRIu32
             " final=high-z",
             pin, !inactive, delay_ms, hold_ms);
    hil_emit("PASS", "peer.gpio-wake-complete", "recovery", detail);
    vTaskDelete(NULL);
}

static esp_err_t hil_gpio_wake_schedule(gpio_num_t pin, int inactive,
                                        uint32_t delay_ms, uint32_t hold_ms)
{
    if (s_gpio_wake_task != NULL)
        return ESP_ERR_INVALID_STATE;

    hil_peer_stop();
    gpio_config_t config = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&config);
    if (err != ESP_OK)
        return err;
    err = gpio_set_level(pin, inactive);
    if (err != ESP_OK) {
        gpio_set_direction(pin, GPIO_MODE_INPUT);
        return err;
    }

    s_gpio_wake_pin = pin;
    s_gpio_wake_inactive = inactive;
    s_gpio_wake_delay_ms = delay_ms;
    s_gpio_wake_hold_ms = hold_ms;
    if (xTaskCreate(hil_gpio_wake_task, "hil-gpio-wake", 3072, NULL, 7,
                    &s_gpio_wake_task) != pdPASS) {
        gpio_set_direction(pin, GPIO_MODE_INPUT);
        gpio_set_pull_mode(pin, GPIO_FLOATING);
        s_gpio_wake_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static esp_err_t hil_edge_counters_start(gpio_num_t first, gpio_num_t second)
{
    pcnt_unit_config_t unit_config = {
        .low_limit = -32768,
        .high_limit = 32767,
        .flags.accum_count = true,
    };
    pcnt_chan_config_t channel_config = {
        .edge_gpio_num = first,
        .level_gpio_num = -1,
    };
    esp_err_t err = pcnt_new_unit(&unit_config, &s_spi_clock_unit);
    if (err == ESP_OK) {
        err = pcnt_new_channel(s_spi_clock_unit, &channel_config,
                               &s_spi_clock_channel);
    }
    if (err == ESP_OK) {
        err = pcnt_channel_set_edge_action(
            s_spi_clock_channel, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_HOLD);
    }
    if (err == ESP_OK) {
        err = pcnt_unit_enable(s_spi_clock_unit);
    }
    if (err == ESP_OK) {
        err = pcnt_unit_clear_count(s_spi_clock_unit);
    }
    if (err == ESP_OK) {
        err = pcnt_unit_start(s_spi_clock_unit);
    }
    if (err == ESP_OK) {
        err = pcnt_new_unit(&unit_config, &s_spi_cs_unit);
    }
    channel_config.edge_gpio_num = second;
    if (err == ESP_OK) {
        err = pcnt_new_channel(s_spi_cs_unit, &channel_config,
                               &s_spi_cs_channel);
    }
    if (err == ESP_OK) {
        err = pcnt_channel_set_edge_action(
            s_spi_cs_channel, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_HOLD);
    }
    if (err == ESP_OK) {
        err = pcnt_unit_enable(s_spi_cs_unit);
    }
    if (err == ESP_OK) {
        err = pcnt_unit_clear_count(s_spi_cs_unit);
    }
    if (err == ESP_OK) {
        err = pcnt_unit_start(s_spi_cs_unit);
    }
    return err;
}

static void hil_i2s_rx_task(void *argument)
{
    (void)argument;
    size_t total = 0;
    size_t discarded = 0;
    size_t matched = 0;
    uint8_t scratch[512];
    const size_t sync_length = 16;
    s_i2s_error = i2s_channel_enable(s_i2s_channel);
    s_i2s_ready = true;
    if (s_i2s_error == ESP_OK) {
        while (total < s_i2s_length && !s_i2s_stop_requested) {
            size_t done = 0;
            esp_err_t err = i2s_channel_read(s_i2s_channel,
                                             scratch, sizeof(scratch), &done,
                                             3000);
            if (err != ESP_OK || done == 0) {
                s_i2s_error = err;
                break;
            }
            if (!s_i2s_raw_length) {
                for (size_t i = 0; i < done; i++) {
                    if (scratch[i]) {
                        s_i2s_raw_length = done - i < sizeof(s_i2s_raw_sample) ?
                                          done - i : sizeof(s_i2s_raw_sample);
                        memcpy(s_i2s_raw_sample, scratch + i, s_i2s_raw_length);
                        break;
                    }
                }
            }
            for (size_t i = 0; i < done && total < s_i2s_length; ++i) {
                uint8_t value = scratch[i];
                if (matched < sync_length) {
                    uint8_t expected = (uint8_t)(matched * 37U + 11U);
                    if (value == expected) {
                        matched++;
                        if (matched == sync_length) {
                            for (size_t j = 0; j < sync_length; ++j) {
                                s_i2s_buffer[j] =
                                    (uint8_t)(j * 37U + 11U);
                            }
                            total = sync_length;
                        }
                    } else {
                        matched = value == 11U ? 1 : 0;
                    }
                    discarded++;
                } else {
                    s_i2s_buffer[total++] = value;
                }
            }
            s_i2s_bytes = total;
            s_i2s_discarded = discarded >= sync_length ?
                              discarded - sync_length : discarded;
            if (matched < sync_length && discarded >= s_i2s_scan_limit) {
                s_i2s_error = ESP_ERR_TIMEOUT;
                break;
            }
        }
    }
    s_i2s_bytes = total;
    s_i2s_discarded = discarded >= sync_length ?
                      discarded - sync_length : discarded;
    s_i2s_mismatch = -1;
    for (size_t i = 0; i < total; ++i) {
        if (s_i2s_buffer[i] != (uint8_t)(i * 37U + 11U)) {
            s_i2s_mismatch = (int)i;
            break;
        }
    }
    s_i2s_peer_task = NULL;
    vTaskDelete(NULL);
}

static void hil_i2s_tx_task(void *argument)
{
    (void)argument;
    size_t total = 0;
    /* Populate DMA before the master starts clocks. API timeouts are ms. */
    s_i2s_error = i2s_channel_preload_data(s_i2s_channel, s_i2s_buffer,
                                         s_i2s_length, &total);
    if (s_i2s_error == ESP_OK) {
        s_i2s_error = i2s_channel_enable(s_i2s_channel);
    }
    s_i2s_ready = true;
    if (s_i2s_error == ESP_OK) {
        while (total < s_i2s_length && !s_i2s_stop_requested) {
            size_t done = 0;
            esp_err_t err = i2s_channel_write(s_i2s_channel,
                                              s_i2s_buffer + total,
                                              s_i2s_length - total, &done,
                                              3000);
            if (err != ESP_OK || done == 0) {
                s_i2s_error = err;
                break;
            }
            total += done;
        }
    }
    s_i2s_bytes = total;
    s_i2s_peer_task = NULL;
    vTaskDelete(NULL);
}

static esp_err_t hil_i2s_peer_start(bool transmit, bool master, size_t length,
                                    uint32_t rate)
{
    hil_peer_stop();
    s_i2s_buffer = malloc(length);
    if (s_i2s_buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_i2s_length = length;
    s_i2s_scan_limit = (size_t)rate * 2U * sizeof(int16_t) * 10U;
    s_i2s_raw_length = 0;
    s_i2s_bytes = 0;
    s_i2s_discarded = 0;
    s_i2s_mismatch = -1;
    s_i2s_ready = false;
    s_i2s_stop_requested = false;
    s_i2s_error = ESP_OK;
    for (size_t i = 0; i < length; ++i) {
        s_i2s_buffer[i] = transmit ? (uint8_t)(i * 37U + 11U) : 0;
    }
    esp_err_t err = hil_edge_counters_start(s_hil_lane_pins[0],
                                             s_hil_lane_pins[1]);
    if (err != ESP_OK) {
        hil_peer_stop();
        return err;
    }
    i2s_chan_config_t chan_config = I2S_CHANNEL_DEFAULT_CONFIG(
        I2S_NUM_0, master ? I2S_ROLE_MASTER : I2S_ROLE_SLAVE);
    err = transmit ?
        i2s_new_channel(&chan_config, &s_i2s_channel, NULL) :
        i2s_new_channel(&chan_config, NULL, &s_i2s_channel);
    if (err != ESP_OK) {
        hil_i2s_peer_stop();
        return err;
    }
    i2s_std_config_t config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = s_hil_lane_pins[0],
            .ws = s_hil_lane_pins[1],
            .dout = transmit ? s_hil_lane_pins[3] : I2S_GPIO_UNUSED,
            .din = transmit ? I2S_GPIO_UNUSED : s_hil_lane_pins[2],
        },
    };
    err = i2s_channel_init_std_mode(s_i2s_channel, &config);
    if (err == ESP_OK && master) {
        err = gpio_set_drive_capability(s_hil_lane_pins[0], GPIO_DRIVE_CAP_2);
    }
    if (err == ESP_OK && master) {
        err = gpio_set_drive_capability(s_hil_lane_pins[1], GPIO_DRIVE_CAP_2);
    }
    if (err == ESP_OK && transmit) {
        err = gpio_set_drive_capability(s_hil_lane_pins[3], GPIO_DRIVE_CAP_2);
    }
    if (err != ESP_OK ||
        xTaskCreate(transmit ? hil_i2s_tx_task : hil_i2s_rx_task,
                    transmit ? "hil-i2s-tx" : "hil-i2s-rx", 4096, NULL, 7,
                    &s_i2s_peer_task) != pdPASS) {
        hil_i2s_peer_stop();
        return err != ESP_OK ? err : ESP_ERR_NO_MEM;
    }
    for (unsigned wait = 0; wait < 100 && !s_i2s_ready; ++wait) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (!s_i2s_ready || s_i2s_error != ESP_OK) {
        err = s_i2s_ready ? s_i2s_error : ESP_ERR_TIMEOUT;
        hil_i2s_peer_stop();
        return err;
    }
    return ESP_OK;
}

static void hil_lines_high_z(void)
{
    hil_peer_stop();
    uint64_t mask = 0;
    for (size_t i = 0; i < sizeof(s_hil_pins) / sizeof(s_hil_pins[0]); ++i) {
        mask |= 1ULL << s_hil_pins[i];
    }
    gpio_config_t config = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    s_armed = false;
    s_arm_deadline_us = 0;
}

static void hil_uart_peer_task(void *argument)
{
    (void)argument;
    uint8_t buffer[256];

    while (true) {
        int received = uart_read_bytes(UART_NUM_1, buffer, sizeof(buffer),
                                       pdMS_TO_TICKS(100));
        if (received <= 0) {
            continue;
        }
        s_uart_bytes += (uint32_t)received;
        s_uart_crc = hil_crc32_update(s_uart_crc, buffer, received);
        int sent = 0;
        while (sent < received) {
            int done = uart_write_bytes(UART_NUM_1, buffer + sent,
                                        received - sent);
            if (done <= 0) {
                break;
            }
            sent += done;
        }
    }
}

static esp_err_t hil_uart_peer_start(uint32_t baud)
{
    hil_peer_stop();
    uart_config_t config = {
        .baud_rate = (int)baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(UART_NUM_1, 4096, 4096, 0, NULL, 0);
    if (err == ESP_OK) {
        err = uart_param_config(UART_NUM_1, &config);
    }
    if (err == ESP_OK) {
        err = uart_set_pin(UART_NUM_1, s_hil_lane_pins[1],
                           s_hil_lane_pins[0], UART_PIN_NO_CHANGE,
                           UART_PIN_NO_CHANGE);
    }
    if (err != ESP_OK) {
        hil_peer_stop();
        return err;
    }
    s_uart_bytes = 0;
    s_uart_crc = ~0U;
    if (xTaskCreate(hil_uart_peer_task, "hil-uart-peer", 3072, NULL, 7,
                    &s_uart_peer_task) != pdPASS) {
        hil_peer_stop();
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static void hil_spi_peer_task(void *argument)
{
    (void)argument;
    while (true) {
        spi_slave_transaction_t *transaction = NULL;

        if (spi_slave_get_trans_result(HIL_SPI_HOST, &transaction,
                                       portMAX_DELAY) == ESP_OK &&
            transaction != NULL) {
            size_t bytes = transaction->trans_len / 8;

            if (bytes > s_spi_length) {
                bytes = s_spi_length;
            }
            if (s_spi_completions == 0) {
                s_spi_first_bits = transaction->trans_len;
                memcpy(s_spi_first_rx, s_spi_rx,
                       bytes < sizeof(s_spi_first_rx) ? bytes :
                       sizeof(s_spi_first_rx));
            }
            s_spi_last_bits = transaction->trans_len;
            ++s_spi_completions;
            s_spi_bytes += bytes;
            s_spi_crc = hil_crc32_update(s_spi_crc, s_spi_rx, bytes);
            memset(s_spi_rx, 0, s_spi_length);
            s_spi_transaction.trans_len = 0;
            if (spi_slave_queue_trans(HIL_SPI_HOST, &s_spi_transaction,
                                      portMAX_DELAY) != ESP_OK) {
                break;
            }
        }
    }
    s_spi_peer_task = NULL;
    vTaskDelete(NULL);
}

static void hil_spi_master_task(void *argument)
{
    (void)argument;
    /* Receiver-first: give Linux time to enter its target ioctl. */
    vTaskDelay(pdMS_TO_TICKS(1500));
    spi_transaction_t transfer = {
        .length = s_spi_length * 8,
        .tx_buffer = s_spi_tx,
        .rx_buffer = s_spi_rx,
    };
    esp_err_t err = spi_device_transmit(s_spi_master_device, &transfer);
    unsigned mismatches = 0;
    if (err == ESP_OK) {
        for (size_t i = 0; i < s_spi_length; i++)
            mismatches += s_spi_rx[i] != (uint8_t)(i * 37U + 11U);
    }
    char detail[128];
    snprintf(detail, sizeof(detail), "bytes=%u mismatches=%u rx_crc32=%08" PRIx32 " result=%s",
             (unsigned)s_spi_length, mismatches,
             ~hil_crc32_update(~0U, s_spi_rx, s_spi_length), esp_err_to_name(err));
    hil_emit(err == ESP_OK && !mismatches ? "PASS" : "FAIL",
             "peer.spi-master-data", "data", detail);
    s_spi_peer_task = NULL;
    vTaskDelete(NULL);
}

static esp_err_t hil_spi_master_start(unsigned mode, size_t length, unsigned speed)
{
    hil_peer_stop();
    s_spi_length = length;
    s_spi_tx = heap_caps_malloc(length, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    s_spi_rx = heap_caps_calloc(1, length, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_spi_tx || !s_spi_rx) {
        hil_peer_stop();
        return ESP_ERR_NO_MEM;
    }
    for (size_t i = 0; i < length; i++)
        s_spi_tx[i] = (uint8_t)(i ^ 0xa5U);
    spi_bus_config_t bus = {
        .mosi_io_num = s_hil_lane_pins[1], .miso_io_num = s_hil_lane_pins[3],
        .sclk_io_num = s_hil_lane_pins[0], .quadwp_io_num = -1,
        .quadhd_io_num = -1, .max_transfer_sz = length,
    };
    spi_device_interface_config_t device = {
        .clock_speed_hz = speed, .mode = mode,
        .spics_io_num = s_hil_lane_pins[2], .queue_size = 1,
    };
    esp_err_t err = spi_bus_initialize(HIL_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err == ESP_OK) {
        s_spi_master_active = true;
        err = spi_bus_add_device(HIL_SPI_HOST, &device, &s_spi_master_device);
    }
    if (err == ESP_OK && xTaskCreate(hil_spi_master_task, "hil-spi-master", 4096,
                                   NULL, 7, &s_spi_peer_task) != pdPASS)
        err = ESP_ERR_NO_MEM;
    if (err != ESP_OK)
        hil_peer_stop();
    return err;
}

static esp_err_t hil_spi_peer_start(unsigned mode, size_t length,
                                    uint32_t speed_hz, int rsck_override,
                                    int tsck_override, int clk13_override)
{
    hil_peer_stop();
    s_spi_length = length;
    s_spi_tx = heap_caps_malloc(length, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    s_spi_rx = heap_caps_malloc(length, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (s_spi_tx == NULL || s_spi_rx == NULL) {
        hil_peer_stop();
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = ESP_OK;
    spi_bus_config_t bus = {
        .mosi_io_num = s_hil_lane_pins[1],
        .miso_io_num = s_hil_lane_pins[3],
        .sclk_io_num = s_hil_lane_pins[0],
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = length,
    };
    spi_slave_interface_config_t slave = {
        .spics_io_num = s_hil_lane_pins[2],
        .mode = mode,
        .queue_size = 1,
    };
    /*
     * P4's non-DMA slave path is limited to the 64-byte hardware FIFO.
     * The HIL matrix deliberately includes 257, 1024, and 4096-byte
     * transactions, so let the driver allocate a GDMA channel.  The buffers
     * carry MALLOC_CAP_DMA and ALIGN_AUTO also handles non-word test lengths.
     */
    err = spi_slave_initialize(HIL_SPI_HOST, &bus, &slave, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        hil_peer_stop();
        return err;
    }
    gpio_config_t sclk_pad = {
        .pin_bit_mask = BIT64(s_hil_lane_pins[0]),
        .mode = GPIO_MODE_INPUT,
        .hys_ctrl_mode = GPIO_HYS_SOFT_ENABLE,
    };
    err = gpio_config(&sclk_pad);
    if (err != ESP_OK) {
        hil_peer_stop();
        return err;
    }
    /* ESP32-P4 rev 1.3 needs a separate transmit-edge override for stable
     * 20 MHz full-duplex slave DMA in modes 1/2.  Optional explicit values are
     * accepted by the diagnostic RPC so all four internal edge combinations
     * can be characterized without rebuilding the tester. */
    spi_dev_t *hw = SPI_LL_GET_HW(HIL_SPI_HOST);

    if (rsck_override >= 0) {
        hw->user.rsck_i_edge = rsck_override;
    }
    if (tsck_override >= 0) {
        hw->user.tsck_i_edge = tsck_override;
    } else if (mode == 1 || mode == 2) {
        hw->user.tsck_i_edge = 0;
    }
    if (clk13_override >= 0) {
        hw->slave.clk_mode_13 = clk13_override;
    }
    s_spi_active = true;
    gpio_set_pull_mode(s_hil_lane_pins[2], GPIO_PULLUP_ONLY);
    for (size_t i = 0; i < length; ++i) {
        s_spi_tx[i] = (uint8_t)(i ^ 0xa5U);
        s_spi_rx[i] = 0;
    }
    memset(&s_spi_transaction, 0, sizeof(s_spi_transaction));
    s_spi_transaction.length = length * 8;
    s_spi_transaction.tx_buffer = s_spi_tx;
    s_spi_transaction.rx_buffer = s_spi_rx;
    s_spi_transaction.flags = SPI_SLAVE_TRANS_DMA_BUFFER_ALIGN_AUTO;
    err = spi_slave_queue_trans(HIL_SPI_HOST, &s_spi_transaction,
                                portMAX_DELAY);
    if (err != ESP_OK) {
        hil_peer_stop();
        return err;
    }
    s_spi_bytes = 0;
    s_spi_crc = ~0U;
    s_spi_completions = 0;
    s_spi_first_bits = 0;
    s_spi_last_bits = 0;
    memset(s_spi_first_rx, 0, sizeof(s_spi_first_rx));
    if (xTaskCreate(hil_spi_peer_task, "hil-spi-peer", 4096, NULL, 7,
                    &s_spi_peer_task) != pdPASS) {
        hil_peer_stop();
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static bool hil_i2c_receive_cb(i2c_slave_dev_handle_t handle,
                               const i2c_slave_rx_done_event_data_t *event,
                               void *argument)
{
    (void)handle;
    (void)argument;
    BaseType_t woken = pdFALSE;
    enum hil_i2c_event kind = HIL_I2C_RECEIVE;
    size_t length = event->length < 4097 ? event->length : 4097;

    if (length != 0) {
        memcpy(s_i2c_rx, event->buffer, length);
        s_i2c_register = s_i2c_rx[0];
    }
    s_i2c_rx_length = length;
    xQueueSendFromISR(s_i2c_events, &kind, &woken);
    return woken == pdTRUE;
}

static bool hil_i2c_request_cb(i2c_slave_dev_handle_t handle,
                               const i2c_slave_request_event_data_t *event,
                               void *argument)
{
    (void)handle;
    (void)event;
    (void)argument;
    BaseType_t woken = pdFALSE;
    enum hil_i2c_event kind = HIL_I2C_REQUEST;

    xQueueSendFromISR(s_i2c_events, &kind, &woken);
    return woken == pdTRUE;
}

static void hil_i2c_peer_task(void *argument)
{
    (void)argument;
    enum hil_i2c_event event;
    uint8_t *response = s_i2c_response;
    size_t response_length = s_i2c_response_length;

    while (true) {
        if (xQueueReceive(s_i2c_events, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (event == HIL_I2C_RECEIVE) {
            size_t length = s_i2c_rx_length;
            uint8_t reg = s_i2c_register;
            if (length > 1) {
                for (size_t i = 1; i < length; ++i) {
                    s_i2c_registers[(uint8_t)(reg + i - 1)] = s_i2c_rx[i];
                }
                ++s_i2c_writes;
                for (size_t i = 0; i < response_length; ++i) {
                    response[i] = s_i2c_registers[(uint8_t)(reg + i)];
                }
                uint32_t written = 0;
                i2c_slave_reset_tx_fifo(s_i2c_slave);
                i2c_slave_write(s_i2c_slave, response, response_length,
                                &written, 1000);
            }
        } else {
            uint8_t reg = s_i2c_register;
            for (size_t i = 0; i < response_length; ++i) {
                response[i] = s_i2c_registers[(uint8_t)(reg + i)];
            }
            uint32_t written = 0;
            i2c_slave_write(s_i2c_slave, response, response_length,
                            &written, 1000);
            ++s_i2c_reads;
        }
    }
}

static esp_err_t hil_i2c_peer_start(size_t response_length, unsigned scl_lane)
{
    hil_peer_stop();
    s_i2c_rx = malloc(4097);
    s_i2c_response = malloc(response_length);
    if (!s_i2c_rx || !s_i2c_response) {
        hil_peer_stop();
        return ESP_ERR_NO_MEM;
    }
    s_i2c_response_length = response_length;
    for (size_t i = 0; i < sizeof(s_i2c_registers); ++i) {
        s_i2c_registers[i] = (uint8_t)(i ^ 0x5aU);
    }
    s_i2c_register = 0;
    s_i2c_rx_length = 0;
    s_i2c_reads = 0;
    s_i2c_writes = 0;
    s_i2c_events = xQueueCreate(16, sizeof(enum hil_i2c_event));
    if (s_i2c_events == NULL) {
        hil_peer_stop();
        return ESP_ERR_NO_MEM;
    }
    i2c_slave_config_t config = {
        .i2c_port = -1,
        .sda_io_num = s_hil_lane_pins[1],
        .scl_io_num = s_hil_lane_pins[scl_lane],
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .send_buf_depth = response_length + 128,
        .receive_buf_depth = response_length + 128,
        .slave_addr = 0x42,
        .addr_bit_len = I2C_ADDR_BIT_LEN_7,
        .flags.enable_internal_pullup = 1,
    };
    esp_err_t err = i2c_new_slave_device(&config, &s_i2c_slave);
    if (err == ESP_OK) {
        i2c_slave_event_callbacks_t callbacks = {
            .on_request = hil_i2c_request_cb,
            .on_receive = hil_i2c_receive_cb,
        };
        err = i2c_slave_register_event_callbacks(s_i2c_slave, &callbacks, NULL);
    }
    if (err == ESP_OK &&
        xTaskCreate(hil_i2c_peer_task, "hil-i2c-peer", 4096, NULL, 15,
                    &s_i2c_peer_task) != pdPASS) {
        err = ESP_ERR_NO_MEM;
    }
    if (err == ESP_OK) {
        uint8_t *initial = s_i2c_response;
        uint32_t written = 0;
        for (size_t i = 0; i < response_length; ++i) {
            initial[i] = s_i2c_registers[(uint8_t)i];
        }
        err = i2c_slave_write(s_i2c_slave, initial, response_length,
                              &written, 1000);
        if (err == ESP_OK && written != response_length) {
            err = ESP_ERR_INVALID_SIZE;
        }
    }
    if (err != ESP_OK) {
        hil_peer_stop();
    }
    return err;
}

static void hil_pulse_isr(void *argument)
{
    gpio_num_t pin = (gpio_num_t)(uintptr_t)argument;
    int level = gpio_get_level(pin);
    int64_t now = esp_timer_get_time();
    int64_t elapsed = s_pulse_last_edge ? now - s_pulse_last_edge : 0;

    if (level) {
        if (elapsed > 0 && !s_pulse_level) {
            s_pulse_low_us += elapsed;
        }
        if (s_pulse_rises == 0) {
            s_pulse_first_rise = now;
        }
        s_pulse_last_rise = now;
        ++s_pulse_rises;
    } else {
        if (elapsed > 0 && s_pulse_level) {
            s_pulse_high_us += elapsed;
        }
        ++s_pulse_falls;
    }
    s_pulse_level = level;
    s_pulse_last_edge = now;
}

static esp_err_t hil_pulse_monitor_start(void)
{
    if (s_pulse_monitor_active) {
        gpio_isr_handler_remove(s_hil_lane_pins[0]);
        s_pulse_monitor_active = false;
    }
    s_pulse_rises = 0;
    s_pulse_falls = 0;
    s_pulse_first_rise = 0;
    s_pulse_last_rise = 0;
    s_pulse_last_edge = 0;
    s_pulse_high_us = 0;
    s_pulse_low_us = 0;
    s_pulse_level = gpio_get_level(s_hil_lane_pins[0]);
    gpio_config_t config = {
        .pin_bit_mask = 1ULL << s_hil_lane_pins[0],
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    esp_err_t err = gpio_config(&config);
    if (err == ESP_OK) {
        err = gpio_install_isr_service(0);
        if (err == ESP_ERR_INVALID_STATE) {
            err = ESP_OK;
        }
    }
    if (err == ESP_OK) {
        err = gpio_isr_handler_add(s_hil_lane_pins[0], hil_pulse_isr,
                                   (void *)(uintptr_t)s_hil_lane_pins[0]);
    }
    s_pulse_monitor_active = err == ESP_OK;
    return err;
}

static void hil_pulse_monitor_stop(void)
{
    if (s_pulse_monitor_active) {
        gpio_isr_handler_remove(s_hil_lane_pins[0]);
        s_pulse_monitor_active = false;
    }
    gpio_set_direction(s_hil_lane_pins[0], GPIO_MODE_INPUT);
    gpio_set_pull_mode(s_hil_lane_pins[0], GPIO_FLOATING);
}

static void hil_arm_watchdog(void *argument)
{
    (void)argument;
    while (true) {
        if (s_armed && esp_timer_get_time() >= s_arm_deadline_us) {
            hil_lines_high_z();
            hil_emit("PASS", "safety.auto-disarm", "safety",
                     "arm window expired; all tester pins are high-Z");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void hil_wifi_event(void *argument, esp_event_base_t base,
                           int32_t event_id, void *event_data)
{
    (void)argument;
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_ble_events, WIFI_IP_BIT);
        xEventGroupSetBits(s_ble_events, WIFI_DISCONNECTED_BIT);
    } else if (base == WIFI_EVENT &&
               event_id == WIFI_EVENT_AP_STACONNECTED) {
        s_wifi_ap_connects++;
    } else if (base == WIFI_EVENT &&
               event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        const wifi_event_ap_stadisconnected_t *event = event_data;
        s_wifi_ap_disconnects++;
        s_wifi_ap_last_disconnect_reason = event->reason;
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = event_data;
        s_wifi_ip = event->ip_info;
        xEventGroupSetBits(s_ble_events, WIFI_IP_BIT);
    }
}

static esp_err_t hil_wifi_init(void)
{
    if (s_wifi_ready) {
        return ESP_OK;
    }
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    if (esp_netif_get_handle_from_ifkey("WIFI_STA_DEF") == NULL) {
        esp_netif_create_default_wifi_sta();
    }
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&config);
    if (err != ESP_OK) {
        return err;
    }
    if ((err = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK ||
        (err = esp_wifi_start()) != ESP_OK) {
        return err;
    }
    if ((err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                          hil_wifi_event, NULL)) != ESP_OK ||
        (err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                          hil_wifi_event, NULL)) != ESP_OK) {
        return err;
    }
    s_wifi_ready = true;
    return ESP_OK;
}

static void hil_wifi_ap_udp_echo_task(void *argument)
{
    (void)argument;
    uint8_t buffer[1536];
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        s_wifi_ap_echo_errors++;
        s_wifi_ap_echo_task = NULL;
        vTaskDelete(NULL);
        return;
    }
    struct timeval timeout = { .tv_sec = 0, .tv_usec = 200000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(3334),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&address, sizeof(address)) != 0) {
        s_wifi_ap_echo_errors++;
        close(sock);
        s_wifi_ap_echo_task = NULL;
        vTaskDelete(NULL);
        return;
    }
    while (s_wifi_ap_active) {
        struct sockaddr_storage source;
        socklen_t source_length = sizeof(source);
        int received = recvfrom(sock, buffer, sizeof(buffer), 0,
                                (struct sockaddr *)&source, &source_length);
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                s_wifi_ap_echo_errors++;
            }
            continue;
        }
        if (received == 0) {
            continue;
        }
        uint32_t packet = s_wifi_ap_packets;
        for (int i = 0; i < received; ++i) {
            uint8_t expected = (uint8_t)(i * 37U + packet * 13U + 11U);
            if (buffer[i] != expected) {
                s_wifi_ap_pattern_errors++;
                break;
            }
        }
        s_wifi_ap_packets++;
        s_wifi_ap_bytes += received;
        if (sendto(sock, buffer, received, 0,
                   (struct sockaddr *)&source, source_length) != received) {
            s_wifi_ap_echo_errors++;
        }
    }
    close(sock);
    s_wifi_ap_echo_task = NULL;
    vTaskDelete(NULL);
}

static esp_err_t hil_wifi_ap_start(const char *ssid, const char *password,
                                   unsigned channel)
{
    size_t ssid_length = strlen(ssid);
    size_t password_length = strlen(password);
    if (ssid_length == 0 || ssid_length >= 32 ||
        (password_length != 0 && password_length < 8) ||
        password_length >= 64 || channel < 1 || channel > 13) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = hil_wifi_init();
    if (err != ESP_OK) {
        return err;
    }
    esp_wifi_disconnect();
    err = esp_wifi_stop();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    if (s_wifi_ap_netif == NULL) {
        s_wifi_ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
        if (s_wifi_ap_netif == NULL) {
            s_wifi_ap_netif = esp_netif_create_default_wifi_ap();
        }
        if (s_wifi_ap_netif == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    wifi_config_t config = {0};
    memcpy(config.ap.ssid, ssid, ssid_length);
    memcpy(config.ap.password, password, password_length);
    config.ap.ssid_len = ssid_length;
    config.ap.channel = channel;
    config.ap.authmode = password_length == 0 ? WIFI_AUTH_OPEN :
                                                 WIFI_AUTH_WPA2_PSK;
    config.ap.max_connection = 4;
    config.ap.pmf_cfg.capable = true;
    config.ap.pmf_cfg.required = false;
    if ((err = esp_wifi_set_mode(WIFI_MODE_AP)) != ESP_OK ||
        (err = esp_wifi_set_config(WIFI_IF_AP, &config)) != ESP_OK ||
        (err = esp_wifi_start()) != ESP_OK) {
        return err;
    }
    /* esp_wifi_stop()/start() reuses the default AP netif across HIL runs.
     * Reset the DHCP server explicitly so a stale stopped state or lease
     * cannot turn a valid second association into a no-address failure. */
    vTaskDelay(pdMS_TO_TICKS(100));
    err = esp_netif_dhcps_stop(s_wifi_ap_netif);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        esp_wifi_stop();
        return err;
    }
    err = esp_netif_dhcps_start(s_wifi_ap_netif);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
        esp_wifi_stop();
        return err;
    }
    s_wifi_ap_packets = 0;
    s_wifi_ap_bytes = 0;
    s_wifi_ap_pattern_errors = 0;
    s_wifi_ap_echo_errors = 0;
    s_wifi_ap_connects = 0;
    s_wifi_ap_disconnects = 0;
    s_wifi_ap_last_disconnect_reason = 0;
    s_wifi_ap_active = true;
    if (xTaskCreate(hil_wifi_ap_udp_echo_task, "hil-wifi-echo", 4096, NULL, 5,
                    &s_wifi_ap_echo_task) != pdPASS) {
        s_wifi_ap_active = false;
        esp_wifi_stop();
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static esp_err_t hil_wifi_ap_stop(void)
{
    s_wifi_ap_active = false;
    for (unsigned i = 0; i < 15 && s_wifi_ap_echo_task != NULL; ++i) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (s_wifi_ap_echo_task != NULL) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t dhcp_err = esp_netif_dhcps_stop(s_wifi_ap_netif);
    if (dhcp_err != ESP_OK &&
        dhcp_err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        return dhcp_err;
    }
    esp_err_t err = esp_wifi_stop();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    if ((err = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK ||
        (err = esp_wifi_start()) != ESP_OK) {
        return err;
    }
    return ESP_OK;
}

static esp_err_t hil_wifi_connect_once(const char *ssid, const char *password)
{
    wifi_config_t config = {0};
    size_t ssid_length = strlen(ssid);
    size_t password_length = strlen(password);
    if (ssid_length == 0 || ssid_length >= sizeof(config.sta.ssid) ||
        password_length >= sizeof(config.sta.password)) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(config.sta.ssid, ssid, ssid_length + 1);
    memcpy(config.sta.password, password, password_length + 1);
    config.sta.threshold.authmode = password_length == 0 ?
                                    WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    config.sta.pmf_cfg.capable = true;
    config.sta.pmf_cfg.required = false;
    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &config);
    if (err != ESP_OK) {
        return err;
    }
    xEventGroupClearBits(s_ble_events, WIFI_IP_BIT | WIFI_DISCONNECTED_BIT);
    err = esp_wifi_connect();
    if (err != ESP_OK) {
        return err;
    }
    EventBits_t bits = xEventGroupWaitBits(s_ble_events, WIFI_IP_BIT,
                                           pdFALSE, pdTRUE,
                                           pdMS_TO_TICKS(20000));
    return (bits & WIFI_IP_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

static esp_err_t hil_wifi_station_echo_stop(void)
{
    s_wifi_ap_active = false;
    for (unsigned i = 0; i < 15 && s_wifi_ap_echo_task; i++)
        vTaskDelay(pdMS_TO_TICKS(100));
    if (s_wifi_ap_echo_task)
        return ESP_ERR_TIMEOUT;
    esp_wifi_disconnect();
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif)
        esp_netif_dhcpc_start(netif);
    return ESP_OK;
}

static esp_err_t hil_wifi_station_echo_start(const char *ssid, const char *password,
                                            bool keep_ap)
{
    esp_err_t err = hil_wifi_init();
    if (err != ESP_OK || (s_wifi_ap_echo_task && !keep_ap) ||
        (keep_ap && (!s_wifi_ap_active || !s_wifi_ap_echo_task)))
        return err != ESP_OK ? err : ESP_ERR_INVALID_STATE;
    esp_wifi_disconnect();
    if (!keep_ap)
        esp_wifi_stop();
    if ((err = esp_wifi_set_mode(keep_ap ? WIFI_MODE_APSTA : WIFI_MODE_STA)) != ESP_OK)
        return err;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif)
        return ESP_ERR_INVALID_STATE;
    esp_netif_dhcpc_stop(netif);
    esp_netif_ip_info_t ip = {0};
    ip.ip.addr = inet_addr("192.168.77.1");
    ip.gw.addr = inet_addr("192.168.77.2");
    ip.netmask.addr = inet_addr("255.255.255.0");
    if ((err = esp_netif_set_ip_info(netif, &ip)) != ESP_OK ||
        (!keep_ap && (err = esp_wifi_start()) != ESP_OK) ||
        (err = hil_wifi_connect_once(ssid, password)) != ESP_OK) {
        if (!keep_ap)
            hil_wifi_station_echo_stop();
        return err;
    }
    if (keep_ap)
        return ESP_OK;
    s_wifi_ap_packets = s_wifi_ap_bytes = s_wifi_ap_pattern_errors = s_wifi_ap_echo_errors = 0;
    s_wifi_ap_active = true;
    if (xTaskCreate(hil_wifi_ap_udp_echo_task, "hil-sta-echo", 4096, NULL, 5,
                   &s_wifi_ap_echo_task) != pdPASS) {
        hil_wifi_station_echo_stop();
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static esp_err_t hil_wifi_dns_probe(const char *hostname)
{
    struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *result = NULL;
    int rc = getaddrinfo(hostname, NULL, &hints, &result);
    if (result != NULL) {
        freeaddrinfo(result);
    }
    return rc == 0 ? ESP_OK : ESP_FAIL;
}

static const char *hil_wifi_auth_name(wifi_auth_mode_t mode)
{
    switch (mode) {
    case WIFI_AUTH_OPEN: return "open";
    case WIFI_AUTH_WEP: return "wep";
    case WIFI_AUTH_WPA_PSK: return "wpa-psk";
    case WIFI_AUTH_WPA2_PSK: return "wpa2-psk";
    case WIFI_AUTH_WPA_WPA2_PSK: return "wpa-wpa2-psk";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "wpa2-enterprise";
    case WIFI_AUTH_WPA3_PSK: return "wpa3-psk";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "wpa2-wpa3-psk";
    default: return "other";
    }
}

static esp_err_t hil_wifi_find_ap(const char *ssid, wifi_ap_record_t *target)
{
    wifi_scan_config_t scan = {
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    };
    esp_err_t err = esp_wifi_scan_start(&scan, true);
    if (err != ESP_OK) {
        return err;
    }
    uint16_t count = 0;
    if ((err = esp_wifi_scan_get_ap_num(&count)) != ESP_OK) {
        return err;
    }
    wifi_ap_record_t *records = calloc(count, sizeof(*records));
    if (count != 0 && records == NULL) {
        return ESP_ERR_NO_MEM;
    }
    uint16_t returned = count;
    err = count == 0 ? ESP_OK : esp_wifi_scan_get_ap_records(&returned, records);
    if (err == ESP_OK) {
        err = ESP_ERR_NOT_FOUND;
        for (uint16_t i = 0; i < returned; ++i) {
            if (strcmp((const char *)records[i].ssid, ssid) == 0) {
                *target = records[i];
                err = ESP_OK;
                break;
            }
        }
    }
    free(records);
    return err;
}

static esp_err_t hil_wifi_connection_test(const char *ssid, const char *password,
                                          const char *hostname, char *detail,
                                          size_t detail_size)
{
    esp_err_t err = hil_wifi_init();
    if (err != ESP_OK) {
        return err;
    }
    wifi_ap_record_t target = {0};
    err = hil_wifi_find_ap(ssid, &target);
    if (err != ESP_OK) {
        snprintf(detail, detail_size, "ssid=%s not found", ssid);
        return err;
    }
    if (target.authmode != WIFI_AUTH_OPEN && password[0] == '\0') {
        snprintf(detail, detail_size,
                 "ssid=%s rssi=%d channel=%u auth=%s credential=missing",
                 ssid, target.rssi, target.primary,
                 hil_wifi_auth_name(target.authmode));
        return ESP_ERR_INVALID_ARG;
    }
    esp_wifi_disconnect();
    xEventGroupWaitBits(s_ble_events, WIFI_DISCONNECTED_BIT, pdTRUE, pdTRUE,
                        pdMS_TO_TICKS(2000));
    if ((err = hil_wifi_connect_once(ssid, password)) != ESP_OK ||
        (err = hil_wifi_dns_probe(hostname)) != ESP_OK) {
        esp_wifi_disconnect();
        return err;
    }
    esp_netif_ip_info_t first_ip = s_wifi_ip;
    xEventGroupClearBits(s_ble_events, WIFI_DISCONNECTED_BIT);
    if ((err = esp_wifi_disconnect()) != ESP_OK) {
        return err;
    }
    EventBits_t bits = xEventGroupWaitBits(s_ble_events, WIFI_DISCONNECTED_BIT,
                                           pdTRUE, pdTRUE,
                                           pdMS_TO_TICKS(5000));
    if (!(bits & WIFI_DISCONNECTED_BIT) ||
        (err = hil_wifi_connect_once(ssid, password)) != ESP_OK ||
        (err = hil_wifi_dns_probe(hostname)) != ESP_OK) {
        esp_wifi_disconnect();
        return err == ESP_OK ? ESP_ERR_TIMEOUT : err;
    }
    char ip[16], gateway[16];
    esp_ip4addr_ntoa(&s_wifi_ip.ip, ip, sizeof(ip));
    esp_ip4addr_ntoa(&s_wifi_ip.gw, gateway, sizeof(gateway));
    char first_ip_text[16];
    esp_ip4addr_ntoa(&first_ip.ip, first_ip_text, sizeof(first_ip_text));
    snprintf(detail, detail_size,
             "ssid=%s auth=%s rssi=%d channel=%u first_ip=%s ip=%s gateway=%s dns=%s reconnect=pass",
             ssid, hil_wifi_auth_name(target.authmode), target.rssi,
             target.primary, first_ip_text, ip, gateway, hostname);
    esp_wifi_disconnect();
    return ESP_OK;
}

static esp_err_t hil_network_stack_init(void)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    return ESP_OK;
}

static void hil_eth_event(void *argument, esp_event_base_t base,
                          int32_t event_id, void *event_data)
{
    (void)argument;
    (void)base;
    (void)event_data;
    if (event_id == ETHERNET_EVENT_CONNECTED) {
        xEventGroupSetBits(s_ble_events, ETH_LINK_BIT);
    } else if (event_id == ETHERNET_EVENT_DISCONNECTED ||
               event_id == ETHERNET_EVENT_STOP) {
        xEventGroupClearBits(s_ble_events, ETH_LINK_BIT);
    }
}

static void hil_eth_udp_echo_task(void *argument)
{
    (void)argument;
    uint8_t buffer[1536];
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        s_eth_echo_task = NULL;
        vTaskDelete(NULL);
        return;
    }
    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(3333),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&address, sizeof(address)) != 0) {
        close(sock);
        s_eth_echo_task = NULL;
        vTaskDelete(NULL);
        return;
    }
    while (true) {
        struct sockaddr_storage source;
        socklen_t source_length = sizeof(source);
        int received = recvfrom(sock, buffer, sizeof(buffer), 0,
                                (struct sockaddr *)&source, &source_length);
        if (received > 0) {
            sendto(sock, buffer, received, 0,
                   (struct sockaddr *)&source, source_length);
        }
    }
}

static esp_err_t hil_eth_start(void)
{
    esp_err_t err;

    if (s_eth_handle != NULL) {
        if (s_eth_phy != NULL && s_eth_phy->pwrctl != NULL &&
            (err = s_eth_phy->pwrctl(s_eth_phy, true)) != ESP_OK) {
            return err;
        }
        err = esp_eth_start(s_eth_handle);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            return err;
        }
    } else {
        err = hil_network_stack_init();
        if (err != ESP_OK) {
            return err;
        }

        eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
        eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
        eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
        phy_config.phy_addr = 1;
        phy_config.reset_gpio_num = 51;
        s_eth_mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
        s_eth_phy = esp_eth_phy_new_generic(&phy_config);
        if (s_eth_mac == NULL || s_eth_phy == NULL) {
            return ESP_ERR_NO_MEM;
        }
        esp_eth_config_t config = ETH_DEFAULT_CONFIG(s_eth_mac, s_eth_phy);
        err = esp_eth_driver_install(&config, &s_eth_handle);
        if (err != ESP_OK) {
            return err;
        }

        esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
        s_eth_netif = esp_netif_new(&netif_config);
        s_eth_glue = esp_eth_new_netif_glue(s_eth_handle);
        if (s_eth_netif == NULL || s_eth_glue == NULL) {
            return ESP_ERR_NO_MEM;
        }
        if ((err = esp_netif_attach(s_eth_netif, s_eth_glue)) != ESP_OK ||
            (err = esp_netif_dhcpc_stop(s_eth_netif)) != ESP_OK) {
            return err;
        }
        esp_netif_ip_info_t ip = {
            .ip.addr = ipaddr_addr("192.168.77.1"),
            .netmask.addr = ipaddr_addr("255.255.255.0"),
            .gw.addr = ipaddr_addr("0.0.0.0"),
        };
        if ((err = esp_netif_set_ip_info(s_eth_netif, &ip)) != ESP_OK ||
            (err = esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                              hil_eth_event, NULL)) != ESP_OK ||
            (err = esp_eth_start(s_eth_handle)) != ESP_OK) {
            return err;
        }
    }

    EventBits_t bits = xEventGroupWaitBits(s_ble_events, ETH_LINK_BIT,
                                           pdFALSE, pdTRUE,
                                           pdMS_TO_TICKS(10000));
    if ((bits & ETH_LINK_BIT) && s_eth_echo_task == NULL &&
        xTaskCreate(hil_eth_udp_echo_task, "hil-eth-echo", 4096, NULL, 5,
                    &s_eth_echo_task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return (bits & ETH_LINK_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

static esp_err_t hil_eth_stop(void)
{
    if (s_eth_handle == NULL) {
        return ESP_OK;
    }
    esp_err_t err = esp_eth_stop(s_eth_handle);
    if (err == ESP_ERR_INVALID_STATE) {
        err = ESP_OK;
    }
    if (err == ESP_OK && s_eth_phy != NULL && s_eth_phy->pwrctl != NULL) {
        err = s_eth_phy->pwrctl(s_eth_phy, false);
    }
    xEventGroupClearBits(s_ble_events, ETH_LINK_BIT);
    return err;
}

static esp_err_t hil_wifi_scan(uint16_t *ap_count, int8_t *strongest_rssi,
                               uint8_t *strongest_channel)
{
    esp_err_t err = hil_wifi_init();
    if (err != ESP_OK) {
        return err;
    }
    wifi_scan_config_t scan = {
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    };
    if ((err = esp_wifi_scan_start(&scan, true)) != ESP_OK) {
        return err;
    }
    uint16_t count = 0;
    if ((err = esp_wifi_scan_get_ap_num(&count)) != ESP_OK) {
        return err;
    }
    uint16_t returned = count > 32 ? 32 : count;
    wifi_ap_record_t records[32] = {0};
    if (returned > 0 &&
        (err = esp_wifi_scan_get_ap_records(&returned, records)) != ESP_OK) {
        return err;
    }
    int8_t rssi = -127;
    uint8_t channel = 0;
    for (uint16_t i = 0; i < returned; ++i) {
        if (records[i].rssi > rssi) {
            rssi = records[i].rssi;
            channel = records[i].primary;
        }
    }
    *ap_count = count;
    *strongest_rssi = rssi;
    *strongest_channel = channel;
    return ESP_OK;
}

static void hil_ble_sync(void)
{
    xEventGroupSetBits(s_ble_events, BLE_SYNC_BIT);
}

static void hil_ble_reset(int reason)
{
    (void)reason;
    xEventGroupClearBits(s_ble_events, BLE_SYNC_BIT);
}

static void hil_ble_host_task(void *argument)
{
    (void)argument;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static esp_err_t hil_ble_init_and_wait(void)
{
    if (!s_ble_started) {
        esp_err_t err = nimble_port_init();
        if (err != ESP_OK) {
            return err;
        }
        ble_hs_cfg.reset_cb = hil_ble_reset;
        ble_hs_cfg.sync_cb = hil_ble_sync;
        nimble_port_freertos_init(hil_ble_host_task);
        s_ble_started = true;
    }
    EventBits_t bits = xEventGroupWaitBits(s_ble_events, BLE_SYNC_BIT,
                                           pdFALSE, pdTRUE,
                                           pdMS_TO_TICKS(10000));
    return (bits & BLE_SYNC_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

static int hil_ble_gap_event(struct ble_gap_event *event, void *argument)
{
    (void)argument;
    if (event->type == BLE_GAP_EVENT_DISC) {
        struct ble_hs_adv_fields fields = {0};
        if (ble_hs_adv_parse_fields(&fields, event->disc.data,
                                    event->disc.length_data) == 0 &&
            fields.name_len == strlen("S31 Radio") &&
            memcmp(fields.name, "S31 Radio", fields.name_len) == 0) {
            s_ble_target = event->disc.addr;
            ble_gap_disc_cancel();
            xEventGroupSetBits(s_ble_events, BLE_FOUND_BIT);
        }
    } else if (event->type == BLE_GAP_EVENT_CONNECT) {
        s_ble_status = event->connect.status;
        if (event->connect.status == 0) {
            s_ble_conn_handle = event->connect.conn_handle;
            xEventGroupSetBits(s_ble_events, BLE_CONNECTED_BIT);
        } else {
            xEventGroupSetBits(s_ble_events, BLE_DISCONNECTED_BIT);
        }
    } else if (event->type == BLE_GAP_EVENT_DISCONNECT) {
        s_ble_status = event->disconnect.reason;
        xEventGroupSetBits(s_ble_events, BLE_DISCONNECTED_BIT);
    }
    return 0;
}

static int hil_ble_service_cb(uint16_t conn_handle,
                              const struct ble_gatt_error *error,
                              const struct ble_gatt_svc *service, void *argument)
{
    (void)conn_handle;
    (void)argument;
    if (error->status == 0 && service != NULL) {
        s_ble_service_start = service->start_handle;
        s_ble_service_end = service->end_handle;
    } else if (error->status == BLE_HS_EDONE && s_ble_service_start != 0) {
        s_ble_status = 0;
        xEventGroupSetBits(s_ble_events, BLE_SERVICE_BIT);
    } else if (error->status != 0) {
        s_ble_status = error->status;
        xEventGroupSetBits(s_ble_events, BLE_SERVICE_BIT);
    }
    return 0;
}

static int hil_ble_characteristic_cb(uint16_t conn_handle,
                                     const struct ble_gatt_error *error,
                                     const struct ble_gatt_chr *characteristic,
                                     void *argument)
{
    (void)conn_handle;
    (void)argument;
    if (error->status == 0 && characteristic != NULL) {
        s_ble_value_handle = characteristic->val_handle;
    } else if (error->status == BLE_HS_EDONE && s_ble_value_handle != 0) {
        s_ble_status = 0;
        xEventGroupSetBits(s_ble_events, BLE_CHARACTERISTIC_BIT);
    } else if (error->status != 0) {
        s_ble_status = error->status;
        xEventGroupSetBits(s_ble_events, BLE_CHARACTERISTIC_BIT);
    }
    return 0;
}

static int hil_ble_read_cb(uint16_t conn_handle,
                           const struct ble_gatt_error *error,
                           struct ble_gatt_attr *attribute, void *argument)
{
    (void)conn_handle;
    (void)argument;
    s_ble_status = error->status;
    memset(s_ble_value, 0, sizeof(s_ble_value));
    if (error->status == 0 && attribute != NULL && attribute->om != NULL) {
        uint16_t length = OS_MBUF_PKTLEN(attribute->om);
        if (length >= sizeof(s_ble_value)) {
            length = sizeof(s_ble_value) - 1;
        }
        os_mbuf_copydata(attribute->om, 0, length, s_ble_value);
    }
    xEventGroupSetBits(s_ble_events, BLE_READ_BIT);
    return 0;
}

static esp_err_t hil_ble_connect_and_read(const ble_addr_t *address)
{
    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        return ESP_FAIL;
    }
    xEventGroupClearBits(s_ble_events, BLE_CONNECTED_BIT |
                         BLE_DISCONNECTED_BIT | BLE_SERVICE_BIT |
                         BLE_CHARACTERISTIC_BIT | BLE_READ_BIT);
    s_ble_service_start = 0;
    s_ble_service_end = 0;
    s_ble_value_handle = 0;
    s_ble_status = 0;
    rc = ble_gap_connect(own_addr_type, address, 10000, NULL,
                         hil_ble_gap_event, NULL);
    if (rc != 0) {
        return ESP_FAIL;
    }
    EventBits_t bits = xEventGroupWaitBits(s_ble_events, BLE_CONNECTED_BIT |
                                           BLE_DISCONNECTED_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(12000));
    if (!(bits & BLE_CONNECTED_BIT) || s_ble_status != 0) {
        return ESP_ERR_TIMEOUT;
    }
    rc = ble_gattc_disc_svc_by_uuid(s_ble_conn_handle,
                                     BLE_UUID16_DECLARE(0xff10),
                                     hil_ble_service_cb, NULL);
    if (rc != 0 || !(xEventGroupWaitBits(s_ble_events, BLE_SERVICE_BIT,
                                         pdFALSE, pdTRUE,
                                         pdMS_TO_TICKS(5000)) &
                     BLE_SERVICE_BIT) || s_ble_status != 0) {
        return ESP_FAIL;
    }
    rc = ble_gattc_disc_chrs_by_uuid(s_ble_conn_handle,
                                      s_ble_service_start,
                                      s_ble_service_end,
                                      BLE_UUID16_DECLARE(0xff11),
                                      hil_ble_characteristic_cb, NULL);
    if (rc != 0 || !(xEventGroupWaitBits(s_ble_events,
                                         BLE_CHARACTERISTIC_BIT,
                                         pdFALSE, pdTRUE,
                                         pdMS_TO_TICKS(5000)) &
                     BLE_CHARACTERISTIC_BIT) || s_ble_status != 0) {
        return ESP_FAIL;
    }
    rc = ble_gattc_read(s_ble_conn_handle, s_ble_value_handle,
                        hil_ble_read_cb, NULL);
    if (rc != 0 || !(xEventGroupWaitBits(s_ble_events, BLE_READ_BIT,
                                         pdFALSE, pdTRUE,
                                         pdMS_TO_TICKS(5000)) &
                     BLE_READ_BIT) || s_ble_status != 0 ||
        strcmp(s_ble_value, "ready") != 0) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t hil_ble_gatt_test(char *detail, size_t detail_size, bool hold)
{
    esp_err_t err = hil_ble_init_and_wait();
    if (err != ESP_OK) {
        return err;
    }
    uint8_t own_addr_type;
    if (ble_hs_id_infer_auto(0, &own_addr_type) != 0) {
        return ESP_FAIL;
    }
    struct ble_gap_disc_params params = {
        .filter_duplicates = 1,
        .passive = 0,
    };
    xEventGroupClearBits(s_ble_events, BLE_FOUND_BIT);
    if (ble_gap_disc(own_addr_type, 10000, &params,
                     hil_ble_gap_event, NULL) != 0 ||
        !(xEventGroupWaitBits(s_ble_events, BLE_FOUND_BIT, pdFALSE, pdTRUE,
                              pdMS_TO_TICKS(12000)) & BLE_FOUND_BIT)) {
        return ESP_ERR_NOT_FOUND;
    }
    if ((err = hil_ble_connect_and_read(&s_ble_target)) != ESP_OK) {
        return err;
    }
    if (hold) {
        snprintf(detail, detail_size,
                 "peer=S31 Radio service=ff10 characteristic=ff11 value=%s connected=1", s_ble_value);
        return ESP_OK;
    }
    xEventGroupClearBits(s_ble_events, BLE_DISCONNECTED_BIT);
    if (ble_gap_terminate(s_ble_conn_handle, BLE_ERR_REM_USER_CONN_TERM) != 0 ||
        !(xEventGroupWaitBits(s_ble_events, BLE_DISCONNECTED_BIT,
                              pdTRUE, pdTRUE, pdMS_TO_TICKS(5000)) &
          BLE_DISCONNECTED_BIT)) {
        return ESP_FAIL;
    }
    if ((err = hil_ble_connect_and_read(&s_ble_target)) != ESP_OK) {
        return err;
    }
    snprintf(detail, detail_size,
             "peer=S31 Radio service=ff10 characteristic=ff11 value=%s reconnect=pass",
             s_ble_value);
    ble_gap_terminate(s_ble_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    xEventGroupWaitBits(s_ble_events, BLE_DISCONNECTED_BIT,
                        pdTRUE, pdTRUE, pdMS_TO_TICKS(5000));
    return ESP_OK;
}

static void hil_run_selftest(void)
{
    unsigned pass = 0;
    unsigned fail = 0;
    char detail[192];
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    if (chip.cores == 2) {
        snprintf(detail, sizeof(detail), "cores=%u revision=%u features=0x%08" PRIx32,
                 chip.cores, chip.revision, chip.features);
        hil_emit("PASS", "chip.identity", "firmware", detail);
        ++pass;
    } else {
        snprintf(detail, sizeof(detail), "expected 2 HP cores, got %u", chip.cores);
        hil_emit("FAIL", "chip.identity", "firmware", detail);
        ++fail;
    }

    uint32_t flash_size = 0;
    esp_err_t err = esp_flash_get_size(NULL, &flash_size);
    if (err == ESP_OK && flash_size >= 8 * 1024 * 1024) {
        snprintf(detail, sizeof(detail), "flash=%" PRIu32 " heap=%u",
                 flash_size, (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT));
        hil_emit("PASS", "memory.inventory", "firmware", detail);
        ++pass;
    } else {
        snprintf(detail, sizeof(detail), "flash query failed: %s",
                 esp_err_to_name(err));
        hil_emit("FAIL", "memory.inventory", "firmware", detail);
        ++fail;
    }

    int64_t start = esp_timer_get_time();
    /* Include the current partial tick: two 10 ms ticks may last 10–20 ms. */
    vTaskDelay(pdMS_TO_TICKS(20) + 1);
    int64_t elapsed = esp_timer_get_time() - start;
    if (elapsed >= 20000 && elapsed <= 100000) {
        snprintf(detail, sizeof(detail), "20ms delay measured %lldus",
                 (long long)elapsed);
        hil_emit("PASS", "timer.monotonic", "firmware", detail);
        ++pass;
    } else {
        snprintf(detail, sizeof(detail), "unexpected delay %lldus",
                 (long long)elapsed);
        hil_emit("FAIL", "timer.monotonic", "firmware", detail);
        ++fail;
    }

    hil_lines_high_z();
    hil_emit("PASS", "gpio.safe-state", "safety",
             "GPIO2,3,4,5,20-23,46-48 input/no-pull/high-Z");
    ++pass;

    err = hil_wifi_init();
    if (err == ESP_OK) {
        esp_hosted_coprocessor_fwver_t version = {0};
        err = esp_hosted_get_coprocessor_fwversion(&version);
        if (err == ESP_OK) {
            snprintf(detail, sizeof(detail), "C6 hosted firmware %" PRIu32 ".%" PRIu32 ".%" PRIu32,
                     version.major1, version.minor1, version.patch1);
            hil_emit("PASS", "c6.hosted-ready", "firmware", detail);
            ++pass;
        } else {
            snprintf(detail, sizeof(detail), "version RPC failed: %s",
                     esp_err_to_name(err));
            hil_emit("FAIL", "c6.hosted-ready", "firmware", detail);
            ++fail;
        }
    } else {
        snprintf(detail, sizeof(detail), "Wi-Fi init failed: %s",
                 esp_err_to_name(err));
        hil_emit("FAIL", "c6.hosted-ready", "firmware", detail);
        ++fail;
    }

    uint16_t ap_count = 0;
    int8_t strongest_rssi = -127;
    uint8_t strongest_channel = 0;
    err = hil_wifi_scan(&ap_count, &strongest_rssi, &strongest_channel);
    if (err == ESP_OK) {
        snprintf(detail, sizeof(detail), "aps=%u strongest_rssi=%d channel=%u",
                 ap_count, strongest_rssi, strongest_channel);
        hil_emit("PASS", "c6.wifi-scan", "firmware", detail);
        ++pass;
    } else {
        snprintf(detail, sizeof(detail), "scan failed: %s", esp_err_to_name(err));
        hil_emit("FAIL", "c6.wifi-scan", "firmware", detail);
        ++fail;
    }

    err = hil_ble_init_and_wait();
    if (err == ESP_OK) {
        hil_emit("PASS", "c6.ble-hci-reset", "firmware",
                 "NimBLE synchronized through ESP-Hosted VHCI");
        ++pass;
    } else {
        snprintf(detail, sizeof(detail), "BLE sync failed: %s", esp_err_to_name(err));
        hil_emit("FAIL", "c6.ble-hci-reset", "firmware", detail);
        ++fail;
    }

    hil_emit("SKIP", "peer.loopback", "electrical",
             "requires host-coordinated lane stimulus; standalone selftest stays high-Z");
    snprintf(detail, sizeof(detail), "pass=%u fail=%u skip=1", pass, fail);
    hil_emit(fail == 0 ? "PASS" : "FAIL", "summary", "summary", detail);
}

static int hil_parse_pin(const char *text)
{
    if (text == NULL || *text == '\0') {
        return -1;
    }
    char *end = NULL;
    errno = 0;
    long value = strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || value < 0 || value > 63) {
        return -1;
    }
    return (int)value;
}

static bool hil_require_arm(void)
{
    if (!s_armed || esp_timer_get_time() >= s_arm_deadline_us) {
        hil_lines_high_z();
        hil_emit("FAIL", "safety.arm", "safety",
                 "output command rejected; issue hil arm <token>");
        return false;
    }
    return true;
}

static void hil_command(char *line)
{
    char *save = NULL;
    char *prefix = strtok_r(line, " \t\r\n", &save);
    char *command = strtok_r(NULL, " \t\r\n", &save);
    if (prefix == NULL || strcmp(prefix, "hil") != 0 || command == NULL) {
        return;
    }
    if (strcmp(command, "hello") == 0 || strcmp(command, "status") == 0) {
        char detail[192];
        snprintf(detail, sizeof(detail),
                 "protocol=%u armed=%u token=%08" PRIx32
                 " pins=2,3,4,5,20,21,22,23,46,47,48"
                 " lanes=L0:23,L1:22,L2:21,L3:20",
                 HIL_PROTOCOL_VERSION, s_armed, s_arm_token);
        hil_emit("PASS", "rpc.hello", "firmware", detail);
    } else if (strcmp(command, "selftest") == 0) {
        hil_run_selftest();
    } else if (strcmp(command, "wifi-scan") == 0) {
        uint16_t count = 0;
        int8_t rssi = -127;
        uint8_t channel = 0;
        esp_err_t err = hil_wifi_scan(&count, &rssi, &channel);
        char detail[128];
        snprintf(detail, sizeof(detail), "result=%s aps=%u rssi=%d channel=%u",
                 esp_err_to_name(err), count, rssi, channel);
        hil_emit(err == ESP_OK ? "PASS" : "FAIL", "c6.wifi-scan",
                 "firmware", detail);
    } else if (strcmp(command, "wifi-test") == 0) {
        char *ssid = strtok_r(NULL, " \t\r\n", &save);
        char *password = strtok_r(NULL, " \t\r\n", &save);
        char *hostname = strtok_r(NULL, " \t\r\n", &save);
        char detail[192] = "";
        esp_err_t err = ESP_ERR_INVALID_ARG;
        if (ssid != NULL && password != NULL && hostname != NULL) {
            if (strcmp(password, "-") == 0) {
                password = "";
            }
            err = hil_wifi_connection_test(ssid, password, hostname,
                                           detail, sizeof(detail));
            if (err != ESP_OK && detail[0] == '\0') {
                snprintf(detail, sizeof(detail),
                         "ssid=%s association/data/reconnect failed: %s",
                         ssid, esp_err_to_name(err));
            }
        } else {
            snprintf(detail, sizeof(detail), "invalid arguments");
        }
        hil_emit(err == ESP_OK ? "PASS" : "FAIL", "c6.wifi-test",
                 "data", detail);
    } else if (strcmp(command, "wifi-sta-echo-start") == 0 ||
               strcmp(command, "wifi-apsta-connect") == 0) {
        char *ssid = strtok_r(NULL, " \t\r\n", &save);
        char *password = strtok_r(NULL, " \t\r\n", &save);
        esp_err_t err = ssid && password ? hil_wifi_station_echo_start(
            ssid, strcmp(password, "-") ? password : "",
            strcmp(command, "wifi-apsta-connect") == 0) : ESP_ERR_INVALID_ARG;
        char detail[96];
        snprintf(detail, sizeof(detail), "ip=192.168.77.1 udp=3334 result=%s", esp_err_to_name(err));
        hil_emit(err == ESP_OK ? "PASS" : "FAIL", "c6.wifi-sta-start", "electrical", detail);
    } else if (strcmp(command, "wifi-sta-echo-report") == 0) {
        wifi_ap_record_t ap;
        esp_err_t err = esp_wifi_sta_get_ap_info(&ap);
        char detail[160];
        snprintf(detail, sizeof(detail), "connected=%u packets=%" PRIu32
                 " bytes=%" PRIu32 " pattern_errors=%" PRIu32 " echo_errors=%" PRIu32,
                 err == ESP_OK, s_wifi_ap_packets, s_wifi_ap_bytes,
                 s_wifi_ap_pattern_errors, s_wifi_ap_echo_errors);
        hil_emit(err == ESP_OK && !s_wifi_ap_pattern_errors && !s_wifi_ap_echo_errors ? "PASS" : "FAIL",
                 "c6.wifi-sta-report", "data", detail);
    } else if (strcmp(command, "wifi-sta-echo-stop") == 0) {
        esp_err_t err = hil_wifi_station_echo_stop();
        hil_emit(err == ESP_OK ? "PASS" : "FAIL", "c6.wifi-sta-stop", "recovery", esp_err_to_name(err));
    } else if (strcmp(command, "wifi-ap-start") == 0) {
        char *ssid = strtok_r(NULL, " \t\r\n", &save);
        char *password = strtok_r(NULL, " \t\r\n", &save);
        char *channel_text = strtok_r(NULL, " \t\r\n", &save);
        unsigned channel = channel_text != NULL ?
                           strtoul(channel_text, NULL, 0) : 0;
        if (password != NULL && strcmp(password, "-") == 0) {
            password = "";
        }
        esp_err_t err = ssid != NULL && password != NULL ?
                        hil_wifi_ap_start(ssid, password, channel) :
                        ESP_ERR_INVALID_ARG;
        char detail[176];
        snprintf(detail, sizeof(detail),
                 "ssid=%s auth=%s channel=%u ip=192.168.4.1 dhcp=started udp=3334 result=%s",
                 ssid != NULL ? ssid : "-",
                 password != NULL && password[0] == '\0' ? "open" : "wpa2",
                 channel, esp_err_to_name(err));
        hil_emit(err == ESP_OK ? "PASS" : "FAIL", "c6.wifi-ap-start",
                 "firmware", detail);
    } else if (strcmp(command, "wifi-ap-report") == 0) {
        wifi_sta_list_t stations = {0};
        esp_err_t station_err = esp_wifi_ap_get_sta_list(&stations);
        esp_netif_dhcp_status_t dhcp_status = ESP_NETIF_DHCP_INIT;
        esp_err_t dhcp_err = s_wifi_ap_netif != NULL ?
            esp_netif_dhcps_get_status(s_wifi_ap_netif, &dhcp_status) :
            ESP_ERR_INVALID_STATE;
        char detail[256];
        snprintf(detail, sizeof(detail),
                 "clients=%u dhcp=%s connects=%" PRIu32 " disconnects=%" PRIu32
                 " last_reason=%" PRIu32 " packets=%" PRIu32 " bytes=%" PRIu32
                 " pattern_errors=%" PRIu32 " echo_errors=%" PRIu32,
                 station_err == ESP_OK ? stations.num : 0,
                 dhcp_err == ESP_OK && dhcp_status == ESP_NETIF_DHCP_STARTED ?
                    "started" : "stopped",
                 s_wifi_ap_connects, s_wifi_ap_disconnects,
                 s_wifi_ap_last_disconnect_reason,
                 s_wifi_ap_packets, s_wifi_ap_bytes,
                 s_wifi_ap_pattern_errors, s_wifi_ap_echo_errors);
        bool pass = s_wifi_ap_packets != 0 &&
                    s_wifi_ap_pattern_errors == 0 &&
                    s_wifi_ap_echo_errors == 0;
        hil_emit(pass ? "PASS" : "FAIL", "c6.wifi-ap-report", "data",
                 detail);
    } else if (strcmp(command, "wifi-ap-stop") == 0) {
        esp_err_t err = hil_wifi_ap_stop();
        char detail[64];
        snprintf(detail, sizeof(detail), "result=%s", esp_err_to_name(err));
        hil_emit(err == ESP_OK ? "PASS" : "FAIL", "c6.wifi-ap-stop",
                 "recovery", detail);
    } else if (strcmp(command, "ble-test") == 0 || strcmp(command, "ble-hold") == 0) {
        char detail[192] = "BLE GATT test failed";
        esp_err_t err = hil_ble_gatt_test(detail, sizeof(detail), strcmp(command, "ble-hold") == 0);
        if (err != ESP_OK) {
            snprintf(detail, sizeof(detail),
                     "S31 Radio scan/GATT/reconnect failed: %s status=%d",
                     esp_err_to_name(err), s_ble_status);
        }
        hil_emit(err == ESP_OK ? "PASS" : "FAIL", "c6.ble-test",
                 "data", detail);
    } else if (strcmp(command, "ble-link-report") == 0 || strcmp(command, "ble-disconnect") == 0) {
        struct ble_gap_conn_desc desc;
        bool connected = ble_gap_conn_find(s_ble_conn_handle, &desc) == 0;
        if (connected && strcmp(command, "ble-disconnect") == 0)
            ble_gap_terminate(s_ble_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        hil_emit("PASS", "c6.ble-link", "diagnostic", connected ? "connected=1" : "connected=0");
    } else if (strcmp(command, "ethernet-start") == 0) {
        esp_err_t err = hil_eth_start();
        char detail[112];
        snprintf(detail, sizeof(detail),
                 "ip=192.168.77.1 phy=generic@1 mdc=31 mdio=52 reset=51 result=%s",
                 esp_err_to_name(err));
        hil_emit(err == ESP_OK ? "PASS" : "FAIL", "peer.ethernet-start",
                 "electrical", detail);
    } else if (strcmp(command, "ethernet-stop") == 0) {
        esp_err_t err = hil_eth_stop();
        char detail[64];
        snprintf(detail, sizeof(detail), "result=%s", esp_err_to_name(err));
        hil_emit(err == ESP_OK ? "PASS" : "FAIL", "peer.ethernet-stop",
                 "recovery", detail);
    } else if (strcmp(command, "arm") == 0) {
        char *token = strtok_r(NULL, " \t\r\n", &save);
        char expected[9];
        snprintf(expected, sizeof(expected), "%08" PRIx32, s_arm_token);
        if (token != NULL && strcmp(token, expected) == 0) {
            s_armed = true;
            s_arm_deadline_us = esp_timer_get_time() + HIL_ARM_WINDOW_US;
            hil_emit("PASS", "safety.arm", "safety",
                     "outputs armed for 10 seconds");
        } else {
            hil_emit("FAIL", "safety.arm", "safety", "token mismatch");
        }
    } else if (strcmp(command, "disarm") == 0 ||
               strcmp(command, "reset-lines") == 0) {
        hil_lines_high_z();
        hil_emit("PASS", "safety.disarm", "safety",
                 "all tester pins input/no-pull/high-Z");
    } else if (strcmp(command, "peer-stop") == 0) {
        hil_lines_high_z();
        hil_emit("PASS", "peer.stop", "safety",
                 "peer peripheral stopped; all HIL lines high-Z");
    } else if (strcmp(command, "uart-echo-start") == 0) {
        char *baud_text = strtok_r(NULL, " \t\r\n", &save);
        uint32_t baud = baud_text != NULL ? strtoul(baud_text, NULL, 0) : 0;
        esp_err_t err = hil_require_arm() && baud >= 9600 && baud <= 2000000 ?
                        hil_uart_peer_start(baud) : ESP_ERR_INVALID_ARG;
        char detail[96];
        snprintf(detail, sizeof(detail), "baud=%" PRIu32 " result=%s",
                 baud, esp_err_to_name(err));
        hil_emit(err == ESP_OK ? "PASS" : "FAIL", "peer.uart-start",
                 "electrical", detail);
    } else if (strcmp(command, "uart-report") == 0) {
        char detail[96];
        snprintf(detail, sizeof(detail), "bytes=%" PRIu32 " crc32=%08" PRIx32,
                 s_uart_bytes, ~s_uart_crc);
        hil_emit(s_uart_bytes != 0 ? "PASS" : "FAIL", "peer.uart-report",
                 "data", detail);
    } else if (strcmp(command, "spi-master-start") == 0) {
        char *mode_text = strtok_r(NULL, " \t\r\n", &save);
        char *length_text = strtok_r(NULL, " \t\r\n", &save);
        char *speed_text = strtok_r(NULL, " \t\r\n", &save);
        unsigned mode = mode_text ? strtoul(mode_text, NULL, 0) : 99;
        unsigned length = length_text ? strtoul(length_text, NULL, 0) : 0;
        unsigned speed = speed_text ? strtoul(speed_text, NULL, 0) : 0;
        esp_err_t err = hil_require_arm() && mode <= 3 && length && length <= 4096 &&
                        speed >= 10000 && speed <= 5000000 ?
                        hil_spi_master_start(mode, length, speed) : ESP_ERR_INVALID_ARG;
        char detail[96];
        snprintf(detail, sizeof(detail), "mode=%u bytes=%u speed=%u delay_ms=1500 result=%s",
                 mode, length, speed, esp_err_to_name(err));
        hil_emit(err == ESP_OK ? "PASS" : "FAIL", "peer.spi-master-start", "electrical", detail);
    } else if (strcmp(command, "spi-start") == 0) {
        char *mode_text = strtok_r(NULL, " \t\r\n", &save);
        char *length_text = strtok_r(NULL, " \t\r\n", &save);
        char *speed_text = strtok_r(NULL, " \t\r\n", &save);
        char *rsck_text = strtok_r(NULL, " \t\r\n", &save);
        char *tsck_text = strtok_r(NULL, " \t\r\n", &save);
        char *clk13_text = strtok_r(NULL, " \t\r\n", &save);
        unsigned mode = mode_text != NULL ? strtoul(mode_text, NULL, 0) : 99;
        size_t length = length_text != NULL ? strtoul(length_text, NULL, 0) :
                                              4096;
        uint32_t speed_hz = speed_text != NULL ?
                            strtoul(speed_text, NULL, 0) : 1000000;
        int rsck = rsck_text != NULL ? (int)strtol(rsck_text, NULL, 0) : -1;
        int tsck = tsck_text != NULL ? (int)strtol(tsck_text, NULL, 0) : -1;
        int clk13 = clk13_text != NULL ? (int)strtol(clk13_text, NULL, 0) : -1;
        esp_err_t err = hil_require_arm() && mode <= 3 && length >= 1 &&
                        length <= 4096 && speed_hz <= 40000000 &&
                        rsck >= -1 && rsck <= 1 && tsck >= -1 && tsck <= 1 &&
                        clk13 >= -1 && clk13 <= 1 ?
                        hil_spi_peer_start(mode, length, speed_hz, rsck,
                                           tsck, clk13) :
                                         ESP_ERR_INVALID_ARG;
        char detail[128];
        snprintf(detail, sizeof(detail),
                 "mode=%u length=%u speed=%" PRIu32
                 " rsck=%d tsck=%d clk13=%d result=%s", mode,
                 (unsigned)length, speed_hz, rsck, tsck, clk13,
                 esp_err_to_name(err));
        hil_emit(err == ESP_OK ? "PASS" : "FAIL", "peer.spi-start",
                 "electrical", detail);
    } else if (strcmp(command, "spi-report") == 0) {
        char detail[256];
        char head[sizeof(s_spi_first_rx) * 2 + 1] = "";
        int clock_edges = -1;
        int cs_edges = -1;
        if (s_spi_clock_unit != NULL) {
            pcnt_unit_get_count(s_spi_clock_unit, &clock_edges);
        }
        if (s_spi_cs_unit != NULL) {
            pcnt_unit_get_count(s_spi_cs_unit, &cs_edges);
        }
        size_t head_len = s_spi_bytes < sizeof(s_spi_first_rx) ?
                          s_spi_bytes : sizeof(s_spi_first_rx);
        for (size_t i = 0; i < head_len; ++i) {
            snprintf(head + i * 2, sizeof(head) - i * 2, "%02x",
                     s_spi_first_rx[i]);
        }
        snprintf(detail, sizeof(detail), "bytes=%" PRIu32
                 " expected=%u completions=%" PRIu32
                 " first_bits=%u last_bits=%u active-clocks=%d cs-rises=%d"
                 " rx_crc32=%08" PRIx32 " tx_crc32=%08" PRIx32
                 " first_rx=%s",
                 s_spi_bytes, (unsigned)s_spi_length, s_spi_completions,
                 (unsigned)s_spi_first_bits, (unsigned)s_spi_last_bits,
                 clock_edges, cs_edges, ~s_spi_crc,
                 s_spi_tx != NULL ? hil_crc32_update(~0U, s_spi_tx,
                                                     s_spi_length) ^ ~0U : 0,
                 head);
        hil_emit(s_spi_bytes == s_spi_length && s_spi_completions == 1 ?
                 "PASS" : "FAIL", "peer.spi-report",
                 "data", detail);
    } else if (strcmp(command, "i2c-start") == 0) {
        char *length_text = strtok_r(NULL, " \t\r\n", &save);
        unsigned length = length_text ? strtoul(length_text, NULL, 0) : 16;
        char *lane_text = strtok_r(NULL, " \t\r\n", &save);
        unsigned lane = lane_text ? strtoul(lane_text, NULL, 0) : 0;
        esp_err_t err = hil_require_arm() && length && length <= 4096 &&
                        lane < HIL_LANE_COUNT && lane != 1 ? hil_i2c_peer_start(length, lane) :
                        ESP_ERR_INVALID_STATE;
        char detail[96];
        snprintf(detail, sizeof(detail),
                 "address=0x42 scl=L%u sda=L1 length=%u result=%s",
                 lane, length, esp_err_to_name(err));
        hil_emit(err == ESP_OK ? "PASS" : "FAIL", "peer.i2c-start",
                 "electrical", detail);
    } else if (strcmp(command, "i2c-report") == 0) {
        char detail[128];
        snprintf(detail, sizeof(detail), "reads=%" PRIu32 " writes=%" PRIu32
                 " rx_bytes=%u rx_crc32=%08" PRIx32,
                 s_i2c_reads, s_i2c_writes, (unsigned)s_i2c_rx_length,
                 ~hil_crc32_update(~0U, s_i2c_rx, s_i2c_rx_length));
        hil_emit(s_i2c_reads != 0 ? "PASS" : "FAIL", "peer.i2c-report",
                 "data", detail);
    } else if (strcmp(command, "i2s-rx-start") == 0 ||
               strcmp(command, "i2s-rx-master-start") == 0 ||
               strcmp(command, "i2s-tx-start") == 0) {
        bool transmit = command[4] == 't';
        bool master = transmit || strcmp(command, "i2s-rx-master-start") == 0;
        char *length_text = strtok_r(NULL, " \t\r\n", &save);
        char *rate_text = strtok_r(NULL, " \t\r\n", &save);
        size_t length = length_text != NULL ? strtoul(length_text, NULL, 0) : 0;
        uint32_t rate = rate_text != NULL ? strtoul(rate_text, NULL, 0) : 48000;
        esp_err_t err = hil_require_arm() && length >= 64 && length <= 65536 &&
                        rate >= 8000 && rate <= 48000 ?
            hil_i2s_peer_start(transmit, master, length, rate) : ESP_ERR_INVALID_ARG;
        char detail[112];
        snprintf(detail, sizeof(detail),
                 "direction=%s role=%s rate=%u format=S16_LE channels=2 bytes=%u result=%s",
                 transmit ? "P4-to-S31" : "S31-to-P4",
                 master ? "master" : "slave", (unsigned)rate, (unsigned)length,
                 esp_err_to_name(err));
        hil_emit(err == ESP_OK ? "PASS" : "FAIL", "peer.i2s-start",
                 "electrical", detail);
    } else if (strcmp(command, "i2s-report") == 0 ||
               strcmp(command, "i2s-wait-report") == 0) {
        if (strcmp(command, "i2s-wait-report") == 0) {
            for (unsigned wait = 0; wait < 1200 && s_i2s_peer_task != NULL;
                 ++wait) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        int bclk_edges = 0;
        int ws_edges = 0;
        char head[33] = "";
        char mismatch_data[65] = "";
        if (s_spi_clock_unit != NULL) {
            pcnt_unit_get_count(s_spi_clock_unit, &bclk_edges);
        }
        if (s_spi_cs_unit != NULL) {
            pcnt_unit_get_count(s_spi_cs_unit, &ws_edges);
        }
        if (s_i2s_buffer != NULL) {
            size_t head_len = s_i2s_bytes < 16 ? s_i2s_bytes : 16;
            for (size_t i = 0; i < head_len; ++i) {
                snprintf(head + i * 2, sizeof(head) - i * 2, "%02x",
                         s_i2s_buffer[i]);
            }
        }
        if (s_i2s_buffer != NULL && s_i2s_mismatch >= 0) {
            size_t start = (size_t)s_i2s_mismatch;
            size_t count = s_i2s_bytes - start;
            if (count > 32) count = 32;
            for (size_t i = 0; i < count; i++) {
                snprintf(mismatch_data + i * 2, sizeof(mismatch_data) - i * 2,
                         "%02x", s_i2s_buffer[start + i]);
            }
        }
        char detail[384];
        snprintf(detail, sizeof(detail),
                 "bytes=%u expected=%u discarded=%u active=%u mismatch=%d error=%s bclk_edges=%d ws_edges=%d crc32=%08" PRIx32 " head=%s mismatch_data=%s",
                 (unsigned)s_i2s_bytes, (unsigned)s_i2s_length,
                 (unsigned)s_i2s_discarded, s_i2s_peer_task != NULL,
                 s_i2s_mismatch,
                 esp_err_to_name(s_i2s_error), bclk_edges,
                 ws_edges,
                 s_i2s_buffer != NULL ?
                 hil_crc32_update(~0U, s_i2s_buffer, s_i2s_bytes) ^ ~0U : 0,
                 head, mismatch_data);
        bool pass = s_i2s_peer_task == NULL &&
                    s_i2s_bytes == s_i2s_length && s_i2s_mismatch < 0;
        hil_emit(pass ? "PASS" : "FAIL", "peer.i2s-report", "data", detail);
    } else if (strcmp(command, "i2s-raw-report") == 0) {
        char detail[96];
        size_t used = snprintf(detail, sizeof(detail), "bytes=%u data=", (unsigned)s_i2s_raw_length);
        for (size_t i = 0; i < s_i2s_raw_length; i++)
            used += snprintf(detail + used, sizeof(detail) - used, "%02x", s_i2s_raw_sample[i]);
        hil_emit(s_i2s_raw_length ? "PASS" : "FAIL", "peer.i2s-raw", "diagnostic", detail);
    } else if (strcmp(command, "pulse-monitor-start") == 0) {
        esp_err_t err = hil_pulse_monitor_start();
        char detail[96];
        snprintf(detail, sizeof(detail), "input=L0/gpio23 result=%s",
                 esp_err_to_name(err));
        hil_emit(err == ESP_OK ? "PASS" : "FAIL", "peer.pulse-monitor-start",
                 "electrical", detail);
    } else if (strcmp(command, "pulse-monitor-report") == 0) {
        uint32_t rises = s_pulse_rises;
        uint32_t falls = s_pulse_falls;
        int64_t span = s_pulse_last_rise - s_pulse_first_rise;
        uint32_t frequency = rises > 1 && span > 0 ?
            (uint32_t)(((uint64_t)(rises - 1) * 1000000ULL) / span) : 0;
        uint64_t total = s_pulse_high_us + s_pulse_low_us;
        uint32_t duty_permille = total ?
            (uint32_t)(s_pulse_high_us * 1000ULL / total) : 0;
        hil_pulse_monitor_stop();
        char detail[128];
        snprintf(detail, sizeof(detail),
                 "rises=%" PRIu32 " falls=%" PRIu32
                 " frequency_hz=%" PRIu32 " duty_permille=%" PRIu32,
                 rises, falls, frequency, duty_permille);
        hil_emit(rises > 2 && falls > 2 ? "PASS" : "FAIL",
                 "peer.pulse-monitor-report", "electrical", detail);
    } else if (strcmp(command, "pulse-generate") == 0) {
        char *hz_text = strtok_r(NULL, " \t\r\n", &save);
        char *count_text = strtok_r(NULL, " \t\r\n", &save);
        uint32_t hz = hz_text ? strtoul(hz_text, NULL, 0) : 0;
        uint32_t count = count_text ? strtoul(count_text, NULL, 0) : 0;
        bool valid = hil_require_arm() && hz >= 10 && hz <= 10000 &&
                     count >= 1 && count <= 10000;
        if (valid) {
            uint32_t half_us = 500000U / hz;
            gpio_set_direction(s_hil_lane_pins[1], GPIO_MODE_OUTPUT);
            gpio_set_level(s_hil_lane_pins[1], 0);
            for (uint32_t i = 0; i < count; ++i) {
                gpio_set_level(s_hil_lane_pins[1], 1);
                esp_rom_delay_us(half_us);
                gpio_set_level(s_hil_lane_pins[1], 0);
                esp_rom_delay_us(half_us);
            }
            gpio_set_direction(s_hil_lane_pins[1], GPIO_MODE_INPUT);
            gpio_set_pull_mode(s_hil_lane_pins[1], GPIO_FLOATING);
        }
        char detail[96];
        snprintf(detail, sizeof(detail), "output=L1/gpio22 hz=%" PRIu32
                 " pulses=%" PRIu32, hz, count);
        hil_emit(valid ? "PASS" : "FAIL", "peer.pulse-generate",
                 "electrical", detail);
    } else if (strcmp(command, "gpio-read") == 0 ||
               strcmp(command, "lane-read") == 0) {
        bool lane_mode = strcmp(command, "lane-read") == 0;
        int target = hil_parse_pin(strtok_r(NULL, " \t\r\n", &save));
        int pin = lane_mode && target >= 0 && target < HIL_LANE_COUNT ?
                  (int)s_hil_lane_pins[target] : target;
        if (!hil_pin_allowed(pin)) {
            hil_emit("FAIL", lane_mode ? "lane.read" : "gpio.read",
                     "electrical", lane_mode ? "invalid lane" :
                     "pin is not in safe pool");
        } else {
            char detail[64];
            if (lane_mode) {
                snprintf(detail, sizeof(detail), "lane=%d gpio=%d level=%d",
                         target, pin, gpio_get_level((gpio_num_t)pin));
            } else {
                snprintf(detail, sizeof(detail), "gpio=%d level=%d", pin,
                         gpio_get_level((gpio_num_t)pin));
            }
            hil_emit("PASS", lane_mode ? "lane.read" : "gpio.read",
                     "electrical", detail);
        }
    } else if (strcmp(command, "gpio-input") == 0 ||
               strcmp(command, "lane-input") == 0) {
        bool lane_mode = strcmp(command, "lane-input") == 0;
        int target = hil_parse_pin(strtok_r(NULL, " \t\r\n", &save));
        int pin = lane_mode && target >= 0 && target < HIL_LANE_COUNT ?
                  (int)s_hil_lane_pins[target] : target;
        char *pull = strtok_r(NULL, " \t\r\n", &save);
        if (!hil_pin_allowed(pin) || pull == NULL) {
            hil_emit("FAIL", lane_mode ? "lane.input" : "gpio.input",
                     "electrical", lane_mode ? "invalid lane or pull" :
                     "invalid pin or pull");
        } else {
            gpio_pullup_t up = strcmp(pull, "up") == 0 ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
            gpio_pulldown_t down = strcmp(pull, "down") == 0 ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
            gpio_config_t config = {
                .pin_bit_mask = 1ULL << pin,
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = up,
                .pull_down_en = down,
                .intr_type = GPIO_INTR_DISABLE,
            };
            esp_err_t err = gpio_config(&config);
            char detail[64];
            if (lane_mode) {
                snprintf(detail, sizeof(detail), "lane=%d gpio=%d result=%s",
                         target, pin, esp_err_to_name(err));
            } else {
                snprintf(detail, sizeof(detail), "gpio=%d result=%s", pin,
                         esp_err_to_name(err));
            }
            hil_emit(err == ESP_OK ? "PASS" : "FAIL",
                     lane_mode ? "lane.input" : "gpio.input",
                     "electrical", detail);
        }
    } else if (strcmp(command, "gpio-write") == 0 ||
               strcmp(command, "lane-write") == 0) {
        bool lane_mode = strcmp(command, "lane-write") == 0;
        int target = hil_parse_pin(strtok_r(NULL, " \t\r\n", &save));
        int pin = lane_mode && target >= 0 && target < HIL_LANE_COUNT ?
                  (int)s_hil_lane_pins[target] : target;
        char *level_text = strtok_r(NULL, " \t\r\n", &save);
        int level = level_text != NULL ? atoi(level_text) : -1;
        if (!hil_pin_allowed(pin) || (level != 0 && level != 1)) {
            hil_emit("FAIL", lane_mode ? "lane.write" : "gpio.write",
                     "electrical", lane_mode ? "invalid lane or level" :
                     "invalid pin or level");
        } else if (hil_require_arm()) {
            gpio_config_t config = {
                .pin_bit_mask = 1ULL << pin,
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE,
            };
            esp_err_t err = gpio_config(&config);
            if (err == ESP_OK) {
                err = gpio_set_level((gpio_num_t)pin, level);
            }
            char detail[64];
            if (lane_mode) {
                snprintf(detail, sizeof(detail),
                         "lane=%d gpio=%d level=%d result=%s", target, pin,
                         level, esp_err_to_name(err));
            } else {
                snprintf(detail, sizeof(detail), "gpio=%d level=%d result=%s",
                         pin, level, esp_err_to_name(err));
            }
            hil_emit(err == ESP_OK ? "PASS" : "FAIL",
                     lane_mode ? "lane.write" : "gpio.write",
                     "electrical", detail);
        }
    } else if (strcmp(command, "gpio-wake") == 0) {
        int pin = hil_parse_pin(strtok_r(NULL, " \t\r\n", &save));
        char *inactive_text = strtok_r(NULL, " \t\r\n", &save);
        char *delay_text = strtok_r(NULL, " \t\r\n", &save);
        char *hold_text = strtok_r(NULL, " \t\r\n", &save);
        int inactive = inactive_text != NULL ? atoi(inactive_text) : -1;
        uint32_t delay_ms = delay_text != NULL ?
                            strtoul(delay_text, NULL, 0) : 0;
        uint32_t hold_ms = hold_text != NULL ?
                           strtoul(hold_text, NULL, 0) : 0;
        esp_err_t err = ESP_ERR_INVALID_ARG;
        if (hil_pin_allowed(pin) && (inactive == 0 || inactive == 1) &&
            delay_ms >= 250 && delay_ms <= 8000 &&
            hold_ms >= 10 && hold_ms <= 1000 && hil_require_arm())
            err = hil_gpio_wake_schedule((gpio_num_t)pin, inactive,
                                         delay_ms, hold_ms);
        char detail[128];
        snprintf(detail, sizeof(detail),
                 "gpio=%d inactive=%d delay_ms=%" PRIu32
                 " hold_ms=%" PRIu32 " result=%s",
                 pin, inactive, delay_ms, hold_ms, esp_err_to_name(err));
        hil_emit(err == ESP_OK ? "PASS" : "FAIL",
                 "peer.gpio-wake-schedule", "electrical", detail);
    } else {
        hil_emit("FAIL", "rpc.command", "firmware", "unknown command");
    }
}

static void hil_console_task(void *argument)
{
    (void)argument;
    char line[192];
    size_t used = 0;

    while (true) {
        uint8_t byte;
        int received = uart_read_bytes(UART_NUM_0, &byte, 1,
                                       pdMS_TO_TICKS(1000));
        if (received <= 0) {
            continue;
        }
        if (byte == '\r' || byte == '\n') {
            if (used != 0) {
                line[used] = '\0';
                hil_command(line);
                used = 0;
            }
            continue;
        }
        if (used + 1 < sizeof(line)) {
            line[used++] = (char)byte;
        } else {
            used = 0;
            hil_emit("FAIL", "rpc.command", "firmware",
                     "input line exceeded 191 bytes and was discarded");
        }
    }
}

void app_main(void)
{
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    s_output_lock = xSemaphoreCreateMutex();
    s_ble_events = xEventGroupCreate();
    s_arm_token = esp_random();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    hil_lines_high_z();
    if (!uart_is_driver_installed(UART_NUM_0)) {
        ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 512, 0, 0, NULL, 0));
    }
    ESP_ERROR_CHECK(uart_set_baudrate(UART_NUM_0,
                                     CONFIG_ESP_CONSOLE_UART_BAUDRATE));
    xTaskCreate(hil_arm_watchdog, "hil-safety", 3072, NULL, 8, NULL);
    /* Hosted Wi-Fi scan/RPC callbacks run synchronously in this task and use
     * more than the generic command parser's 4 KiB stack.  Keep enough headroom
     * for the full self-test instead of relying on a command-dependent margin. */
    xTaskCreate(hil_console_task, "hil-console", 8192, NULL, 5, NULL);

    hil_emit("PASS", "boot.ready", "firmware",
             "RPC console ready; all tester pins are high-Z");
}
