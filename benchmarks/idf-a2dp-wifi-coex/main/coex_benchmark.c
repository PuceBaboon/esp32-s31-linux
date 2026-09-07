/* SPDX-License-Identifier: Unlicense OR CC0-1.0 */

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "coex_benchmark.h"

#define METRIC_TASK_SLOTS 64
#define TCP_READ_SIZE 16384

static const char *TAG = "COEX_BENCH";
static volatile uint32_t s_wifi_bytes;
static volatile uint32_t s_a2dp_packets;
static volatile uint32_t s_a2dp_bytes;
static volatile bool s_a2dp_connected;
static volatile bool s_a2dp_streaming;

static void coex_timer_registers_dump(void)
{
    volatile const uint32_t *global = (volatile const uint32_t *)0x2010f000U;
    unsigned int idx;

    ESP_LOGI(TAG, "COEX_REG global=%08" PRIx32 "/%08" PRIx32
             "/%08" PRIx32 "/%08" PRIx32,
             global[0], global[1], global[2], global[3]);
    for (idx = 0; idx < 4; idx++) {
        volatile const uint32_t *timer =
            (volatile const uint32_t *)(0x2010f400U + (idx << 4));

        ESP_LOGI(TAG, "COEX_REG timer%u=%08" PRIx32 "/%08" PRIx32
                 "/%08" PRIx32 "/%08" PRIx32,
                 idx, timer[0], timer[1], timer[2], timer[3]);
    }
}

extern void coex_bt_app_main(void);

void coex_bench_record_a2dp_data(uint32_t length)
{
    __atomic_add_fetch(&s_a2dp_packets, 1, __ATOMIC_RELAXED);
    __atomic_add_fetch(&s_a2dp_bytes, length, __ATOMIC_RELAXED);
}

void coex_bench_set_a2dp_connected(bool connected)
{
    __atomic_store_n(&s_a2dp_connected, connected, __ATOMIC_RELEASE);
    if (!connected) {
        __atomic_store_n(&s_a2dp_streaming, false, __ATOMIC_RELEASE);
    }
}

void coex_bench_set_a2dp_streaming(bool streaming)
{
    __atomic_store_n(&s_a2dp_streaming, streaming, __ATOMIC_RELEASE);
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id,
                       void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        ESP_ERROR_CHECK(esp_wifi_connect());
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected; reconnecting");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = data;
        ESP_LOGI(TAG, "BENCH_READY ip=" IPSTR " tcp_port=%d",
                 IP2STR(&event->ip_info.ip), CONFIG_BENCH_TCP_PORT);
    }
}

static void wifi_start(void)
{
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t config = {0};

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               wifi_event, NULL));
    strlcpy((char *)config.sta.ssid, CONFIG_BENCH_WIFI_SSID,
            sizeof(config.sta.ssid));
    strlcpy((char *)config.sta.password, CONFIG_BENCH_WIFI_PASSWORD,
            sizeof(config.sta.password));
    config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    config.sta.pmf_cfg.capable = true;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void tcp_sink_task(void *arg)
{
    uint8_t *buffer = heap_caps_malloc(TCP_READ_SIZE, MALLOC_CAP_8BIT);
    (void)arg;

    if (!buffer) {
        ESP_LOGE(TAG, "cannot allocate TCP receive buffer");
        vTaskDelete(NULL);
    }

    for (;;) {
        struct sockaddr_in address = {
            .sin_family = AF_INET,
            .sin_port = htons(CONFIG_BENCH_TCP_PORT),
            .sin_addr.s_addr = htonl(INADDR_ANY),
        };
        int server = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        int one = 1;
        int rcvbuf = CONFIG_BENCH_TCP_RCVBUF;

        if (server < 0) {
            ESP_LOGE(TAG, "socket failed: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        setsockopt(server, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
        if (bind(server, (struct sockaddr *)&address, sizeof(address)) < 0 ||
            listen(server, 1) < 0) {
            ESP_LOGE(TAG, "bind/listen failed: errno=%d", errno);
            close(server);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        ESP_LOGI(TAG, "TCP sink listening on port %d", CONFIG_BENCH_TCP_PORT);

        for (;;) {
            struct sockaddr_in peer;
            socklen_t peer_len = sizeof(peer);
            int client = accept(server, (struct sockaddr *)&peer, &peer_len);

            if (client < 0) {
                ESP_LOGW(TAG, "accept failed: errno=%d", errno);
                break;
            }
            ESP_LOGI(TAG, "TCP client connected from %s",
                     inet_ntoa(peer.sin_addr));
            for (;;) {
                int received = recv(client, buffer, TCP_READ_SIZE, 0);
                if (received <= 0) {
                    break;
                }
                __atomic_add_fetch(&s_wifi_bytes, (uint32_t)received,
                                   __ATOMIC_RELAXED);
            }
            shutdown(client, SHUT_RDWR);
            close(client);
            ESP_LOGI(TAG, "TCP client closed");
        }
        close(server);
    }
}

static uint32_t cpu_busy_millicores(void)
{
    static configRUN_TIME_COUNTER_TYPE previous_total;
    static uint64_t previous_idle;
    static TaskStatus_t status[METRIC_TASK_SLOTS];
    configRUN_TIME_COUNTER_TYPE total;
    uint64_t idle = 0;
    uint32_t busy = 0;
    UBaseType_t count;

    count = uxTaskGetSystemState(status, METRIC_TASK_SLOTS, &total);
    for (UBaseType_t i = 0; i < count; i++) {
        if (!strncmp(status[i].pcTaskName, "IDLE", 4)) {
            idle += status[i].ulRunTimeCounter;
        }
    }
    if (previous_total && total != previous_total) {
        uint64_t elapsed = total - previous_total;
        uint64_t idle_elapsed = idle - previous_idle;
        uint64_t capacity = elapsed * CONFIG_FREERTOS_NUMBER_OF_CORES;
        if (idle_elapsed < capacity) {
            busy = (uint32_t)(((capacity - idle_elapsed) * 1000) / elapsed);
        }
    }
    previous_total = total;
    previous_idle = idle;
    return busy;
}

static void metrics_task(void *arg)
{
    uint32_t previous_wifi = 0;
    uint32_t previous_a2dp_bytes = 0;
    uint32_t previous_a2dp_packets = 0;
    int64_t previous_us = esp_timer_get_time();
    unsigned int samples = 0;
    (void)arg;

    for (;;) {
        wifi_ap_record_t ap;
        uint32_t wifi = __atomic_load_n(&s_wifi_bytes, __ATOMIC_RELAXED);
        uint32_t a2dp_bytes = __atomic_load_n(&s_a2dp_bytes,
                                               __ATOMIC_RELAXED);
        uint32_t a2dp_packets = __atomic_load_n(&s_a2dp_packets,
                                                 __ATOMIC_RELAXED);
        int64_t now_us;
        uint64_t elapsed_us;
        uint32_t wifi_kbps;
        uint32_t a2dp_kbps;
        uint32_t busy_millicores;
        int rssi = 0;

        vTaskDelay(pdMS_TO_TICKS(1000));
        now_us = esp_timer_get_time();
        elapsed_us = now_us - previous_us;
        wifi = __atomic_load_n(&s_wifi_bytes, __ATOMIC_RELAXED);
        a2dp_bytes = __atomic_load_n(&s_a2dp_bytes, __ATOMIC_RELAXED);
        a2dp_packets = __atomic_load_n(&s_a2dp_packets, __ATOMIC_RELAXED);
        wifi_kbps = elapsed_us ?
            (uint32_t)(((uint64_t)(wifi - previous_wifi) * 8000) /
                       elapsed_us) : 0;
        a2dp_kbps = elapsed_us ?
            (uint32_t)(((uint64_t)(a2dp_bytes - previous_a2dp_bytes) * 8000) /
                       elapsed_us) : 0;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            rssi = ap.rssi;
        }
        busy_millicores = cpu_busy_millicores();

        ESP_LOGI(TAG,
                 "BENCH wifi=%" PRIu32 ".%03" PRIu32 "Mbps total=%" PRIu32
                 " a2dp=%" PRIu32 "kbps packets=%" PRIu32
                 " connected=%u streaming=%u cpu=%" PRIu32 ".%03" PRIu32
                 "cores heap=%" PRIu32 " min_heap=%" PRIu32 " rssi=%d",
                 wifi_kbps / 1000, wifi_kbps % 1000, wifi,
                 a2dp_kbps, a2dp_packets - previous_a2dp_packets,
                 __atomic_load_n(&s_a2dp_connected, __ATOMIC_ACQUIRE),
                 __atomic_load_n(&s_a2dp_streaming, __ATOMIC_ACQUIRE),
                 busy_millicores / 1000,
                 busy_millicores % 1000,
                 heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 esp_get_minimum_free_heap_size(), rssi);
        if (++samples <= 8)
            coex_timer_registers_dump();

        previous_wifi = wifi;
        previous_a2dp_bytes = a2dp_bytes;
        previous_a2dp_packets = a2dp_packets;
        previous_us = now_us;
    }
}

void app_main(void)
{
    coex_bt_app_main();
    wifi_start();
    xTaskCreate(tcp_sink_task, "tcp_sink", 4096, NULL, 5, NULL);
    xTaskCreate(metrics_task, "bench_metrics", 4096, NULL, 4, NULL);
}
