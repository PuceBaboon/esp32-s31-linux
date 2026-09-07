/* SPDX-License-Identifier: Unlicense OR CC0-1.0 */

#include <stdint.h>

#include "esp_rom_sys.h"

extern int __real_coex_enable(void);
extern void __real_coex_disable(void);
extern int __real_coex_wifi_request(uint32_t event, uint32_t latency,
                                    uint32_t duration);
extern int __real_coex_wifi_release(uint32_t event);
extern int __real_coex_bt_request(uint32_t event, uint32_t latency,
                                  uint32_t duration);
extern int __real_coex_bt_release(uint32_t event);

static unsigned int enable_calls;
static unsigned int disable_calls;
static unsigned int wifi_requests;
static unsigned int wifi_releases;
static unsigned int bt_requests;
static unsigned int bt_releases;

int __wrap_coex_enable(void)
{
    int rc = __real_coex_enable();

    esp_rom_printf("COEX_TRACE enable=%u rc=%d\n", ++enable_calls, rc);
    return rc;
}

void __wrap_coex_disable(void)
{
    __real_coex_disable();
    esp_rom_printf("COEX_TRACE disable=%u\n", ++disable_calls);
}

int __wrap_coex_wifi_request(uint32_t event, uint32_t latency,
                             uint32_t duration)
{
    int rc = __real_coex_wifi_request(event, latency, duration);
    unsigned int count = ++wifi_requests;

    if (count <= 24 || (count % 128) == 0)
        esp_rom_printf("COEX_TRACE wifi_req=%u event=%u latency=%u duration=%u rc=%d\n",
                       count, event, latency, duration, rc);
    return rc;
}

int __wrap_coex_wifi_release(uint32_t event)
{
    int rc = __real_coex_wifi_release(event);
    unsigned int count = ++wifi_releases;

    if (count <= 24 || (count % 128) == 0)
        esp_rom_printf("COEX_TRACE wifi_rel=%u event=%u rc=%d\n",
                       count, event, rc);
    return rc;
}

int __wrap_coex_bt_request(uint32_t event, uint32_t latency,
                           uint32_t duration)
{
    int rc = __real_coex_bt_request(event, latency, duration);
    unsigned int count = ++bt_requests;

    if (count <= 24 || (count % 128) == 0)
        esp_rom_printf("COEX_TRACE bt_req=%u event=%u latency=%u duration=%u rc=%d\n",
                       count, event, latency, duration, rc);
    return rc;
}

int __wrap_coex_bt_release(uint32_t event)
{
    int rc = __real_coex_bt_release(event);
    unsigned int count = ++bt_releases;

    if (count <= 24 || (count % 128) == 0)
        esp_rom_printf("COEX_TRACE bt_rel=%u event=%u rc=%d\n",
                       count, event, rc);
    return rc;
}
