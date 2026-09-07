/* SPDX-License-Identifier: Unlicense OR CC0-1.0 */
#pragma once

#include <stdint.h>
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len);
void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event,
                     esp_avrc_ct_cb_param_t *param);
void bt_app_rc_tg_cb(esp_avrc_tg_cb_event_t event,
                     esp_avrc_tg_cb_param_t *param);
