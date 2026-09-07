/* SPDX-License-Identifier: Unlicense OR CC0-1.0 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

void coex_bench_record_a2dp_data(uint32_t length);
void coex_bench_set_a2dp_connected(bool connected);
void coex_bench_set_a2dp_streaming(bool streaming);
