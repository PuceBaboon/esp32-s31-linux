# ESP-IDF S31 A2DP/Wi-Fi Coexistence Benchmark

This project provides a native ESP-IDF reference for the Linux shared-radio
work. It deliberately uses ESP-IDF's unmodified S31 Bluedroid coexistence
example for Classic A2DP plus BLE GATT, then adds a Wi-Fi station, a raw TCP
receive sink, and one-second measurements.

The serial `BENCH` line reports Wi-Fi receive throughput, decoded A2DP callback
rate and packet count, A2DP connection/streaming state, aggregate dual-core CPU
use, internal free/minimum heap, and Wi-Fi RSSI. `BENCH_READY` reports the DHCP
address and TCP port.

## Configure and build

Use the ESP-IDF checkout configured by this repository. Select target
`esp32s31`, then set the station SSID and password under **S31 A2DP/Wi-Fi
coexistence benchmark** in `idf.py menuconfig`. Credentials are written only to
the ignored local `sdkconfig`.

Build and flash with the normal `idf.py build flash monitor` workflow. This is
a standalone IDF image; flashing it temporarily replaces the Linux boot image.
The app partition ends at `0x290000`, below the Linux radio bundle partition.

The Bluetooth devices are the official example names
`ESP_COEX_BTDM_DEMO`/`ESP_COEX_BLE_DEMO`. Pair the Windows host with the Classic
device and start A2DP playback. BLE advertising remains enabled by the same
Bluedroid instance.

On ESP32-S31, `sdkconfig.defaults.esp32s31` disables
`CONFIG_BT_BLUEDROID_ESP_COEX_VSC`. Bluedroid otherwise sends the host-side
common coexistence VSC (`0xfc82`), but the S31 controller does not implement
that command. The controller's internal `btdm_coex` path and the shared Wi-Fi
coexistence library remain enabled, as do BR/EDR, A2DP, and BLE. This target
override avoids treating an unsupported host command as a controller feature;
it does not disable radio coexistence.

## Run concurrent Wi-Fi traffic

After `BENCH_READY`, run `tools/tcp_push.py <board-ip> --seconds 60` on the
Windows host while A2DP is playing. The host tool reports transmitted bytes and
average TCP throughput; the board's one-second lines show whether A2DP remained
streaming and provide the CPU/heap baseline.

For comparison with Linux, retain at least these values: median and minimum
one-second Wi-Fi Mbps, host total bytes, continuous A2DP packet callbacks,
aggregate CPU cores, minimum free internal heap, RSSI, and any disconnect or
watchdog event. Association alone is not a bandwidth result.

## Current S31 reference result

The current app was tested on real S31 hardware with a Windows A2DP source and
the Wi-Fi station associated to the same external access point as the traffic
generator. No host hotspot was used.

| Workload | Host TCP result | A2DP state | Aggregate CPU | Minimum internal heap |
| --- | --- | --- | --- | --- |
| Wi-Fi only, 60 s | 2.738 Mbps | Not streaming | about 0.18-0.24 cores | not recorded for this comparison |
| A2DP + Wi-Fi, two earlier runs | 2.682 and 2.534 Mbps | Continuous | about 0.31-0.38 cores | 141,288 bytes |
| A2DP + Wi-Fi after S31 VSC override | 18,087,936 bytes in 60.029 s, 2.411 Mbps, payload hash completed | Continuous `connected=1 streaming=1`; 1.34-1.44 Mbps decoded PCM callback rate | about 0.28-0.34 cores | 137,480 bytes |

The A2DP number is the decoded PCM callback data rate, not the SBC bitrate on
the air. During the final 60-second payload window there was no `0xfc82`, A2DP
suspend, disconnect, reset, or watchdog event. BLE remained enabled and a
short BLE GATT connect/disconnect also occurred without interrupting A2DP.
