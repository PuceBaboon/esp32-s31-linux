# ESP32-S31 radio bundle licensing boundary

`esp32s31-radio.ko` combines the Linux Wi-Fi/HCI frontends, the S31
compatibility runtime, and a narrow loader. The locally generated ESP-IDF
radio dependency closure remains a separate, versioned
`esp32s31-radio-fw-v1.o` file in the radio partition. It is loaded and
relocated at runtime; it is not statically linked into the kernel module.

The deployable module, radio overlays, build configuration and this notice are
also stored together in the read-only `radio.sqfs` image. The image occupies a
dedicated `radio-bundle` MTD partition and is mounted only when a radio overlay
selects the driver.

That separation is useful for deployment and for keeping exact source
identities together, but it is only an engineering boundary and is not a
legal conclusion. The module imports GPL-only kernel symbols and is therefore
marked `GPL v2` for Linux module-loader compatibility. The external payload
can include Apache-2.0 and Espressif-supplied components whose redistribution
and runtime-combination terms must be reviewed separately.

The default package mode is consequently `engineering-only`. Public release
mode is intentionally gated on both:

1. written permission or a compatible licensing grant for the complete radio
   payload; and
2. an exact corresponding-source archive for the redistributable payload and
   the module sources.

Keep the grant, source archive, toolchain identity, ESP-IDF commit, generated
sdkconfig, module, overlays, notices and checksums together for each release.
