# ESP32-S31 LP-core firmware

This project builds the Linux-managed LP-core payload with ESP-IDF's native
`ulp_embed_binary()` flow. The LP application uses the native hardware mailbox
runtime and the shared protocol in `../../shared/s31_lp_protocol.h`.

Build and stage the ELF and raw binary into the root filesystem overlay:

```sh
make -C firmware/lp stage
```

Linux remoteproc loads `s31-lp-core.elf` into the 32 KiB LP SRAM. The raw binary
is staged as a debugging and recovery artifact, but Linux boots the ELF image.
