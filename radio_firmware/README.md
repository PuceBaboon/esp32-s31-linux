# ESP32-S31 Linux S-mode radio firmware

This directory links the ESP-IDF Wi-Fi, Bluetooth, coexistence and PHY
objects into a relocatable ILP32F payload consumed by the Linux radio module's
runtime loader. OpenSBI only prepares the platform and delegates interrupts;
it does not execute the radio blobs.

The nested `idf_deps/` directory is a build-only, minimal ESP-IDF project.
`make radio-idf-deps` compiles the matching component archives into
`idf_deps/build-radio/`; the resulting ESP-IDF application and second-stage
bootloader are never flashed. `make radio-linux-payload` then relinks the
required archives into `build/esp32s31-radio-fw-v1.o`. This external payload
is packaged unchanged apart from XZ compression in the radio partition; it is
not copied into the kernel tree or statically linked into
`esp32s31-radio.ko`.

`make linux-kbuild` produces the external payload and a generated, explicit
import table for the module loader. The payload uses the small `s31_rtos`
compatibility scheduler instead of FreeRTOS. Its allocators, tasks, queues and
timers are backed by the loader-carved internal HP-SRAM pool managed by the
Linux driver.

The Linux driver is the only execution owner. TIMG1 ticks, deferred radio
interrupts and typed requests from Linux front ends are all consumed by the
`s31-radio` kthread. Every ESP-IDF entry runs with local interrupts masked,
the ILP32F register state owned by the kernel thread, and the synchronous-trap
stack moved into the reserved SRAM tail. Callers must use the typed API in
`include/linux/esp32s31-radio.h`; raw ESP-IDF symbols are not a driver API.

The loader accepts only ELF32 little-endian RISC-V relocatable objects with the
expected firmware ABI. Undefined symbols must be present in the generated
allowlist; the loader never performs an unrestricted kernel symbol lookup.
Bluetooth HCI and cfg80211 operations remain typed requests rather than a
generic function-pointer gateway.

The executable arena is intentionally allocated once and retained at the same
virtual address until reboot. Private ESP-IDF process-lifetime callbacks can
survive a controller shutdown with payload function pointers, so changing the
payload address across module reloads is unsafe. Reload still consumes the
external ELF from the radio partition and performs a fresh copy and relocation;
the payload is not linked into the module, and all Linux-facing runtime state is
torn down normally.

The `boot_*.txt` and `idf_includes.rsp` files capture the ESP-IDF component
link closure used by this target. The Makefile resolves the installed ESP-IDF
and matching picolibc toolchain under `~/.espressif` at build time, and fails
early if either is unavailable. Generated objects and symbol reports are
ignored and can be removed with the top-level `make clean`.
