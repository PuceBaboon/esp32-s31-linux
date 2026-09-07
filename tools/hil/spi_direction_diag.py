#!/usr/bin/env python3
"""Isolate S31 SPI TX and RX paths against the P4 HIL peer."""

import argparse
import re
import sys
import time
import zlib

import s31_hil as hil


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("direction", choices=("tx", "rx", "full"))
    parser.add_argument("--repeat", type=int, default=20)
    parser.add_argument("--mode", type=int, default=2)
    parser.add_argument("--peer-mode", type=int, choices=(0, 1, 2, 3),
                        help="P4 slave mode override for fixture diagnosis")
    parser.add_argument("--speed", type=int, default=20_000_000)
    parser.add_argument("--length", type=int, default=4096)
    parser.add_argument("--rsck", type=int, choices=(0, 1))
    parser.add_argument("--tsck", type=int, choices=(0, 1))
    parser.add_argument("--clk13", type=int, choices=(0, 1))
    parser.add_argument("--controller", choices=("gpspi2", "gpspi3", "both"),
                        default="both")
    args = parser.parse_args()

    expected_bits = args.length * 8
    host_tx_crc = zlib.crc32(bytes((i * 37 + 11) & 0xff
                                  for i in range(args.length)))
    peer_tx_crc = zlib.crc32(bytes((i ^ 0xa5) & 0xff
                                  for i in range(args.length)))

    s31 = hil.open_serial("/dev/ttyUSB0")
    p4 = hil.open_serial("COM6")
    overlays = (("gpspi2", "/dev/spidev2.0"),
                ("gpspi3", "/dev/spidev3.0"))
    if args.controller != "both":
        overlays = tuple(pair for pair in overlays
                         if pair[0] == args.controller)
    passed = 0
    total = 0
    try:
        if not hil.wait_for_shell(s31, 30.0):
            raise RuntimeError("S31 shell prompt was not detected")
        hello = hil.p4_command(p4, "hello", "rpc.hello")
        match = re.search(r"token=([0-9a-fA-F]+)", hello.detail)
        if match is None:
            raise RuntimeError("P4 arm token missing")
        token = match.group(1)
        print(f"P4 {hello.detail}")

        for overlay, device in overlays:
            for stale, _ in overlays:
                hil.s31_shell_command(
                    s31, f"s31-overlay remove {stale} --volatile "
                    ">/dev/null 2>&1 || true")
            rc, output = hil.s31_shell_command(
                s31,
                f"s31-overlay apply {overlay} {overlay}.sclk=42 "
                f"{overlay}.mosi=43 {overlay}.miso=45 --volatile")
            if rc:
                print(f"FAIL {overlay} overlay: {output}")
                continue

            board_passed = 0
            for iteration in range(1, args.repeat + 1):
                hil.p4_command(p4, f"arm {token}", "safety.arm")
                edge_args = ""
                if any(value is not None for value in
                       (args.rsck, args.tsck, args.clk13)):
                    edge_args = (f" {args.rsck if args.rsck is not None else -1}"
                                 f" {args.tsck if args.tsck is not None else -1}"
                                 f" {args.clk13 if args.clk13 is not None else -1}")
                peer_mode = args.mode if args.peer_mode is None else args.peer_mode
                hil.p4_command(
                    p4,
                    f"spi-start {peer_mode} {args.length} {args.speed}"
                    f"{edge_args}",
                    "peer.spi-start")
                time.sleep(0.2)
                tool_case = "spi" if args.direction == "full" else \
                            f"spi-{args.direction}"
                rc, output = hil.s31_shell_command(
                    s31,
                    f"s31-hil-io {tool_case} {device} "
                    f"{args.mode} {args.speed} {args.length}",
                    20.0)
                time.sleep(0.1)
                report = hil.p4_command(p4, "spi-report", "peer.spi-report")
                hil.p4_command(p4, "peer-stop", "peer.stop")

                if args.direction in ("tx", "full"):
                    peer_ok = (f"first_bits={expected_bits}" in report.detail and
                               f"last_bits={expected_bits}" in report.detail and
                               f"rx_crc32={host_tx_crc:08x}" in report.detail)
                else:
                    # The P4 still records its MOSI input, but RX-only leaves
                    # that input idle.  The S31-side pattern check is decisive.
                    peer_ok = (f"first_bits={expected_bits}" in report.detail and
                               f"last_bits={expected_bits}" in report.detail)
                if args.direction in ("rx", "full"):
                    peer_ok = peer_ok and \
                              f"tx_crc32={peer_tx_crc:08x}" in report.detail
                host_ok = rc == 0 and f"PASS {tool_case}" in output
                ok = host_ok and peer_ok
                total += 1
                passed += int(ok)
                board_passed += int(ok)
                print(f"{'PASS' if ok else 'FAIL'} {overlay} "
                      f"{args.direction} {iteration}/{args.repeat}: "
                      f"host_rc={rc} peer={report.detail} "
                      f"host={' | '.join(output.splitlines()[-2:])}")
            print(f"SUMMARY {overlay} {board_passed}/{args.repeat}")
            hil.s31_shell_command(
                s31, f"s31-overlay remove {overlay} --volatile")
    finally:
        try:
            hil.p4_command(p4, "peer-stop", "peer.stop")
        except Exception:
            pass
        for overlay, _ in overlays:
            try:
                hil.s31_shell_command(
                    s31, f"s31-overlay remove {overlay} --volatile "
                    ">/dev/null 2>&1 || true", 3.0)
            except Exception:
                pass
        p4.close()
        s31.close()
    print(f"TOTAL {passed}/{total}")
    return 0 if total and passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
