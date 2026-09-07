#!/usr/bin/env python3
"""Push one compiled DT overlay to a running S31 over its console."""

import argparse
import base64
import time
from pathlib import Path

from s31_hil import open_serial, s31_shell_command, wait_for_shell


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    args = parser.parse_args()

    payload = base64.b64encode(args.source.read_bytes()).decode("ascii")
    serial = open_serial(args.port)
    try:
        if not wait_for_shell(serial, 20.0):
            raise RuntimeError("S31 shell prompt was not detected")
        temporary = "/tmp/s31-overlay-upload.dtbo"
        rc, output = s31_shell_command(serial, f": > {temporary}", 5.0)
        if rc:
            print(output)
            return rc
        for offset in range(0, len(payload), 80):
            chunk = payload[offset:offset + 80]
            command = f"printf %s {chunk} | base64 -d >> {temporary}"
            rc, output = s31_shell_command(serial, command, 5.0)
            if rc:
                print(output)
                return rc
            time.sleep(0.05)
        rc, output = s31_shell_command(serial,
            f"cp {temporary} {args.destination} && rm {temporary} && "
            f"wc -c < {args.destination}", 20.0)
        print(output)
        return rc
    finally:
        serial.close()


if __name__ == "__main__":
    raise SystemExit(main())
