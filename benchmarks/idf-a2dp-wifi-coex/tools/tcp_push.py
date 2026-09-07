#!/usr/bin/env python3
"""Push a deterministic byte stream into the S31 IDF coexist benchmark."""

from __future__ import annotations

import argparse
import hashlib
import socket
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("host")
    parser.add_argument("--port", type=int, default=3333)
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--chunk", type=int, default=64 * 1024)
    args = parser.parse_args()

    block = bytes((i * 17 + 23) & 0xFF for i in range(args.chunk))
    digest = hashlib.sha256()
    sent = 0
    started = time.perf_counter()
    deadline = started + args.seconds

    with socket.create_connection((args.host, args.port), timeout=10) as sock:
        sock.settimeout(10)
        while time.perf_counter() < deadline:
            sock.sendall(block)
            digest.update(block)
            sent += len(block)

    elapsed = time.perf_counter() - started
    print(
        f"sent={sent} elapsed={elapsed:.3f}s "
        f"throughput={sent * 8 / elapsed / 1_000_000:.3f}Mbps "
        f"sha256={digest.hexdigest()}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
