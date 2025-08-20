#!/usr/bin/env python3
"""
Update the TLE file watched by mock_lidar_ws.py to trigger live Drone updates.

This script alternates between two example TLEs every few seconds and writes
them atomically to the target file so the WebSocket server broadcasts changes.

Usage:
  python scripts/tle_writer.py

Env vars:
  TLE_PATH   Path to the TLE file to write (default: ./drone.tle)
  INTERVAL_S Interval in seconds between updates (default: 10)
"""

import os
import tempfile
import time


def write_tle_atomic(path: str, tle1: str, tle2: str) -> None:
    directory = os.path.dirname(path) or "."
    fd, tmp = tempfile.mkstemp(prefix="tle_", dir=directory, text=True)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(tle1.rstrip() + "\n")
            f.write(tle2.rstrip() + "\n")
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except Exception:
        try:
            os.remove(tmp)
        except OSError:
            pass
        raise


EXAMPLE_TLES = [
    (
        # Example A (synthetic-like values)
        "1 00005U 58002B   25020.45269444  .00001234  00000-0  12345-4 0  9991",
        "2 00005  34.5678 123.4567 0012345  12.3456 345.6789 14.12345678123456",
    ),
    (
        # Example B (slightly different orbit to visualize changes)
        "1 00005U 58002B   25020.55555555  .00002345  00000-0  22345-4 0  9992",
        "2 00005  37.0000 180.0000 0020000  45.0000 300.0000 14.30000000100000",
    ),
]


def main() -> None:
    tle_path = os.getenv("TLE_PATH", os.path.join(os.getcwd(), "drone.tle"))
    interval_s = float(os.getenv("INTERVAL_S", "10"))
    idx = 0
    print(f"Writing alternating TLEs to {tle_path} every {interval_s}s. Press Ctrl+C to stop.")
    try:
        while True:
            tle1, tle2 = EXAMPLE_TLES[idx % len(EXAMPLE_TLES)]
            write_tle_atomic(tle_path, tle1, tle2)
            print(f"Wrote TLE set #{idx % len(EXAMPLE_TLES)} at {time.strftime('%X')}")
            idx += 1
            time.sleep(interval_s)
    except KeyboardInterrupt:
        print("Stopped.")


if __name__ == "__main__":
    main()


