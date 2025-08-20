#!/usr/bin/env python3
import asyncio
import json
import os
import time
from typing import Tuple

import websockets

HOST = os.getenv("HOST", "0.0.0.0")
PORT = int(os.getenv("PORT", "8770"))

# Optional: path to a TLE file that another process updates.
# The file should contain exactly two lines: TLE1 and TLE2.
TLE_PATH = os.getenv("TLE_PATH", "drone.tle")

# Mutable current TLE values (populated strictly from file on startup and on changes)
TLE1 = None  # type: ignore
TLE2 = None  # type: ignore

CLIENTS = set()


def _read_tle_file(path: str) -> Tuple[str, str]:
    """Read two-line TLE from file. Returns (tle1, tle2).
    Raises FileNotFoundError or ValueError if not valid.
    """
    with open(path, "r", encoding="utf-8") as f:
        lines = [ln.strip() for ln in f.read().splitlines() if ln.strip()]
    if len(lines) < 2:
        raise ValueError("TLE file must contain at least two non-empty lines")
    return lines[0], lines[1]


def _build_msg(tle1: str, tle2: str) -> str:
    return json.dumps(
        {
            "type": "tle_update",
            "id": "Drone",
            "name": "Drone",
            "tle1": tle1,
            "tle2": tle2,
            "timestamp": int(time.time() * 1000),
        }
    )


async def broadcast_tle(tle1: str, tle2: str):
    """Send current TLE to all connected clients."""
    if not CLIENTS:
        return
    msg = _build_msg(tle1, tle2)
    to_remove = []
    for ws in list(CLIENTS):
        try:
            await ws.send(msg)
        except Exception:
            to_remove.append(ws)
    for ws in to_remove:
        try:
            CLIENTS.remove(ws)
        except KeyError:
            pass


async def tle_file_watcher(path: str, poll_sec: float = 1.0):
    """Watch a TLE file and broadcast whenever the content changes."""
    global TLE1, TLE2
    last_content = (TLE1, TLE2)
    last_mtime = None

    while True:
        try:
            stat = os.stat(path)
            mtime = stat.st_mtime
            if last_mtime is None or mtime != last_mtime:
                tle1, tle2 = _read_tle_file(path)
                last_mtime = mtime
                if (tle1, tle2) != last_content:
                    TLE1, TLE2 = tle1, tle2
                    last_content = (tle1, tle2)
                    print(f"[TLE] File change detected → broadcasting update")
                    await broadcast_tle(TLE1, TLE2)
        except FileNotFoundError:
            # File not present yet; ignore and retry
            pass
        except Exception:
            # Ignore malformed/partial writes and retry next tick
            pass
        await asyncio.sleep(poll_sec)


async def client_handler(ws):
    CLIENTS.add(ws)
    try:
        # Send the current TLE immediately to the connecting client only
        try:
            await ws.send(_build_msg(TLE1, TLE2))
            print(f"[WS] Sent current TLE to new client ({len(CLIENTS)} total)")
        except Exception:
            pass
        await ws.wait_closed()
    finally:
        try:
            CLIENTS.remove(ws)
        except KeyError:
            pass


async def main():
    # Load initial TLEs from file before accepting connections (no defaults)
    async def wait_for_initial_tle(path: str, poll_sec: float = 0.5):
        global TLE1, TLE2
        print(f"[TLE] Waiting for initial TLE at {path}...")
        while True:
            try:
                tle1, tle2 = _read_tle_file(path)
                TLE1, TLE2 = tle1, tle2
                print(f"[TLE] Loaded initial TLE from {path}")
                return
            except Exception:
                await asyncio.sleep(poll_sec)

    await wait_for_initial_tle(TLE_PATH)

    server = websockets.serve(client_handler, HOST, PORT)
    async with server:
        # Start watcher task in background
        print(f"[WS] TLE server on ws://{HOST}:{PORT} watching {TLE_PATH}")
        watcher = asyncio.create_task(tle_file_watcher(TLE_PATH, poll_sec=0.5))
        try:
            await asyncio.Future()  # run forever
        finally:
            watcher.cancel()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass


































# #!/usr/bin/env python3
# import asyncio
# import json
# import math
# import os
# import random
# import time

# try:
#     import websockets  # pip install websockets
# except ImportError:
#     raise SystemExit("Please install dependency: pip install websockets")

# HOST = os.getenv("HOST", "0.0.0.0")
# PORT = int(os.getenv("PORT", "8765"))
# HZ = float(os.getenv("HZ", "10"))  # messages per second
# MODE = os.getenv("MODE", "moving")  # moving | random

# # Ranges typical for a short/medium range sensor (adjust as desired)
# MIN_RANGE_M = float(os.getenv("MIN_RANGE_M", "200"))
# MAX_RANGE_M = float(os.getenv("MAX_RANGE_M", "1500"))


# def generate_reading_moving(t: float) -> dict:
#     # Smoothly varying azimuth/elevation and range for a stable moving target
#     base_az = (t * 12.0) % 360.0  # deg/s
#     el = 10.0 + 7.5 * math.sin(t * 0.25)
#     rng = 900.0 + 350.0 * math.sin(t * 0.18)
#     return {
#         "timestamp": int(time.time() * 1000),
#         "range_m": float(max(MIN_RANGE_M, min(MAX_RANGE_M, rng))),
#         "az_deg": float(base_az),
#         "el_deg": float(max(0.1, min(85.0, el))),
#     }


# def generate_reading_random(_: float) -> dict:
#     return {
#         "timestamp": int(time.time() * 1000),
#         "range_m": float(random.uniform(MIN_RANGE_M, MAX_RANGE_M)),
#         "az_deg": float(random.uniform(0.0, 360.0)),
#         "el_deg": float(random.uniform(2.0, 60.0)),
#     }


# async def client_handler(ws):
#     print(f"Client connected: {ws.remote_address}")
#     t0 = time.time()
#     try:
#         while True:
#             t = time.time() - t0
#             reading = (
#                 generate_reading_moving(t)
#                 if MODE == "moving"
#                 else generate_reading_random(t)
#             )
#             await ws.send(json.dumps(reading))
#             await asyncio.sleep(max(0.0, 1.0 / HZ))
#     except websockets.ConnectionClosed:
#         print("Client disconnected")


# async def main():
#     print(f"Starting mock LiDAR WebSocket on ws://{HOST}:{PORT} (mode={MODE}, {HZ} Hz)")
#     async with websockets.serve(client_handler, HOST, PORT):
#         await asyncio.Future()  # run forever


# if __name__ == "__main__":
#     try:
#         asyncio.run(main())
#     except KeyboardInterrupt:
#         pass






















# #!/usr/bin/env python3
# import asyncio
# import json
# import math
# import os
# import random
# import time

# try:
#     import websockets  # pip install websockets
# except ImportError:
#     raise SystemExit("Please install dependency: pip install websockets")

# HOST = os.getenv("HOST", "0.0.0.0")
# PORT = int(os.getenv("PORT", "8765"))
# HZ = float(os.getenv("HZ", "10"))  # messages per second
# MODE = os.getenv("MODE", "moving")  # moving | random

# # Ranges typical for a short/medium range sensor (adjust as desired)
# MIN_RANGE_M = float(os.getenv("MIN_RANGE_M", "200"))
# MAX_RANGE_M = float(os.getenv("MAX_RANGE_M", "1500"))


# def generate_reading_moving(t: float) -> dict:
#     # Smoothly varying azimuth/elevation and range for a stable moving target
#     base_az = (t * 12.0) % 360.0  # deg/s
#     el = 10.0 + 7.5 * math.sin(t * 0.25)
#     rng = 900.0 + 350.0 * math.sin(t * 0.18)
#     return {
#         "timestamp": int(time.time() * 1000),
#         "range_m": float(max(MIN_RANGE_M, min(MAX_RANGE_M, rng))),
#         "az_deg": float(base_az),
#         "el_deg": float(max(0.1, min(85.0, el))),
#     }


# def generate_reading_random(_: float) -> dict:
#     return {
#         "timestamp": int(time.time() * 1000),
#         "range_m": float(random.uniform(MIN_RANGE_M, MAX_RANGE_M)),
#         "az_deg": float(random.uniform(0.0, 360.0)),
#         "el_deg": float(random.uniform(2.0, 60.0)),
#     }


# async def client_handler(ws):
#     print(f"Client connected: {ws.remote_address}")
#     t0 = time.time()
#     try:
#         while True:
#             t = time.time() - t0
#             reading = (
#                 generate_reading_moving(t)
#                 if MODE == "moving"
#                 else generate_reading_random(t)
#             )
#             await ws.send(json.dumps(reading))
#             await asyncio.sleep(max(0.0, 1.0 / HZ))
#     except websockets.ConnectionClosed:
#         print("Client disconnected")


# async def main():
#     print(f"Starting mock LiDAR WebSocket on ws://{HOST}:{PORT} (mode={MODE}, {HZ} Hz)")
#     async with websockets.serve(client_handler, HOST, PORT):
#         await asyncio.Future()  # run forever


# if __name__ == "__main__":
#     try:
#         asyncio.run(main())
#     except KeyboardInterrupt:
#         pass
