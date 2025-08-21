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
INITIAL_TLE_PATH = os.getenv("INITIAL_TLE_PATH", "initial.tle")

# Mutable current TLE values (populated strictly from file on startup and on changes)
TLE_DRONE1 = None  # type: ignore
TLE_DRONE2 = None  # type: ignore
TLE_INIT1 = None  # type: ignore
TLE_INIT2 = None  # type: ignore

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


def _build_msg(tle1: str, tle2: str, sat_id: str, name: str) -> str:
    return json.dumps(
        {
            "type": "tle_update",
            "id": sat_id,
            "name": name,
            "tle1": tle1,
            "tle2": tle2,
            "timestamp": int(time.time() * 1000),
        }
    )


async def broadcast_tle(tle1: str, tle2: str, sat_id: str, name: str):
    """Send current TLE to all connected clients."""
    if not CLIENTS:
        return
    msg = _build_msg(tle1, tle2, sat_id, name)
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


async def tle_file_watcher(path: str, sat_id: str, name: str, poll_sec: float = 1.0):
    """Watch a TLE file and broadcast whenever the content changes."""
    global TLE_DRONE1, TLE_DRONE2, TLE_INIT1, TLE_INIT2
    if sat_id == "Drone":
        last_content = (TLE_DRONE1, TLE_DRONE2)
    else:
        last_content = (TLE_INIT1, TLE_INIT2)
    last_mtime = None

    while True:
        try:
            stat = os.stat(path)
            mtime = stat.st_mtime
            if last_mtime is None or mtime != last_mtime:
                tle1, tle2 = _read_tle_file(path)
                last_mtime = mtime
                if (tle1, tle2) != last_content:
                    if sat_id == "Drone":
                        TLE_DRONE1, TLE_DRONE2 = tle1, tle2
                        last_content = (TLE_DRONE1, TLE_DRONE2)
                    else:
                        TLE_INIT1, TLE_INIT2 = tle1, tle2
                        last_content = (TLE_INIT1, TLE_INIT2)
                    print(f"[TLE] {name} file change detected → broadcasting update")
                    await broadcast_tle(tle1, tle2, sat_id, name)
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
            if TLE_INIT1 and TLE_INIT2:
                await ws.send(_build_msg(TLE_INIT1, TLE_INIT2, "DroneInitial", "Drone (Initial)")) 
            if TLE_DRONE1 and TLE_DRONE2:
                await ws.send(_build_msg(TLE_DRONE1, TLE_DRONE2, "Drone", "Drone"))
            print(f"[WS] Sent current TLEs to new client ({len(CLIENTS)} total)")
        except Exception:
            pass
        await ws.wait_closed()
    finally:
        try:
            CLIENTS.remove(ws)
        except KeyError:
            pass


async def main():
    # Load initial TLEs from files before accepting connections (no defaults)
    async def wait_for_file_tle(path: str, set_fn, label: str, poll_sec: float = 0.5):
        print(f"[TLE] Waiting for initial TLE at {label} ({path})...")
        while True:
            try:
                tle1, tle2 = _read_tle_file(path)
                set_fn(tle1, tle2)
                print(f"[TLE] Loaded initial TLE for {label} from {path}")
                return
            except Exception:
                await asyncio.sleep(poll_sec)

    def set_drone(t1, t2):
        global TLE_DRONE1, TLE_DRONE2
        TLE_DRONE1, TLE_DRONE2 = t1, t2

    def set_init(t1, t2):
        global TLE_INIT1, TLE_INIT2
        TLE_INIT1, TLE_INIT2 = t1, t2

    await asyncio.gather(
        wait_for_file_tle(INITIAL_TLE_PATH, set_init, "initial"),
        wait_for_file_tle(TLE_PATH, set_drone, "current"),
    )

    server = websockets.serve(client_handler, HOST, PORT)
    async with server:
        # Start watcher task in background
        print(f"[WS] TLE server on ws://{HOST}:{PORT} watching initial={INITIAL_TLE_PATH}, current={TLE_PATH}")
        watcher_initial = asyncio.create_task(tle_file_watcher(INITIAL_TLE_PATH, "DroneInitial", "Drone (Initial)", poll_sec=0.5))
        watcher_current = asyncio.create_task(tle_file_watcher(TLE_PATH, "Drone", "Drone", poll_sec=0.5))
        try:
            await asyncio.Future()  # run forever
        finally:
            watcher_initial.cancel()
            watcher_current.cancel()


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
