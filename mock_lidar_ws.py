#!/usr/bin/env python3
import asyncio, json, os, time
import websockets

HOST = os.getenv("HOST", "0.0.0.0")
PORT = int(os.getenv("PORT", "8770"))
TLE1 = os.getenv("DRONE_TLE1", "1 00000U 00000A   25001.00000000  .00000000  00000-0  00000-0 0  9991")
TLE2 = os.getenv("DRONE_TLE2", "2 00000  98.0000  10.0000 0010000  10.0000 350.0000 14.00000000100001")

async def client_handler(ws):
    while True:
        await ws.send(json.dumps({
            "type": "tle_update",
            "id": "Drone",
            "name": "Drone",
            "tle1": TLE1,
            "tle2": TLE2,
            "timestamp": int(time.time() * 1000),
        }))
        await asyncio.sleep(5)

async def main():
    async with websockets.serve(client_handler, HOST, PORT):
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())


































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
<<<<<<< HEAD






















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
=======
>>>>>>> 86c9692b5619842ddd1ef2524c6b4131a876adbc
