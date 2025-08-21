#!/usr/bin/env python3
"""
WebSocket server that streams 3D point data (x, y, z, timestamp).

Two modes:
1) Generated from polar/spherical measurements (range, azimuth, elevation)
2) Read from a file containing x,y,z,timestamp records

Environment variables (optional):
  HOST           Bind address (default: 0.0.0.0)
  PORT           Port (default: 8765)
  HZ             Messages per second (default: 20)
  MODE           moving | random (default: moving)  [only used if DATA_FILE unset]
  MIN_RANGE_M    Minimum range in meters (default: 0.5)
  MAX_RANGE_M    Maximum range in meters (default: 12.0)
  DATA_FILE      Path to CSV/JSON/JSONL file with x,y,z,timestamp
  LOOP           If "1", loop the file when reaching EOF (default: 1)

Coordinate convention (for generated mode):
  - Y is up.
  - Azimuth rotates around +Y axis, with 0° along +X and increasing toward +Z.
  - Elevation is angle above the XZ plane (0° at horizon, +90° straight up).
  - Conversion: x = r*cos(el)*cos(az), z = r*cos(el)*sin(az), y = r*sin(el)

Requires: pip install websockets
"""

import asyncio
import json
import csv
import math
import os
import random
import time

try:
    import websockets  # type: ignore
except ImportError as exc:
    raise SystemExit("Please install dependency: pip install websockets") from exc


HOST = os.getenv("HOST", "0.0.0.0")
PORT = int(os.getenv("PORT", "8765"))
HZ = float(os.getenv("HZ", "20"))  # messages per second
MODE = os.getenv("MODE", "moving").strip().lower()  # moving | random

# Match the UI's default sphere radius of ~12 units as meters
MIN_RANGE_M = float(os.getenv("MIN_RANGE_M", "0.5"))
MAX_RANGE_M = float(os.getenv("MAX_RANGE_M", "12.0"))

DATA_FILE = os.getenv("DATA_FILE")
LOOP = os.getenv("LOOP", "1") in ("1", "true", "True", "yes", "YES")


def spherical_to_cartesian(range_m: float, az_deg: float, el_deg: float) -> tuple[float, float, float]:
    az = math.radians(az_deg)
    el = math.radians(el_deg)
    cos_el = math.cos(el)
    x = range_m * cos_el * math.cos(az)
    z = range_m * cos_el * math.sin(az)
    y = range_m * math.sin(el)
    return x, y, z


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def generate_reading_moving(t: float) -> dict:
    base_az = (t * 12.0) % 360.0  # degrees per second around Y
    el = 10.0 + 7.5 * math.sin(t * 0.25)
    rng = 6.0 + 3.0 * math.sin(t * 0.18)
    rng = clamp(rng, MIN_RANGE_M, MAX_RANGE_M)
    x, y, z = spherical_to_cartesian(rng, base_az, clamp(el, 0.0, 85.0))
    return {
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "timestamp": int(time.time() * 1000),
    }


def generate_reading_random(_: float) -> dict:
    az = random.uniform(0.0, 360.0)
    el = random.uniform(2.0, 60.0)
    rng = random.uniform(MIN_RANGE_M, MAX_RANGE_M)
    x, y, z = spherical_to_cartesian(rng, az, el)
    return {
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "timestamp": int(time.time() * 1000),
    }


async def client_handler(ws):
    print(f"[WS] Client connected: {ws.remote_address}")
    t0 = time.time()
    try:
        if DATA_FILE:
            async for reading in stream_points_from_file(DATA_FILE, hz=HZ, loop_file=LOOP):
                await ws.send(json.dumps(reading))
        else:
            while True:
                t = time.time() - t0
                reading = (
                    generate_reading_moving(t)
                    if MODE == "moving"
                    else generate_reading_random(t)
                )
                await ws.send(json.dumps(reading))
                await asyncio.sleep(max(0.0, 1.0 / HZ))
    except websockets.ConnectionClosed:
        print("[WS] Client disconnected")


async def main():
    if DATA_FILE:
        print(f"[WS] Starting point file server on ws://{HOST}:{PORT} (file='{DATA_FILE}', {HZ} Hz, loop={LOOP})")
    else:
        print(f"[WS] Starting polar point server on ws://{HOST}:{PORT} (mode={MODE}, {HZ} Hz)")
    async with websockets.serve(client_handler, HOST, PORT):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass


# ------------------------------ File streaming ------------------------------ #

def _as_point(obj: dict) -> dict | None:
    if not isinstance(obj, dict):
        return None
    x = obj.get("x") if "x" in obj else obj.get("X")
    y = obj.get("y") if "y" in obj else obj.get("Y")
    z = obj.get("z") if "z" in obj else obj.get("Z")
    ts = obj.get("timestamp")
    if ts is None:
        ts = obj.get("time") if "time" in obj else obj.get("t") or obj.get("T")
    try:
        px = float(x)
        py = float(y)
        pz = float(z)
        pt = int(float(ts))
    except Exception:
        return None
    return {"x": px, "y": py, "z": pz, "timestamp": pt}


def _iter_points_from_csv(path: str):
    with open(path, "r", encoding="utf-8") as f:
        try:
            # Try DictReader with headers
            reader = csv.DictReader(f)
            # If fieldnames are None, there was no header; fall back below
            if reader.fieldnames and any(h.lower() in ("x", "y", "z", "timestamp") for h in reader.fieldnames):
                for row in reader:
                    pt = _as_point(row)
                    if pt:
                        yield pt
                return
        except Exception:
            pass

    # Fallback: simple CSV with 4 columns: x,y,z,timestamp (optional header skipped)
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) != 4:
                # Skip non data lines (e.g., a header)
                continue
            try:
                x, y, z, ts = float(parts[0]), float(parts[1]), float(parts[2]), int(float(parts[3]))
            except Exception:
                continue
            yield {"x": x, "y": y, "z": z, "timestamp": ts}


def _iter_points_from_json(path: str):
    # Try JSON Lines first
    try:
        with open(path, "r", encoding="utf-8") as f:
            any_line = False
            for line in f:
                s = line.strip()
                if not s:
                    continue
                any_line = True
                try:
                    obj = json.loads(s)
                except Exception:
                    # Not JSONL; fall back to array mode below
                    any_line = False
                    break
                pt = _as_point(obj)
                if pt:
                    yield pt
            if any_line:
                return
    except Exception:
        pass

    # Try full JSON array
    try:
        with open(path, "r", encoding="utf-8") as f:
            payload = json.load(f)
        if isinstance(payload, dict) and isinstance(payload.get("points"), list):
            for obj in payload["points"]:
                pt = _as_point(obj)
                if pt:
                    yield pt
        elif isinstance(payload, list):
            for obj in payload:
                pt = _as_point(obj)
                if pt:
                    yield pt
    except Exception:
        return


def _iter_points(path: str):
    lower = path.lower()
    if lower.endswith(".csv"):
        yield from _iter_points_from_csv(path)
    elif lower.endswith(".json") or lower.endswith(".jsonl") or lower.endswith(".ndjson"):
        yield from _iter_points_from_json(path)
    else:
        # Try CSV first, then JSON as a fallback
        yield from _iter_points_from_csv(path)
        yield from _iter_points_from_json(path)


async def stream_points_from_file(path: str, hz: float, loop_file: bool = True):
    delay = max(0.0, 1.0 / hz) if hz > 0 else 0.0
    while True:
        any_sent = False
        for pt in _iter_points(path):
            any_sent = True
            yield pt
            if delay:
                await asyncio.sleep(delay)
        if not loop_file:
            break
        # If file contained no points, avoid tight loop
        if not any_sent:
            await asyncio.sleep(0.5)


