"""
Raspberry Pi WebSocket server for Unified Tracker (React client).

Run this on the Pi. The React app connects to ws://<pi-ip>:8765.

How to use with your real program (no simulation):
1) Import Bridge and create one instance (global is fine)
2) Call bridge.push_sync({...}) whenever you have new real data
3) Use the JSON shapes below (the frontend accepts these fields)

JSON payloads (send any subset as you have it):
- Initial TLE: { "initial_tle1": "...", "initial_tle2": "..." }
- New TLE:     { "new_tle1": "...", "new_tle2": "..." }
- Trajectory:  { "trajectory_points": [{"x": km, "y": km, "z": km, "timestamp": ms}, ...] }
- Unified status/telemetry:
  {
    "status": "TRACKING_TARGET",
    "progress": 42,
    "scanner_pos": [x, y, z],               # meters in local scene if you use local view
    "measured_pos": [x, y, z],              # meters (optional)
    "live_telemetry": { "pan_angle": deg, "tilt_angle": deg, "distance": m },
    "position_history": [ { "position": [x, y, z], "timestamp": ms } ],
    "predicted_orbit_params": { ... }       # optional
  }

If you have ECEF positions, prefer sending Trajectory points (km) — they are drawn in the global view.
"""

import asyncio
import json
import signal
import threading
from typing import Any, Dict, List, Optional, Set

import websockets
from pathlib import Path


# ======================== Configuration ========================
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8765


# ===================== Optional geo helpers =====================
# Use these only if you need to convert sensor readings to ECEF.
import math

EARTH_RADIUS_KM = 6371.0


def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_km: float = 0.0):
    lat = deg2rad(lat_deg)
    lon = deg2rad(lon_deg)
    r = EARTH_RADIUS_KM + alt_km
    x = r * math.cos(lat) * math.cos(lon)
    y = r * math.cos(lat) * math.sin(lon)
    z = r * math.sin(lat)
    return x, y, z


def enu_basis_at(lat_deg: float, lon_deg: float):
    lat = deg2rad(lat_deg)
    lon = deg2rad(lon_deg)
    E = (-math.sin(lon), math.cos(lon), 0.0)
    N = (-math.sin(lat) * math.cos(lon), -math.sin(lat) * math.sin(lon), math.cos(lat))
    U = (math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat))
    return E, N, U


def lidar_spherical_to_ecef_delta(
    range_m: float,
    az_deg_from_north_cw: float,
    el_deg: float,
    lat_deg: float,
    lon_deg: float,
):
    range_km = max(0.0, float(range_m)) / 1000.0
    az = deg2rad(az_deg_from_north_cw)
    el = deg2rad(el_deg)
    E, N, U = enu_basis_at(lat_deg, lon_deg)

    east = math.cos(el) * math.sin(az)
    north = math.cos(el) * math.cos(az)
    up = math.sin(el)

    dx = range_km * (east * E[0] + north * N[0] + up * U[0])
    dy = range_km * (east * E[1] + north * N[1] + up * U[1])
    dz = range_km * (east * E[2] + north * N[2] + up * U[2])
    return dx, dy, dz


# ======================= Bridge implementation =======================
class Bridge:
    """Thread/async-safe channel to broadcast real data to React clients."""

    def __init__(self) -> None:
        self._clients: Set[websockets.WebSocketServerProtocol] = set()
        self._queue: "asyncio.Queue[Dict[str, Any]]" = asyncio.Queue()
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def set_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    async def register(self, ws: websockets.WebSocketServerProtocol) -> None:
        self._clients.add(ws)

    async def unregister(self, ws: websockets.WebSocketServerProtocol) -> None:
        self._clients.discard(ws)

    async def push(self, payload: Dict[str, Any]) -> None:
        """Async: enqueue payload to be sent to all clients."""
        await self._queue.put(payload)

    def push_sync(self, payload: Dict[str, Any]) -> None:
        """Sync/thread-safe enqueue. Call this from your real code."""
        if not self._loop:
            raise RuntimeError("Bridge loop not set yet. Call after server starts.")
        asyncio.run_coroutine_threadsafe(self.push(payload), self._loop)

    async def _broadcast_loop(self) -> None:
        while True:
            payload = await self._queue.get()
            if payload is None:
                continue
            message = json.dumps(payload)
            if not self._clients:
                continue
            dead: List[websockets.WebSocketServerProtocol] = []
            for ws in list(self._clients):
                try:
                    await ws.send(message)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                await self.unregister(ws)


bridge = Bridge()


# ========================= Run control (UI cmds) =========================
class RunControl:
    """Shared control signals for firmware run lifecycle within same process.

    Firmware can import this and coordinate start/stop/restart with the UI.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._start_event = threading.Event()
        self._stop_event = threading.Event()
        self._running = False

    # UI → Firmware
    def request_start(self) -> None:
        with self._lock:
            self._stop_event.clear()
            self._start_event.set()

    def request_stop(self) -> None:
        with self._lock:
            self._stop_event.set()

    def request_restart(self) -> None:
        with self._lock:
            self._stop_event.set()
            self._start_event.set()

    # Firmware side helpers
    def wait_for_start_blocking(self) -> None:
        self._start_event.wait()
        with self._lock:
            self._running = True

    def clear_start(self) -> None:
        with self._lock:
            self._start_event.clear()

    def is_stop_requested(self) -> bool:
        return self._stop_event.is_set()

    def acknowledge_stopped(self) -> None:
        with self._lock:
            self._running = False
            self._stop_event.clear()
            # Leave start_event state as-is; if set, firmware can start again

    def is_running(self) -> bool:
        with self._lock:
            return self._running


control = RunControl()


# ===================== WebSocket server handler =====================
async def handle_client(ws: websockets.WebSocketServerProtocol):
    await bridge.register(ws)
    try:
        async for msg in ws:
            # Handle commands from UI (optional)
            try:
                data = json.loads(msg)
            except Exception:
                continue

            cmd = (data.get("command") or data.get("cmd") or "").lower()
            if cmd == "start":
                control.request_start()
                # Inform UI
                await bridge.push({
                    "status": "STANDBY",
                    "warning": "Start requested by UI"
                })
            elif cmd == "stop":
                control.request_stop()
                await bridge.push({
                    "status": "STANDBY",
                    "warning": "Stop requested by UI"
                })
            elif cmd in ("restart", "reset"):
                control.request_restart()
                await bridge.push({
                    "status": "STANDBY",
                    "warning": "Restart requested by UI"
                })
            elif cmd == "reset":
                # TODO: trigger your program reset
                pass
            else:
                # Custom commands if needed
                pass
    finally:
        await bridge.unregister(ws)


async def start_server(run_forever: bool = False):
    """Start the WebSocket server. If run_forever=True, block the task forever.

    Import and await this from your program if you already use asyncio.
    Otherwise, see the thread-based example below.
    """
    loop = asyncio.get_running_loop()
    bridge.set_loop(loop)

    # Start broadcast task
    asyncio.create_task(bridge._broadcast_loop())

    server = await websockets.serve(
        handle_client, SERVER_HOST, SERVER_PORT, ping_interval=20, ping_timeout=20
    )
    print(f"RPi WS server listening on ws://{SERVER_HOST}:{SERVER_PORT}")

    if run_forever:
        await asyncio.Future()
    return server


async def main() -> None:
    # Start in standalone mode and handle signals (main thread only)
    server_task = asyncio.create_task(start_server(run_forever=False))
    await server_task

    # Keep running until interrupted
    stop = asyncio.Future()
    loop = asyncio.get_running_loop()
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, lambda s=sig: stop.set_result(True))
    except NotImplementedError:
        # Signal handlers may not be supported (e.g., on Windows or in threads)
        pass
    await stop


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass


