"""Shared in-memory snapshot of the drone state.

Filled by the pump's polling threads, read by both the BLE digest path
and the debug HTTP endpoint. One process, one lock, one dataclass.

Treat this as the single source of truth for "what does the drone look
like right now." Anything that needs telemetry imports `state` from here
and grabs `state_lock` for the read.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class Snapshot:
    # --- Drone-control half (filled by pump.poll_drone) ---
    armed:        bool = False
    mode:         str = ""
    connected:    bool = False
    fix_type:     int = 0
    home_set:     bool = False

    voltage_v:    float = 0.0
    battery_pct:  float = 0.0    # 0..1 from MAVROS, scaled to 0..100 at pack time

    lat:          float = 0.0
    lon:          float = 0.0
    alt_amsl_m:   float = 0.0
    alt_rel_m:    float = 0.0

    ground_speed_mps: float = 0.0
    heading_deg:  float = 0.0
    roll_rad:     float = 0.0
    pitch_rad:    float = 0.0

    drone_last_ts: Optional[float] = None
    drone_errors:  int = 0

    # --- ELRS half (filled by pump.poll_elrs) ---
    rssi_dbm:     int = 0
    uplink_lq:    int = 0
    elrs_last_ts: Optional[float] = None
    elrs_errors:  int = 0


state = Snapshot()
state_lock = threading.Lock()
started_at = time.monotonic()


def uptime_s() -> float:
    return time.monotonic() - started_at


def age_ms(ts: Optional[float]) -> Optional[int]:
    if ts is None:
        return None
    return int((time.monotonic() - ts) * 1000)
