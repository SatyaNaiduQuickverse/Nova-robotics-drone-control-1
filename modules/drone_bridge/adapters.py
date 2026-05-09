"""Schema adapters between the phone's spec and drone-control's reality.

Drone-control is treated as a fixed dependency (per project rule: do
not modify it). When its actual API shape differs from what
PROMPT.md §3 documents, the translation lives here — never in
drone-control.

Real divergences caught by the smoke test:

  1. PATH:  /control/mode   → /mode
            (Note: /control/arm and /control/disarm are now NATIVE in
             drone-control api_gateway and do real work — vtx setup +
             FC arming. They are NOT path-rewritten anymore. Only
             /control/mode still rewrites; if /mode ever gets promoted
             to /control/mode in api_gateway, this can go too.)
  2. BODY:  /calibration/motor_test  mode "STANDARD" → "single"
  3. BODY:  /fence/polygon  {"points":[{"lat":x,"lon":y}, ...]}
                          → {"points":[[x, y], ...]}

Per-endpoint timeouts are also set here — the spec's default 5 s
trips a few legitimately slow endpoints (calibration, MAVROS reconnect,
big mission uploads).
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from typing import Any, Optional


log = logging.getLogger("drone_bridge.adapters")


# --- Path rewrites (phone-side path → drone-control path) ----------------

PATH_REWRITES: dict[tuple[str, str], str] = {
    # /control/mode is still phone-side-named differently from drone-control's /mode.
    # No path rewrite for /control/arm or /control/disarm — those exist natively
    # in drone-control now (with virtual_tx orchestration). /control/command also
    # exists natively at the same path with the right shape (throttle/roll/pitch/yaw).
    ("POST", "/control/mode"):   "/mode",
}

# --- Per-endpoint timeouts (seconds) -------------------------------------

# Anything not in here uses DEFAULT_TIMEOUT below.
DEFAULT_TIMEOUT = 5.0

ENDPOINT_TIMEOUTS: dict[str, float] = {
    "/calibration/gyro":           20.0,
    "/calibration/baro":           10.0,
    "/calibration/level_horizon":  20.0,
    "/calibration/compass/start":  20.0,
    "/calibration/accel/start":    10.0,
    "/calibration/accel/position": 30.0,
    "/calibration/accel/simple":   30.0,
    "/calibration/reboot_fc":      10.0,
    "/system/reconnect_mavros":    15.0,
    "/mission":                    15.0,
    "/fence/polygon":              15.0,
    "/fence/config":               10.0,
    "/safety/defaults":            10.0,
}


@dataclass
class AdaptedRequest:
    method:  str
    path:    str
    json:    Any           # decoded JSON body (or None for empty / non-JSON)
    timeout: float


def _adapt_body(path: str, json_body: Any) -> Any:
    """Apply request-body translations. Returns the body shape that
    drone-control expects on the wire."""
    if not isinstance(json_body, dict):
        return json_body

    # /calibration/motor_test: mode "STANDARD" → "single"
    if path == "/calibration/motor_test":
        mode = json_body.get("mode")
        if mode == "STANDARD":
            adapted = dict(json_body)
            adapted["mode"] = "single"
            log.debug("adapter: motor_test mode STANDARD → single")
            return adapted

    # /fence/polygon: [{"lat":x,"lon":y}] → [[x, y]]
    if path == "/fence/polygon":
        points = json_body.get("points")
        if isinstance(points, list) and points and isinstance(points[0], dict):
            try:
                tuples = [[float(p["lat"]), float(p["lon"])] for p in points]
            except (KeyError, TypeError, ValueError):
                return json_body  # let drone-control reject the malformed body
            adapted = dict(json_body)
            adapted["points"] = tuples
            log.debug("adapter: fence.polygon dict-points → array-points (%d pts)",
                      len(tuples))
            return adapted

    return json_body


def adapt(method: str, path: str, body: Optional[bytes]) -> AdaptedRequest:
    """Translate a phone-shaped request into a drone-control-shaped one.

    `method` is the verb string ("GET", "POST", "DELETE", "PUT").
    `body` is raw bytes from the BLE request (or None for empty).
    """
    method = method.upper()

    # Decode JSON if present (most POST/PUT bodies are JSON).
    json_body: Any = None
    if body:
        try:
            json_body = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            # Not JSON — leave as-is. Pass-through endpoints (none today,
            # but reserved for binary uploads later) will see raw bytes.
            json_body = body

    # Path rewrite.
    target_path = PATH_REWRITES.get((method, path), path)

    # Body shape adaptation (keyed off the ORIGINAL phone path, since
    # the schema mismatch was documented against that name).
    json_body = _adapt_body(path, json_body)

    timeout = ENDPOINT_TIMEOUTS.get(target_path,
              ENDPOINT_TIMEOUTS.get(path, DEFAULT_TIMEOUT))

    return AdaptedRequest(method=method, path=target_path,
                          json=json_body, timeout=timeout)
