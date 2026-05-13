"""Shared HTTP client for reading drone-control's /telemetry/* endpoints.

Producers (battery / gps / attitude / flight_mode) all need to poll
drone-control over HTTP and degrade gracefully when:

  - drone-control is down or restarting
  - the FC has just rebooted and values are momentarily missing
  - a single field is None (e.g. GPS lat/lon during fix acquisition)
  - the network call times out

This module wraps `urllib.request` with sane defaults: short timeout,
no internal retries (the producer's outer loop already polls at a
fixed rate), and a `last_error` field producers can use to log
"unreachable" only on transitions rather than every iteration.

Methods return:
  - dict on success (decoded JSON)
  - None on any failure (timeout, HTTP error, connection refused,
    invalid JSON). The caller MUST handle None — never assume success.

No external dependencies. Pure stdlib.
"""
from __future__ import annotations

import json
import os
import time
import urllib.error
import urllib.request
from typing import Optional


DEFAULT_BASE_URL = os.environ.get("DRONE_CONTROL_URL", "http://localhost:8080")
DEFAULT_TIMEOUT_S = float(os.environ.get("FC_CLIENT_TIMEOUT_S", "0.5"))


class FcClient:
    """Reads drone-control HTTP telemetry. Thread-safe (urllib + no shared state).

    Tracks `last_error` (str | None) and `consecutive_failures` (int)
    so producers can log "FC unreachable" only on transitions and
    surface a clean health signal.
    """

    def __init__(
        self,
        base_url: str = DEFAULT_BASE_URL,
        timeout_s: float = DEFAULT_TIMEOUT_S,
    ):
        self.base_url = base_url.rstrip("/")
        self.timeout_s = timeout_s
        self.last_error: Optional[str] = None
        self.consecutive_failures: int = 0
        self.consecutive_successes: int = 0

    def _get(self, path: str) -> Optional[dict]:
        url = f"{self.base_url}{path}"
        try:
            with urllib.request.urlopen(url, timeout=self.timeout_s) as resp:
                if resp.status != 200:
                    self._fail(f"http {resp.status} on {path}")
                    return None
                body = resp.read()
            data = json.loads(body)
            if not isinstance(data, dict):
                self._fail(f"non-dict response on {path}: {type(data).__name__}")
                return None
            self._ok()
            return data
        except urllib.error.HTTPError as e:
            self._fail(f"http error {e.code} on {path}")
            return None
        except urllib.error.URLError as e:
            self._fail(f"url error on {path}: {e.reason}")
            return None
        except TimeoutError:
            self._fail(f"timeout ({self.timeout_s}s) on {path}")
            return None
        except (json.JSONDecodeError, ValueError) as e:
            self._fail(f"json decode failed on {path}: {e}")
            return None
        except Exception as e:
            # Defensive — never let an unexpected error kill the producer.
            self._fail(f"unexpected {type(e).__name__} on {path}: {e}")
            return None

    def _ok(self):
        self.last_error = None
        self.consecutive_failures = 0
        self.consecutive_successes += 1

    def _fail(self, msg: str):
        self.last_error = msg
        self.consecutive_failures += 1
        self.consecutive_successes = 0

    # --- typed accessors --------------------------------------------------
    # Each returns None on any failure. Producers must handle None.

    def get_battery(self) -> Optional[dict]:
        """{voltage, current, percentage}. percentage is 0..1 fraction."""
        return self._get("/telemetry/battery")

    def get_gps(self) -> Optional[dict]:
        """{fix_type, latitude, longitude, altitude, satellites, hdop}.
        fix_type=-1 / satellites=0 indicates no GPS lock."""
        return self._get("/telemetry/gps")

    def get_vfr_hud(self) -> Optional[dict]:
        """{airspeed, groundspeed, heading, throttle, altitude, climb_rate}.
        heading in degrees 0..360, groundspeed in m/s."""
        return self._get("/telemetry/vfr_hud")

    def get_orientation(self) -> Optional[dict]:
        """{roll, pitch, yaw} — radians."""
        return self._get("/telemetry/orientation")

    def get_state(self) -> Optional[dict]:
        """{connected, armed, mode, system_status}. mode is the
        ArduPilot mode string."""
        return self._get("/telemetry/state")


# --- module-level convenience helpers -------------------------------------

def is_valid_battery(b: Optional[dict]) -> bool:
    """True if all three fields present and numeric (not None)."""
    if not isinstance(b, dict):
        return False
    return all(
        isinstance(b.get(k), (int, float))
        for k in ("voltage", "current", "percentage")
    )


def is_valid_gps_fix(g: Optional[dict]) -> bool:
    """True if fix_type ≥ 2 (2D or better) AND satellites > 0.
    Producers may still SEND frames when this returns False (with
    zeros), so consumers see a 'still acquiring' signal — that's
    a policy choice in the producer, not this helper."""
    if not isinstance(g, dict):
        return False
    fix_type = g.get("fix_type")
    sats = g.get("satellites")
    return (
        isinstance(fix_type, int) and fix_type >= 2
        and isinstance(sats, int) and sats > 0
    )


def is_valid_orientation(o: Optional[dict]) -> bool:
    """True if all three angles present and numeric."""
    if not isinstance(o, dict):
        return False
    return all(
        isinstance(o.get(k), (int, float))
        for k in ("roll", "pitch", "yaw")
    )


def is_valid_state(s: Optional[dict]) -> bool:
    """True if connected + has a mode string."""
    if not isinstance(s, dict):
        return False
    return bool(s.get("connected")) and isinstance(s.get("mode"), str)


# --- self-test ------------------------------------------------------------

if __name__ == "__main__":
    import sys
    c = FcClient()
    print(f"fc_client smoke test against {c.base_url}", file=sys.stderr)
    for name, fn in [
        ("battery",    c.get_battery),
        ("gps",        c.get_gps),
        ("vfr_hud",    c.get_vfr_hud),
        ("orientation", c.get_orientation),
        ("state",      c.get_state),
    ]:
        t0 = time.monotonic()
        result = fn()
        dt_ms = (time.monotonic() - t0) * 1000
        status = "OK" if result is not None else f"FAIL ({c.last_error})"
        print(f"  {name:12} {dt_ms:5.1f} ms  {status}  {result}")
    print(f"\nsuccesses={c.consecutive_successes}  failures={c.consecutive_failures}")
