"""Polling threads that fill the shared snapshot.

Two background threads:

  * `poll_drone` — pulls drone-control's `/telemetry` at DRONE_POLL_HZ.
                   Drone-control is read-only from our side; nothing here
                   ever POSTs to it.
  * `poll_elrs`  — pulls elrs-telemetry's `/link` at ELRS_POLL_HZ for
                   RSSI / LQ.

Both threads are best-effort: if the upstream is briefly unreachable we
log once, bump an error counter, and keep the last-known snapshot in
place. The 32-byte digest's `monotonic_ms` field is what callers use to
detect staleness — the snapshot itself never goes blank.
"""

from __future__ import annotations

import logging
import threading
import time

import requests

from . import snapshot


log = logging.getLogger("drone_bridge.pump")


# --- Drone-control poller -------------------------------------------------

def poll_drone(api_base: str, hz: float) -> None:
    """Background loop. Updates the drone-side fields of the snapshot."""
    period = 1.0 / hz
    sess = requests.Session()
    last_log = 0.0

    while True:
        t0 = time.monotonic()
        try:
            r = sess.get(f"{api_base}/telemetry", timeout=1.0)
            r.raise_for_status()
            d = r.json()
            now = time.monotonic()

            state = d.get("state") or {}
            batt  = d.get("battery") or {}
            gps   = d.get("gps") or {}
            ori   = d.get("orientation") or {}
            vfr   = d.get("vfr_hud") or {}
            home  = d.get("home") or {}
            local = d.get("local_position") or {}

            with snapshot.state_lock:
                s = snapshot.state
                s.connected   = bool(state.get("connected", False))
                s.armed       = bool(state.get("armed", False))
                s.mode        = str(state.get("mode") or "")

                s.voltage_v   = float(batt.get("voltage", 0.0) or 0.0)
                s.battery_pct = float(batt.get("percentage", 0.0) or 0.0)

                s.fix_type    = int(gps.get("fix_type", 0) or 0)
                s.lat         = float(gps.get("latitude", 0.0) or 0.0)
                s.lon         = float(gps.get("longitude", 0.0) or 0.0)
                s.alt_amsl_m  = float(gps.get("altitude", 0.0) or 0.0)

                # MAVROS local_position uses ENU (z up); good enough as
                # "altitude above EKF origin" for the 16-bit decimeter
                # field. The FC computes the canonical relative altitude.
                s.alt_rel_m   = float(local.get("z", 0.0) or 0.0)

                s.ground_speed_mps = float(vfr.get("groundspeed", 0.0) or 0.0)
                s.heading_deg = float(vfr.get("heading", 0.0) or 0.0)
                s.roll_rad    = float(ori.get("roll", 0.0) or 0.0)
                s.pitch_rad   = float(ori.get("pitch", 0.0) or 0.0)

                s.home_set    = bool(
                    float(home.get("latitude", 0.0) or 0.0) != 0.0
                    or float(home.get("longitude", 0.0) or 0.0) != 0.0
                )
                s.drone_last_ts = now

            if now - last_log >= 30.0:
                last_log = now
                with snapshot.state_lock:
                    s = snapshot.state
                    log.info(
                        "drone-poll OK: connected=%s mode=%s armed=%s "
                        "batt=%.1fV %.0f%% fix=%d errors=%d",
                        s.connected, s.mode or "-", s.armed,
                        s.voltage_v, s.battery_pct * 100,
                        s.fix_type, s.drone_errors,
                    )
        except Exception as e:
            with snapshot.state_lock:
                snapshot.state.drone_errors += 1
                err_count = snapshot.state.drone_errors
            if err_count == 1 or err_count % 50 == 0:
                log.warning("drone-poll failed (%s): %s", type(e).__name__, e)

        # Sleep the remainder of the period (no drift on slow responses)
        time.sleep(max(0.0, period - (time.monotonic() - t0)))


# --- ELRS link poller -----------------------------------------------------

def poll_elrs(api_base: str, hz: float) -> None:
    """Background loop. Updates RSSI / LQ from elrs-telemetry."""
    period = 1.0 / hz
    sess = requests.Session()

    while True:
        t0 = time.monotonic()
        try:
            r = sess.get(f"{api_base}/link", timeout=1.0)
            r.raise_for_status()
            link = (r.json() or {}).get("link") or {}
            if link:
                with snapshot.state_lock:
                    s = snapshot.state
                    s.rssi_dbm  = int(link.get("uplink_rssi_ant1") or 0)
                    s.uplink_lq = int(link.get("uplink_lq") or 0)
                    s.elrs_last_ts = time.monotonic()
        except Exception as e:
            with snapshot.state_lock:
                snapshot.state.elrs_errors += 1
                err_count = snapshot.state.elrs_errors
            if err_count == 1 or err_count % 200 == 0:
                log.warning("elrs-poll failed (%s): %s", type(e).__name__, e)

        time.sleep(max(0.0, period - (time.monotonic() - t0)))


# --- Thread launchers -----------------------------------------------------

def start(drone_api: str, drone_hz: float, elrs_api: str, elrs_hz: float) -> None:
    """Launch both pollers as daemon threads. Returns immediately."""
    threading.Thread(
        target=poll_drone, args=(drone_api, drone_hz),
        daemon=True, name="poll-drone",
    ).start()
    threading.Thread(
        target=poll_elrs, args=(elrs_api, elrs_hz),
        daemon=True, name="poll-elrs",
    ).start()
    log.info("pump started: drone=%s @ %.1f Hz, elrs=%s @ %.1f Hz",
             drone_api, drone_hz, elrs_api, elrs_hz)
