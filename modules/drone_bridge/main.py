"""drone_bridge entrypoint.

Wires together:
  * pump:        2 background threads polling drone-control + elrs-telemetry
  * Flask debug: thread serving /healthz + /telemetry/digest{,/json} on 5004
  * BLE server:  asyncio task — peripheral that the phone connects to
  * translator:  asyncio task — CRSF→MAVLink, runs in same loop as BLE

One process, one container. Shared state via `snapshot.py`. Three
asyncio tasks (BLE, translator's WS reader, translator's 50 Hz sender)
plus three threads (pump-drone, pump-elrs, Flask).

Crashes anywhere should bring the process down — Docker `restart:
unless-stopped` brings it back. We don't try to mask partial failures.
"""

from __future__ import annotations

import asyncio
import logging
import math
import os
import signal
import threading
from typing import Optional

from flask import Flask, Response, jsonify

from . import ble, digest, pump, serializers, services, snapshot, translator


# --- Config (env-driven; sensible defaults for this Pi) ------------------

DRONE_API     = os.environ.get("DRONE_API",  "http://127.0.0.1:8080")
ELRS_API      = os.environ.get("ELRS_API",   "http://127.0.0.1:5003")
ELRS_WS       = os.environ.get("ELRS_WS",    "ws://127.0.0.1:5003/ws/channels")

DRONE_POLL_HZ = float(os.environ.get("DRONE_POLL_HZ", "10"))
ELRS_POLL_HZ  = float(os.environ.get("ELRS_POLL_HZ",  "5"))

DEBUG_BIND    = os.environ.get("DEBUG_BIND", "0.0.0.0")
DEBUG_PORT    = int(os.environ.get("DEBUG_PORT", "5004"))

ADV_NAME      = os.environ.get("BLE_ADV_NAME", ble.ADV_NAME)
BLE_MTU       = int(os.environ.get("BLE_MTU", "247"))

ENABLE_TRANSLATOR = os.environ.get("ENABLE_TRANSLATOR", "1") not in ("0", "false", "False")
ENABLE_BLE        = os.environ.get("ENABLE_BLE",        "1") not in ("0", "false", "False")


# --- Logging -------------------------------------------------------------

logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO").upper(),
    format="%(asctime)s %(levelname)s %(name)s %(message)s",
)
log = logging.getLogger("drone_bridge")
logging.getLogger("werkzeug").setLevel(logging.WARNING)
logging.getLogger("urllib3").setLevel(logging.WARNING)
logging.getLogger("httpx").setLevel(logging.WARNING)
logging.getLogger("websockets").setLevel(logging.INFO)


# --- Debug HTTP server ---------------------------------------------------

debug_app = Flask("drone_bridge.debug")


@debug_app.route("/healthz")
def healthz():
    """Aggregate health: 200 iff every registered service is healthy.
    Returns 503 with the list of failing services otherwise. Per-service
    detail at /services or /healthz/<name>."""
    with snapshot.state_lock:
        s = snapshot.state
    body = {
        "ok": services.registry.healthy(),
        "uptime_s": round(snapshot.uptime_s(), 2),
        "drone_age_ms": snapshot.age_ms(s.drone_last_ts),
        "elrs_age_ms": snapshot.age_ms(s.elrs_last_ts),
        "drone_errors": s.drone_errors,
        "elrs_errors": s.elrs_errors,
        "services_summary": [
            {"name": h.name, "healthy": h.healthy}
            for h in services.registry.all()
        ],
    }
    status = 200 if body["ok"] else 503
    return jsonify(body), status


@debug_app.route("/healthz/<name>")
def healthz_service(name: str):
    """Per-service health detail. 200 if healthy, 503 if not, 404 if
    unknown service. The body always contains the full ServiceHealth
    snapshot (extras + counters) so a 503 response is debuggable."""
    h = services.registry.get(name)
    if h is None:
        return jsonify(error=f"unknown service: {name}",
                       known=[s.name for s in services.registry.all()]), 404
    return jsonify(h.to_dict()), 200 if h.healthy else 503


@debug_app.route("/services")
def services_list():
    """Full inventory of every registered subsystem with health + extras.
    Designed for human debugging — pretty-print friendly."""
    return jsonify(services.registry.aggregate_dict())


@debug_app.route("/telemetry/digest")
def digest_binary():
    """The same 32-byte payload the BLE handler returns. Useful for
    smoke-testing digest packing without going through BLE."""
    return Response(digest.pack(), mimetype="application/octet-stream")


@debug_app.route("/telemetry/digest/json")
def digest_json():
    """Human-readable view of the snapshot + the unpacked digest fields.
    Not on the BLE forwarding path.

    Routes through the canonical serializers (DigestSerializer +
    JsonFullSerializer) — adding a field to Snapshot + JsonFullSerializer
    auto-flows here. Don't duplicate field-formatting logic in this
    endpoint."""
    import dataclasses as _dc
    with snapshot.state_lock:
        snap = _dc.replace(snapshot.state)  # immutable copy
    raw = serializers.DIGEST.serialize(snap)
    return jsonify(
        hex=raw.hex(),
        bytes=len(raw),
        unpacked=serializers.DIGEST.deserialize(raw),
        raw_snapshot=serializers.JSON_FULL.serialize(snap),
        wire_format=serializers.DIGEST.wire_format,
    )


def _run_debug_http():
    debug_app.run(host=DEBUG_BIND, port=DEBUG_PORT, threaded=True,
                  debug=False, use_reloader=False)


# --- Async services (BLE + translator) -----------------------------------

async def _async_main() -> None:
    tasks = []
    server: Optional[ble.BleServer] = None

    if ENABLE_BLE:
        server = ble.BleServer(drone_api=DRONE_API,
                               adv_name=ADV_NAME,
                               mtu=BLE_MTU)
        await server.start()
    else:
        log.info("BLE server disabled by env")

    if ENABLE_TRANSLATOR:
        tasks.append(asyncio.create_task(
            translator.run(DRONE_API, ELRS_WS),
            name="translator",
        ))
    else:
        log.info("CRSF translator disabled by env")

    # Hook SIGTERM/SIGINT for clean shutdown (Docker stop sends SIGTERM).
    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGTERM, signal.SIGINT):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except NotImplementedError:
            pass  # Windows / certain restricted runtimes

    if not tasks and server is None:
        log.error("nothing enabled — exiting")
        return

    log.info("drone_bridge async services up; waiting for shutdown signal")
    if tasks:
        # Race tasks against the shutdown event. Whichever completes
        # first triggers full teardown (we treat any task exit as fatal,
        # consistent with PROMPT.md §2 supervisor pattern).
        done, _pending = await asyncio.wait(
            [asyncio.create_task(stop_event.wait(), name="stop"), *tasks],
            return_when=asyncio.FIRST_COMPLETED,
        )
        for t in done:
            if t.get_name() != "stop":
                log.warning("async task %s exited; tearing down",
                            t.get_name())
    else:
        await stop_event.wait()

    if server is not None:
        log.info("stopping BLE server")
        await server.stop()


def main() -> None:
    log.info("drone_bridge starting")
    log.info("  drone_api   = %s", DRONE_API)
    log.info("  elrs_api    = %s", ELRS_API)
    log.info("  elrs_ws     = %s", ELRS_WS)
    log.info("  debug http  = %s:%d", DEBUG_BIND, DEBUG_PORT)
    log.info("  ble adv     = %s (enabled=%s)", ADV_NAME, ENABLE_BLE)
    log.info("  translator  = enabled=%s", ENABLE_TRANSLATOR)

    # 1. pump pollers (threads — they don't block the asyncio loop)
    pump.start(DRONE_API, DRONE_POLL_HZ, ELRS_API, ELRS_POLL_HZ)

    # 2. debug HTTP (thread — Flask, low traffic)
    threading.Thread(target=_run_debug_http,
                     daemon=True, name="debug-http").start()

    # 3. async services (this thread)
    asyncio.run(_async_main())


if __name__ == "__main__":
    main()
