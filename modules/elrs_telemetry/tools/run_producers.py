#!/usr/bin/env python3
"""Supervisor for the 5 ELRS telemetry producers.

Runs each producer as a managed child subprocess. If a child dies,
restart it with exponential backoff (1 s → 2 s → 4 s → 8 s → 30 s
cap). Streams all child output to stdout with a [name] prefix so
`docker logs elrs-producers` shows everything in one pane.

This is the container entrypoint inside the elrs-producers image.
Container-level supervision (Docker `restart: unless-stopped`) handles
the case where this supervisor itself dies; inner supervision handles
single-producer crashes without restarting the container.

Pattern lifted from drone-control's entrypoint-supervisor (see memory
`entrypoint-supervisor.md`).
"""
from __future__ import annotations

import glob
import os
import signal
import subprocess
import sys
import threading
import time

# (logical-name, script-path-in-container, extra-env)
PRODUCERS = [
    ("rc-keeper", "/app/elrs_producer.py",         {"ELRS_PRODUCER_HZ": "10"}),
    ("battery",   "/app/fc_battery_producer.py",    {}),
    ("gps",       "/app/fc_gps_producer.py",        {}),
    ("attitude",  "/app/fc_attitude_producer.py",   {}),
    ("flightmode","/app/fc_flightmode_producer.py", {}),
]

BACKOFF_INITIAL_S = 1.0
BACKOFF_MAX_S = 30.0
HEALTHY_RUNTIME_S = 30.0   # crash after this much runtime resets backoff

stop_flag = threading.Event()


def utc_stamp() -> str:
    return time.strftime("%H:%M:%S", time.gmtime()) + "Z"


def log(prefix: str, msg: str) -> None:
    print(f"[{utc_stamp()}] [{prefix}] {msg}", flush=True)


def supervise(name: str, script: str, env_extra: dict) -> None:
    """Run one producer in a loop. Backs off exponentially on rapid crashes,
    resets backoff when a process lives at least HEALTHY_RUNTIME_S."""
    backoff = BACKOFF_INITIAL_S
    while not stop_flag.is_set():
        env = os.environ.copy()
        env.update(env_extra)
        # Default to localhost — caller can override via DRONE_CONTROL_URL in compose.
        env.setdefault("DRONE_CONTROL_URL", "http://localhost:8080")

        log(name, f"starting {script}")
        started_at = time.monotonic()
        try:
            proc = subprocess.Popen(
                ["python3", "-u", script],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except Exception as e:
            log(name, f"launch failed: {e}; retry in {backoff:.0f}s")
            _sleep_interruptible(backoff)
            backoff = min(backoff * 2, BACKOFF_MAX_S)
            continue

        # Stream child output line by line with [name] prefix
        try:
            for line in proc.stdout:
                if stop_flag.is_set():
                    break
                print(f"[{name}] {line.rstrip()}", flush=True)
        except Exception as e:
            log(name, f"output read failed: {e}")

        # Process ended; collect rc
        if stop_flag.is_set():
            try:
                proc.terminate()
                proc.wait(timeout=5)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass
            log(name, "shutdown")
            return

        rc = proc.wait()
        runtime = time.monotonic() - started_at
        log(name, f"exited rc={rc} after {runtime:.1f}s")

        # If it ran long enough, treat as healthy — reset backoff.
        if runtime >= HEALTHY_RUNTIME_S:
            backoff = BACKOFF_INITIAL_S

        if stop_flag.is_set():
            return
        _sleep_interruptible(backoff)
        backoff = min(backoff * 2, BACKOFF_MAX_S)


def _sleep_interruptible(seconds: float) -> None:
    """Sleep but wake on stop_flag."""
    end_at = time.monotonic() + seconds
    while not stop_flag.is_set() and time.monotonic() < end_at:
        time.sleep(min(0.5, end_at - time.monotonic()))


def _shutdown(signum, _frame) -> None:
    log("supervisor", f"signal {signum} — stopping all children")
    stop_flag.set()


def _purge_stale_pidfiles() -> None:
    """Remove producer pidfiles left over from a previous supervisor run.

    Inside the container, /tmp persists across `docker restart` and PIDs
    get reused — the OLD pidfile may point at a live PID that's actually
    a different process (often this supervisor itself). producer_safety
    treats that as a held pidfile and refuses to start. Since the
    supervisor is the sole owner of these pidfiles when it starts, it's
    safe to clear them all at boot.
    """
    pattern = "/tmp/novaros_elrs_producer_*.pid"
    removed = []
    for path in glob.glob(pattern):
        try:
            os.unlink(path)
            removed.append(path)
        except OSError as e:
            log("supervisor", f"could not remove stale pidfile {path}: {e}")
    if removed:
        log("supervisor", f"purged {len(removed)} stale pidfile(s)")


def main() -> int:
    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    _purge_stale_pidfiles()

    log("supervisor", f"starting {len(PRODUCERS)} producers")
    threads: list[threading.Thread] = []
    for name, script, env in PRODUCERS:
        t = threading.Thread(
            target=supervise,
            args=(name, script, env),
            name=f"sup-{name}",
            daemon=True,
        )
        t.start()
        threads.append(t)
        # Stagger startup: RC keeper goes first + gets a 1-second head start
        # so Tx is hot before fc_* start POSTing telemetry. Other producers
        # spread 300 ms apart so logs are readable.
        time.sleep(1.0 if name == "rc-keeper" else 0.3)

    log("supervisor", "all children launched; running until SIGTERM")
    while not stop_flag.is_set():
        time.sleep(1.0)

    for t in threads:
        t.join(timeout=10.0)
    log("supervisor", "all children stopped — exit")
    return 0


if __name__ == "__main__":
    sys.exit(main())
