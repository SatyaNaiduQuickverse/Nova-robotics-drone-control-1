"""Lightweight service registry for drone-bridge subsystems.

Why this exists: drone-bridge runs five concurrent concerns in one
container — BLE GATT peripheral, CRSF→HTTP translator, two telemetry
pumps, debug HTTP. They're tightly coupled by failure model (a BLE
death without a translator is useless to the phone, etc.) so they
deserve to live in one process. But they're loosely coupled in code
and have very different lifecycles (asyncio tasks vs. daemon threads).

Forcing them into a single ABC was the wrong move when I considered
it — they'd need ill-fitting wrappers. Instead this module gives them
a *shared health-reporting surface* without dictating their lifecycle.

Each subsystem:
  1. Calls `registry.register("its-name")` on startup
  2. Calls `health.report_error(msg)` when something goes wrong
  3. Calls `health.report_recovery()` when it self-heals
  4. Optionally writes service-specific metrics to `health.extra`
     (frame counts, queue depths, last-seen-anything timestamps)

The debug HTTP server then reads `registry.all()` and exposes:
  GET /healthz             — aggregate (200 if everything healthy, else 503)
  GET /healthz/<name>      — one service (200/503 + per-service detail)
  GET /services            — full list with all extras for debugging

This is observability, not orchestration. If a service is unhealthy,
it stays in the registry and keeps serving traffic; the registry just
*reports* the state. Restart policy lives at the Docker layer.

Concurrency: the registry is locked because reports come from threads
(pump pollers) AND the asyncio event loop (BLE, translator). Lock
scope is small — register/lookup/mutate-flag — never around long ops.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class ServiceHealth:
    """Per-service health record. Fields are mutable; the registry hands
    you a reference and you write to it directly. Reads from other
    threads see snapshots taken under the registry's lock.

    `extra` is a free-form dict for service-specific metrics. Convention:
      - keep keys flat (no nested dicts), one level deep
      - keep values JSON-serializable (the /services endpoint dumps it)
      - update atomically — set a new value, don't read-modify-write
        without external coordination
    """
    name: str
    started_at_mono: float
    healthy: bool = True
    last_error: Optional[str] = None
    last_error_at_mono: Optional[float] = None
    last_recovery_at_mono: Optional[float] = None
    error_count: int = 0
    extra: dict = field(default_factory=dict)

    def report_error(self, msg: str) -> None:
        """Mark unhealthy. Idempotent — repeated calls bump the counter
        but don't multiply effects. Caller doesn't need to deduplicate."""
        self.healthy = False
        self.last_error = msg
        self.last_error_at_mono = time.monotonic()
        self.error_count += 1

    def report_recovery(self) -> None:
        """Mark healthy again. No-op if already healthy."""
        if self.healthy:
            return
        self.healthy = True
        self.last_recovery_at_mono = time.monotonic()

    def uptime_s(self) -> float:
        return time.monotonic() - self.started_at_mono

    def to_dict(self) -> dict:
        """Serializable snapshot for /services responses. Monotonic times
        are converted to "seconds ago" relative to now so the consumer
        doesn't need to know our uptime base."""
        now = time.monotonic()
        return {
            "name": self.name,
            "healthy": self.healthy,
            "uptime_s": round(self.uptime_s(), 1),
            "error_count": self.error_count,
            "last_error": self.last_error,
            "last_error_age_s": (
                round(now - self.last_error_at_mono, 1)
                if self.last_error_at_mono is not None else None
            ),
            "last_recovery_age_s": (
                round(now - self.last_recovery_at_mono, 1)
                if self.last_recovery_at_mono is not None else None
            ),
            "extra": dict(self.extra),  # shallow copy for the response
        }


class ServiceRegistry:
    """Shared lookup for all subsystem health records. One instance,
    module-level singleton (`registry` below).

    Thread-safe. The lock guards the dict; the ServiceHealth objects
    themselves do their own atomic-ish writes via simple field
    assignment (Python's GIL makes single-field writes atomic).
    """

    def __init__(self) -> None:
        self._services: dict[str, ServiceHealth] = {}
        self._lock = threading.Lock()

    def register(self, name: str) -> ServiceHealth:
        """Register a service. Returns its ServiceHealth — caller keeps
        the reference and writes through it. Calling register() twice
        with the same name returns the existing record (idempotent;
        useful for services that restart in-place)."""
        with self._lock:
            existing = self._services.get(name)
            if existing is not None:
                return existing
            h = ServiceHealth(name=name, started_at_mono=time.monotonic())
            self._services[name] = h
            return h

    def get(self, name: str) -> Optional[ServiceHealth]:
        with self._lock:
            return self._services.get(name)

    def all(self) -> list[ServiceHealth]:
        """Snapshot of every registered service. Returns a list (not the
        underlying dict) so the caller can iterate without holding the
        lock or mutating the registry."""
        with self._lock:
            return list(self._services.values())

    def healthy(self) -> bool:
        """True iff every registered service is currently healthy.
        Aggregate signal for the top-level /healthz endpoint."""
        with self._lock:
            return all(h.healthy for h in self._services.values())

    def aggregate_dict(self) -> dict:
        """JSON-friendly aggregate for /services."""
        services = self.all()
        return {
            "ok": all(h.healthy for h in services),
            "service_count": len(services),
            "services": [h.to_dict() for h in services],
        }


# Module-level singleton. All subsystems share this.
registry = ServiceRegistry()
