"""VTX broadcast routing layer.

Owns the question "which video source is currently being broadcast over
the analog VTX?" — separated from "which physical cameras exist?" (that
lives in camera.py's CAMERAS registry). The renderer pulls a single URL
(/vtx/snapshot) and never needs to know about source switching.

Sources (registered at api_gateway startup):

    front   — drone-control camera "tracking" (Lenovo front cam, raw)
    vision  — vision-detect /frame (YOLO + tracker overlays baked in)
    ground  — placeholder for downward-facing cam / ground annotation;
              if VTX_GROUND_URL env is set, fetches from that. Otherwise
              returns a static "GROUND CAM OFFLINE" indicator frame.
    black   — solid black frame; useful for privacy / pause / blanking
              the VTX feed without stopping the renderer.

Adding a new source = one register() call. The router doesn't know or
care what's behind each name — just calls the function and gets bytes
(or None on failure) back.

Design notes:
- All public methods are thread-safe via a single self._lock.
- Source fetcher functions can do I/O (HTTP, file) but should fail
  gracefully — return None instead of raising. The router falls back to
  a stable "no-signal" frame when the active source returns None.
- No frame caching here; that's the source function's responsibility if
  it needs it. Keeps this layer state-free except for the active-name.
"""

from __future__ import annotations

import logging
import threading
from typing import Callable, Optional


log = logging.getLogger("drone_control.vtx_broadcast")


# Source-fetcher signature: takes nothing, returns JPEG bytes, or None on
# failure (camera offline, vision-detect down, etc.). The router treats
# None uniformly — always falls back to a "no signal" frame for the
# current source so the broadcast never goes truly empty.
SourceFetcher = Callable[[], Optional[bytes]]


class BroadcastRouter:
    """Single owner of "what's currently going out the VTX".

    Public surface intentionally tiny — the goal is for endpoints to
    delegate everything here without holding state of their own.
    """

    def __init__(self, default_source: str = "front"):
        self._lock = threading.Lock()
        self._sources: dict[str, SourceFetcher] = {}
        # Per-source last-good frame cache. Lets us serve a brief stale
        # frame across a transient source failure instead of going blank.
        self._last_good: dict[str, bytes] = {}
        self._current: str = default_source

    # --- source registration ---------------------------------------

    def register(self, name: str, fetch_fn: SourceFetcher) -> None:
        """Register or replace a source. Safe to call at any time."""
        with self._lock:
            self._sources[name] = fetch_fn
            log.info("registered source: %s", name)

    def list_sources(self) -> list[str]:
        with self._lock:
            return sorted(self._sources.keys())

    # --- routing ---------------------------------------------------

    def current(self) -> str:
        with self._lock:
            return self._current

    def set_source(self, name: str) -> bool:
        """Switch the active source. Returns False if name unknown."""
        with self._lock:
            if name not in self._sources:
                log.warning("set_source rejected: unknown source %r "
                            "(known: %s)", name, sorted(self._sources))
                return False
            old = self._current
            self._current = name
        if old != name:
            log.info("source switched: %s → %s", old, name)
        return True

    # --- frame retrieval ------------------------------------------

    def get_frame(self) -> Optional[bytes]:
        """Return the latest JPEG from the active source.

        Behavior on source failure:
          1. Source fetcher returns None → fall back to last-good frame
             from this source if we ever got one.
          2. No last-good frame and source returns None → return None
             (caller decides — endpoint sends 503).
        """
        with self._lock:
            name = self._current
            fn = self._sources.get(name)
        if fn is None:
            return None
        try:
            jpg = fn()
        except Exception:
            log.exception("source %r raised in fetch", name)
            jpg = None
        if jpg:
            with self._lock:
                self._last_good[name] = jpg
            return jpg
        # Fallback: serve last-good for THIS source (never cross-pollinate
        # between sources — a source going offline shouldn't show stale
        # content from a different source).
        with self._lock:
            return self._last_good.get(name)


# Module-level singleton. Used by api_gateway endpoints.
router = BroadcastRouter(default_source="front")
