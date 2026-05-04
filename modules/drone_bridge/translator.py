"""CRSF → MAVLink translator (PROMPT.md §2 Deliverable 1).

Subscribes to elrs-telemetry's `/ws/channels` WebSocket (which already
unpacks the 16 × 11-bit CRSF channels into both raw and µs forms) and
translates them into HTTP calls against drone-control's existing
`/control/command`, `/arm`, `/disarm`, `/mode`, and `/land/precision`
endpoints.

This is the SAFETY-CRITICAL service. Three v0 traps (caught in the
prompt's review section) are explicitly avoided here:

  1. **No double-flip on pitch.** The phone has already negated pitch
     in `FlightRepository.mapSticksToDrone` so the drone receives
     drone-convention values. We pass through unchanged.

  2. **Hard link-loss guard at 300 ms.** If we stop seeing channel
     updates we STOP calling `/control/command`. We do NOT send
     "neutral" sticks — that defeats ArduPilot's RC failsafe (which
     needs to see *no* RC input to trigger).

  3. **Fully async HTTP via httpx.AsyncClient.** Never blocks the
     channel reader on a slow upstream call.

Channel mapping (12-channel Wide mode; CH13-16 not transmitted):
  CH1 throttle 0..1   CH2 roll -1..+1     CH3 pitch -1..+1     CH4 yaw -1..+1
  CH5 arm ≥1500       CH6 force-disarm ≥1500 (rising edge)
  CH7 mode 12-pos rotary (handled here)
  CH8 layered ops field — owned ENTIRELY by vision_lock
       (button_code 3b | box_valid 1b | source_code 2b)
  CH9-CH12 box coords  — owned by vision_lock
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import time
from typing import Optional

import httpx
import websockets

from . import vision_lock


log = logging.getLogger("drone_bridge.translator")


# --- Tuning knobs --------------------------------------------------------

LINK_LOSS_S       = 0.3     # PROMPT.md §2.1 — hard guard, do not raise
CONTROL_SEND_HZ   = 50.0    # PROMPT.md §2.1 — internal throttle for /control/command
WS_RECONNECT_S    = 1.0     # exponential backoff if elrs-telemetry is restarting
WS_FRAME_STALL_S  = 2.0     # if recv() goes this long without a frame, force
                            # reconnect — covers "WS alive but server stopped
                            # broadcasting to us" failures that ping/pong misses

# Phone mode encoding: 12 modes carried directly on CH7 as a 12-position
# rotary (Wide-mode µs evenly spaced). CH8 is owned ENTIRELY by vision_lock
# (button_code | box_valid | source_code layered ops field) — translator
# does not read CH8 at all. Edge fires on ch7 index change.
PHONE_MODE_MAP: dict[int, str] = {
    0:  "STABILIZE",
    1:  "ALT_HOLD",
    2:  "LOITER",
    3:  "AUTO",
    4:  "GUIDED",
    5:  "BRAKE",
    6:  "RTL",
    7:  "LAND",
    8:  "POSHOLD",
    9:  "ACRO",
    10: "SMART_RTL",
    11: "AUTOTUNE",
}


# --- Channel decode helpers ---------------------------------------------

def axis_pm1(crsf_val: int) -> float:
    """CRSF unit (172..1811) → -1.0..+1.0 axis. Clamped."""
    v = (crsf_val - 992) / 819.5
    return max(-1.0, min(1.0, v))


def throttle_01(crsf_val: int) -> float:
    """CRSF unit → 0.0..1.0 throttle. Clamped."""
    v = (crsf_val - 172) / 1639.0
    return max(0.0, min(1.0, v))


def mode_index_from_us(us: int) -> int:
    """12-bucket mode decoder with MIDPOINT thresholds.

    Phone encodes 12 modes evenly spaced across the Wide-mode µs range
    [988..2012] using `Channels.positionToPwm(p, 12)`. Step is 1024/11
    ≈ 93.09 µs (NOT 1024/12 — 12 positions create 11 gaps, not 12).
    Decision thresholds belong at the MIDPOINTS between adjacent positions;
    that gives ~46.5 µs symmetric margin per side ≈ 4× the empirically
    measured ~11 µs wire drift on this radio.

    Prior implementation used 1024/12-derived thresholds, which left
    pos 10 (SMART_RTL, us=1919) with only 8 µs of upper margin — below
    the 11 µs drift floor. That misdecoded as AUTOTUNE on drift events;
    fixed here.
    """
    if us < 1035: return 0   # STABILIZE   (pos 0  ≈ 988  µs)
    if us < 1128: return 1   # ALT_HOLD    (pos 1  ≈ 1081 µs)
    if us < 1221: return 2   # LOITER      (pos 2  ≈ 1174 µs)
    if us < 1314: return 3   # AUTO        (pos 3  ≈ 1267 µs)
    if us < 1407: return 4   # GUIDED      (pos 4  ≈ 1360 µs)
    if us < 1500: return 5   # BRAKE       (pos 5  ≈ 1453 µs)
    if us < 1593: return 6   # RTL         (pos 6  ≈ 1547 µs)
    if us < 1686: return 7   # LAND        (pos 7  ≈ 1640 µs)
    if us < 1779: return 8   # POSHOLD     (pos 8  ≈ 1733 µs)
    if us < 1872: return 9   # ACRO        (pos 9  ≈ 1826 µs)
    if us < 1965: return 10  # SMART_RTL   (pos 10 ≈ 1919 µs)
    return 11                # AUTOTUNE    (pos 11 ≈ 2012 µs)


# --- State (shared between WS reader and the 50 Hz control sender) ------

class TranslatorState:
    def __init__(self):
        self.channels_crsf: list[int] = [992] * 16  # neutral, will be overwritten on first WS msg
        self.channels_us:   list[int] = [1500] * 16
        self.last_rc_ts:    Optional[float] = None
        self.last_ch5_armed:  Optional[bool] = None    # arm (CH5 ≥1500)
        self.last_ch6_forced: Optional[bool] = None    # force-disarm (CH6 ≥1500)
        self.last_mode_idx:   Optional[int]  = None    # 12-pos CH7 mode index
        # Counters for /healthz
        self.frames_in:     int = 0
        self.commands_sent: int = 0
        self.commands_skipped_link_loss: int = 0
        self.events_arm:    int = 0
        self.events_disarm: int = 0
        self.events_mode:   int = 0

        # Vision-lock sub-state (CH8 layered ops field + CH9-CH12).
        # Lives on TranslatorState so the WS reader can pass a single
        # state object to all handlers; the vision_lock module is the
        # only thing that touches this attribute.
        self.vision_lock = vision_lock.VisionLockState()


# --- WebSocket reader ----------------------------------------------------

async def _ws_reader_loop(ws_url: str, state: TranslatorState,
                          on_channels) -> None:
    """Subscribe to elrs-telemetry /ws/channels and keep the state fresh.

    `on_channels(state)` is called synchronously after each frame is
    decoded; it should be cheap (edge detection only — actual HTTP work
    is fired into background tasks)."""
    backoff = WS_RECONNECT_S
    while True:
        try:
            log.info("connecting to %s", ws_url)
            async with websockets.connect(
                ws_url, ping_interval=10, ping_timeout=5, close_timeout=2,
            ) as ws:
                log.info("WS connected")
                backoff = WS_RECONNECT_S  # reset on successful connect
                while True:
                    try:
                        raw = await asyncio.wait_for(
                            ws.recv(), timeout=WS_FRAME_STALL_S,
                        )
                    except asyncio.TimeoutError:
                        log.warning("WS stall — no frame in %.1fs, forcing reconnect",
                                    WS_FRAME_STALL_S)
                        break
                    try:
                        msg = json.loads(raw)
                    except json.JSONDecodeError:
                        continue
                    crsf = msg.get("channels_crsf")
                    us   = msg.get("channels_us")
                    if not crsf or not us or len(crsf) < 16:
                        continue
                    state.channels_crsf = list(crsf)
                    state.channels_us   = list(us)
                    state.last_rc_ts    = time.monotonic()
                    state.frames_in    += 1
                    on_channels(state)
        except (websockets.exceptions.WebSocketException, OSError) as e:
            log.warning("WS error (%s); reconnecting in %.1fs",
                        type(e).__name__, backoff)
        except Exception:
            log.exception("WS reader unexpected error")

        await asyncio.sleep(backoff)
        backoff = min(backoff * 2, 5.0)


# --- Control sender (50 Hz) ---------------------------------------------

async def _control_sender_loop(client: httpx.AsyncClient,
                               state: TranslatorState) -> None:
    """Send /control/command at CONTROL_SEND_HZ.

    Skips silently if no fresh channel data (PROMPT.md §2.1 link-loss
    guard) — ArduPilot's onboard FS_THR_* failsafe takes over. We do
    NOT send neutral sticks as a substitute."""
    period = 1.0 / CONTROL_SEND_HZ
    last_log = 0.0
    while True:
        t0 = time.monotonic()

        if state.last_rc_ts is None or (t0 - state.last_rc_ts) > LINK_LOSS_S:
            state.commands_skipped_link_loss += 1
            if state.commands_skipped_link_loss == 1 \
               or state.commands_skipped_link_loss % int(CONTROL_SEND_HZ * 5) == 0:
                age = "never" if state.last_rc_ts is None \
                      else f"{(t0 - state.last_rc_ts):.2f}s"
                log.warning("link-loss guard active (age=%s) — not sending /control/command", age)
        else:
            ch = state.channels_crsf
            # Stick channel mapping matches the phone-side ElrsControlApi.kt
            # (CH_THROTTLE=0, CH_ROLL=1, CH_PITCH=2, CH_YAW=3) — NOT the
            # ArduPilot RC convention. Pitch is sent un-negated because the
            # phone already applies the drone-convention negation upstream.
            payload = {
                "timestamp": time.time(),       # server uses this for clock skew
                "throttle": throttle_01(ch[0]),  # CH1 (idx 0)
                "roll":     axis_pm1(ch[1]),     # CH2 (idx 1)
                "pitch":    axis_pm1(ch[2]),     # CH3 (idx 2)
                "yaw":      axis_pm1(ch[3]),     # CH4 (idx 3)
                "duration": 0.1,                 # short hold; we re-send every 20 ms
            }
            try:
                await client.post("/control/command", json=payload, timeout=0.4)
                state.commands_sent += 1
            except httpx.RequestError as e:
                # Don't let a one-off post failure stop the loop. Log
                # rate-limited; FC is still flying on its own state.
                if state.commands_sent < 5 or state.commands_sent % 200 == 0:
                    log.warning("/control/command post failed: %s", e)

        if t0 - last_log >= 10.0:
            last_log = t0
            vl = state.vision_lock
            log.info("translator stats: frames_in=%d sent=%d skipped=%d "
                     "arm=%d disarm=%d mode=%d "
                     "vlock=[engage=%d follow=%d abort=%d cancel=%d "
                     "source=%d esc=%d sub=%d] "
                     "vlock_rejected=[no_sel=%d gate=%d link=%d state=%d "
                     "idem=%d reserved=%d esc_to=%d unstable=%d]",
                     state.frames_in, state.commands_sent,
                     state.commands_skipped_link_loss,
                     state.events_arm, state.events_disarm, state.events_mode,
                     vl.events_engage, vl.events_follow,
                     vl.events_abort, vl.events_cancel_lock,
                     vl.events_source_change, vl.events_escape, vl.events_sub_opcode,
                     vl.rejections_no_selection, vl.rejections_engage_gate,
                     vl.rejections_link_age, vl.rejections_state_stale,
                     vl.rejections_idempotent,
                     vl.rejections_reserved, vl.rejections_escape_timeout,
                     vl.suppressed_unstable)

        await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))


# --- Edge-event handler --------------------------------------------------

class EdgeEventHandler:
    """Detects edges on CH5/6/7 and fires HTTP commands.

    Called from the WebSocket reader's `on_channels` hook. Posts are
    fire-and-forget (background tasks) so the WS reader stays responsive.

    Channel ownership in Wide-mode wire format (12 channels):
      CH1-CH4   sticks         — handled by `_control_sender_loop`
      CH5       arm            — here
      CH6       force-disarm   — here
      CH7       mode 12-pos    — here (single int 0..11)
      CH8       layered ops    — owned ENTIRELY by vision_lock
      CH9-CH12  box coords     — owned by vision_lock
      CH13-CH16 not transmitted (Wide mode = 12 channels)
    """

    def __init__(self, client: httpx.AsyncClient,
                 mode_map: dict[int, str]):
        self._client = client
        self._mode_map = mode_map

    def on_channels(self, state: TranslatorState) -> None:
        us = state.channels_us

        # CH5: arm switch (binary). >=1500 means "armed".
        ch5_armed = us[4] >= 1500
        if state.last_ch5_armed is not None and ch5_armed != state.last_ch5_armed:
            if ch5_armed:
                state.events_arm += 1
                asyncio.create_task(self._post("/arm",
                                               label="arm (CH5↑)"))
            else:
                state.events_disarm += 1
                asyncio.create_task(self._post("/disarm",
                                               label="disarm (CH5↓)"))
        state.last_ch5_armed = ch5_armed

        # CH6: force-disarm (rising edge only). >=1500 forces disarm.
        ch6_forced = us[5] >= 1500
        if state.last_ch6_forced is not None and ch6_forced and not state.last_ch6_forced:
            state.events_disarm += 1
            asyncio.create_task(self._post("/disarm/force",
                                           label="force-disarm (CH6↑)"))
        state.last_ch6_forced = ch6_forced

        # CH7: 12-position mode rotary. Single int (0..11) keys the mode
        # map directly — no more (ch7_pos, mode_overflow) tuple. Edge
        # fires on bucket change.
        ch7_idx = mode_index_from_us(us[6])
        if state.last_mode_idx is not None and ch7_idx != state.last_mode_idx:
            mode_name = self._mode_map.get(ch7_idx)
            if mode_name is None:
                log.warning("unmapped mode index %d — no /mode emit", ch7_idx)
            else:
                state.events_mode += 1
                asyncio.create_task(self._post(
                    "/mode",
                    json_body={"mode": mode_name},
                    label=f"mode → {mode_name} (CH7 idx={ch7_idx})",
                ))
        state.last_mode_idx = ch7_idx

        # CH8 (button_code | box_valid | source_code) and CH9-CH12 (box
        # coords) are handled by vision_lock — see the fanout in `run()`.

    async def _post(self, path: str,
                    json_body: Optional[dict] = None,
                    label: str = "") -> None:
        try:
            r = await self._client.post(path, json=json_body, timeout=2.0)
            log.info("%s → %s %d", label, path, r.status_code)
        except httpx.RequestError as e:
            log.warning("%s → %s FAILED: %s", label, path, e)


# --- Public entry --------------------------------------------------------

async def run(drone_api: str, ws_url: str) -> None:
    """Start the translator. Runs forever; reconnects WS on errors;
    keeps the 50 Hz sender alive even when WS is down (sender will
    just hit link-loss and not send)."""
    state = TranslatorState()
    log.info("translator starting (mode map: %d entries)", len(PHONE_MODE_MAP))
    log.info("stick mapping: throttle=ch[0], roll=ch[1], pitch=ch[2], yaw=ch[3]")

    async with httpx.AsyncClient(base_url=drone_api, timeout=2.0) as client:
        edge   = EdgeEventHandler(client, PHONE_MODE_MAP)
        vlock  = vision_lock.VisionLockHandler(client)

        # Both handlers run on every frame. Existing edge handler keeps
        # CH1-CH8 semantics; vlock owns CH9-CH16 (vision-lock contract).
        def _fanout(s: TranslatorState) -> None:
            edge.on_channels(s)
            vlock.on_channels(s)

        await asyncio.gather(
            _ws_reader_loop(ws_url, state, _fanout),
            _control_sender_loop(client, state),
        )
