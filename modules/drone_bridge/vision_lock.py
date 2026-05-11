"""Vision-lock channel state machine — 12-channel Wide-mode wire format
(CH7 12-pos mode + CH8 layered ops field, redesigned 2026-05-02).

Drone-side of the FINAL CONTRACT for phone-driven vision-lock control
over ELRS. Receives CRSF channels via translator.py; dispatches HTTP
commands to drone-control's /vision/* and /vtx/* endpoints.

WIRE FORMAT (NovaApp's CH1=throttle convention; ELRS 100 Hz Full + Wide):

  CH1-CH4   sticks                       (10-bit analog, owned by translator)
  CH5       arm                          (binary, owned by translator)
  CH6       force-disarm                 (momentary, owned by translator)
  CH7       mode 12-pos rotary           (owned by translator)
  CH8       LAYERED OPS FIELD            (6-bit; owned ENTIRELY by vision_lock)
  CH9       Box X1                       (6-bit → norm 0..1023)
  CH10      Box Y1                       (6-bit → norm 0..1023)
  CH11      Box X2                       (6-bit → norm 0..1023)
  CH12      Box Y2                       (6-bit → norm 0..1023)
  CH13-CH16 NOT TRANSMITTED              (Wide mode carries 12 channels)

CH8 LAYERED OPS FIELD (6 bits, decoded with quantize_64 → three sub-fields):

  bits 0-2 (3 bits, mask 0x07): button_code  — 8 codes (momentary, idle→non-zero edge)
    0  IDLE
    1  ENGAGE        rising → /vision/engage     (gated by 6-rule contract)
    2  FOLLOW        rising → /vision/follow
    3  ABORT         rising → /vision/abort
    4  CANCEL_LOCK   rising → /vision/cancel-lock
    5  RESERVED      log + reject (must never be sent by phone)
    6  RESERVED      log + reject
    7  ESCAPE        next frame's full 6 bits = sub-opcode 0..63

  bit 3   (1 bit, mask 0x08): box_valid       — persistent rule-1 gate
  bits 4-5 (2 bits, mask 0x30): source_code   — persistent VTX source select
    0 BLACK   1 FRONT   2 GROUND   3 VISION

The button_code field is MOMENTARY (phone sets the code, holds ~150 ms,
returns to IDLE). Drone fires on idle → non-zero transition. Only ONE
button can be active per frame (3-bit field, not bitfield). box_valid
and source_code are PERSISTENT levels.

ESCAPE pattern: button_code == 7 puts the decoder into ESCAPE_PENDING
state. The next frame's full quantize_64 value is interpreted as a
sub-opcode (0..63). 200 ms timeout if the next frame doesn't arrive
in time — abandons the wait rather than misinterpreting a stale frame.
No sub-opcodes wired up today; logged + discarded.

Coord space: NORMALIZED 0..1023 over the source camera frame. drone-control
scales to pixels at engage time using frame_w/frame_h from /vision/state.
Phone NEVER sends raw pixels. 6-bit precision = 16 quantization steps
on each axis ≈ 20 px snap on a 1280-wide source.

Failsafe coupling: firmware drives CH8 → 0 (all sub-fields zero: button
IDLE, box invalid, source BLACK) on host silence. Link drops cannot
fabricate a valid box, button press, or source switch. This module
relies on that property; reverting the firmware patch makes this state
machine unsafe.

Six-rule behavior contract: rules 2a/2b/2c/2d, 4, and 5 are gates
applied in `_on_engage_rising` / `_engage_io_phase`. Search this file
for `# Rule N` to find each one — wire format changed; contract did not.
"""

from __future__ import annotations

import asyncio
import enum
import json as _json
import logging
import os
import time
import urllib.error as _urllib_error
import urllib.request as _urllib_request
from dataclasses import dataclass, field
from typing import Optional

import httpx


log = logging.getLogger("drone_bridge.vision_lock")


# --- Channel layout (zero-indexed) --------------------------------------

CH8_INDEX  = 7      # CH8  — layered ops field (see decode_ch8 below)
CH_BOX_X1  = 8      # CH9  — 6-bit normalized X1
CH_BOX_Y1  = 9      # CH10 — 6-bit normalized Y1
CH_BOX_X2  = 10     # CH11 — 6-bit normalized X2
CH_BOX_Y2  = 11     # CH12 — 6-bit normalized Y2

# --- CH8 layered-ops field (applied AFTER quantize_64 to position 0..63) ---
# Three sub-fields packed into 6 bits:
#   bits 0-2 button_code  (8 codes — only one button active per frame)
#   bit  3   box_valid    (rule-1 gate, persistent level)
#   bits 4-5 source_code  (4 VTX sources, persistent level)
# Composed phone-side as `code | (box_valid << 3) | (source_code << 4)`,
# carried over Wide-mode CRSF as a 64-position channel, recovered drone-
# side via quantize_64 + mask-and-shift in decode_ch8().

OPS_BUTTON_MASK   = 0b000111         # bits 0-2
OPS_BOX_VALID_BIT = 0b001000         # bit 3
OPS_SOURCE_MASK   = 0b110000         # bits 4-5
OPS_SOURCE_SHIFT  = 4

# Button codes (3-bit field, idle→non-zero is the rising edge).
BUTTON_CODE_IDLE        = 0
BUTTON_CODE_ENGAGE      = 1
BUTTON_CODE_FOLLOW      = 2
BUTTON_CODE_ABORT       = 3
BUTTON_CODE_CANCEL_LOCK = 4
BUTTON_CODE_RESERVED_5  = 5          # must never be sent by phone — log + reject
BUTTON_CODE_RESERVED_6  = 6          # must never be sent by phone — log + reject
BUTTON_CODE_ESCAPE      = 7          # opens 2-frame extension protocol

# VTX source codes (2-bit field, persistent). Mapped to /vtx/source names.
SOURCE_CODE_BLACK  = 0
SOURCE_CODE_FRONT  = 1
SOURCE_CODE_GROUND = 2
SOURCE_CODE_VISION = 3
SOURCE_NAME_BY_CODE = {
    SOURCE_CODE_BLACK:  "black",
    SOURCE_CODE_FRONT:  "front",
    SOURCE_CODE_GROUND: "ground",
    SOURCE_CODE_VISION: "vision",
}

# ESCAPE timeout: if the sub-opcode frame doesn't arrive within this
# window, abandon the wait. Bounded by next-frame arrival (timeout check
# runs on each on_channels tick, not a separate timer) — at 50 Hz CRSF
# rate the actual fire-time may be one frame (~20 ms) late.
ESCAPE_TIMEOUT_MS = 200.0

# After accepting a sub-opcode, the phone-side hold may extend past
# acceptance — multiple CH8 frames continue to carry the same reserved
# code value. Suppress reserved-code rejections for matches inside this
# grace window so the counter signature reflects intent (one sub-op
# event), not wire residue.
SUBOP_TRAILING_GRACE_MS = 250.0


# --- CH8 OPTION D: 16-code flat table (drift-robust replacement) --------
#
# The bit-packed layered field above (button_code | box_valid | source_code)
# was empirically insufficient for sustained wire drift on this radio.
# Stage B end-to-end smoke (UTC 09:21-09:37 on 2026-05-02) showed:
#   - VISION 60 s hold: drift held every keep-alive frame at pos 49
#     (= ENGAGE+VISION). Stability gate accepted (3+ identical frames).
#     Two ENGAGE rising-edges fired internally; rule 2d caught both.
#   - FRONT 60 s hold: drift held at pos 15 (= ESCAPE+box+BLACK). 24×
#     ESCAPE/sub-opcode 15 cycles. FRONT button silently non-functional
#     on the wire.
#
# Option D collapses the 64-position bit-packed space to 16 flat codes
# at 4-position spacing (~65 µs centers). Each code occupies four wire
# positions; drift up to ~24 µs upper / ~33 µs lower (asymmetric due
# to integer rounding) cannot cross a code boundary. Margin vs measured
# ~11 µs drift: ~2-3×, well above the previous ~16 µs button-boundary
# margin of the bit-packed layout.
#
# Codes 0..9 are assigned semantics; 10..15 are reserved for future
# allocation OR sub-opcode space (when entered via ESCAPE prefix).
# 16 × 16 = 256 extended codes via 2-frame ESCAPE protocol when needed.

OPS_CODE_IDLE             = 0   # source BLACK / no overlay (matches failsafe MIN)
OPS_CODE_SRC_FRONT        = 1   # source FRONT
OPS_CODE_SRC_GROUND       = 2   # source GROUND
OPS_CODE_SRC_VISION       = 3   # source VISION (no box drawn yet)
OPS_CODE_VISION_BOX       = 4   # source VISION + box committed (CH9-12 carry coords)
OPS_CODE_BTN_ENGAGE       = 5   # ENGAGE pulse (only valid from prev=VISION_BOX)
OPS_CODE_BTN_FOLLOW       = 6   # FOLLOW pulse (only valid from prev=VISION_BOX)
OPS_CODE_BTN_ABORT        = 7   # ABORT  pulse (only valid from prev=VISION_BOX)
OPS_CODE_BTN_CANCEL_LOCK  = 8   # CANCEL_LOCK pulse (only valid from prev=VISION_BOX)
OPS_CODE_ESCAPE           = 9   # 2-frame extension prefix (next frame = sub-opcode)

# Codes 10..15: reserved. Outside an ESCAPE window they are rejected
# (stats counter increments, no dispatch). Inside ESCAPE_PENDING they
# are interpreted as sub-opcodes 10..15.

OPS_CODE_MIN_RESERVED = 10
OPS_CODE_MAX          = 15

# Button codes (5..8) require the IMMEDIATELY-PRECEDING decoded code to
# be VISION_BOX (4). Phone always emits the button briefly then returns
# to VISION_BOX; this constraint catches any other-state button frame
# as a wire fault (e.g., direct 0→5 transition = impossible from phone,
# would mean a multi-position drift event).
BUTTON_CODES_REQUIRING_VISION_BOX = frozenset({
    OPS_CODE_BTN_ENGAGE, OPS_CODE_BTN_FOLLOW,
    OPS_CODE_BTN_ABORT,  OPS_CODE_BTN_CANCEL_LOCK,
})

# Source-state codes (codes that set the prev_source level).
SOURCE_NAME_BY_OPS_CODE = {
    OPS_CODE_IDLE:        "black",
    OPS_CODE_SRC_FRONT:   "front",
    OPS_CODE_SRC_GROUND:  "ground",
    OPS_CODE_SRC_VISION:  "vision",
    OPS_CODE_VISION_BOX:  "vision",   # same source as SRC_VISION; box is the addition
}

# --- µs → 6-bit quantization bounds (Wide mode) -------------------------
# CRSF µs span on the wire (failsafe MIN / valid-frame MAX).
WIDE_US_MIN  = 988
WIDE_US_MAX  = 2012
WIDE_POS_MAX = 63          # 6-bit channel = 64 positions [0..63]

# Edge guard bands. ELRS 12ch Mixed mode adds ~11 µs of upward drift on
# MIN (observed: phone sends CRSF 172 = 988 µs, RX outputs 999 µs). With
# strict round() decoding this lands at position 1, which under the new
# layered ops field would falsely decode button_code=1 (ENGAGE) on every
# idle frame — silently firing /vision/engage rejections (or worse,
# commits) on any frame where the operator hasn't pressed anything.
# The guard absorbs this drift without disturbing legitimate pos=1+
# encodings (phone's pos=1 sits at 1004 µs ≥ 1000). Symmetric guard
# on MAX edge for the same reason.
MIN_GUARD_US = 1000        # us ≤ this → pos 0 (absorbs ~12 µs MIN drift)
MAX_GUARD_US = 2000        # us ≥ this → pos 63 (symmetric MAX drift)

NORM_RANGE   = 1024        # normalized output coord range expected by api_gateway


# --- Contract thresholds (env-tunable; see contract rule numbers) -------

# Rule 1: stable across this many consecutive frames before READY (~60 ms at 50 Hz).
SELECTION_DEBOUNCE_FRAMES = 3

# Rule 2a: minimum stability time before ENGAGE may commit (overlaps with
# rule 1's debounce — they don't stack).
ENGAGE_GATE_MS = 100.0

# Rule 2b: /vision/state seq must have advanced within this window (heartbeat).
STATE_FRESHNESS_MS = 500.0

# Rule 2c: max channel-frame age at DISPATCH time (not receive time).
LINK_AGE_MS = 500.0

# Rule 3: IoU × confidence threshold to choose class-lock over KCF.
IOU_CLASS_THRESHOLD = float(os.environ.get("VISION_LOCK_IOU_THRESHOLD", "0.3"))

# Rule 4: IoU threshold for "selection matches current target → no-op".
# Distinct from rule 3's threshold — different job, different value.
IOU_IDEMPOTENCY_THRESHOLD = float(
    os.environ.get("VISION_LOCK_IDEMPOTENCY_THRESHOLD", "0.5")
)

# Vision state probe — used by hardening gates (rules 2b, 4) at engage
# time to read {seq, locked_box, frame_w, frame_h}. Hits vision-detect
# directly (sibling on host network), not through drone-control: this
# read is purely informational, doesn't need to traverse the command
# plane, and works even if drone-control is restart-looping (no FC).
VISION_STATE_URL = os.environ.get(
    "VISION_STATE_URL", "http://127.0.0.1:8081/state"
)
# Cap on /state probe latency so an engage never stalls the WS reader
# on a slow vision pipeline. 250 ms is well under the 500 ms freshness
# rule — if /state takes longer, we treat it as stale anyway.
VISION_STATE_PROBE_TIMEOUT_S = 0.25


# --- Decoder helpers (Wide-mode 6-bit channel decoding) -----------------

def quantize_64(us: int) -> int:
    """Wide mode quantizes 988..2012 µs to 64 positions (6-bit). Pure.

    Inverse of the phone-side encoder: phone picks `pos ∈ [0..63]`,
    encodes to µs, ELRS quantizes the wire-form back to a position;
    the RX outputs the µs at that step. We map back to position.

    Math: linear scale (us - 988) / 1024 * 63 (= 64 positions over a
    span of 1024 µs, so 1024/63 ≈ 16.25 µs per step). Verified
    round-trip identity across all 64 phone-encoder outputs and
    bitwise-equivalent to the phone-side reference formulation
    `round((us - 988) * 63 / 1024)`. Do NOT "simplify" to `(us-988)/16`
    — that's the 1024/64 form and accumulates a position-step error
    at higher µs values.

    Edge guards (MIN_GUARD_US / MAX_GUARD_US) absorb the ~11 µs of
    upward drift introduced by ELRS 12ch Mixed mode quantization —
    without them, phone-intended pos=0 (us=988) reads back as us=999
    and decodes to pos=1, falsely setting bit 0 (which under the new
    layered ops field is the LSB of button_code, decoding to ENGAGE).
    The middle range still uses the precise scaling; pos=1 (us≈1004)
    is preserved because 1004 > MIN_GUARD_US.
    """
    if us <= MIN_GUARD_US:
        return 0
    if us >= MAX_GUARD_US:
        return WIDE_POS_MAX
    return int(round((us - WIDE_US_MIN) / (WIDE_US_MAX - WIDE_US_MIN) * WIDE_POS_MAX))


def decode_box_norm_from_us(coord_us: int) -> int:
    """CH9-12 box coord: µs → 6-bit position → normalized 0..1023.

    Two-step mapping: first quantize to the same 6-bit grid the phone
    encoded on, then linearly stretch [0..63] to [0..1023] for the
    downstream IoU resolver. The phone's encode is the inverse — it
    takes a normalized 0..1023 coord, maps to a position 0..63, and
    sends that as a Wide-mode µs.

    Precision: 64 quantization steps over (typically) 1280-wide source
    → ~20 px snap on each axis. Documented intentional loss vs CH13-16
    full 10-bit; tradeoff for delivering CH9-12 reliably.
    """
    pos = quantize_64(coord_us)
    return int(round(pos * (NORM_RANGE - 1) / WIDE_POS_MAX))


def decode_ch8(ch8_us: int) -> dict:
    """CH8 µs → layered ops field {button_code, box_valid, source_code}.

    DEPRECATED — kept for the cutover window only. Replaced by
    `decode_ops_code` (16-code flat table). Will be removed once
    Option D is shipped on both sides AND validated on the wire.

    Pure function. Quantizes µs to a 6-bit position, then splits into
    the three sub-fields. Callers do not see the raw position — the
    contract is the named fields, not the bit packing.
    """
    pos = quantize_64(ch8_us)
    return {
        "button_code": pos & OPS_BUTTON_MASK,
        "box_valid":   bool(pos & OPS_BOX_VALID_BIT),
        "source_code": (pos & OPS_SOURCE_MASK) >> OPS_SOURCE_SHIFT,
    }


def decode_ops_code(us: int) -> int:
    """CH8 µs → flat ops code in [0..15] (Option D).

    Each code occupies 4 wire positions (~65 µs centers). Decoder snaps
    the incoming 6-bit position to the nearest 4-multiple via
    `(pos + 2) // 4`. Code C is centered at pos = 4*C and owns positions
    [4C - 2, 4C + 1] inclusive (asymmetric due to integer rounding).

    Drift survival: each code center has ~33 µs lower margin and ~24 µs
    upper margin before crossing into a neighbor code. Measured wire
    drift on this radio is ±~11 µs, so margin is ~2-3× — well above
    the bit-packed layout's ~16 µs margin (which empirically failed
    on Stage B VISION + FRONT holds).

    Failsafe: us <= 988 (MIN) → code 0 = OPS_CODE_IDLE = source BLACK.
    Matches firmware `failsafe_channels[7] = MIN`. Link drops cannot
    fabricate a button or non-BLACK source.

    Pure function. Caller is responsible for state-transition logic
    (which transitions are legal, which fire dispatches).
    """
    if us <= MIN_GUARD_US:
        return OPS_CODE_IDLE
    if us >= MAX_GUARD_US:
        return OPS_CODE_MAX
    pos = quantize_64(us)
    code = (pos + 2) // 4
    return max(0, min(OPS_CODE_MAX, code))


# --- Selection state machine --------------------------------------------

class SelectionPhase(enum.Enum):
    """Per-frame selection state. Explicit; no implicit-counter logic.

    Transitions (driven once per WS frame by `_advance_selection_fsm`):
      IDLE        — CH8 bit 5 (BOX_VALID) clear. Box ignored.
      STABILIZING — bit 5 set + CH9-12 not yet stable for SELECTION_DEBOUNCE_FRAMES.
      READY       — Stable; box may be committed by an engage_pulse rising edge.
      DISPATCHED  — engage_pulse rising edge fired and we've forwarded to
                    drone-control. Returns to IDLE/STABILIZING/READY on
                    the next frame based on current selection state —
                    DISPATCHED is a one-frame marker for logging only.
    """
    IDLE        = "idle"
    STABILIZING = "stabilizing"
    READY       = "ready"
    DISPATCHED  = "dispatched"


@dataclass
class NormBox:
    """Normalized 0..1023 coords as sent over the wire. Origin top-left."""
    x1: int
    y1: int
    x2: int
    y2: int


# --- Button stability gate (defense-in-depth against CH8 drift) ---------

@dataclass
class ButtonStabilityGate:
    """3-frame consecutive-stability filter for CH8 button codes 1..4.

    ## What this filters
    TRANSIENT (1-2 frame) drift events that briefly mis-quantize CH8
    to an adjacent position. The CH8 layered ops field has only ~8 µs
    of symmetric margin per position — BELOW the empirically measured
    ~11 µs upward drift on this radio. Without this gate, a single
    drift frame on a "source-only" hold (e.g., source=GROUND at pos 32)
    can read as pos 33 (source=GROUND + button_code=ENGAGE) and falsely
    fire /vision/engage on a target the operator did not select.

    ## What this does NOT filter
    SUSTAINED drift — persistent multi-frame µs offset like the 11 µs
    MIN-edge offset observed at startup (988 → 999 µs sustained for
    ~5 seconds). If a mid-range CH8 position exhibits sustained drift,
    the same code persists across REQUIRED_STABLE_FRAMES and the gate
    DOES dispatch it. The MIN-edge case is already covered by the
    MIN_GUARD_US edge guard in quantize_64; mid-range positions are
    NOT covered.

    ## Why this is acceptable
    Defense-in-depth, not a complete fix. Phase 4 of the coordinated
    smoke (60-sec hold per source-only CH8 position; count spurious
    button dispatches) confirms whether mid-range drift is transient
    (gate sufficient) or sustained (escalate to wire-format mitigation:
    PAD bit between fields, or compress CH8 to 32-position 5-bit field).
    Do NOT treat the gate's existence as evidence that the drift problem
    is "solved" — it is one layer.

    ## Latency cost
    Up to 3 CRSF frames ≈ 30 ms at 100 Hz. Phone holds button codes for
    ≥150 ms, so operator-perceived latency is unchanged. ABORT and
    CANCEL_LOCK pay the same 30 ms; still well inside human reaction
    time and orders of magnitude under any FC failsafe window.

    ## Lifecycle
    - consume(0): clears candidate + last-dispatched. A button must be
      released to IDLE between presses to dispatch the same code twice.
    - consume(non-zero): counts consecutive identical frames.
      Dispatches on the REQUIRED_STABLE_FRAMES-th identical frame, but
      only if the candidate differs from the last dispatched code (so
      a held button dispatches once, not every frame).
    - Direct code-to-code transitions (1 → 3 with no IDLE between)
      are supported: the candidate switches and counts up to stable
      from frame 1 of the new code, dispatching ABORT 30 ms later.

    ## Integration constraint
    Reserved codes (5, 6) and ESCAPE (7) MUST bypass this gate. They
    use a 1-frame edge fast path on the caller's side. The gate sees
    only IDLE (0) and codes 1..4. If a reserved or ESCAPE code somehow
    reaches consume(), it would be treated as a normal button code —
    the caller is responsible for routing.
    """
    REQUIRED_STABLE_FRAMES: int = 3
    candidate_code:        int = 0
    candidate_count:       int = 0
    last_dispatched_code:  int = 0

    def consume(self, current_code: int) -> Optional[int]:
        """Returns the code to dispatch (1..4) or None if not yet stable.

        Pure state-machine — no logging, no I/O. Caller logs the
        dispatch + suppression events as needed.
        """
        if current_code == 0:
            self.candidate_code = 0
            self.candidate_count = 0
            self.last_dispatched_code = 0
            return None
        if current_code == self.candidate_code:
            self.candidate_count += 1
        else:
            self.candidate_code = current_code
            self.candidate_count = 1
        if (self.candidate_count >= self.REQUIRED_STABLE_FRAMES
                and self.candidate_code != self.last_dispatched_code):
            self.last_dispatched_code = self.candidate_code
            return self.candidate_code
        return None

    def reset(self) -> None:
        """Hard reset — used when ESCAPE consumes a frame and we want
        the next normal frame to start counting fresh, not pick up a
        stale candidate from before the ESCAPE."""
        self.candidate_code = 0
        self.candidate_count = 0
        self.last_dispatched_code = 0


@dataclass
class VisionLockState:
    """All vision-lock sub-state lives here; attached to TranslatorState
    as state.vision_lock so we don't touch TranslatorState's existing fields.

    Counters are exposed via translator's stats logger via `snapshot()`."""
    phase: SelectionPhase = SelectionPhase.IDLE
    candidate_box:    Optional[NormBox] = None   # latest decoded selection (any phase)
    stable_box:       Optional[NormBox] = None   # last box that has been READY
    stable_since_mono: Optional[float]  = None   # monotonic ts of READY entry
    debounce_frames_remaining: int = 0           # rule 1 countdown

    # Button-code edge tracker. Single 3-bit field; idle (0) → non-zero
    # is the rising edge. Initialized to IDLE so the very first frame
    # after startup with code=0 fires nothing (correct), and a first
    # frame already carrying a non-zero code fires the dispatcher (also
    # correct — operator may have been holding the button across the
    # restart, and we should honor it).
    last_button_code: int = BUTTON_CODE_IDLE

    # Source-code level tracker. Default 0 (BLACK) matches both the
    # firmware failsafe value and api_gateway's BroadcastRouter default,
    # so a startup frame carrying source=0 fires no spurious POST.
    last_source_code: int = SOURCE_CODE_BLACK

    # ESCAPE state machine. NORMAL = read button + source as usual;
    # ESCAPE_PENDING = next frame is sub-opcode (codes 10..15 in Option D
    # flat-table layout; raw 6-bit value in legacy bit-packed layout).
    # Timeout in monotonic seconds; ESCAPE_TIMEOUT_MS guards against
    # missing the follow-up frame.
    escape_state:        str   = "NORMAL"
    escape_started_mono: float = 0.0

    # Option D flat-code state. Replaces (last_button_code, last_source_code,
    # box_valid bit) with a single ops-code level + helper bookkeeping.
    # Defaults match the firmware failsafe (code 0 = IDLE = source BLACK).
    prev_ops_code:       int  = OPS_CODE_IDLE   # last decoded code (0..15)
    prev_source_name:    str  = "black"         # last source name we POSTed
    prev_box_committed:  bool = False           # True iff prev_ops_code was
                                                # VISION_BOX or a button-from-VISION_BOX
    pre_escape_ops_code: int  = OPS_CODE_IDLE   # snapshot at ESCAPE entry so the
                                                # post-sub-opcode resync doesn't
                                                # spuriously re-fire transitions
    last_subop_code:        int   = -1          # last sub-opcode consumed; -1 = none
    last_subop_consumed_mono: float = 0.0       # monotonic time of consumption

    # LEGACY (Round 2 bit-packed CH8 layered field).
    # Stability gate for button codes 1..4 — kept around but no longer wired
    # into Option D dispatch. The gate was empirically insufficient against
    # sustained drift (Stage B VISION 60 s hold fired 2 spurious ENGAGE
    # rising-edges). Replaced by the 4-position spacing of the flat-code
    # layout, which makes drift-into-button-position structurally impossible.
    # Class definition kept in case operator-feedback later shows we need
    # double-tap debounce — re-enable by wrapping `consume()` at dispatch.
    last_button_code: int = BUTTON_CODE_IDLE
    last_source_code: int = SOURCE_CODE_BLACK
    button_gate: ButtonStabilityGate = field(default_factory=ButtonStabilityGate)

    # Rule 5 telemetry. The contract says "during FOLLOW, selection-
    # channel changes alone never re-lock" — that's already structurally
    # true because commit requires engage_pulse rising edge regardless
    # of FOLLOW state. This flag is purely for logging / future autonomy.
    is_follow_active: bool = False

    # Counters
    events_engage:        int = 0
    events_follow:        int = 0
    events_abort:         int = 0
    events_cancel_lock:   int = 0
    events_source_change: int = 0       # /vtx/source POSTs fired
    events_escape:        int = 0       # ESCAPE button received
    events_sub_opcode:    int = 0       # sub-opcode frame received
    rejections_no_selection: int = 0    # rule 2d
    rejections_state_stale:  int = 0    # rule 2b
    rejections_link_age:     int = 0    # rule 2c
    rejections_engage_gate:  int = 0    # rule 2a
    rejections_idempotent:   int = 0    # rule 4
    rejections_reserved:     int = 0    # button code 5 or 6
    rejections_escape_timeout: int = 0  # ESCAPE without follow-up frame
    suppressed_unstable:     int = 0    # codes 1-4 filtered by button_gate
                                        # (transient-drift mitigation;
                                        # high counter = real frame instability)


# --- Channel decode + state-machine tick ---------------------------------

def _decode_selection(us: list[int]) -> Optional[NormBox]:
    """Read CH8 ops code + CH9-CH12 → NormBox, else None.

    Validity gate (Option D): CH8 must decode to OPS_CODE_VISION_BOX or
    one of the button codes (5..8). Buttons inherit the box state from
    the VISION_BOX they came from — phone always returns to VISION_BOX
    after a button pulse, and box coords on CH9-12 stay set across the
    pulse. If CH8 is anything else (IDLE / SRC_FRONT / SRC_GROUND /
    SRC_VISION-without-box / ESCAPE / reserved), CH9-12 contents are
    ignored regardless of value. Caller debounces.

    Pure function — no state mutation.
    """
    code = decode_ops_code(us[CH8_INDEX])
    if code != OPS_CODE_VISION_BOX and code not in BUTTON_CODES_REQUIRING_VISION_BOX:
        return None
    return NormBox(
        x1=decode_box_norm_from_us(us[CH_BOX_X1]),
        y1=decode_box_norm_from_us(us[CH_BOX_Y1]),
        x2=decode_box_norm_from_us(us[CH_BOX_X2]),
        y2=decode_box_norm_from_us(us[CH_BOX_Y2]),
    )


def _advance_selection_fsm(vl: VisionLockState, decoded: Optional[NormBox]) -> None:
    """Advance the selection FSM by one frame.

    Transitions:
      decoded is None                       → IDLE; clear stable_box.
      decoded != candidate_box              → STABILIZING; reset debounce.
      decoded == candidate_box, frames left → STABILIZING; tick down.
      decoded == candidate_box, frames done → READY; freeze stable_box.
    """
    if decoded is None:
        vl.phase = SelectionPhase.IDLE
        vl.candidate_box = None
        vl.stable_box = None
        vl.stable_since_mono = None
        vl.debounce_frames_remaining = 0
        return

    if vl.candidate_box != decoded:
        # New candidate (or first one) — restart debounce.
        vl.candidate_box = decoded
        vl.debounce_frames_remaining = SELECTION_DEBOUNCE_FRAMES
        vl.phase = SelectionPhase.STABILIZING
        vl.stable_box = None
        vl.stable_since_mono = None
        return

    # Same candidate as last frame — tick the debounce.
    if vl.debounce_frames_remaining > 0:
        vl.debounce_frames_remaining -= 1
        if vl.debounce_frames_remaining == 0:
            vl.phase = SelectionPhase.READY
            vl.stable_box = decoded
            vl.stable_since_mono = time.monotonic()
        else:
            vl.phase = SelectionPhase.STABILIZING


# --- Hardening helpers (rules 2b, 4) ------------------------------------

def _probe_vision_state() -> Optional[dict]:
    """Synchronous GET /vision/state for engage-time freshness + idempotency
    gates. Returns None on any failure (timeout, connection error, parse).

    Always invoked via `loop.run_in_executor(None, _probe_vision_state)`
    so the WS reader hot loop never blocks on the urllib call. Timeout
    bounded by VISION_STATE_PROBE_TIMEOUT_S.
    """
    try:
        with _urllib_request.urlopen(
            VISION_STATE_URL, timeout=VISION_STATE_PROBE_TIMEOUT_S
        ) as r:
            return _json.loads(r.read().decode("utf-8"))
    except (_urllib_error.URLError, OSError, ValueError):
        return None


def _iou_xyxy(a: list[int], b: list[int]) -> float:
    """Standard IoU on [x1,y1,x2,y2] boxes. Returns 0.0 for malformed
    or non-overlapping input — never raises. (Pure function — same shape
    as the one in api_gateway.py, deliberately duplicated to keep this
    module zero-dep on drone-control.)"""
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1 = max(ax1, bx1); iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2); iy2 = min(ay2, by2)
    iw = max(0, ix2 - ix1); ih = max(0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    a_area = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    b_area = max(0, bx2 - bx1) * max(0, by2 - by1)
    union = a_area + b_area - inter
    if union <= 0:
        return 0.0
    return inter / union


def _norm_box_to_pixels(box: NormBox, frame_w: int, frame_h: int) -> list[int]:
    """Convert NormBox (0..1023) to pixel-space [x1,y1,x2,y2] using the
    live frame_w/frame_h from /vision/state. Mirrors the canonical
    scaling in api_gateway.py — phone NEVER sends raw pixels.
    """
    x1 = int(round(min(box.x1, box.x2) * frame_w / NORM_RANGE))
    y1 = int(round(min(box.y1, box.y2) * frame_h / NORM_RANGE))
    x2 = int(round(max(box.x1, box.x2) * frame_w / NORM_RANGE))
    y2 = int(round(max(box.y1, box.y2) * frame_h / NORM_RANGE))
    x1 = max(0, min(frame_w - 2, x1))
    y1 = max(0, min(frame_h - 2, y1))
    x2 = max(x1 + 1, min(frame_w - 1, x2))
    y2 = max(y1 + 1, min(frame_h - 1, y2))
    return [x1, y1, x2, y2]


# --- Edge-event handler --------------------------------------------------

class VisionLockHandler:
    """Owns the HTTP client to drone-control and reacts to channel frames.

    `on_channels(state)` is called from translator's WS reader after every
    decoded frame. All HTTP work goes into background tasks via
    asyncio.create_task — never block the WS reader.
    """

    def __init__(self, client: httpx.AsyncClient):
        self._client = client

    def on_channels(self, ts_state) -> None:
        """Called per WS frame. Reads ts_state.channels_us +
        ts_state.last_rc_ts; owns ts_state.vision_lock for sub-state.

        Option D dispatch — flat 16-code CH8:
          1. Selection FSM (CH8 = VISION_BOX or button code + CH9-CH12)
             → IDLE/STABILIZING/READY for rule-1 box debounce.
          2. ESCAPE timeout check + sub-opcode consumption (ESCAPE prefix
             → next frame as sub-opcode 10..15).
          3. Flat-code state machine on `code != prev_ops_code` transitions:
             - source codes (1..3): POST /vtx/source AND /inet/source
               (code 0 is pulse-shape IDLE — no source dispatch)
             - VISION_BOX (4): set source=vision + box_committed=True
             - button codes (5..8): require prev=VISION_BOX, fire 6-rule contract
             - ESCAPE (9): enter ESCAPE_PENDING
             - reserved (10..15) outside ESCAPE: log + reject

        Drift-robust: each code occupies 4 wire positions. Drift can no
        longer cross code boundaries (vs the bit-packed layout's narrow
        button-code adjacency that needed the stability gate).
        """
        vl: VisionLockState = ts_state.vision_lock
        us = ts_state.channels_us

        # --- 1. Selection FSM (rule 1 box debounce) ------------------
        _advance_selection_fsm(vl, _decode_selection(us))

        # --- 2. ESCAPE timeout check ---------------------------------
        # If we're waiting for a sub-opcode and too much time has passed,
        # abandon the wait. Check fires at most once per frame; if frames
        # stop entirely, the warning is delayed until link recovery.
        now_mono = time.monotonic()
        if vl.escape_state == "ESCAPE_PENDING":
            if (now_mono - vl.escape_started_mono) * 1000.0 > ESCAPE_TIMEOUT_MS:
                vl.rejections_escape_timeout += 1
                log.warning("CH8 ESCAPE timeout — abandoning sub-opcode wait")
                vl.escape_state = "NORMAL"
                # Resync prev_ops_code to the snapshot we took at ESCAPE
                # entry. Any post-timeout transition will compare against
                # the pre-ESCAPE state, not the ESCAPE code itself, so we
                # don't spuriously re-fire.
                vl.prev_ops_code = vl.pre_escape_ops_code

        code = decode_ops_code(us[CH8_INDEX])

        # --- 3. ESCAPE_PENDING: this frame IS the sub-opcode ---------
        if vl.escape_state == "ESCAPE_PENDING":
            if OPS_CODE_MIN_RESERVED <= code <= OPS_CODE_MAX:
                vl.events_sub_opcode += 1
                log.info("CH8 ESCAPE sub-opcode received: %d (no handlers; reserved)",
                         code)
                vl.escape_state = "NORMAL"
                vl.prev_ops_code = vl.pre_escape_ops_code
                # Record consumption so the trailing portion of the
                # phone-side hold doesn't fire spurious reserved warnings.
                vl.last_subop_code = code
                vl.last_subop_consumed_mono = now_mono
            elif code == OPS_CODE_ESCAPE:
                # Continued ESCAPE — Tx slot contention absorber.
                # Why: the Ranger Micro Tx in 100 Hz Mixed mode runs its own
                # RF-slot scheduler with a single channel-source buffer.
                # If the phone-side writes ESCAPE / sub-op / VISION_BOX into
                # the Tx faster than one slot period (~10 ms), the Tx may
                # emit ESCAPE on multiple consecutive RF frames before the
                # sub-op write reaches the next slot. Treating repeated
                # ESCAPE as "still waiting" absorbs this without weakening
                # the safety guard: reserved codes (10..15) still require an
                # OPEN window for sub-op acceptance. Refresh the timer so
                # the 200 ms budget restarts from now.
                vl.escape_started_mono = now_mono
            else:
                # Unexpected follow-up: phone returned to VISION_BOX or
                # another non-reserved code before sub-op landed. Wire-level
                # fault or operator released ESCAPE early. Abandon.
                vl.rejections_escape_timeout += 1
                log.warning("CH8 ESCAPE follow-up frame was code %d, not a "
                            "sub-opcode (10..15) — abandoning", code)
                vl.escape_state = "NORMAL"
                vl.prev_ops_code = vl.pre_escape_ops_code
            return

        # --- 4. NORMAL state: dispatch on code transitions -----------
        # Idempotent on no-change: held VISION_BOX (5 Hz keep-alive)
        # only fires source dispatch once.
        if code == vl.prev_ops_code:
            return

        # Transition handlers — exactly one branch fires per call.
        if code == OPS_CODE_IDLE:
            # Code 0 is the pulse-shape IDLE between rising-edge ops codes —
            # the phone sends IDLE → CODE → IDLE (~200 ms total) for source
            # / button taps. The trailing IDLE must NOT be re-interpreted
            # as "switch to BLACK source" or every tap ends at BLACK
            # regardless of the operator's selection. To go BLACK explicitly,
            # the phone must use a dedicated path (BLE direct POST to
            # /inet/source name=black, or a future dedicated ops code).
            #
            # Selection-state side effects still need to clear though: any
            # operator return to IDLE means no box is committed for an
            # upcoming button press.
            vl.prev_box_committed = False

        elif code == OPS_CODE_SRC_FRONT:
            self._dispatch_source_change(vl, "front", code)
            vl.prev_box_committed = False

        elif code == OPS_CODE_SRC_GROUND:
            self._dispatch_source_change(vl, "ground", code)
            vl.prev_box_committed = False

        elif code == OPS_CODE_SRC_VISION:
            # source=VISION but no box committed (or box just cleared)
            self._dispatch_source_change(vl, "vision", code)
            vl.prev_box_committed = False

        elif code == OPS_CODE_VISION_BOX:
            # source=VISION + box committed. Source POST only if changing FROM
            # a non-vision source — VISION_BOX inherits "vision" from SRC_VISION.
            self._dispatch_source_change(vl, "vision", code)
            vl.prev_box_committed = True

        elif code in BUTTON_CODES_REQUIRING_VISION_BOX:
            # Buttons 5..8: only valid IMMEDIATELY after VISION_BOX state.
            # If we got here from anywhere else, it's a wire fault (drift
            # cannot cross 4 code positions; only an actual phone-side bug
            # or multi-position correlated wire glitch could explain it).
            if vl.prev_ops_code != OPS_CODE_VISION_BOX:
                vl.rejections_no_selection += 1
                log.warning("CH8 button code %d ignored — prev was code %d, "
                            "not VISION_BOX (4)", code, vl.prev_ops_code)
            else:
                if code == OPS_CODE_BTN_ENGAGE:
                    self._on_engage_rising(ts_state)
                elif code == OPS_CODE_BTN_FOLLOW:
                    self._on_follow_rising(vl)
                elif code == OPS_CODE_BTN_ABORT:
                    self._on_abort_rising(vl)
                elif code == OPS_CODE_BTN_CANCEL_LOCK:
                    self._on_cancel_lock_rising(vl)
            # Button frame doesn't update source/box; phone returns to
            # VISION_BOX after the pulse and source/box state continues.

        elif code == OPS_CODE_ESCAPE:
            vl.escape_state = "ESCAPE_PENDING"
            vl.escape_started_mono = now_mono
            vl.pre_escape_ops_code = vl.prev_ops_code  # snapshot for resync
            vl.events_escape += 1
            log.info("CH8 ESCAPE entered — awaiting sub-opcode (codes 10..15) "
                     "within %.0f ms", ESCAPE_TIMEOUT_MS)
            # Source/box state is unchanged through ESCAPE.

        elif OPS_CODE_MIN_RESERVED <= code <= OPS_CODE_MAX:
            # Codes 10..15 outside an ESCAPE window. Phone-side never sends
            # these except as sub-opcodes — receiving one here means either
            # a future feature without coordinated update or a wire fault.
            # Exception: if we JUST consumed this exact code as a sub-opcode,
            # the wire is still carrying the trailing portion of the same
            # phone-side hold. Suppress to avoid spurious counter increments.
            if (code == vl.last_subop_code
                and (now_mono - vl.last_subop_consumed_mono) * 1000.0
                    < SUBOP_TRAILING_GRACE_MS):
                pass  # idempotent: state advances via prev_ops_code below
            else:
                vl.rejections_reserved += 1
                log.warning("CH8 reserved code %d outside ESCAPE window — ignored",
                            code)

        vl.prev_ops_code = code

    # --- Source-change dispatch helper -------------------------------

    def _dispatch_source_change(self, vl: "VisionLockState", name: str,
                                code: int) -> None:
        """Fire /vtx/source AND /inet/source POSTs iff the source name
        actually changes. VISION_BOX and SRC_VISION share name 'vision'
        so transitions between them don't re-POST.

        Why both channels in lockstep: the operator has a single source-
        chip UI on the phone (FRONT/GROUND/VISION). In Radio mode the
        only feed they see is the internet stream on the tablet; in
        analog-VTX-also mode they see the same source on the 5.8 GHz
        side. Driving both keeps operator mental model coherent. The
        underlying routers stay independent at the API level — anyone
        wanting to decouple can POST /vtx/source and /inet/source
        separately."""
        if vl.prev_source_name == name:
            return
        vl.events_source_change += 1
        log.info("source change → %s (code=%d)", name, code)
        asyncio.create_task(self._post_source(name))
        asyncio.create_task(self._post_inet_source(name))
        vl.prev_source_name = name

    # --- ENGAGE: rules 2a/2c/2d are sync (local state, free);
    #           rules 2b/4 are async (HTTP probe to /vision/state).
    #
    # The synchronous part runs in the WS reader's hot loop and must
    # never do I/O — a stall here drops CRSF frames at 50 Hz. The async
    # part is fired via create_task so the WS reader returns immediately.

    def _on_engage_rising(self, ts_state) -> None:
        """Synchronous fast-gate phase. Returns without I/O."""
        vl: VisionLockState = ts_state.vision_lock
        box = vl.stable_box

        # Rule 2d: no valid selection (CH8 bit 5 clear OR not yet stable).
        if box is None or vl.phase != SelectionPhase.READY:
            vl.rejections_no_selection += 1
            log.info("engage rejected (rule 2d): no selection (phase=%s)",
                     vl.phase.value)
            return

        # Rule 2a: selection stable for at least ENGAGE_GATE_MS. Overlaps
        # with rule 1's debounce — they don't stack. If the user paused
        # ≥100 ms between selection and pressing ENGAGE, this is satisfied
        # immediately on the next frame after engage_pulse rises.
        now = time.monotonic()
        stable_for_ms = (
            (now - vl.stable_since_mono) * 1000.0
            if vl.stable_since_mono is not None
            else 0.0
        )
        if stable_for_ms < ENGAGE_GATE_MS:
            vl.rejections_engage_gate += 1
            log.info("engage rejected (rule 2a): stable for %.0f ms < %.0f ms",
                     stable_for_ms, ENGAGE_GATE_MS)
            return

        # Rule 2c: channel-frame freshness at DISPATCH time. Reads
        # translator's last_rc_ts (already maintained by the WS reader).
        last_rc_ts = getattr(ts_state, "last_rc_ts", None)
        link_age_ms = (
            (now - last_rc_ts) * 1000.0
            if last_rc_ts is not None
            else float("inf")
        )
        if link_age_ms > LINK_AGE_MS:
            vl.rejections_link_age += 1
            log.warning("engage rejected (rule 2c): link age %.0f ms > %.0f ms",
                        link_age_ms, LINK_AGE_MS)
            return

        # Snapshot what the async path needs. We pass `vl` so the async
        # task can update counters / phase / log on completion. `vl`
        # mutates only on this single asyncio loop, so no lock needed.
        asyncio.create_task(self._engage_io_phase(vl, box))

    async def _engage_io_phase(self, vl: VisionLockState, box: NormBox) -> None:
        """Async phase: rules 2b + 4 + dispatch. Runs off the WS hot loop
        so the synchronous /state probe never stalls channel ingest."""
        # Rules 2b + 4 share a single /state probe — fetch once. We
        # call the synchronous helper via run_in_executor to keep this
        # truly off the event loop's main thread.
        loop = asyncio.get_running_loop()
        vstate = await loop.run_in_executor(None, _probe_vision_state)
        if vstate is None:
            vl.rejections_state_stale += 1
            log.warning("engage rejected (rule 2b): /vision/state unreachable")
            return

        seq_age_ms = float(vstate.get("frame_age_ms") or -1)
        if seq_age_ms < 0 or seq_age_ms > STATE_FRESHNESS_MS:
            vl.rejections_state_stale += 1
            log.warning("engage rejected (rule 2b): /state frame_age_ms=%.0f "
                        "(threshold %.0f)", seq_age_ms, STATE_FRESHNESS_MS)
            return

        # Rule 4: idempotency vs current target.last_box. New selection
        # is in NORM space; locked_box is in PIXEL space — scale before
        # comparing.
        locked_box_px = vstate.get("locked_box")
        frame_w = int(vstate.get("frame_w") or 0)
        frame_h = int(vstate.get("frame_h") or 0)
        if (locked_box_px and isinstance(locked_box_px, list)
                and len(locked_box_px) == 4
                and frame_w > 0 and frame_h > 0):
            new_box_px = _norm_box_to_pixels(box, frame_w, frame_h)
            iou = _iou_xyxy(new_box_px, locked_box_px)
            if iou >= IOU_IDEMPOTENCY_THRESHOLD:
                vl.rejections_idempotent += 1
                log.info("engage no-op (rule 4): IoU=%.2f ≥ %.2f vs current target",
                         iou, IOU_IDEMPOTENCY_THRESHOLD)
                return

        # All gates passed — commit.
        vl.events_engage += 1
        vl.phase = SelectionPhase.DISPATCHED
        log.info("engage commit — box(norm)=(%d,%d,%d,%d) follow=%s",
                 box.x1, box.y1, box.x2, box.y2,
                 vl.is_follow_active)
        await self._post_engage(box)

    async def _post_engage(self, box: NormBox) -> None:
        body = {"x1": box.x1, "y1": box.y1, "x2": box.x2, "y2": box.y2}
        try:
            r = await self._client.post("/vision/engage", json=body, timeout=2.0)
            log.info("engage → /vision/engage %d", r.status_code)
        except httpx.RequestError as e:
            log.warning("engage → /vision/engage FAILED: %s", e)

    # --- FOLLOW / ABORT / CANCEL_LOCK: simple rising-edge POSTs --------

    def _on_follow_rising(self, vl: VisionLockState) -> None:
        vl.events_follow += 1
        vl.is_follow_active = True
        log.info("follow rising — POST /vision/follow (is_follow_active=True)")
        asyncio.create_task(self._post_simple("/vision/follow"))

    def _on_abort_rising(self, vl: VisionLockState) -> None:
        vl.events_abort += 1
        # ABORT halts movement; per contract it's orthogonal to lock state,
        # so we leave the lock alone. is_follow_active reset is not strictly
        # required by the contract but matches operator expectation
        # (abort = full stop, including following).
        vl.is_follow_active = False
        log.info("abort rising — POST /vision/abort (is_follow_active=False)")
        asyncio.create_task(self._post_simple("/vision/abort"))

    def _on_cancel_lock_rising(self, vl: VisionLockState) -> None:
        vl.events_cancel_lock += 1
        # CANCEL_LOCK clears the target. FOLLOW without a target is
        # nonsensical, so we also clear the flag.
        vl.is_follow_active = False
        log.info("cancel_lock rising — POST /vision/cancel-lock "
                 "(is_follow_active=False)")
        asyncio.create_task(self._post_simple("/vision/cancel-lock"))

    async def _post_simple(self, path: str) -> None:
        try:
            r = await self._client.post(path, json={}, timeout=2.0)
            log.info("%s → %d", path, r.status_code)
        except httpx.RequestError as e:
            log.warning("%s FAILED: %s", path, e)

    # --- VTX source switch: persistent-level dispatcher ---------------

    async def _post_source(self, name: str) -> None:
        """POST /vtx/source with the resolved source name. Body matches
        api_gateway's VtxSourceRequest schema: {"name": "front|vision|ground|black"}.
        """
        try:
            r = await self._client.post(
                "/vtx/source", json={"name": name}, timeout=2.0,
            )
            log.info("/vtx/source(%s) → %d", name, r.status_code)
        except httpx.RequestError as e:
            log.warning("/vtx/source(%s) FAILED: %s", name, e)

    async def _post_inet_source(self, name: str) -> None:
        """POST /inet/source — internet/Android feed channel. Same schema
        as /vtx/source. Fired in parallel from _dispatch_source_change so
        an /inet failure doesn't block the /vtx POST (and vice versa).
        """
        try:
            r = await self._client.post(
                "/inet/source", json={"name": name}, timeout=2.0,
            )
            log.info("/inet/source(%s) → %d", name, r.status_code)
        except httpx.RequestError as e:
            log.warning("/inet/source(%s) FAILED: %s", name, e)
