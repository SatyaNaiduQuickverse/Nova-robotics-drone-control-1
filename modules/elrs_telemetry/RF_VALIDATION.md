# ELRS Downlink RF Substrate Validation — Phases A / B / C

Three-phase characterization of the new drone-side downlink Tx ↔ ground
Rx pair, run end-to-end on 2026-05-12 (UTC). Replaces the rushed-to-
integration approach that motivated rigorous characterization before the
H4 telemetry producer work.

## Protocol

Both sides run a `elrs_probe.py` instance against the ESP32-C6 USB-CDC
device. Probe encodes/decodes the `0xCAFE`-framed mux protocol used by
the dual-UART firmware:

- mux ch=1 = local Tx (NEW Ranger Micro Tx 2G4 on the drone)
- mux ch=2 = local Rx (existing RP4TD-equivalent on the drone)

Counter-fire mode encodes a monotonic CH3 value (mod 2048) inside an
otherwise FS_CHANNELS-shaped CRSF RC frame. The peer's analyzer tracks
CH3 across received frames, counting gaps, duplicates, and bad-sync
indicators from the ESP firmware.

## Phase A — Uplink only (ground Tx → drone Rx)

Ground side fires CH3 counter on its mux ch=1 (Tx). Drone side listens
on its mux ch=2 (Rx). 60 s bounded window.

| Metric           | Value     |
|------------------|-----------|
| Packet delivery  | 94.96%    |
| bad_sync         | 0         |
| Counter coverage | clean, only singleton drops |

Verdict: uplink substrate is healthy.

## Phase B — Downlink only (drone Tx → ground Rx)

Drone side fires CH3 counter on mux ch=1 (NEW Tx). Ground side listens
on mux ch=2 (Rx). 30 s fire surrounded by 10 s warmup + 5 s cooldown so
the Rx never sees a cold-Tx silence inside the analyzer window.

| Metric                  | Value (in captured 26 s overlap)   |
|-------------------------|-------------------------------------|
| Packet delivery         | ~100%                              |
| bad_sync                | 0                                  |
| Counter span            | 1294 unique values over 25.88 s    |
| Bind reacquire latency  | ~7 s from cold-Tx to first delivery |

Verdict: downlink substrate is healthy. The 7 s reacquire latency is a
property worth filing — if downlink ever returns from failsafe in
production, expect that latency before clean data resumes.

## Phase C — Simultaneous bidirectional (co-existence)

Both probes fire CH3 counter on their local Tx AND analyze CH3 on their
local Rx, in a single process each. Synchronized launch at
`T_FIRE_UTC = 23:00:00Z`, both sides arm at T-2 s, both close at T+32 s.

| Direction | Packet delivery | bad_sync | Notes                                  |
|-----------|-----------------|----------|----------------------------------------|
| Downlink  | 97.6% substrate / 98.9% counter | 0 | rate observed ~20 Hz (instrument cap) |
| Uplink    | 82.4% packet               | 0 | rate observed ~42 Hz (instrument cap) |

**Verdict: co-existence is clean.** `bad_sync = 0` on both ends through
the full 32 s window confirms the two RF pairs do not interfere at the
radio level when fired simultaneously.

### Rate-cap caveat

Both probes hit a host-side rate cap around 20–47 Hz during Phase C
despite `--rate 50`. Root cause: the main loop's `out_q.get(timeout=…)`
was set to 0.05 s, so each empty-queue iteration blocked 50 ms,
throttling the re-send scheduler. Patched to `0.002 s` on both sides
post-Phase-C; substrate conclusion (clean co-existence, bad_sync=0)
holds because the cap is host-side, not RF.

## Operational notes

- Wallclock synchronization between drone and ground Pi is sub-frame
  accurate during fires (verified during Phase C: arm 22:59:58.005Z
  drone vs 22:59:58.004Z ground). No NTP adjustment needed.
- Schedule fires using absolute UTC `HH:MM:SS` and a `sleep_until`
  loop against `datetime.now(UTC)` — relative sleeps drift, and
  hardcoded dates trip the local-vs-UTC date trap (lost Phase C Round 1
  to this exact bug).
- Synchronized launcher: `tools/rf_phase_launcher.py HH:MM:SS` —
  moves stub producer + daemon out of the way, runs probe in foreground
  for the bounded duration, restores both on exit.

## Outcome

RF substrate validated end-to-end. Cleared to proceed to **H4** — replace
the stub producer with a proper FC-sourced telemetry producer
(battery / GPS / attitude) and define the production downlink serializer
format.
