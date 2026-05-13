#!/usr/bin/env python3
"""Rate-scheduled producer for downlink LINK_STATISTICS decode validation.

Usage:
    link_stats_rate_cycle.py HH:MM:SS

Argv is the UTC wallclock instant T_FIRE_UTC at which the ground-side
listener will start its --duration 300 capture. We:

  1. Start firing FS_CHANNELS at 50 Hz IMMEDIATELY on launch (warmup).
     Keeps the new Tx carrier hot so by T_FIRE the ground Rx is already
     bound — avoids the ~7 s cold-Tx bind-reacquire latency observed in
     Phase B.

  2. At T_FIRE, execute the rate schedule:
        t=  0..60   50 Hz   (matches warmup, seamless at wire)
        t= 60..120   0 Hz   (silence — expect Rx LQ to crater + bind loss)
        t=120..180 100 Hz   (recovery + max rate)
        t=180..300  50 Hz   (tail, total 300 s)

  3. After T+300 s exit.

Frame format: FS_CHANNELS with CH9 carrying a monotonic counter mod 2048
(identical to elrs_producer.py so any decoder accepts both).

UTC-correct: target parsed via 'next future occurrence' against
datetime.now(timezone.utc) — does not hardcode the date.
"""
import datetime
import json
import sys
import time
import urllib.request

URL = "http://localhost:5003/telemetry/raw"
FS = [
    172, 992, 992, 992, 992, 992, 172, 172,
    992, 992, 992, 992, 992, 992, 992, 992,
]

# (rate_hz, duration_s) — applied sequentially from T_FIRE
SCHEDULE = [
    (50.0,  60.0),
    (0.0,   60.0),
    (100.0, 60.0),
    (50.0, 120.0),
]


def parse_target_utc(spec):
    h, m, s = (int(x) for x in spec.split(":"))
    now = datetime.datetime.now(datetime.timezone.utc)
    target = now.replace(hour=h, minute=m, second=s, microsecond=0)
    if target <= now:
        target += datetime.timedelta(days=1)
    return target


def utc_stamp():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%H:%M:%S.%f")[:-3] + "Z"


def crc8_dvbs2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def build_rc_frame(channels):
    bits = 0
    nbits = 0
    payload = bytearray()
    for ch in channels:
        bits |= (ch & 0x7FF) << nbits
        nbits += 11
        while nbits >= 8:
            payload.append(bits & 0xFF)
            bits >>= 8
            nbits -= 8
    crc = crc8_dvbs2(bytes([0x16]) + bytes(payload))
    return bytes([0xC8, 0x18, 0x16]) + bytes(payload) + bytes([crc])


def post(frame_hex):
    body = json.dumps({"hex": frame_hex}).encode()
    req = urllib.request.Request(
        URL, data=body, method="POST",
        headers={"Content-Type": "application/json"},
    )
    try:
        urllib.request.urlopen(req, timeout=0.05).read()
    except Exception:
        pass


def emit_until(end_mono, rate_hz, counter_start, label):
    """Emit FS+counter frames at rate_hz until end_mono. Return next counter."""
    counter = counter_start
    if rate_hz <= 0:
        print(f"  [{utc_stamp()}] {label} — rate=0 Hz, UART1 silent", flush=True)
        while time.monotonic() < end_mono:
            time.sleep(min(0.5, end_mono - time.monotonic()))
        return counter
    period = 1.0 / rate_hz
    next_tick = time.monotonic()
    sent = 0
    t0 = time.monotonic()
    print(f"  [{utc_stamp()}] {label} — rate={rate_hz:.0f} Hz", flush=True)
    while time.monotonic() < end_mono:
        chans = list(FS)
        chans[8] = counter & 0x7FF
        post(build_rc_frame(chans).hex())
        counter += 1
        sent += 1
        next_tick += period
        sleep = next_tick - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_tick = time.monotonic()
    elapsed = time.monotonic() - t0
    print(f"  [{utc_stamp()}] {label} done — sent {sent} frames in {elapsed:.1f}s "
          f"(effective {sent/elapsed:.1f} Hz)", flush=True)
    return counter


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} HH:MM:SS  (UTC T_FIRE)", file=sys.stderr)
        sys.exit(2)

    t_fire = parse_target_utc(sys.argv[1])
    print(f"[{utc_stamp()}] rate-cycle armed. T_FIRE={t_fire.strftime('%H:%M:%S')}Z", flush=True)

    # Warmup: emit 50 Hz from launch until T_FIRE
    warmup_end_mono = (
        time.monotonic()
        + (t_fire - datetime.datetime.now(datetime.timezone.utc)).total_seconds()
    )
    counter = emit_until(warmup_end_mono, 50.0, 0, "WARMUP")

    # Execute schedule
    t_now_mono = time.monotonic()
    print(f"[{utc_stamp()}] T_FIRE reached — executing schedule", flush=True)
    cumulative = 0.0
    for i, (rate_hz, duration_s) in enumerate(SCHEDULE, start=1):
        phase_end_mono = t_now_mono + cumulative + duration_s
        counter = emit_until(
            phase_end_mono, rate_hz, counter,
            f"PHASE {i}/{len(SCHEDULE)} (t={cumulative:.0f}..{cumulative+duration_s:.0f}s)",
        )
        cumulative += duration_s

    print(f"[{utc_stamp()}] complete — total schedule {cumulative:.0f}s "
          f"final counter={counter}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"[{utc_stamp()}] interrupted", flush=True)
