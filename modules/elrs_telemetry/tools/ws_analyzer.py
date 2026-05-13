#!/usr/bin/env python3
"""Drone-side WS analyzer: counter-on-uplink + LinkStats aggregator.

Avoids serial-contention with the running daemon + producers by
subscribing to the daemon's WebSocket endpoints instead of opening
the ESP32 serial directly.

Usage:
    ws_analyzer.py HH:MM:SS

Sleeps until T_FIRE_UTC, then for the bounded duration:
  - /ws/channels  → CH3 counter delivery (uplink: ground→drone)
  - /ws/link      → LinkStats aggregate (drone-side view of the link)

After the window closes, prints both reports.
"""
import asyncio
import datetime
import json
import statistics
import sys
import websockets

DAEMON_HOST = "ws://localhost:5003"
DURATION_S = 30.0
CH_INDEX = 2   # CH3 (1-based) = index 2 (0-based)
COUNTER_MOD = 2048


def parse_target_utc(spec):
    h, m, s = (int(x) for x in spec.split(":"))
    now = datetime.datetime.now(datetime.timezone.utc)
    target = now.replace(hour=h, minute=m, second=s, microsecond=0)
    if target <= now:
        target += datetime.timedelta(days=1)
    return target


def utc_stamp():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%H:%M:%S.%f")[:-3] + "Z"


async def analyze_channels(duration_s):
    counter_vals = []
    first_msg_ts = None
    last_msg_ts = None
    async with websockets.connect(f"{DAEMON_HOST}/ws/channels") as ws:
        loop = asyncio.get_event_loop()
        end_at = loop.time() + duration_s
        while loop.time() < end_at:
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=0.5)
            except asyncio.TimeoutError:
                continue
            try:
                data = json.loads(msg)
            except json.JSONDecodeError:
                continue
            ch = data.get("channels_crsf")
            if not ch or len(ch) <= CH_INDEX:
                continue
            counter_vals.append(ch[CH_INDEX])
            if first_msg_ts is None:
                first_msg_ts = loop.time()
            last_msg_ts = loop.time()
    n = len(counter_vals)
    if n == 0:
        return {"received": 0, "wallclock_s": 0.0, "rate_hz": 0.0}
    # Counter gap analysis (mod 2048)
    gaps = 0
    dupes = 0
    for i in range(1, n):
        diff = (counter_vals[i] - counter_vals[i - 1]) % COUNTER_MOD
        if diff == 0:
            dupes += 1
        elif diff > 1:
            gaps += diff - 1
    span = (counter_vals[-1] - counter_vals[0]) % COUNTER_MOD
    wall_s = (last_msg_ts - first_msg_ts) if first_msg_ts else 0.0
    rate = n / wall_s if wall_s > 0 else 0.0
    return {
        "received": n,
        "first": counter_vals[0],
        "last": counter_vals[-1],
        "counter_span": span,
        "gaps_missing": gaps,
        "duplicates": dupes,
        "wallclock_s": round(wall_s, 3),
        "rate_hz": round(rate, 2),
    }


async def analyze_link(duration_s):
    samples = []
    async with websockets.connect(f"{DAEMON_HOST}/ws/link") as ws:
        loop = asyncio.get_event_loop()
        end_at = loop.time() + duration_s
        while loop.time() < end_at:
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=0.5)
            except asyncio.TimeoutError:
                continue
            try:
                data = json.loads(msg)
            except json.JSONDecodeError:
                continue
            link = data.get("link")
            if isinstance(link, dict):
                samples.append(link)
    if not samples:
        return {"received": 0}
    fields = list(samples[0].keys())
    summary = {"received": len(samples)}
    for f in fields:
        vals = [s.get(f) for s in samples if isinstance(s.get(f), (int, float))]
        if vals:
            summary[f] = {
                "min": min(vals),
                "avg": round(statistics.mean(vals), 2),
                "max": max(vals),
                "last": vals[-1],
            }
    return summary


async def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} HH:MM:SS  (UTC T_FIRE)", file=sys.stderr)
        sys.exit(2)

    target = parse_target_utc(sys.argv[1])
    print(f"[{utc_stamp()}] ws_analyzer armed. T_FIRE={target.strftime('%H:%M:%S')}Z  "
          f"duration={DURATION_S}s", flush=True)

    # Sleep until T_FIRE
    while True:
        now = datetime.datetime.now(datetime.timezone.utc)
        remaining = (target - now).total_seconds()
        if remaining <= 0:
            break
        await asyncio.sleep(min(remaining, 0.5))

    print(f"[{utc_stamp()}] T_FIRE — analyzing for {DURATION_S}s", flush=True)
    ch_task = asyncio.create_task(analyze_channels(DURATION_S))
    ls_task = asyncio.create_task(analyze_link(DURATION_S))
    ch_result, ls_result = await asyncio.gather(ch_task, ls_task)
    print(f"[{utc_stamp()}] analysis window closed", flush=True)

    print("\n=== UPLINK CH3 counter delivery (ground→drone) ===")
    print(json.dumps(ch_result, indent=2))

    print("\n=== Drone-side LINK_STATISTICS (incoming from uplink Rx) ===")
    print(json.dumps(ls_result, indent=2))


if __name__ == "__main__":
    asyncio.run(main())
