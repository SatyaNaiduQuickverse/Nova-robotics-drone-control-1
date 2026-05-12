#!/usr/bin/env python3
"""Synchronized RF phase fire launcher (used for Phase A/B/C substrate
validation; retained for future co-existence / regression runs).

Takes T_FIRE wallclock HH:MM:SS UTC on argv. Computes the next
occurrence (today if still future, else tomorrow). Avoids the
hardcoded-date trap that wasted Phase C round 1.

Sleeps until T_FIRE_UTC - 2s, then:
  1. Stops stub producer (pkill elrs_producer.py)
  2. Stops elrs-telemetry daemon (docker stop -t 0 elrs-telemetry)
  3. Execs elrs_probe.py with --counter-channel 3 --duration 32

Probe fires CH3 counter on mux ch=1 (downlink Tx out) AND analyzes
incoming CH3 counter on mux ch=2 (uplink Rx in) in the same process.

After probe exits at T+30s, we restart the daemon and stub producer
so Rx stays bound for any post-fire diagnostics.
"""
import datetime
import os
import subprocess
import sys
import time


def parse_target_utc(spec: str) -> datetime.datetime:
    """Parse 'HH:MM:SS' and return next future occurrence in UTC."""
    h, m, s = (int(x) for x in spec.split(":"))
    now = datetime.datetime.now(datetime.timezone.utc)
    target = now.replace(hour=h, minute=m, second=s, microsecond=0)
    if target <= now:
        target += datetime.timedelta(days=1)
    return target


if len(sys.argv) != 2:
    print(f"usage: {sys.argv[0]} HH:MM:SS  (UTC wallclock)", file=sys.stderr)
    sys.exit(2)

TARGET_UTC = parse_target_utc(sys.argv[1])
ARM_LEAD_S = 2.0   # launch probe 2s before T_FIRE
PROBE_DURATION_S = 32.0
ESP_PORT = (
    "/dev/serial/by-id/"
    "usb-Espressif_USB_JTAG_serial_debug_unit_FC:01:2C:E8:BC:64-if00"
)
PROBE_PATH = "/home/novaedge1/elrs_probe.py"


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc)


def log(msg):
    print(f"[{utc_now().strftime('%H:%M:%S.%f')[:-3]}Z] {msg}", flush=True)


def main():
    arm_time = TARGET_UTC - datetime.timedelta(seconds=ARM_LEAD_S)
    log(f"Phase C launcher armed. T_FIRE={TARGET_UTC.strftime('%H:%M:%S')}Z  "
        f"arm_at={arm_time.strftime('%H:%M:%S')}Z")

    # Sleep until arm time using wallclock (no relative-sleep drift)
    while True:
        now = utc_now()
        remaining = (arm_time - now).total_seconds()
        if remaining <= 0:
            break
        # Coarse sleep until close, then fine-grain
        time.sleep(min(remaining, 0.5))

    log(f"ARM. now={utc_now().strftime('%H:%M:%S.%f')[:-3]}Z — handoff begins")

    # Stop stub producer first (so it can't try to POST while daemon goes down)
    subprocess.run(["pkill", "-f", "elrs_producer.py"], check=False)
    log("stub producer killed")

    # Stop daemon to release ESP serial. -t 0 = immediate SIGKILL after grace
    subprocess.run(
        ["docker", "stop", "-t", "0", "elrs-telemetry"],
        check=False,
        capture_output=True,
    )
    log("daemon stopped — ESP serial released")

    # Probe takes over. Use exec so we inherit logging cleanly.
    log(f"launching probe — duration={PROBE_DURATION_S}s")
    cmd = [
        "python3", "-u", PROBE_PATH,
        "--port", ESP_PORT,
        "--ch", "1",
        "--counter-channel", "3",
        "--rate", "50",
        "--duration", str(PROBE_DURATION_S),
    ]
    log("cmd: " + " ".join(cmd))

    # Run probe in foreground so we can post-process after it exits
    result = subprocess.run(cmd, check=False)
    log(f"probe exited rc={result.returncode}")

    # Restore daemon + stub so Rx stays visibly bound for next test
    subprocess.run(
        ["docker", "start", "elrs-telemetry"],
        check=False, capture_output=True,
    )
    log("daemon restarted")

    # Give daemon ~3s to claim ESP serial before stub starts posting
    time.sleep(3.0)

    stub_log = "/tmp/elrs_producer.log"
    with open(stub_log, "ab") as f:
        subprocess.Popen(
            ["python3", "-u",
             "/home/novaedge1/novaros/modules/elrs_telemetry/tools/elrs_producer.py"],
            stdout=f, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    log(f"stub producer restarted (log: {stub_log})")
    log("Phase C launcher complete.")


if __name__ == "__main__":
    sys.exit(main())
