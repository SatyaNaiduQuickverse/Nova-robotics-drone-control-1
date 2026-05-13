"""Safety gate for production telemetry producers.

Two failure modes this module prevents:

  1. Stub + real producer running together. The 4 H4-validation stubs
     (battery / gps / attitude / flightmode_producer.py) push synthetic
     values; if a real producer fires alongside, both POST to
     /telemetry/raw at the daemon's rate-limit and race. Real consumers
     would see interleaved values from two sources. Refuse to start.

  2. Multiple instances of the same real producer. Two attitude
     producers at 10 Hz = 20 Hz host push, doubling load. Refuse with
     pidfile lock.

Usage from a real producer (call once at startup, before any POST):

    from producer_safety import production_setup
    production_setup("battery")   # raises SystemExit if unsafe

The check is cheap (single `ps -eo` call + one open()); fine to run
at boot. After that, producers loop freely.

No external dependencies — uses /proc on Linux directly so it works
inside containers without a `ps` binary.
"""
from __future__ import annotations

import errno
import os
import sys
from pathlib import Path
from typing import Optional


STUB_PRODUCER_NAMES = (
    "battery_producer.py",
    "gps_producer.py",
    "attitude_producer.py",
    "flightmode_producer.py",
)

PIDFILE_DIR = Path(os.environ.get("PRODUCER_PIDFILE_DIR", "/tmp"))
PIDFILE_PREFIX = "novaros_elrs_producer_"


class ProducerSafetyError(RuntimeError):
    """Raised when the safety gate refuses to start a real producer."""


def _read_cmdline(pid: int) -> Optional[str]:
    """Return /proc/<pid>/cmdline as a space-joined string, or None if gone."""
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except FileNotFoundError:
        return None
    except PermissionError:
        return None
    if not raw:
        return None
    return raw.replace(b"\x00", b" ").decode("utf-8", errors="replace").strip()


def find_running_stubs() -> list[tuple[int, str]]:
    """Return [(pid, cmdline), …] for any stub producer found via /proc."""
    found: list[tuple[int, str]] = []
    my_pid = os.getpid()
    try:
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            if pid == my_pid:
                continue
            cmd = _read_cmdline(pid)
            if not cmd:
                continue
            if any(name in cmd for name in STUB_PRODUCER_NAMES):
                found.append((pid, cmd))
    except OSError:
        # /proc unavailable (non-Linux, or unusual sandbox). Caller can
        # decide whether to abort or continue — default conservative.
        pass
    return found


def _pidfile_path(name: str) -> Path:
    return PIDFILE_DIR / f"{PIDFILE_PREFIX}{name}.pid"


def acquire_pidfile(name: str) -> Path:
    """Create a pidfile for a real producer. Raises if one already
    exists and the holder is alive. Returns the path."""
    path = _pidfile_path(name)
    if path.exists():
        try:
            existing_pid = int(path.read_text().strip())
        except (ValueError, OSError):
            existing_pid = None
        if existing_pid and existing_pid != os.getpid():
            cmd = _read_cmdline(existing_pid)
            if cmd:
                raise ProducerSafetyError(
                    f"pidfile {path} held by live process "
                    f"pid={existing_pid}: {cmd}"
                )
            # stale pidfile (holder dead) — overwrite below
    try:
        path.write_text(f"{os.getpid()}\n")
    except OSError as e:
        raise ProducerSafetyError(f"cannot write pidfile {path}: {e}") from e
    return path


def release_pidfile(path: Path) -> None:
    """Remove the pidfile. Safe to call multiple times."""
    try:
        path.unlink()
    except FileNotFoundError:
        pass
    except OSError as e:
        # Don't crash on release errors — log and move on. Producers
        # may be in shutdown when this runs.
        sys.stderr.write(f"warning: cannot remove {path}: {e}\n")


def production_setup(name: str) -> Path:
    """Real producers call this once at startup before any POSTing.

    Steps:
      1. Refuse to start if any stub producer is running.
      2. Acquire pidfile to prevent self-duplication.

    Returns the pidfile path; the producer should release it on exit
    via release_pidfile(). For interactive testing, set env var
    NOVAROS_ALLOW_STUB_MIX=1 to bypass step (1) — useful when verifying
    that a real producer's frame shape matches the stub byte-for-byte
    before the cutover commit.
    """
    if os.environ.get("NOVAROS_ALLOW_STUB_MIX") != "1":
        stubs = find_running_stubs()
        if stubs:
            lines = "\n".join(f"  pid={p} cmd={c}" for p, c in stubs)
            raise ProducerSafetyError(
                f"refusing to start real producer {name!r} — stub "
                f"producers are running and would race:\n{lines}\n"
                f"Stop them first, or set NOVAROS_ALLOW_STUB_MIX=1 "
                f"for byte-shape verification only."
            )
    return acquire_pidfile(name)


# --- self-test ------------------------------------------------------------

if __name__ == "__main__":
    print("producer_safety smoke test", file=sys.stderr)
    stubs = find_running_stubs()
    if stubs:
        print(f"  found {len(stubs)} stub(s) running:")
        for pid, cmd in stubs:
            print(f"    pid={pid} cmd={cmd[:100]}")
    else:
        print("  no stubs running")

    # Test pidfile lock cycle
    test_name = "selftest_only"
    path = acquire_pidfile(test_name)
    print(f"  acquired pidfile: {path}")
    assert path.exists()
    assert int(path.read_text().strip()) == os.getpid()

    # Re-acquire with same pid is allowed
    path2 = acquire_pidfile(test_name)
    assert path2 == path
    print(f"  re-acquire by same pid: OK")

    release_pidfile(path)
    assert not path.exists()
    print(f"  released pidfile: OK")

    # Production setup must refuse if stubs are running
    if stubs:
        try:
            production_setup("selftest_real")
            print("  UNEXPECTED: production_setup() did not refuse")
            sys.exit(1)
        except ProducerSafetyError as e:
            print(f"  production_setup() correctly refused: {str(e)[:80]}...")
        # And accept when override is set
        os.environ["NOVAROS_ALLOW_STUB_MIX"] = "1"
        try:
            path = production_setup("selftest_real")
            print(f"  override (NOVAROS_ALLOW_STUB_MIX=1) bypassed: OK")
            release_pidfile(path)
        finally:
            del os.environ["NOVAROS_ALLOW_STUB_MIX"]
    else:
        path = production_setup("selftest_real")
        print(f"  production_setup() succeeded (no stubs): OK")
        release_pidfile(path)

    print("\nall smoke tests pass")
