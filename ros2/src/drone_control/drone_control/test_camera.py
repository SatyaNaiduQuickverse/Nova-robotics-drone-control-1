#!/usr/bin/env python3
"""
Quick test: verify video and audio capture from USB camera.
Grabs a few video frames and a short audio clip, reports results.
Usage: python3 test_camera.py
"""

import time
import wave
import subprocess
import tempfile
import os

import cv2


VIDEO_DEVICE = 0       # /dev/video0
AUDIO_DEVICE = "hw:2,0"  # Lenovo FHD Webcam Audio
AUDIO_SECONDS = 2
AUDIO_RATE = 44100
AUDIO_CHANNELS = 2

FRAME_COUNT = 30       # Grab this many frames to measure real FPS


def test_video():
    """Open camera, grab frames, report resolution and FPS."""
    print("=== Video Test ===")
    cap = cv2.VideoCapture(VIDEO_DEVICE, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"FAIL: cannot open /dev/video{VIDEO_DEVICE}")
        return False

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # Warm up — first frame can be slow
    ok, frame = cap.read()
    if not ok or frame is None:
        print("FAIL: first frame read failed")
        cap.release()
        return False

    h, w = frame.shape[:2]
    print(f"  Resolution : {w}x{h}")

    # Measure real FPS over FRAME_COUNT frames
    start = time.monotonic()
    for _ in range(FRAME_COUNT):
        ok, frame = cap.read()
        if not ok:
            print("FAIL: dropped frame during FPS test")
            cap.release()
            return False
    elapsed = time.monotonic() - start
    fps = FRAME_COUNT / elapsed

    cap.release()
    print(f"  Frames     : {FRAME_COUNT}")
    print(f"  Elapsed    : {elapsed:.2f}s")
    print(f"  Real FPS   : {fps:.1f}")
    print("  Status     : OK")
    return True


def test_audio():
    """Record a short clip with arecord, verify we got data."""
    print("\n=== Audio Test ===")
    tmp = tempfile.mktemp(suffix=".wav")
    cmd = [
        "arecord",
        "-D", AUDIO_DEVICE,
        "-f", "S16_LE",
        "-r", str(AUDIO_RATE),
        "-c", str(AUDIO_CHANNELS),
        "-d", str(AUDIO_SECONDS),
        tmp,
    ]

    try:
        result = subprocess.run(cmd, capture_output=True, timeout=AUDIO_SECONDS + 5)
    except FileNotFoundError:
        print("FAIL: arecord not found (install alsa-utils)")
        return False
    except subprocess.TimeoutExpired:
        print("FAIL: arecord timed out")
        return False

    if result.returncode != 0:
        print(f"FAIL: arecord exited {result.returncode}")
        print(f"  stderr: {result.stderr.decode().strip()}")
        return False

    if not os.path.exists(tmp):
        print("FAIL: output file not created")
        return False

    try:
        with wave.open(tmp, "rb") as wf:
            channels = wf.getnchannels()
            rate = wf.getframerate()
            frames = wf.getnframes()
            duration = frames / rate
            # Read raw samples to check for silence
            raw = wf.readframes(frames)
            peak = max(abs(int.from_bytes(raw[i:i+2], "little", signed=True))
                       for i in range(0, min(len(raw), 20000), 2))
    except Exception as e:
        print(f"FAIL: cannot read wav — {e}")
        return False
    finally:
        os.unlink(tmp)

    print(f"  Channels   : {channels}")
    print(f"  Sample rate: {rate} Hz")
    print(f"  Duration   : {duration:.2f}s")
    print(f"  Frames     : {frames}")
    print(f"  Peak sample: {peak}")
    if peak < 10:
        print("  Warning    : audio looks silent (peak < 10) — mic may be muted")
    print("  Status     : OK")
    return True


if __name__ == "__main__":
    print(f"Camera test — {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
    v = test_video()
    a = test_audio()
    print("\n=== Summary ===")
    print(f"  Video: {'PASS' if v else 'FAIL'}")
    print(f"  Audio: {'PASS' if a else 'FAIL'}")
