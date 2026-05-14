# NovaROS deployment to a fresh Raspberry Pi 5

Step-by-step bringup for installing this stack on a new Pi. Stops at
verifiable checkpoints — easy to roll back if any step fails.

**Estimated total time:** 1–1.5 hours, mostly waiting on `make build`.

**Prerequisites:**
- Raspberry Pi 5 (8 GB RAM recommended)
- SD card, 32 GB minimum, 64 GB recommended
- Ethernet cable for the build (for faster apt/pip pulls)
- Flight hardware: CubeOrange+ FC, ESP32-C6 ELRS bridge, USB cameras,
  optionally Hailo-8 HAT+

---

## Quick reference (the whole flow)

```bash
# Hour 1 — OS bootstrap
ssh novaedge1@<new-pi-ip>
git clone <repo url> novaros
cd novaros
sudo ./install.sh
# (set boot config in step 4 below if install.sh didn't)
sudo reboot

# Hour 2 — edit hardware paths
ssh novaedge1@<new-pi-ip>
cd novaros
ls /dev/serial/by-id/         # find your ESP serial
ls /dev/v4l/by-id/             # find your camera by-id
# edit the two files in step 7 below

# Hour 3 — build
make verify                    # confirm hardware enumerates
make build                     # ~25–40 min on first run

# Hour 4 — go live
make up                        # production stack hot
make ps                        # confirm all 6 containers Up
# Power Tx ONLY AFTER this point (avoid the scan-mode quirk)
```

---

## Detailed walkthrough

### Step 1: Pi OS image (10 min)

1. Flash Raspberry Pi OS Bookworm or Trixie 64-bit to the SD card
   using Pi Imager.
2. In Pi Imager's "Edit Settings" before flashing:
   - Set hostname
   - Enable SSH (with key auth ideally)
   - Set username (recommend `novaedge1` to match memory paths) +
     password
   - Configure Wi-Fi if used
3. Boot the Pi, SSH in.

### Step 2: Clone the repo (1 min)

```bash
git clone <your-repo-url> novaros
cd novaros
```

### Step 3: OS bootstrap — install.sh (10–15 min)

```bash
sudo ./install.sh
```

What it does (best-effort summary — verify on first run):
- `apt update && apt upgrade`
- Installs Docker + docker compose plugin
- Installs build essentials, git, curl, python3
- Sets up the project directory

**Reboot after this step** if `install.sh` updated the kernel.

### Step 4: Boot config for analog VTX (manual; one-time)

Required if you want the analog 5.8 GHz VTX path (composite-out from
Pi 5 J7 pads). If you're not using VTX, skip this step.

```bash
sudo nano /boot/firmware/config.txt
# add these three lines:
#   dtoverlay=vc4-kms-v3d,composite
#   enable_tvout=1
#   sdtv_mode=2                  # PAL (use 0 for NTSC)

sudo nano /boot/firmware/cmdline.txt
# append (on the SAME existing line):
#   video=Composite-1:720x576i@50

sudo reboot
```

Verify after reboot:
```bash
ls /sys/class/drm/ | grep Composite       # should show card1-Composite-1
```

### Step 5: Hailo NPU (only for vision-detect — opt-in)

Skip this step if you don't want YOLO vision. The flight stack works
without it.

```bash
sudo apt install hailo-all              # provides hailo_pci driver + HEF model
sudo modprobe hailo_pci                 # if not auto-loaded
lsmod | grep hailo_pci                  # confirm loaded
ls /usr/share/hailo-models/yolov8s_h8.hef    # confirm HEF present
ls /dev/hailo0                          # confirm device node
```

### Step 6: Connect hardware

Plug in:
- CubeOrange+ via USB-C
- ESP32-C6 ELRS bridge via USB
- USB cameras (front + ground)
- Hailo HAT+ (if using vision)

Wait ~5 s for udev to create by-id symlinks, then:

```bash
ls /dev/serial/by-id/
# should list:
#   usb-CubePilot_CubeOrange+_<long-serial>-if00
#   usb-CubePilot_CubeOrange+_<long-serial>-if02
#   usb-Espressif_USB_JTAG_serial_debug_unit_<your-mac>-if00

ls /dev/v4l/by-id/
# should list 1 or 2 webcam entries

ls /dev/hailo0    # only if vision wanted
```

### Step 7: Per-Pi hardware path edits (manual)

**Required if your hardware has different USB serials than the
reference Pi.** Two files to update:

#### 7a. ELRS bridge — modules/elrs_telemetry/docker-compose.yml

Find your ESP32 serial:
```bash
ls /dev/serial/by-id/usb-Espressif_*
```
Edit `modules/elrs_telemetry/docker-compose.yml` line 29:
```yaml
- ELRS_SERIAL_DEVICE=/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_<YOUR_MAC>-if00
```

Also update the matching default in `modules/elrs_telemetry/service.py`
(line ~33) so the daemon can find the device when launched without the
env var set.

#### 7b. Front camera — ros2/src/drone_control/drone_control/camera.py

Only edit if your front webcam differs from the reference Lenovo FHD:
```bash
ls /dev/v4l/by-id/
```
Find the right by-id for your front camera and update `FRONT_DEVICE`
(~line 50). The ground camera (`GROUND_DEVICE`) uses a generic Jieli
path that usually works without edits.

**CubeOrange+ does NOT need editing** — `ros2/entrypoint.sh` uses a
glob that matches any CubePilot serial.

### Step 8: Preflight verify

```bash
make verify
```

Should print 14 `[OK]` lines. Any `[FAIL]` tells you exactly what's
wrong. If the disk check warns, you have less than 6 GB free — risky
for builds.

### Step 9: Build (~25-40 min, one-time)

```bash
make build
```

The Makefile builds stacks sequentially with `docker builder prune -f`
between each to keep disk usage manageable. Sequence:
1. mongodb image pull
2. elrs_telemetry + elrs-producers (~5 min)
3. drone_bridge (~5 min)
4. ros2 stack — drone-control 2.2 GB + vision-detect 1.7 GB (~20-30 min)

Have ethernet plugged in. Coffee break.

### Step 10: Start the production stack

```bash
make up
```

This brings up 6 containers:
- mongodb
- drone-control + web-control
- elrs-telemetry + elrs-producers
- drone-bridge

`vision-detect` is **NOT** auto-started (Pi throttles). To enable it:
```bash
make up-with-vision
```

### Step 11: Smoke test

```bash
make ps                                       # all 6 containers Up
curl localhost:8080/telemetry/state           # drone-control responsive
curl localhost:5003/stats                     # elrs daemon counters
curl localhost:5004/healthz                   # drone-bridge translator
curl localhost:5000/                          # web UI loads
```

If any endpoint 502s or hangs, check `make logs` for the misbehaving
service.

### Step 12: Power on the ELRS Tx

**Order matters.** Power the Tx ONLY AFTER `make up` has completed
and `curl localhost:5003/stats` shows `bytes_out` climbing at
~492 B/s.

Reason: if the Tx is powered before the Pi is feeding RC frames to
it, the Tx enters a "scan mode" and won't recover via software — needs
a physical power-cycle.

Permanent fix for this quirk: wire Tx 5V from the Pi 5V rail (same
trick the ground side uses). Then the Tx physically can't power up
before the Pi does.

### Step 13: Reboot survival test

```bash
sudo reboot
```

After ~30 s the Pi comes back. All 6 containers auto-start via
Docker's `restart: unless-stopped`. Verify:
```bash
make ps
```

Should show all 6 containers `Up` within a minute of reboot.

---

## Troubleshooting

### `make build` fails on disk

`docker builder prune -f` between stacks should prevent this, but if
it triggers anyway:
```bash
make prune          # frees unused images + builder cache
df -h /             # need at least 6 GB free
```

### `make verify` reports `[FAIL] no flight controller`

The FC is either disconnected, in bootloader mode, or has a different
USB descriptor. Check:
```bash
ls /dev/serial/by-id/
lsusb                              # look for CubePilot/Pixhawk vendor
dmesg | tail -20                   # USB enumeration errors?
```

If FC is in bootloader (`PX4 BL FMU` in lsusb), power-cycle the FC
fully (battery removed for 10 s, then reconnect).

### `make verify` reports `[FAIL] no ESP32`

```bash
ls /dev/serial/by-id/usb-Espressif_*
```
If listed but different serial, update `ELRS_SERIAL_DEVICE` in step
7a.

### Containers start but ground-side sees no telemetry

Likely Tx power-on order — see step 12. Power-cycle the Tx physically.

### Vision-detect throttles the Pi

Expected with all containers running. Either:
- Stop other heavy services temporarily
- Don't use vision-detect until you have a more permissive thermal
  setup

---

## What's verified vs needs fresh-Pi validation

| Item | Status |
|---|---|
| Makefile targets (verify, build, up, down, ps, logs) | verified on reference Pi |
| Per-stack docker compose build | verified on reference Pi |
| Container restart survival (docker restart) | verified |
| Component-failure tolerance (FC, ELRS, drone-control crashes) | verified |
| Full `make build` end-to-end (>6 GB peak) | **needs fresh-Pi validation** |
| install.sh boot-config setup | **needs fresh-Pi validation** |
| install.sh package completeness | **needs fresh-Pi validation** |
| Pi reboot survival (full system) | partially verified via container restart |
| BLE (BlueZ) availability on fresh Pi OS | **needs fresh-Pi validation** |

The Makefile + verify_deployment.sh defaults are conservative
enough that the first deploy will fail-loud at the right spot rather
than silently break something. Expect 2–3 small fixes on the first
deploy — that's normal for any system that hasn't been mass-deployed
yet. Each fix gets committed back; the next deploy is cleaner.

---

## Reference: 7 services + ports

| Container | Image | Port | Purpose |
|---|---|---|---|
| `mongodb` | mongo:7.0 | 27017 | Mission log storage |
| `drone-control` | novaros/drone-control:humble | 8080 | MAVROS + FastAPI |
| `web-control` | python:3.11-slim | 5000 | Flask UI |
| `vision-detect` | novaros/vision-detect:hailo | 8081 | YOLO on Hailo NPU (opt-in) |
| `elrs-telemetry` | novaros/elrs-telemetry:latest | 5003 | ELRS CRSF bridge |
| `elrs-producers` | novaros/elrs-producers:latest | — | RC keeper + 4 fc_* producers |
| `drone-bridge` | novaros/drone-bridge:latest | 5004 | BLE GATT + CRSF translator |
