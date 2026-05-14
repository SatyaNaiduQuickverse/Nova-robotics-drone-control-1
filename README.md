# NovaROS

Drone control + telemetry platform for Raspberry Pi 5. Police / public-safety
focused. ROS 2 Humble + MAVROS + FastAPI on the drone Pi, ELRS RF link to a
ground Pi, Android tablet over fibre / internet / BLE / AOA.

---

## One-command deployment on a fresh Pi

After `install.sh` has bootstrapped the OS (Docker + boot config + Hailo
driver), the application stack is brought up via the top-level `Makefile`:

```bash
make verify    # hardware preflight (FC, ESP, cameras, hailo, boot config, disk)
make build     # build all 7 service images (≈30 min, one-time)
make up        # production stack — drone + cameras + web + elrs + bridge + mongo
```

That's the full flight-ready stack. Vision-detect (Hailo NPU YOLO) is
**opt-in** because the Pi throttles when it runs alongside everything else:

```bash
make up-with-vision   # everything above + vision-detect
```

`make help` prints the full target list.

---

## Architecture (containers)

```
docker-compose.yml                    mongodb
ros2/docker-compose.yml               drone-control, pi-cam, vision-detect, web-control
modules/elrs_telemetry/docker-compose.yml   elrs-telemetry, elrs-producers
modules/drone_bridge/docker-compose.yml     drone-bridge
```

All seven services restart automatically on Pi reboot (Docker `restart:
unless-stopped`), except `vision-detect` which is `restart: "no"`
intentionally — start it manually when needed.

| Container        | What it does                                              | Port  |
|------------------|-----------------------------------------------------------|-------|
| `drone-control`  | MAVROS + FastAPI; talks to flight controller via USB     | 8080  |
| `vision-detect`  | YOLOv8s on Hailo NPU; HUD overlay + drift detection     | 8081  |
| `web-control`    | Flask UI; reverse-proxies `/api/*` → drone-control     | 5000  |
| `elrs-telemetry` | CRSF / mux bridge for the ESP32-C6 ELRS link            | 5003  |
| `elrs-producers` | RC keeper + 4 FC-sourced CRSF producers (telemetry)     | —     |
| `drone-bridge`   | BLE GATT + CRSF→MAVROS translator + AOA pump            | 5004  |
| `mongodb`        | Mission log storage                                       | 27017 |

---

## Host prerequisites

The Pi must have these set up before `make up` will work. `install.sh`
handles most of this on a fresh Pi.

**Disk**: Need at least **6 GB free** for a clean build of the ros2 stack
(drone-control image alone is 2.2 GB, vision-detect is 1.7 GB). `make
verify` enforces a minimum free check and fails fast.

**Docker** + docker compose v2 plugin.

**Boot config** (`/boot/firmware/config.txt`):
```
dtoverlay=vc4-kms-v3d,composite     # composite-out for analog VTX
enable_tvout=1
sdtv_mode=2                          # PAL (use 0 for NTSC)
```

**Kernel cmdline** (`/boot/firmware/cmdline.txt`):
```
video=Composite-1:720x576i@50        # PAL; for NTSC use 720x480i@60
```

**For vision-detect** (opt-in): `hailo_pci` kernel module loaded + `hailo-all`
package installed (provides the HEF model at `/usr/share/hailo-models/`).

---

## Portability gotchas (when deploying to a different Pi)

Two device paths are **hardcoded to specific hardware serials** and will need
updating for a different Pi + ESP / webcam combination:

1. **ESP32 USB serial** in `modules/elrs_telemetry/docker-compose.yml`:
   ```
   ELRS_SERIAL_DEVICE=/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_<MAC>-if00
   ```
   The `<MAC>` portion is the ESP32's USB-CDC serial — different chip,
   different serial. Edit to match the actual device on the target Pi:
   ```bash
   ls /dev/serial/by-id/usb-Espressif_USB_JTAG_*
   ```

2. **Lenovo webcam serial** in `ros2/src/drone_control/drone_control/camera.py`:
   The `FRONT_DEVICE` constant contains `..._SN0001-...`. Different webcam =
   different SN. Use `ls /dev/v4l/by-id/` on the target Pi.

The **CubeOrange / Pixhawk** device path is already glob-resolved in
`ros2/entrypoint.sh`, so it works across different FC serials without
editing.

---

## Resilience

The stack tolerates component failures without cascading:

- **FC USB disconnect** → drone-control restart-loops until FC reappears;
  other containers unaffected; producers auto-resume on recovery.
- **ELRS ESP disconnect** → elrs-telemetry catches `SerialException`, retries
  with backoff, marks `serial_connected=False`; consumers see `stale=true`
  flag, no false-fresh data.
- **drone-control crash** → producers see `fc_failures` climb (skip cycles
  rather than emit stale data); web-control `/api/*` returns 502; recovers
  on next container restart.
- **Camera USB disconnect** → `_capture_loop` reconnect loop with 1 s
  backoff; vision-detect snapshot pulls 502 during gap.
- **Pi full reboot** → all containers Docker-auto-start in the right order.

See `scripts/verify_deployment.sh` for the full preflight check.

---

## Daily operations

```bash
make ps        # status of all NovaROS containers
make logs      # tail logs across all services
make restart   # restart production stack (rarely needed; containers auto-restart)
make down      # stop everything (volumes preserved)
make prune     # free disk: remove unused images + builder cache
```

---

## Subsystem documentation

- `ros2/` — drone-control container, ROS 2 Humble + MAVROS + FastAPI
- `modules/elrs_telemetry/` — ELRS RF bridge + 5 telemetry producers
- `modules/drone_bridge/` — BLE GATT + CRSF translator + AOA pump
- `web/` — Flask UI (mounted into `web-control` container)
