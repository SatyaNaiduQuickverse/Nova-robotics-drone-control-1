# elrs_telemetry

Drone-side bridge for an ELRS RF link. Reads CRSF frames from an ESP32-C6
sitting between the RP4TD RX and the Pi over USB-CDC, exposes RC channels +
link statistics over REST + WebSocket, and accepts telemetry POSTs that
get framed as CRSF and pushed back upstream toward the ground station.

Standalone Docker container, port **5003**, completely separate from the
existing `drone` / `vision` / `web` containers.

```
GROUND                                                          DRONE  (this Pi)
──────                                                          ─────────────────
Host PC ─┐                                                      ┌─ Pi 5
         │USB CDC                                               │
         ▼                                                      ▼
      ESP32-C6 (ground_bridge)                              ESP32-C6 (drone_bridge — this firmware)
         │UART 420kbaud (CRSF)                                  │UART 420kbaud (CRSF)
         ▼                                                      ▼
      Ranger Micro TX  ─── 2.4 GHz LoRa, 150 Hz, "novadrone" ─► RP4TD RX
```

## Layout

```
modules/elrs_telemetry/
├── crsf.py                              # CRSF protocol codec (pure Python)
├── service.py                           # Flask + flask_sock + serial reader
├── ble_server.py                        # BLE GATT server (stub for later)
├── Dockerfile
├── docker-compose.yml                   # bring up with: docker compose up -d
├── firmware/drone_bridge/
│   └── drone_bridge.ino                 # ESP32-C6 USB↔UART bridge
└── tests/
    ├── test_service.py                  # unit tests (no hardware)
    └── integration_test.py              # needs ESP32 + ground side up
```

## Wiring

ESP32-C6 dev board (native USB-C) sits between the Pi and the RP4TD:

| ESP32-C6 pin       | →    | RP4TD pad             | Direction                        |
|--------------------|------|-----------------------|----------------------------------|
| GPIO 7 (UART1 RX)  | ←    | TX (data OUT from RX) | uplink — RC channels, link stats |
| GPIO 5 (UART1 TX)  | →    | RX (data IN to RX)    | downlink — drone telemetry       |
| GND                | ←→   | GND                   | shared                           |

Power the **RP4TD directly from the Pi GPIO header**, NOT from the ESP32's VBUS pin.
The RP4TD draws current spikes during RF TX bursts that an ESP32 onboard regulator may
brown out under, presenting as "uplink works, downlink missing."

| Pi GPIO header pin | →    | RP4TD pad   |
|--------------------|------|-------------|
| Pin 2 (5V)         | →    | 5V          |
| Pin 6 (GND)        | →    | GND (shared with ESP32) |

USB-C from the ESP32 to any Pi USB port. The Pi sees the ESP32 as a stable by-id symlink:

```
/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_<MAC>-if00
```

Don't use `/dev/ttyACM0` directly — it will conflict with the CubeOrange
when both are plugged in. The `docker-compose.yml` already pins to the
by-id path.

**ModemManager will probe USB-CDC devices and pollute the CRSF input stream
with AT commands** — install the ignore rule once:

```
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", ENV{ID_MM_DEVICE_IGNORE}="1"' \
    | sudo tee /etc/udev/rules.d/99-esp32-no-modemmanager.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo systemctl restart ModemManager
```

## Build & flash the firmware

Requires `arduino-cli` and the `esp32:esp32` core.

```bash
cd modules/elrs_telemetry/firmware

# One-time:
arduino-cli config add board_manager.additional_urls \
    https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32

# Compile:
arduino-cli compile --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc drone_bridge

# Flash:
arduino-cli upload -p /dev/ttyACM0 \
    --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc drone_bridge
```

After flashing, `cat /dev/ttyACM0` will show the boot banner plus a
`# stats ...` line every 2 seconds.

## Bring up the service

```bash
cd modules/elrs_telemetry
docker compose build
docker compose up -d
docker compose logs -f
```

The container has `restart: unless-stopped`, mounts `/dev`, runs
`network_mode: host`, and listens on port `5003`.

To stop:
```bash
docker compose down
```

## REST API

All endpoints are JSON. POSTs return `202 Accepted` with the CRSF bytes
that were enqueued.

| Method | Path                       | Purpose                                  |
|--------|----------------------------|------------------------------------------|
| GET    | `/healthz`                 | liveness + serial status                 |
| GET    | `/channels`                | latest 16 channels (CRSF + µs)           |
| GET    | `/link`                    | latest LINK_STATS                        |
| GET    | `/state`                   | full state snapshot + counters           |
| POST   | `/telemetry/battery`       | enqueue BATTERY_SENSOR (0x08)            |
| POST   | `/telemetry/flight_mode`   | enqueue FLIGHT_MODE (0x21)               |
| POST   | `/telemetry/gps`           | enqueue GPS (0x02)                       |
| POST   | `/telemetry/attitude`      | enqueue ATTITUDE (0x1E)                  |
| POST   | `/telemetry/raw`           | enqueue a hex-encoded CRSF frame         |
| WS     | `/ws/channels`             | push channel snapshots @ 30 Hz           |
| WS     | `/ws/link`                 | push link snapshots @ 5 Hz               |

### Stale data

Any GET whose underlying timestamp is older than **1.0 s** returns
`"stale": true` rather than erroring. That's how callers know the RF
link has gone silent.

### Auth (optional)

Set `ELRS_API_TOKEN=somesecret` in the env to require
`Authorization: Bearer somesecret` on every POST. GETs stay open.

### Rate limiting

POST telemetry endpoints share a token bucket (default **5 Hz**). Burst
beyond that returns `429 Too Many Requests`. The ELRS downlink physically
caps around 3 Hz, so 5 Hz is a comfortable upper bound that won't waste
RF time.

### Curl examples

```bash
# Health
curl -s http://localhost:5003/healthz

# Channels (live)
curl -s http://localhost:5003/channels | jq

# Link stats
curl -s http://localhost:5003/link | jq

# Full state + counters
curl -s http://localhost:5003/state | jq

# Push a battery telemetry frame
curl -s -X POST http://localhost:5003/telemetry/battery \
    -H 'Content-Type: application/json' \
    -d '{"voltage": 12.4, "current": 1.2, "mah": 100, "percent": 80}'

# Push a flight mode string
curl -s -X POST http://localhost:5003/telemetry/flight_mode \
    -H 'Content-Type: application/json' \
    -d '{"text": "ALT_HOLD"}'

# Push GPS
curl -s -X POST http://localhost:5003/telemetry/gps \
    -H 'Content-Type: application/json' \
    -d '{"lat": 12.9716, "lon": 77.5946, "alt": 100,
         "speed": 5.5, "heading": 180, "satellites": 12}'

# Push attitude (radians)
curl -s -X POST http://localhost:5003/telemetry/attitude \
    -H 'Content-Type: application/json' \
    -d '{"pitch": 0.1, "roll": -0.2, "yaw": 1.5}'

# Watch channels over WebSocket
websocat ws://localhost:5003/ws/channels

# Verify downlink (drone -> ground) actually works.
# DEVICE_PING is a CRSF parameter-discovery query; both the local RP4TD
# and the remote Ranger TX module respond with their own DEVICE_INFO.
# A "Ranger" name in the devices map = round-trip RF link confirmed.
curl -s -X POST http://localhost:5003/telemetry/raw \
    -H 'Content-Type: application/json' \
    -d '{"hex":"c8042800ea54"}'           # DEVICE_PING from FC, broadcast dest, RX origin
sleep 1 && curl -s http://localhost:5003/state | jq '.devices'
```

## Testing

```bash
# Unit tests — no hardware
python3 modules/elrs_telemetry/tests/test_service.py

# Integration — needs ESP32 plugged in + ground side transmitting + container up
python3 modules/elrs_telemetry/tests/integration_test.py
```

## Configuration (env vars)

| Variable               | Default                                                                  | Notes                                       |
|------------------------|--------------------------------------------------------------------------|---------------------------------------------|
| `ELRS_SERIAL_DEVICE`   | `/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*-if00`      | by-id symlink for hot-plug stability        |
| `ELRS_SERIAL_BAUD`     | `1000000`                                                                | USB-CDC, symbolic                           |
| `ELRS_BIND`            | `0.0.0.0`                                                                | host network, private LAN OK                |
| `ELRS_PORT`            | `5003`                                                                   | sibling of 5000 (web), 8080 (drone), 8081 (vision) |
| `ELRS_API_TOKEN`       | *(empty)*                                                                | enable bearer auth on POSTs                 |
| `ELRS_TX_RATE_HZ`      | `5.0`                                                                    | token bucket for telemetry POSTs            |
| `ELRS_LOG_LEVEL`       | `INFO`                                                                   | DEBUG / INFO / WARNING                      |

## Performance reference

Numbers measured on the ground side; drone side mirrors them.

| Path                            | Rate            | Latency       |
|---------------------------------|-----------------|---------------|
| Uplink at drone CRSF wire       | 75 Hz / 1.6 kB/s | 15–20 ms one-way |
| Downlink ground-observable      | 2–3 Hz / 500 B/s | 150–200 ms one-way |
| Round-trip via DEVICE_PING      | 2.6 Hz          | ~385 ms       |
| USB-CDC ceiling                 | ~100 kB/s       | not the bottleneck |
| ESP32 UART ceiling              | 42 kB/s         | not the bottleneck |
| RC channel resolution (CH1–4)   | 2048 positions  | exact round-trip |

## Notes / footguns

* **`link.downlink_lq` is misleading.** ELRS 4.x with low telemetry ratios
  may never tick this counter even when downlink is fully functional. Don't
  use it as a pass/fail signal. The authoritative check is "do Ranger
  DEVICE_INFO frames arrive?" — see the curl example above.
* **ModemManager pollutes the ESP32's CRSF input** with AT command probes
  unless you install the udev ignore rule (see Wiring section). Without
  it, the RP4TD's CRSF parser drops everything as garbage and downlink
  appears broken.
* **Power the RP4TD from Pi GPIO 5V, not from ESP32 VBUS.** Otherwise the
  RP4TD browns out under TX-burst load and downlink becomes intermittent.
* **CubeOrange + ESP32 both enumerate as `/dev/ttyACM*`.** This module
  pins the ESP32 by-id path; `ros2/entrypoint.sh` was also updated to
  pin the CubeOrange by-id path so they coexist regardless of plug order.
* **Pi 5 brownouts during ESP32 flashing if Pixhawk is also plugged in.**
  Symptom: full reboot mid-flash. Fix: unplug the Pixhawk before
  re-flashing the ESP32, or use a known-good 5V/5A USB-C supply.
* **The ESP32 firmware emits `# ...` diagnostic lines** to USB-CDC every
  2 seconds. `#` is `0x23` so the CRSF parser drops them byte-by-byte.
* **Don't merge this with `ros2/docker-compose.yml`.** Bring it up
  separately — that's why it has its own compose file.
* **BLE is a stub** (`ble_server.py`). Mission planning / calibration /
  parameter exchange will eventually go over BLE; flight control +
  telemetry stays on ELRS.
