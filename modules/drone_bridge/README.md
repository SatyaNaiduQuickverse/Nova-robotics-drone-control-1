# drone_bridge

Drone-Pi side of the NovaApp control + telemetry chain. Implements
deliverables 1, 2, and 3 from `~/drone_handoff/PROMPT.md`:

```
phone (NovaApp)               ground bridge Pi
   │  AOA-USB                    │  ESP32 → Ranger TX → ELRS RF
   ▼                             │  BLE central (bleak)
[ground bridge] ──────────────  ─┘
   ├ ELRS (uplink: RC channels) ────────────────────┐
   └ BLE  (config / cal / params / mission / fence) │
                                                    ▼
                          drone-side Pi  ◄── THIS MODULE
                          ┌─────────────────────────────────────┐
                          │ drone_bridge container              │
                          │   ├ pump        (poll drone-control │
                          │   │              + elrs-telemetry)  │
                          │   ├ ble_gatt    (BLE peripheral)    │
                          │   ├ translator  (CRSF → MAVROS)     │
                          │   └ debug http  :5004 /healthz etc. │
                          └─────────────────────────────────────┘
```

**One container, three internal subsystems** sharing an in-memory
snapshot. The original drone-handoff brief recommended three systemd
units; we collapsed to one because (a) the failures of these three
services are tightly coupled — if BLE dies the rest is useless to the
phone — and (b) one container is dramatically simpler to operate
alongside the other four containers on this Pi (`drone-control`,
`vision-detect`, `web-control`, `elrs-telemetry`).

**drone-control is treated as read-only.** Schema mismatches between
the phone-side spec and drone-control's actual API are translated
inside this container's BLE adapter (`adapters.py`) — never by changing
drone-control.

## File layout

```
modules/drone_bridge/
├── main.py              # entry — wires pump threads + asyncio loop
├── snapshot.py          # shared in-memory state (Snapshot dataclass)
├── pump.py              # 2 background threads: poll drone-control + elrs-telemetry
├── digest.py            # 32-byte CRSF telem digest pack/unpack (PROMPT §2.2)
├── rpc.py               # 10-byte BLE RPC header + fragmentation (PROMPT §2.3)
├── adapters.py          # spec → drone-control schema translation
├── ble.py               # bless GATT peripheral; phone tunnel
├── translator.py        # CRSF → MAVROS via /control/command, /arm, etc.
├── Dockerfile
├── docker-compose.yml
├── README.md            # this file
└── tests/
    └── test_unit.py     # rpc / digest / adapters / translator decoders
```

## Build & run

```bash
cd modules/drone_bridge
docker compose build
docker compose up -d
docker compose logs -f drone_bridge
```

## Configuration (env vars)

| Variable             | Default                                 | What it does                                              |
|----------------------|-----------------------------------------|-----------------------------------------------------------|
| `DRONE_API`          | `http://127.0.0.1:8080`                 | drone-control FastAPI base                                |
| `ELRS_API`           | `http://127.0.0.1:5003`                 | elrs-telemetry HTTP base                                  |
| `ELRS_WS`            | `ws://127.0.0.1:5003/ws/channels`       | translator's WebSocket source                             |
| `DRONE_POLL_HZ`      | `10`                                    | pump rate for `/telemetry`                                |
| `ELRS_POLL_HZ`       | `5`                                     | pump rate for `/link`                                     |
| `DEBUG_PORT`         | `5004`                                  | Flask debug HTTP                                          |
| `BLE_ADV_NAME`       | `novadrone-pi`                          | BLE advertised name (must match phone)                    |
| `BLE_MTU`            | `247`                                   | assumed MTU before negotiation                            |
| `DRONE_CH7_MODES`    | `STABILIZE,ALT_HOLD,LOITER,POSHOLD,RTL,LAND` | 6-position mode-switch table                         |
| `ENABLE_BLE`         | `1`                                     | set `0` to bring up without the BLE peripheral            |
| `ENABLE_TRANSLATOR`  | `1`                                     | set `0` to bring up without the CRSF translator           |
| `LOG_LEVEL`          | `INFO`                                  | DEBUG / INFO / WARNING                                    |

## Debug HTTP (port 5004)

For local sanity-checks without going through BLE.

```bash
# liveness + age of last sample from each upstream
curl -s http://localhost:5004/healthz | jq

# Same 32-byte payload the BLE handler returns to the phone
curl -s --output - http://localhost:5004/telemetry/digest | xxd | head

# Human-readable view of the snapshot + unpacked digest
curl -s http://localhost:5004/telemetry/digest/json | jq
```

## What gets forwarded over BLE — fast vs slow path

| Phone request | Drone-side handler | Why                                                |
|---|---|---|
| `GET /telemetry/digest` | **packed inline from snapshot** (no HTTP hop) | shaves ~3-5 ms per BLE poll (3 Hz nominal) |
| Everything else (`/calibration/*`, `/params/*`, `/mission`, `/safety/*`, `/fence/*`, `/system/*`, `/land/*`) | adapted + forwarded to drone-control via `httpx.AsyncClient` | drone-control owns the actual logic; we don't reimplement |

### Schema adaptations applied (in `adapters.py`)

| Phone sends | We forward to drone-control as |
|---|---|
| `POST /control/arm`    | `POST /arm` |
| `POST /control/disarm` | `POST /disarm` |
| `POST /control/mode`   | `POST /mode` |
| `POST /calibration/motor_test` body `{"mode":"STANDARD",…}` | `{"mode":"single",…}` |
| `POST /fence/polygon` body `{"points":[{"lat":x,"lon":y},…]}` | `{"points":[[x,y],…]}` |

## CRSF → MAVROS translator (deliverable 1)

Subscribes to `elrs-telemetry`'s `/ws/channels` (instead of opening
`/dev/serial/by-id/usb-Express_LRS_*` directly — that device is owned
by the elrs-telemetry container).

| CRSF channel | Effect                                                                   |
|--------------|--------------------------------------------------------------------------|
| CH1 (roll)   | `/control/command` `roll`     ∈ [-1, +1]                                 |
| CH2 (pitch)  | `/control/command` `pitch`    ∈ [-1, +1]   *(NO double-flip — phone's already inverted)* |
| CH3 (throttle)| `/control/command` `throttle`∈ [0, 1]                                   |
| CH4 (yaw)    | `/control/command` `yaw`      ∈ [-1, +1]                                 |
| CH5 ≥ 1500   | `POST /arm`, edge-detected                                               |
| CH5 < 1500   | `POST /disarm`, edge-detected                                            |
| CH6 ≥ 1500 ↑ | `POST /disarm/force` (force-disarm, rising edge only)                    |
| CH7 (6-pos)  | `POST /mode` with name from `DRONE_CH7_MODES`, on index change           |
| CH9 ≥ 1500 ↑ | `POST /land/precision`, rising edge only                                 |

**Hard link-loss guard at 300 ms.** If we stop seeing channel updates,
we STOP calling `/control/command`. We do NOT send neutral sticks —
that defeats ArduPilot's onboard RC failsafe. The FC's `FS_THR_*`
parameters then handle the situation.

## Tests

```bash
# Unit tests — no hardware
python3 -m unittest drone_bridge.tests.test_unit -v
# (or, inside the container: python3 -m drone_bridge.tests.test_unit)

# End-to-end smoke test (drone-control reachable, no BLE)
python3 ~/drone_handoff/drone_smoke.py --mode=fastapi

# End-to-end via BLE (run from a SECOND BLE-capable host —
# scanning for SERVICE_UUID 6e400000-...)
python3 ~/drone_handoff/drone_smoke.py --mode=ble
```

## Notes / footguns

* **drone-control must be up first.** The BLE server retries for 30 s
  on `ConnectionRefused` before giving up; if drone-control isn't
  reachable after that, BLE still starts but every forward returns 502.
* **`bless` needs DBus + privileged + host-network** in Docker. The
  compose file mounts `/var/run/dbus` and runs `privileged: true`.
* **Handle one phone request at a time** is by design (PROMPT §2.3 v1).
  An `asyncio.Lock` serializes the upstream HTTP call so the BLE event
  loop stays responsive even on slow endpoints.
* **The digest is packed inline from snapshot** — the BLE handler does
  NOT call back to the debug HTTP. The debug `/telemetry/digest` exists
  only for curl-from-the-LAN sanity checks; the BLE path is fully
  in-process.
* **Fragmentation**: each fragment carries its own header; receiver
  accumulates body slices until EOM. This matches the canonical
  `novabridge/ble/rpc.py` contract that the phone uses. Note that the
  reassembly logic in `drone_smoke.py --mode=ble` is buggy for
  multi-fragment responses (it re-appends the header bytes); that test
  mode reliably handles only single-fragment responses. Real phone
  testing is done via `novabridge/tools/ble_smoke.py` from the ground
  bridge.
