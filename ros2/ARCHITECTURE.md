# NovaROS Drone Control System - Architecture

## Design Philosophy

This system was built with **deliberate, safety-first engineering** - not vibecoding.

Every line of code exists for a reason:
- No unnecessary abstractions
- No premature optimization
- No "nice to have" features
- No copy-paste code
- Direct MAVROS service calls, no wrappers

**Total codebase: ~1,500 lines of Python** - minimal, auditable, maintainable.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         Docker Container                         │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────┐ │
│  │   MAVROS    │◄──►│  Telemetry  │◄──►│    API Gateway      │ │
│  │   Node      │    │   Module    │    │    (FastAPI)        │ │
│  └──────┬──────┘    └─────────────┘    └──────────┬──────────┘ │
│         │                                          │            │
│         │ ROS2 Topics/Services                     │ HTTP :8080 │
│         │                                          │            │
│  ┌──────┴──────┐                                   │            │
│  │ Virtual TX  │───► /mavros/rc/override           │            │
│  └─────────────┘                                   │            │
└─────────┼──────────────────────────────────────────┼────────────┘
          │                                          │
          ▼                                          ▼
    ┌───────────┐                            ┌─────────────┐
    │  Pixhawk  │                            │ MERN App /  │
    │ (Serial)  │                            │ External    │
    └───────────┘                            └─────────────┘
```

---

## File Structure

```
ros2/
├── docker-compose.yml      # Single container deployment
├── Dockerfile              # ROS2 Humble + MAVROS + FastAPI
├── ARCHITECTURE.md         # This file
├── API.md                  # API reference for web app
└── src/
    └── drone_control/
        ├── package.xml             # ROS2 package manifest
        ├── setup.py                # Python package setup
        ├── setup.cfg               # Entry points
        ├── resource/drone_control  # Ament marker
        └── drone_control/
            ├── __init__.py
            ├── api_gateway.py      # FastAPI REST endpoints (~420 lines)
            ├── telemetry.py        # ROS2 subscriptions (~330 lines)
            ├── mission.py          # QGC WPL 110 parser (~50 lines)
            ├── virtual_tx.py       # Virtual RC transmitter (~490 lines)
            └── control_node.py     # Standalone node (optional)
```

---

## Component Details

### 1. Telemetry Module (`telemetry.py`)

**Purpose:** Subscribe to all MAVROS topics, store latest values.

**Subscribed Topics:**
| Topic | Message Type | Data |
|-------|--------------|------|
| `/mavros/state` | State | connected, armed, mode |
| `/mavros/extended_state` | ExtendedState | landed_state, vtol_state |
| `/mavros/battery` | BatteryState | voltage, current, percentage |
| `/mavros/global_position/global` | NavSatFix | lat, lon, alt |
| `/mavros/global_position/raw/satellites` | UInt32 | satellite count |
| `/mavros/global_position/raw/fix` | NavSatFix | HDOP estimate |
| `/mavros/local_position/pose` | PoseStamped | x, y, z, orientation |
| `/mavros/local_position/velocity_local` | TwistStamped | vx, vy, vz |
| `/mavros/global_position/rel_alt` | Float64 | relative altitude |
| `/mavros/global_position/compass_hdg` | Float64 | heading |
| `/mavros/home_position/home` | NavSatFix | home position |
| `/mavros/imu/data` | Imu | accelerometer, gyroscope |
| `/mavros/imu/mag` | MagneticField | magnetometer |
| `/mavros/imu/static_pressure` | FluidPressure | barometric pressure |
| `/mavros/imu/temperature_baro` | Temperature | temperature |
| `/mavros/rc/in` | RCIn | RC input channels |
| `/mavros/rc/out` | RCOut | servo/motor outputs |
| `/mavros/statustext/recv` | StatusText | FCU messages |

**QoS Configuration:**
- State topics: RELIABLE + TRANSIENT_LOCAL (guaranteed delivery)
- Sensor topics: BEST_EFFORT + VOLATILE (low latency)

### 2. API Gateway (`api_gateway.py`)

**Purpose:** REST API for external systems (MERN app, GCS, etc.)

See `API.md` for complete endpoint documentation with request/response examples and JavaScript code.

#### Command Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/arm` | Arm (requires STABILIZE mode) |
| POST | `/disarm` | Normal disarm (may be rejected) |
| POST | `/disarm/force` | Force disarm (bypasses safety) |
| POST | `/mode` | Set flight mode |
| GET | `/modes` | List available modes |

#### Mission Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/mission` | Upload QGC WPL 110, set AUTO, arm |
| DELETE | `/mission` | Clear mission from Pixhawk |

#### Control Endpoints (Virtual TX)

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/control/enable` | Enable virtual TX mode |
| POST | `/control/disable` | Disable and release RC |
| GET | `/control/status` | Get enabled state, hold_remaining, outputs |
| POST | `/control/command` | Send timestamped input with duration |
| GET | `/control/config` | Get tunable parameters |
| POST | `/control/config` | Update parameters |

#### Telemetry Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/telemetry` | All telemetry data |
| GET | `/telemetry/state` | Connection, armed, mode |
| GET | `/telemetry/battery` | Voltage, current, percentage |
| GET | `/telemetry/gps` | Lat, lon, alt, satellites, hdop |
| GET | `/telemetry/local_position` | X, Y, Z position |
| GET | `/telemetry/orientation` | Roll, pitch, yaw |
| GET | `/telemetry/velocity` | Vx, Vy, Vz |
| GET | `/telemetry/imu` | Accelerometer, gyroscope |
| GET | `/telemetry/mag` | Magnetometer |
| GET | `/telemetry/baro` | Pressure, temperature |
| GET | `/telemetry/vfr_hud` | Airspeed, groundspeed, heading, throttle |
| GET | `/telemetry/rc_in` | RC input channels |
| GET | `/telemetry/rc_out` | Motor/servo outputs |
| GET | `/telemetry/home` | Home position |
| GET | `/telemetry/extended_state` | Landed state |

### 3. Mission Module (`mission.py`)

**Purpose:** Pure parser for QGC WPL 110 format. Service calls (clear, upload) are handled by the API gateway.

**QGC WPL 110 Format:**
```
QGC WPL 110
<index> <current> <frame> <command> <p1> <p2> <p3> <p4> <lat> <lon> <alt> <autocontinue>
0       1         0       16        0    0    0    0    28.61 77.20 10   1
1       0         3       16        0    0    0    0    28.62 77.21 15   1
```

**Mission Flow (orchestrated by API gateway under single lock):**
1. Parse WPL text → Waypoint list (`mission.parse_qgc_wpl`)
2. Clear existing mission (`/mavros/mission/clear`)
3. Upload waypoints (`/mavros/mission/push`)
4. Set AUTO mode (`/mavros/set_mode`)
5. Arm if requested (`/mavros/cmd/arming`)

### 4. Virtual TX Module (`virtual_tx.py`)

**Purpose:** Digital RC transmitter designed for internet-based control. Handles latency, packet reordering, duplicate commands, and connection drops safely.

**Published Topic:**
| Topic | Message Type | Rate |
|-------|--------------|------|
| `/mavros/rc/override` | OverrideRCIn | 50 Hz |

**Channel Mapping:**
| Channel | Function | Input Range | PWM Range |
|---------|----------|-------------|-----------|
| 0 | Roll | -1.0 to 1.0 | 1000-2000 |
| 1 | Pitch | -1.0 to 1.0 | 1000-2000 |
| 2 | Throttle | 0.0 to 1.0 | 1100-2000 |
| 3 | Yaw | -1.0 to 1.0 | 1000-2000 |

**Processing Pipeline:**
```
Input → Age Check → Duration Check → Order Check → Dedup → Deadzone → Expo → Smoothing → Rate Limit → PWM
```

**Internet-Safe Design:**
- **Duration hold:** Each command carries for a specified time (default 1s). No need to send continuous updates. If network drops, the last command holds.
- **Out-of-order rejection:** Commands carry timestamps. If an older command arrives after a newer one (network reordering), it is rejected.
- **Dedup:** If the user presses the same button multiple times (due to latency not seeing feedback), duplicate commands just extend the hold timer without re-triggering control transitions. Response says `"Hold"` instead of `"OK"`.
- **Mode-aware failsafe:** When the hold expires with no new command, the TX auto-centers all channels smoothly. Roll/pitch/yaw → 0. Throttle target depends on flight mode: **STABILIZE → idle (0.0)**, **ALT_HOLD/LOITER/GUIDED/AUTO → mid-stick (0.44 ≈ PWM 1500 = hold altitude)**. Channels are NEVER released to 65535 while enabled — this prevents ArduPilot RC failsafe from triggering a disarm.

**Measured Performance:**
| Metric | Value |
|--------|-------|
| Command-to-motor latency | ~63ms median (dominated by 10Hz MAVLink stream) |
| Rate limit transition (full reversal) | ~400ms |
| Smoothing convergence | ~200ms to 90% of target |

---

## Data Structures

### Telemetry Response
```json
{
  "state": {"connected": true, "armed": false, "mode": "STABILIZE", "system_status": 3},
  "extended_state": {"landed_state": 0, "vtol_state": 0},
  "battery": {"voltage": 16.2, "current": 1.5, "percentage": 0.85},
  "gps": {"fix_type": 3, "latitude": 28.6139, "longitude": 77.2090, "altitude": 25.0, "satellites": 12, "hdop": 0.9},
  "local_position": {"x": 0.0, "y": 0.0, "z": -2.5},
  "orientation": {"roll": 0.01, "pitch": -0.02, "yaw": 1.57},
  "velocity": {"vx": 0.0, "vy": 0.0, "vz": 0.0},
  "imu": {"ax": 0.1, "ay": 0.0, "az": 9.8, "gx": 0.0, "gy": 0.0, "gz": 0.0},
  "mag": {"x": 0.2, "y": 0.1, "z": 0.4},
  "baro": {"pressure": 101325.0, "temperature": 25.0},
  "vfr_hud": {"airspeed": 0.0, "groundspeed": 0.0, "heading": 90, "throttle": 0.0, "altitude": 25.0, "climb_rate": 0.0},
  "rc_in": {"channels": [1500, 1500, 1100, 1500, ...]},
  "rc_out": {"channels": [1000, 1000, 1000, 1000, ...]},
  "home": {"latitude": 28.6139, "longitude": 77.2090, "altitude": 0.0},
  "relative_alt": 25.0,
  "heading": 90.0,
  "status_text": "GPS: 3D Fix"
}
```

### Control Status Response
```json
{
  "enabled": true,
  "in_failsafe": false,
  "centering": false,
  "hold_remaining": 0.73,
  "last_command_age": 0.27,
  "outputs": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.5},
  "pwm": {"roll": 1500, "pitch": 1500, "yaw": 1500, "throttle": 1540}
}
```

---

## Flight Mode Mapping

| API Mode | ArduPilot | PX4 | Throttle Behavior | Failsafe Throttle |
|----------|-----------|-----|-------------------|-------------------|
| stabilize | STABILIZE | STABILIZED | Direct motor power | Idle (PWM 1100) |
| alt_hold | ALT_HOLD | ALTCTL | Mid=hold alt, below=descend, above=climb | Mid-stick (PWM 1500) |
| loiter | LOITER | POSCTL | Same as alt_hold + GPS position hold | Mid-stick (PWM 1500) |
| land | LAND | AUTO.LAND | Automatic | Mid-stick (PWM 1500) |
| rtl | RTL | AUTO.RTL | Automatic | Mid-stick (PWM 1500) |
| guided | GUIDED | OFFBOARD | Automatic | Mid-stick (PWM 1500) |
| auto | AUTO | AUTO.MISSION | Automatic | Mid-stick (PWM 1500) |
| brake | BRAKE | AUTO.LOITER | Automatic | Mid-stick (PWM 1500) |

**Note:** LOITER requires GPS fix. ALT_HOLD requires barometer only. For manual flying, ALT_HOLD is recommended over STABILIZE — the FC handles altitude, pilot handles direction.

---

## Deployment

### Start System
```bash
cd ~/novaros/ros2
docker compose up -d
```

### Enable Sensor Streams (required after each restart)
```bash
docker exec drone-control bash -c 'source /opt/ros/humble/setup.bash && \
  ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate \
  "{stream_id: 0, message_rate: 10, on_off: true}"'
```

### Check Status
```bash
curl http://localhost:8080/telemetry/state
```

### View Logs
```bash
docker logs -f drone-control
```

---

## Hardware Configuration

| Component | Connection |
|-----------|------------|
| Flight Controller | CubeOrange+ (Pixhawk) |
| Serial Port | /dev/ttyACM0 @ 115200 baud |
| Companion Computer | Raspberry Pi 5 (8GB) |
| Firmware | ArduCopter V4.6.3 |
| Frame | QUAD/X |

---

## Safety Considerations

### What We Built
- Direct MAVROS calls - no abstraction layers to fail
- Explicit safety checks before arm/mode change
- Force disarm requires explicit endpoint (`/disarm/force`)
- All commands return success/failure status
- Virtual TX never releases channels while enabled (prevents RC failsafe disarm)
- Duration-based hold prevents loss of control on network drops
- Out-of-order rejection prevents stale commands from overriding current state
- Thread-safe service calls: single lock serializes all MAVROS commands, future polling avoids concurrent executor spinning

### What We Did NOT Build
- Automatic failsafes (ArduPilot handles these)
- Redundant safety wrappers (trust the FCU)
- Complex state machines (keep it simple)
- Unnecessary error handling (fail fast)

### ArduPilot Handles
- Pre-arm checks (GPS, battery, sensors)
- Geofencing
- Return-to-Launch on signal loss
- Battery failsafe
- Motor interlock

---

## Resource Usage (Raspberry Pi 5)

| Resource | Usage |
|----------|-------|
| RAM | ~2.1 GB / 8 GB (26%) |
| CPU | ~30% (MAVROS + API) |
| Container Memory | ~200 MB |

---

## Design Decisions

### Why Single Container?
- ROS2 DDS discovery works reliably within single container
- Simpler deployment and debugging
- No inter-container networking issues

### Why FastAPI?
- Minimal, fast, async-capable
- Auto-generates OpenAPI docs
- Pydantic validation built-in

### Why Not ROS2 Actions?
- HTTP REST is universal - works with any client
- MERN stack integration is straightforward
- No ROS2 client library needed on frontend

### Why QGC WPL 110?
- Industry standard format
- Compatible with Mission Planner, QGroundControl
- Human-readable, easy to debug

### Why Duration-Based Commands?
- Internet latency makes continuous streaming unreliable
- A single command with `duration: 2.0` holds for 2 seconds without needing re-sends
- If network recovers, next command seamlessly takes over
- If network stays down, failsafe auto-centers after hold expires

### Why Future Polling Instead of spin_until_future_complete?
- `telemetry.py` runs `rclpy.spin()` in a background thread to receive MAVROS topics
- Calling `spin_until_future_complete()` from FastAPI threads on the same node violates ROS2's single-threaded executor model
- Instead: `call_async()` + poll `future.done()` — the background spin thread resolves the future safely
- A `threading.Lock` serializes all service calls to prevent concurrent MAVROS commands

---

## Future Additions (When Needed)

- [ ] WebSocket for real-time telemetry streaming
- [ ] Video streaming endpoint
- [ ] Geofence management
- [ ] Parameter read/write
- [ ] Log download

---

*Built with deliberate engineering. Every line has a purpose.*
