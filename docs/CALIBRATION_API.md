# NovaROS Calibration API — Detailed Documentation

> **For:** Mobile app developers building the calibration UI  
> **Base URL:** `http://<drone-ip>:8080`  
> **Telemetry polling:** `GET /telemetry/messages` for FCU statustext during calibration flows

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Prerequisites & Safety](#prerequisites--safety)
3. [Calibration Status Check](#calibration-status-check)
4. [Motor Test](#motor-test)
5. [Gyro Calibration](#gyro-calibration)
6. [Level Horizon (AHRS Trim)](#level-horizon-ahrs-trim)
7. [Barometer Calibration](#barometer-calibration)
8. [Compass Calibration (Full System)](#compass-calibration-full-system)
9. [Accelerometer Calibration (6-Position)](#accelerometer-calibration-6-position)
10. [FC Reboot](#fc-reboot)
11. [MAVROS Reconnect](#mavros-reconnect)
12. [State Machine Reference](#state-machine-reference)
13. [Fitness Quality Bands](#fitness-quality-bands)
14. [Mobile App Implementation Guide](#mobile-app-implementation-guide)

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                  drone-control container                  │
│                                                          │
│  ┌─────────────┐   MAV_CMD_*    ┌─────────────────────┐ │
│  │  FastAPI     │──────────────▶│  MAVROS ROS2 node    │ │
│  │  /calibration│               │  /cmd/command        │ │
│  │  endpoints   │               │  (CommandLong.srv)   │ │
│  └──────┬───────┘               └──────────┬──────────┘ │
│         │                                   │            │
│         │  raw MAVLink                      │ serial     │
│         │  ┌────────────────┐               │            │
│         │  │  compass_cal.py │◀─────────────┤            │
│         │  │  MAVLink decoder│              │            │
│         │  │  /uas1/mavlink_ │              ▼            │
│         │  │   source (ROS2) │    ┌─────────────────┐   │
│         │  └────────────────┘    │  CubeOrange+ FC  │   │
│         │                         │  ArduCopter 4.6.3│   │
│         ▼                         │  /dev/ttyACM0    │   │
│  ┌──────────────┐                 └─────────────────┘   │
│  │ GET /calibra-│                                        │
│  │ tion/compass/│  (progress data returned to mobile)    │
│  │ progress     │                                        │
│  └──────────────┘                                        │
└─────────────────────────────────────────────────────────┘
```

### How It Works Under the Hood

1. **API layer** (`api_gateway.py`) exposes REST endpoints for each calibration action.
2. **MAVLink commands** are sent via MAVROS's `CommandLong` service (`/mavros/cmd/command`). Each calibration type maps to a specific `MAV_CMD_*` code.
3. **Progress feedback** comes back differently per calibration type:
   - **Compass:** The FC sends `MAG_CAL_PROGRESS` (msgid 191) and `MAG_CAL_REPORT` (msgid 192) MAVLink messages. MAVROS Humble does NOT publish these as typed ROS topics — instead they appear as raw bytes on `/uas1/mavlink_source`. Our `compass_cal.py` module subscribes there with BEST_EFFORT QoS, manually decodes the binary payload using `struct.unpack`, and exposes thread-safe state via `GET /calibration/compass/progress`.
   - **Gyro/Level/Baro:** Instant (single command, FC responds with success/fail).
   - **Accelerometer:** Multi-step. FC emits statustext prompts ("Place vehicle LEVEL…") that appear in `/telemetry/messages`.
   - **Motor test:** Instant command, motors spin for the requested duration.

4. **Service client recovery:** If the FC reboots (required after compass cal), MAVROS recreates its services with new GIDs. Our code auto-rebuilds all service clients ~12s post-reboot via a background thread. During this window, service calls return null — the mobile app should poll `/telemetry/state` → `connected: true` before retrying.

---

## Prerequisites & Safety

### Before Any Calibration

1. **Drone must be disarmed.** All calibration endpoints return HTTP 409 if armed.
2. **FC must be connected.** Returns HTTP 503 if MAVROS hasn't connected.
3. **For motor tests: REMOVE ALL PROPELLERS.** The API does not enforce this — the mobile app must confirm with the user.

### Pre-check Endpoint

**`GET /calibration/status`**

Call this before showing the calibration screen. Gates the UI.

**Response:**
```json
{
  "connected": true,
  "armed": false,
  "mode": "STABILIZE",
  "available": {
    "motor_test": true,
    "gyro": true,
    "level_horizon": true,
    "baro": true,
    "compass": true,
    "accelerometer": true
  }
}
```

**Mobile app logic:**
- If `connected: false` → show "No FC connection" banner, disable all buttons
- If `armed: true` → show "Disarm drone first" banner, disable all buttons
- If both OK → enable calibration UI

---

## Motor Test

### Purpose
Verify motor wiring, direction, and response. Spin individual motors or a sequence.

### Safety
- **PROPELLERS MUST BE REMOVED.** Add a confirmation dialog in the mobile app.
- FC rejects if armed.
- Emergency stop is always available.

### Start Motor Test

**`POST /calibration/motor_test`**

**Request:**
```json
{
  "motor": 1,
  "throttle_pct": 10.0,
  "duration_s": 2.0,
  "mode": "single",
  "motor_count": 1
}
```

| Field | Type | Range | Default | Description |
|---|---|---|---|---|
| `motor` | int | 1–12 | 1 | Motor number (1-based, frame order) |
| `throttle_pct` | float | 0–100 | 10.0 | Throttle percentage. **Start low (5–10%) for bench tests.** |
| `duration_s` | float | 0.1–10.0 | 2.0 | How long each motor spins |
| `mode` | string | `"single"` / `"sequence"` | `"single"` | Single motor or sequential through count |
| `motor_count` | int | 1–12 | 1 | Number of motors in sequence mode |

**Modes explained:**
- **`single`**: Spins motor number `motor` at `throttle_pct` for `duration_s` seconds.
- **`sequence`**: Spins motors sequentially in frame order, M1 → M2 → … → M`motor_count`, each for `duration_s` seconds. Lets you verify all motors + rotation direction in one shot.

**Response:**
```json
{
  "success": true,
  "message": "Motor test started: single m1 @ 10% for 2.0s",
  "result_code": 0
}
```

**`result_code` values:**
| Code | Meaning |
|---|---|
| 0 | MAV_RESULT_ACCEPTED |
| 4 | MAV_RESULT_DENIED (armed, or FC rejected) |

### Emergency Stop

**`POST /calibration/motor_test/stop`**

Immediately sends 0% throttle AND force-disarms as belt-and-suspenders. **Always show this button prominently during motor tests.**

**Request:** no body

**Response:**
```json
{ "success": true, "message": "Motor test stopped" }
```

### Motor Numbering (ArduCopter Quad-X)

```
    FRONT
  M1      M2
    \    /
     \  /
      \/
      /\
     /  \
    /    \
  M4      M3
     REAR
```

| Motor | Position | Rotation |
|---|---|---|
| M1 | Front-Right | CCW |
| M2 | Front-Left | CW |
| M3 | Rear-Left | CCW |
| M4 | Rear-Right | CW |

---

## Gyro Calibration

### Purpose
Zero the gyroscope bias. Required when drift accumulates or after temperature changes.

### Procedure
1. Place drone on a **completely still, flat surface**
2. Do not touch the drone during calibration (~5 seconds)

### API

**`POST /calibration/gyro`**

**Request:** no body  
**Timeout:** Allow up to 20 seconds for the response.

**Response:**
```json
{ "success": true, "message": "Gyro calibration complete", "result_code": 0 }
```

**Failure case:**
```json
{ "success": false, "message": "Gyro calibration rejected (MAV_RESULT=4)", "result_code": 4 }
```
Usually means vibration detected — the surface wasn't still enough.

---

## Level Horizon (AHRS Trim)

### Purpose
Sets the "level" reference for the attitude sensor. Corrects if the flight controller isn't perfectly parallel to the drone frame. Sets `AHRS_TRIM_X` and `AHRS_TRIM_Y` parameters.

### Procedure
1. Place drone on a **known-level surface** (use a bubble level if available)
2. Ensure it's not tilted by any landing gear unevenness

### API

**`POST /calibration/level_horizon`**

**Request:** no body  
**Timeout:** 20 seconds

**Response:**
```json
{ "success": true, "message": "Level horizon captured", "result_code": 0 }
```

---

## Barometer Calibration

### Purpose
Reset the barometer's ground-pressure reference. Sets the current location as altitude zero. Useful after weather changes or location changes.

### API

**`POST /calibration/baro`**

**Request:** no body  
**Timeout:** 10 seconds

**Response:**
```json
{ "success": true, "message": "Barometer zeroed", "result_code": 0 }
```

---

## Compass Calibration (Full System)

This is the most complex calibration. It involves a multi-step flow with real-time progress feedback, a 3D visualization, timeout handling, and an automatic FC reboot on success.

### Purpose
Calibrate the onboard magnetometer (compass). Removes hard-iron and soft-iron interference from the frame, motors, and wiring. Critical for GPS-dependent modes (LOITER, RTL, AUTO).

### How ArduPilot Compass Cal Works (Internal)

1. FC enters a sampling phase where it collects magnetometer readings from many orientations
2. It needs readings distributed across a full sphere of orientations (imagine the drone's nose tracing the surface of a sphere)
3. Progress increases as new unique orientations are sampled
4. When enough coverage is achieved, the FC runs a least-squares sphere fit to compute offsets
5. Results: `COMPASS_OFS_X/Y/Z` + diagonal/off-diagonal correction matrix
6. These are saved to FC flash (if autosave=true) but **require a reboot to take effect**

### Server-Side Decoder: compass_cal.py

MAVROS Humble does NOT expose compass calibration progress as typed ROS topics. We built a custom decoder:

- **Subscribes to:** `/uas1/mavlink_source` (raw MAVLink republished by MAVROS)
- **QoS:** BEST_EFFORT reliability (must match publisher — RELIABLE subscription would get zero messages)
- **Decodes:**
  - `MAG_CAL_PROGRESS` (message ID 191, 27 bytes) — direction vector, completion %, cal status
  - `MAG_CAL_REPORT` (message ID 192, 44 bytes) — fitness, offsets, diag/offdiag matrix, autosaved flag
- **Binary layout decoded with `struct.unpack`** — field order follows MAVLink wire format (largest-first packing)
- **Thread-safe** — the ROS subscription callback writes under `_lock`; the FastAPI endpoint reads under the same lock

### 3D Coverage Sphere Visualization

The web UI includes a **Three.js WebGL sphere** that shows compass calibration coverage in real-time:

#### Concept
- A wireframe unit sphere represents all possible drone orientations
- As the user rotates the drone, green dots appear on the sphere at each sampled orientation
- Uncovered areas remain as dim-red wireframe — easy to see what's missing
- A yellow "current direction" dot shows where the drone is pointing right now
- The sphere auto-rotates slowly so the user can see coverage from all angles

#### How It Works
1. Each `MAG_CAL_PROGRESS` message includes a `direction` vector `[dx, dy, dz]` — the gravity direction in body frame
2. The direction is normalized to unit length: `[x/|v|, y/|v|, z/|v|]`
3. The normalized vector is placed as a point on the surface of a unit sphere
4. Deduplication: vectors are rounded to a grid (`Math.round(x*20)`) so the same orientation doesn't consume buffer slots
5. Maximum 2000 points (Float32Array buffer, pre-allocated)
6. Visual feedback:
   - Sphere starts dim-red (0x552222) = "uncovered"
   - Green dots (0x00ff88) = "covered"
   - Yellow dot (0xffdd44) = "current direction"
   - On success: sphere turns dark-green (0x225522)

#### Mobile App Implementation Guidance for 3D Sphere

For the mobile app, you have two options:

**Option A — Native 3D (recommended):**
Use SceneKit (iOS) or OpenGL/Vulkan (Android) to render:
- A wireframe sphere (radius 1.0)
- XYZ axis helper lines
- A point cloud that grows as `/calibration/compass/progress` returns new direction vectors
- A "current" indicator dot at the latest direction

**Option B — 2D projection:**
If 3D is too heavy, project the direction vectors onto a 2D Mollweide or equirectangular map:
- X axis = yaw angle (atan2(y, x))
- Y axis = pitch angle (asin(z))
- Paint green dots on the 2D map as orientations are covered
- Show gaps as empty regions

Either way, the data source is the same: poll `GET /calibration/compass/progress` every 500ms and read the `direction` field.

### API Endpoints

#### Start Compass Calibration

**`POST /calibration/compass/start`**

**Request:**
```json
{
  "autosave": true,
  "retry_on_fail": false
}
```

| Field | Type | Default | Description |
|---|---|---|---|
| `autosave` | bool | `true` | Automatically save offsets to FC flash on success. **Always use true** — otherwise offsets are lost on reboot. |
| `retry_on_fail` | bool | `false` | FC auto-retries if calibration fails. Generally leave false so the app has control over retry logic. |

**Response:**
```json
{
  "success": true,
  "message": "Compass calibration started — rotate drone through all axes",
  "result_code": 0
}
```

**After calling start:**
1. Begin polling `/calibration/compass/progress` every 500ms
2. Initialize your 3D sphere visualization
3. Start a wall-clock timer (timeout at 300s)
4. Instruct user: "Rotate the drone slowly through all axes — imagine drawing a sphere with the nose. Aim for at least 60 seconds."

#### Poll Progress

**`GET /calibration/compass/progress`**

This is decoded from raw MAVLink MAG_CAL_PROGRESS/REPORT packets. Must be polled — it is not a push endpoint.

**Response during calibration:**
```json
{
  "active": true,
  "progress": {
    "compass_id": 0,
    "cal_status": 3,
    "cal_status_name": "RUNNING_STEP_TWO",
    "completion_pct": 45,
    "attempt": 1,
    "direction": [0.234, -0.567, 0.789]
  },
  "report": null,
  "last_update_t": 1713600045.123
}
```

**Response on success:**
```json
{
  "active": false,
  "progress": {
    "compass_id": 0,
    "cal_status": 4,
    "cal_status_name": "SUCCESS",
    "completion_pct": 100,
    "attempt": 1,
    "direction": [0.1, 0.3, 0.9]
  },
  "report": {
    "compass_id": 0,
    "cal_status": 4,
    "cal_status_name": "SUCCESS",
    "autosaved": true,
    "fitness": 5.23,
    "offsets": [-15.2, 8.7, -3.1],
    "diag": [1.0, 1.0, 1.0],
    "offdiag": [0.0, 0.0, 0.0]
  },
  "last_update_t": 1713600105.456
}
```

**Response on failure:**
```json
{
  "active": false,
  "progress": { "cal_status": 5, "cal_status_name": "FAILED", ... },
  "report": { "cal_status": 5, "cal_status_name": "FAILED", "fitness": 999.0, ... },
  "last_update_t": 1713600080.789
}
```

**Field reference:**

| Field | Type | Description |
|---|---|---|
| `active` | bool | True while samples are still being collected |
| `progress.compass_id` | int | Which compass (0 = primary, 1 = secondary, etc.) |
| `progress.cal_status` | int | See [cal_status enum](#cal_status-enum) |
| `progress.cal_status_name` | string | Human-readable status name |
| `progress.completion_pct` | int | 0–100 completion percentage |
| `progress.attempt` | int | Attempt number (starts at 1) |
| `progress.direction` | [float, float, float] | Current body-frame gravity direction (unit vector). **Use this for the 3D sphere.** |
| `report` | object\|null | Only present when calibration terminates (success or failure) |
| `report.fitness` | float | Calibration fit quality. Lower = better. See [fitness bands](#fitness-quality-bands). |
| `report.offsets` | [float, float, float] | Computed compass offsets [X, Y, Z] |
| `report.diag` | [float, float, float] | Diagonal correction matrix |
| `report.offdiag` | [float, float, float] | Off-diagonal correction matrix |
| `report.autosaved` | bool | Whether offsets were written to FC flash |
| `last_update_t` | float | Unix timestamp of last MAVLink message received |

#### cal_status Enum

| Value | Name | Meaning | Terminal? |
|---|---|---|---|
| 0 | NOT_STARTED | Calibration hasn't begun | No |
| 1 | WAITING_TO_START | FC acknowledged, waiting for first movement | No |
| 2 | RUNNING_STEP_ONE | Collecting initial samples | No |
| 3 | RUNNING_STEP_TWO | Refining fit (needs corner/diagonal coverage) | No |
| 4 | SUCCESS | Calibration complete, offsets computed | **Yes** |
| 5 | FAILED | Calibration failed (bad data or timeout) | **Yes** |
| 6 | BAD_ORIENTATION | Insufficient orientation coverage | **Yes** |
| 7 | BAD_RADIUS | Inconsistent field magnitude (interference) | **Yes** |

#### Accept (Manual)

**`POST /calibration/compass/accept`**

Only needed if `autosave: false` was used at start. Writes computed offsets to FC flash.

**Response:**
```json
{ "success": true, "message": "Compass cal accepted", "result_code": 0 }
```

#### Cancel

**`POST /calibration/compass/cancel`**

Aborts an in-progress calibration. Safe to call at any time.

**Response:**
```json
{ "success": true, "message": "Compass cal cancelled", "result_code": 0 }
```

### Compass Calibration State Machine (for mobile app)

```
                    ┌─────────┐
                    │  IDLE   │
                    └────┬────┘
                         │ POST /compass/start (success)
                         ▼
                    ┌─────────┐
              ┌────▶│ RUNNING │◀────────────────────────┐
              │     └────┬────┘                          │
              │          │                               │
              │          ├─── poll /compass/progress ────┤
              │          │    every 500ms                │
              │          │                               │
              │          ├─── report.cal_status == 4 ───▶ SUCCESS
              │          │                               │    │
              │          ├─── report.cal_status >= 5 ───▶ FAILED
              │          │                                    │
              │          ├─── stall timeout (30s no progress change) ─▶ FAILED
              │          │         (auto-cancel sent)
              │          │
              │          ├─── wall-clock timeout (300s) ─▶ FAILED
              │          │         (auto-cancel sent)
              │          │
              │          └─── user cancels ─────────────▶ CANCELLED
              │
              └─── user clicks "Retry" ─────────────────┘
```

### Timeout Logic

The mobile app should implement these timeouts:

| Timeout | Duration | Action |
|---|---|---|
| **Stall timeout** | 30 seconds without `completion_pct` change | Auto-cancel + show "Stuck — rotate through corners/diagonals" |
| **Wall-clock timeout** | 300 seconds total | Auto-cancel + show "Timeout — try again" |
| **Minimum duration warning** | < 60 seconds | Show warning "Too fast — offsets may be inaccurate" |

### Post-Success Flow

After `cal_status == 4` (SUCCESS):

1. **Check fitness** — show quality band (see below)
2. **If fitness is good** (< 16): auto-trigger FC reboot to apply offsets
3. **If too fast** (< 60s) and **fitness is poor** (>= 8): suggest redo instead of auto-reboot
4. **FC reboot** required — offsets are saved to flash but only loaded into the EKF on boot

```
SUCCESS → show fitness → (good?) → reboot FC → wait 10-15s → verify connected → DONE
                           │
                           └── (poor?) → suggest redo → user decides
```

---

## Fitness Quality Bands

These match ArduPilot's internal thresholds:

| Fitness Value | Quality | Color (hex) | Action |
|---|---|---|---|
| < 4 | Excellent | #00ff88 (green) | Accept and reboot |
| 4 – 8 | Good | #6aff88 (light green) | Accept and reboot |
| 8 – 16 | OK | #ffdd44 (yellow) | Accept, but consider redo if rushed |
| 16 – 32 | Poor | #ffaa00 (orange) | Redo recommended |
| > 32 | Very Poor | #ff4444 (red) | Redo required |

**Tips for good fitness:**
- Rotate slowly — aim for 60+ seconds
- Cover all orientations: roll fully left/right, pitch nose up/down, yaw 360°, AND diagonal/corner orientations
- Move away from metal objects, cars, rebar floors
- Turn off nearby electronics if possible

---

## Accelerometer Calibration (6-Position)

### Purpose
Calibrate the accelerometer by measuring gravity in 6 known orientations. Required for accurate attitude estimation.

### Flow Overview

```
START → FC prompts "LEVEL" → user positions → user confirms → FC samples
      → FC prompts "LEFT" → user positions → user confirms → FC samples
      → ... (6 positions total)
      → FC emits "Calibration successful" (via statustext)
```

### Get Position Descriptions

**`GET /calibration/accel/positions`**

Returns the 6 positions with human-readable instructions. Use these in your UI.

**Response:**
```json
[
  {
    "position": 1,
    "label": "LEVEL",
    "instruction": "Place the drone flat on a level surface, right-side up."
  },
  {
    "position": 2,
    "label": "LEFT SIDE",
    "instruction": "Roll the drone onto its left side."
  },
  {
    "position": 3,
    "label": "RIGHT SIDE",
    "instruction": "Roll the drone onto its right side."
  },
  {
    "position": 4,
    "label": "NOSE DOWN",
    "instruction": "Tip the drone nose-down (pointing at the floor)."
  },
  {
    "position": 5,
    "label": "NOSE UP",
    "instruction": "Tip the drone nose-up (pointing at the ceiling)."
  },
  {
    "position": 6,
    "label": "ON BACK",
    "instruction": "Flip the drone upside-down, on its back."
  }
]
```

### Start Full 6-Position Cal

**`POST /calibration/accel/start`**

**Request:** no body

**Response:**
```json
{ "success": true, "message": "Accel cal started — place drone LEVEL", "result_code": 0 }
```

After starting:
1. Show position 1 ("LEVEL") instruction to user
2. User physically places drone in that orientation
3. User taps "Confirm Position" in the app
4. App calls `POST /calibration/accel/position` with `{ "position": 1 }`
5. FC captures samples for that orientation
6. Advance to next position (2, 3, 4, 5, 6)
7. After position 6, watch `/telemetry/messages` for "Calibration successful" or "Calibration failed"

### Confirm Position

**`POST /calibration/accel/position`**

**Request:**
```json
{ "position": 1 }
```

| Field | Type | Range | Description |
|---|---|---|---|
| `position` | int | 1–6 | Current position number to acknowledge |

**Response:**
```json
{ "success": true, "message": "Position 1 (LEVEL) captured", "result_code": 0 }
```

**Important:** Call positions in order (1 → 2 → 3 → 4 → 5 → 6). The FC expects the sequence. If you skip or repeat, the FC may reject.

### Simple Accel Cal (Level Only)

**`POST /calibration/accel/simple`**

Quick single-position calibration — only calibrates for level orientation. Use when:
- The drone is roughly flat
- You just need to re-zero the horizon after a mounting change
- You don't have time for the full 6-position flow

**Request:** no body

**Response:**
```json
{ "success": true, "message": "Simple accel cal sent", "result_code": 0 }
```

### Monitoring Accel Cal Progress

The FC communicates accel cal status through `statustext` messages. Poll `/telemetry/messages` and pattern-match:

| Pattern | Meaning |
|---|---|
| `/Place vehicle LEVEL/i` | FC ready for position 1 |
| `/on its left/i` | FC ready for position 2 |
| `/on its right/i` | FC ready for position 3 |
| `/nose down/i` | FC ready for position 4 |
| `/nose up/i` | FC ready for position 5 |
| `/on its back/i` | FC ready for position 6 |
| `/[Cc]alibration success/i` | Calibration complete |
| `/[Cc]alibration fail/i` | Calibration failed |

---

## FC Reboot

**`POST /calibration/reboot_fc`**

Reboots the flight controller autopilot. Required after:
- Compass calibration (to load new COMPASS_OFS_* values)
- Some parameter changes that require reboot

**Request:** no body

**Response:**
```json
{ "success": true, "message": "FC reboot sent", "result_code": 0 }
```

### What Happens After Reboot

1. FC disconnects immediately
2. `/telemetry/state` → `connected: false` within 1-2 seconds
3. MAVROS detects disconnection, tears down services
4. FC boots (~8-10 seconds)
5. MAVROS reconnects, recreates services with new GIDs
6. Our auto-recovery rebuilds all cached service clients (~12s post-reboot)
7. `/telemetry/state` → `connected: true` (total: ~10-15 seconds)

**Mobile app flow:**
```
POST /calibration/reboot_fc
  → show "Rebooting..." with spinner
  → poll /telemetry/state every 1s
  → when connected: false → show "FC disconnected, waiting..."
  → when connected: true  → show "FC online ✓"
  → wait 2s extra for services to stabilize
  → calibration complete
```

---

## MAVROS Reconnect (Manual)

**`POST /system/reconnect_mavros`**

Manually rebuild all MAVROS service clients. Use if calibration calls return "Command service unavailable" after a reboot wasn't auto-detected.

**Response:**
```json
{ "success": true, "message": "MAVROS service clients rebuilt" }
```

---

## State Machine Reference

### Overall Calibration UI States

```
DISCONNECTED ──▶ CONNECTED_ARMED ──▶ CONNECTED_DISARMED ──▶ CALIBRATING
     │                                        │                    │
     │                                        │                    ├── SUCCESS
     │                                        │                    ├── FAILED
     │                                        │                    └── CANCELLED
     │                                        │
     └── poll /telemetry/state                └── Show calibration buttons
```

### Per-Calibration State Flows

| Calibration | States | Duration | Notes |
|---|---|---|---|
| Motor test | `idle → running → complete` | 0.1–10s | Fire-and-forget, monitor visually |
| Gyro | `idle → calibrating → done` | ~5s | Single POST, wait for response |
| Level | `idle → calibrating → done` | ~2s | Single POST |
| Baro | `idle → calibrating → done` | ~1s | Single POST |
| Compass | `idle → running → success/failed` | 30–300s | Poll-based, complex state machine |
| Accel (full) | `idle → pos1 → pos2 → ... → pos6 → done` | ~60s | 6 steps, user-paced |
| Accel (simple) | `idle → calibrating → done` | ~2s | Single POST |

---

## Mobile App Implementation Guide

### Recommended UI Layout

```
┌──────────────────────────────────────┐
│  ⚡ Calibration                      │
│  Status: Connected · Disarmed ✓      │
├──────────────────────────────────────┤
│                                      │
│  ┌─ Quick Calibrations ───────────┐  │
│  │ [Gyro]  [Level]  [Baro]       │  │
│  │  One-tap actions               │  │
│  └────────────────────────────────┘  │
│                                      │
│  ┌─ Compass Calibration ──────────┐  │
│  │ [Start] [Cancel]              │  │
│  │ ┌──────────────────────┐      │  │
│  │ │   3D Coverage Sphere  │      │  │
│  │ │   (or 2D projection)  │      │  │
│  │ └──────────────────────┘      │  │
│  │ Progress: ████████░░ 72%      │  │
│  │ Fitness: — (shown on success) │  │
│  │ Elapsed: 45s / min 60s        │  │
│  └────────────────────────────────┘  │
│                                      │
│  ┌─ Motor Test ───────────────────┐  │
│  │ Motor: [1][2][3][4]  Thr: 10% │  │
│  │ [Start]  [STOP]               │  │
│  │ ⚠️ Remove propellers first!    │  │
│  └────────────────────────────────┘  │
│                                      │
│  ┌─ Accelerometer ────────────────┐  │
│  │ [Start 6-Position] [Simple]   │  │
│  │ Position: ● ○ ○ ○ ○ ○         │  │
│  │ Current: "LEVEL"               │  │
│  │ Instruction: Place flat...     │  │
│  │ [Confirm Position]            │  │
│  └────────────────────────────────┘  │
│                                      │
│  ┌─ System ───────────────────────┐  │
│  │ [Reboot FC]  [Reconnect]      │  │
│  └────────────────────────────────┘  │
│                                      │
└──────────────────────────────────────┘
```

### Polling Intervals

| Endpoint | When to poll | Interval |
|---|---|---|
| `GET /calibration/status` | On screen open | Once |
| `GET /telemetry/state` | Always while on cal screen | 1s |
| `GET /calibration/compass/progress` | During compass cal | 500ms |
| `GET /telemetry/messages` | During accel cal | 1s |

### Error Recovery

| Scenario | Detection | Recovery |
|---|---|---|
| FC disconnects mid-cal | `state.connected == false` | Show "FC lost" banner, auto-cancel any active cal |
| Service calls return null | response is null or 502 | Call `POST /system/reconnect_mavros`, retry |
| Compass stalls | progress unchanged 30s | Auto-cancel, show "rotate more axes" |
| Accel stuck | no statustext for 30s | Suggest restarting cal |
| Reboot takes too long | > 30s without reconnect | Show "Check USB cable" message |

### Important Implementation Notes

1. **All calibration endpoints are synchronous** — the HTTP response waits for the FC's MAV_RESULT before returning. Set HTTP timeouts to 30s for calibration calls.

2. **Compass progress is eventually consistent** — there's a ~200ms lag between the FC sending MAG_CAL_PROGRESS and it appearing in `GET /calibration/compass/progress` (MAVLink → MAVROS → ROS topic → our decoder → HTTP response).

3. **The `direction` vector in compass progress is body-frame gravity**, NOT magnetic heading. It tells you which way "down" is relative to the drone body — that's what determines orientation coverage.

4. **After FC reboot, wait for `connected: true` PLUS 2-3 seconds** before making service calls. The service GID rebuild happens asynchronously after the state change.

5. **Motor test does not have a progress endpoint** — the FC just spins the motor for the requested duration. The app should show a timer countdown locally.

6. **Multiple compasses:** The CubeOrange+ has 3 internal compasses. The cal runs all simultaneously but the progress endpoint shows the primary (compass_id=0). The report will show whichever compass the FC reports last.
