# NovaROS API Reference

> **Version:** 1.0  
> **Base URL (drone-control):** `http://<drone-ip>:8080`  
> **Base URL (vision-detect):** `http://<drone-ip>:8081`  
> **Base URL (web-control proxy):** `http://<drone-ip>:5000`  
>
> The web-control Flask server on `:5000` proxies all requests:
> - `/api/<path>` → `drone-control:8080/<path>`
> - `/vision/<path>` → `vision-detect:8081/<path>`
>
> **Mobile app should connect directly to `:8080` and `:8081`** for lower latency (skip the Flask proxy). The proxy exists only for the browser UI.

---

## Table of Contents

1. [Architecture](#architecture)
2. [Common Response Models](#common-response-models)
3. [Section 1: Control](#1-control)
4. [Section 2: Telemetry](#2-telemetry)
5. [Section 3: Camera & Streaming](#3-camera--streaming)
6. [Section 4: Mission](#4-mission)
7. [Section 5: Precision Landing](#5-precision-landing)
8. [Section 6: Vision / Object Tracking](#6-vision--object-tracking)
9. [Section 7: FC Parameters (Settings)](#7-fc-parameters-settings)
10. [Section 8: Calibration](#8-calibration)
11. [Section 9: System](#9-system)
12. [Error Handling](#error-handling)
13. [Versioning & Future APIs](#versioning--future-apis)

---

## Architecture

```
┌──────────────┐       ┌──────────────────┐       ┌──────────────────┐
│  Mobile App  │──────▶│  drone-control   │──────▶│  Flight Controller│
│  (or Web UI) │       │  FastAPI :8080   │       │  (CubeOrange+)   │
│              │──────▶│                  │       └──────────────────┘
│              │       │  MAVROS + ROS2   │
│              │       │  Virtual TX      │
│              │       │  Camera capture  │
│              │       │  Precision land  │
│              │       └──────────────────┘
│              │
│              │──────▶┌──────────────────┐
│              │       │  vision-detect   │
│              │       │  HTTP :8081      │
│              │       │  YOLO (Hailo-8)  │
│              │       │  Object tracker  │
│              │       └──────────────────┘
└──────────────┘
```

**Three independent services:**
| Service | Port | Role |
|---|---|---|
| `drone-control` | 8080 | Flight control, telemetry, camera, calibration, FC params |
| `vision-detect` | 8081 | Object detection + tracking (Hailo-8 NPU) |
| `web-control` | 5000 | Browser UI + proxy (mobile app does NOT need this) |

---

## Common Response Models

Most endpoints return one of these shapes:

### ControlResponse
```json
{ "success": true, "message": "Human-readable status" }
```

### CalibrationResponse
```json
{ "success": true, "message": "...", "result_code": 0 }
```
`result_code` is the MAVLink MAV_RESULT enum (0 = accepted, 4 = denied, etc.). Present on calibration/motor endpoints.

### Error (HTTP 4xx/5xx)
```json
{ "detail": "Error description" }
```

---

## 1. Control

All control endpoints are on **`:8080`**.

### 1.1 Arm

**`POST /arm`**

Arm the flight controller. Fails if already armed or not connected.

**Request:** no body

**Response:**
```json
{ "success": true, "message": "Armed" }
```

**Possible failures:**
- `"Already armed"` (still returns `success: true`)
- `"Not connected to FCU"`
- `"Command rejected by FCU"` (arming checks failed)

---

### 1.2 Disarm

**`POST /disarm`**

Normal disarm. FC may reject if motors are running.

**Request:** no body

**Response:**
```json
{ "success": true, "message": "Disarmed" }
```

---

### 1.3 Force Disarm

**`POST /disarm/force`**

Emergency disarm — bypasses FC safety checks. **Use with extreme caution.** Motors will cut immediately.

**Request:** no body

**Response:**
```json
{ "success": true, "message": "Force disarmed" }
```

---

### 1.4 Set Flight Mode

**`POST /mode`**

**Request:**
```json
{
  "mode": "alt_hold",
  "platform": "ardupilot"
}
```

| Field | Type | Default | Description |
|---|---|---|---|
| `mode` | string | required | One of the mode keys below |
| `platform` | string | `"ardupilot"` | `"ardupilot"` or `"px4"` |

**Available modes:**

| Key | ArduPilot | PX4 | Notes |
|---|---|---|---|
| `stabilize` | STABILIZE | STABILIZED | Manual, no altitude hold |
| `alt_hold` | ALT_HOLD | ALTCTL | Recommended for remote control |
| `loiter` | LOITER | POSCTL | GPS position hold |
| `land` | LAND | AUTO.LAND | Autonomous landing |
| `rtl` | RTL | AUTO.RTL | Return to launch |
| `guided` | GUIDED | OFFBOARD | Requires GPS |
| `auto` | AUTO | AUTO.MISSION | Execute uploaded mission |
| `brake` | BRAKE | AUTO.LOITER | Immediate stop (GPS required) |

**Response:**
```json
{ "success": true, "message": "Mode set to ALT_HOLD", "mode_sent": "ALT_HOLD" }
```

---

### 1.5 List Available Modes

**`GET /modes`**

**Response:**
```json
{ "modes": ["stabilize", "loiter", "alt_hold", ...], "platforms": ["ardupilot", "px4"] }
```

---

### 1.6 Enable Virtual TX (Remote Control)

**`POST /control/enable`**

Enables the virtual transmitter. Must be enabled before sending control commands. The virtual TX publishes RC override messages to the FC at a fixed rate.

**Request:** no body

**Response:**
```json
{ "success": true, "message": "Enabled" }
```

---

### 1.7 Disable Virtual TX

**`POST /control/disable`**

Releases all RC channels. Drone behavior depends on mode:
- ALT_HOLD: holds altitude, drifts with wind
- STABILIZE: motors idle, will descend
- LOITER: holds position (GPS)

**Request:** no body

**Response:**
```json
{ "success": true, "message": "Disabled" }
```

---

### 1.8 Send Control Command

**`POST /control/command`**

Send stick inputs to the drone. Must call `/control/enable` first.

**Request:**
```json
{
  "timestamp": 1713600000.0,
  "throttle": 0.5,
  "roll": 0.0,
  "pitch": 0.0,
  "yaw": 0.0,
  "duration": 1.0
}
```

| Field | Type | Range | Description |
|---|---|---|---|
| `timestamp` | float | epoch seconds | Server uses this to reject stale commands. **Tip:** use server time from telemetry, or let the web proxy inject it. |
| `throttle` | float | 0.0 – 1.0 | 0 = min, 0.5 ≈ hover (ALT_HOLD), 1.0 = max climb |
| `pitch` | float | -1.0 – 1.0 | -1 = full forward, +1 = full backward |
| `roll` | float | -1.0 – 1.0 | -1 = full left, +1 = full right |
| `yaw` | float | -1.0 – 1.0 | -1 = rotate left, +1 = rotate right |
| `duration` | float | seconds | How long to hold this command before centering. Default 1.0 |

**Response:**
```json
{ "success": true, "message": "Command accepted" }
```

**Important notes:**
- Commands must be sent continuously (at least every `command_timeout` seconds, default 2s) or the virtual TX enters failsafe (centers sticks / idles throttle depending on mode).
- Throttle at 0.5 in ALT_HOLD = hover. Below 0.5 = descend, above = climb.
- In STABILIZE mode, throttle directly controls motor power — 0.0 = motors off.

---

### 1.9 Get Virtual TX Status

**`GET /control/status`**

**Response:**
```json
{
  "enabled": true,
  "in_failsafe": false,
  "centering": false,
  "hold_remaining": 0.85,
  "last_command_age": 0.12,
  "outputs": {
    "roll": 1500,
    "pitch": 1500,
    "yaw": 1500,
    "throttle": 1500
  },
  "config": {
    "publish_rate": 50.0,
    "command_timeout": 2.0,
    "failsafe_timeout": 5.0
  }
}
```

| Field | Description |
|---|---|
| `enabled` | Whether virtual TX is active |
| `in_failsafe` | True if no commands received within timeout |
| `centering` | True if sticks are returning to center |
| `hold_remaining` | Seconds left on current command hold |
| `last_command_age` | Seconds since last command received |
| `outputs` | Current PWM values being sent to FC (1000–2000, 1500=center) |

---

### 1.10 Get Virtual TX Config

**`GET /control/config`**

Returns all tunable virtual TX parameters.

**Response:**
```json
{
  "command_timeout": 2.0,
  "failsafe_timeout": 5.0,
  "publish_rate": 50.0,
  "center_rate": 5.0,
  "roll_expo": 0.0,
  "pitch_expo": 0.0,
  "yaw_expo": 0.0,
  "throttle_expo": 0.0,
  "roll_smoothing": 0.0,
  "pitch_smoothing": 0.0,
  "yaw_smoothing": 0.0,
  "throttle_smoothing": 0.0,
  "roll_deadzone": 0.0,
  "pitch_deadzone": 0.0,
  "yaw_deadzone": 0.0,
  "throttle_deadzone": 0.0,
  "roll_rate_limit": 0.0,
  "pitch_rate_limit": 0.0,
  "yaw_rate_limit": 0.0,
  "throttle_rate_limit": 0.0,
  "throttle_pwm_idle": 1000,
  "throttle_pwm_max": 2000
}
```

---

### 1.11 Update Virtual TX Config

**`POST /control/config`**

Update one or more virtual TX parameters. Only include fields you want to change.

**Request:**
```json
{
  "command_timeout": 3.0,
  "roll_expo": 0.3
}
```

**Response:**
```json
{ "success": true, "message": "Config updated" }
```

---

## 2. Telemetry

All on **`:8080`**. Data updates at ~10 Hz from the flight controller.

### 2.1 Get All Telemetry

**`GET /telemetry`**

Returns the complete telemetry snapshot — all categories in one call. Best for initial load or infrequent polling.

**Response:**
```json
{
  "state": {
    "connected": true,
    "armed": false,
    "mode": "STABILIZE",
    "system_status": 3
  },
  "extended_state": {
    "landed_state": 1,
    "vtol_state": 0
  },
  "battery": {
    "voltage": 16.2,
    "current": 1.5,
    "percentage": 0.85
  },
  "gps": {
    "fix_type": 3,
    "latitude": 17.385,
    "longitude": 78.486,
    "altitude": 500.0,
    "satellites": 12,
    "hdop": 1.2
  },
  "local_position": { "x": 0.0, "y": 0.0, "z": 0.0 },
  "orientation": { "roll": 0.5, "pitch": -0.3, "yaw": 45.0 },
  "velocity": { "vx": 0.0, "vy": 0.0, "vz": 0.0 },
  "imu": { "ax": 0.0, "ay": 0.0, "az": -9.81, "gx": 0.0, "gy": 0.0, "gz": 0.0 },
  "mag": { "x": 0.0, "y": 0.0, "z": 0.0 },
  "baro": { "pressure": 101325.0, "temperature": 25.0 },
  "vfr_hud": {
    "airspeed": 0.0,
    "groundspeed": 0.0,
    "heading": 45,
    "throttle": 0.0,
    "altitude": 0.0,
    "climb_rate": 0.0
  },
  "rc_in": { "channels": [1500, 1500, 1100, 1500, 1000, 1000, 1000, 1000] },
  "rc_out": { "channels": [1000, 1000, 1000, 1000] },
  "home": { "latitude": 17.385, "longitude": 78.486, "altitude": 500.0 },
  "relative_alt": 0.0,
  "heading": 45.0,
  "status_text": "PreArm: RC not calibrated",
  "messages": [
    "PreArm: RC not calibrated",
    "EKF3 IMU0 using GPS"
  ]
}
```

### 2.2 Individual Telemetry Categories

Each returns just that section of the telemetry object.

| Endpoint | Returns |
|---|---|
| `GET /telemetry/state` | `{ connected, armed, mode, system_status }` |
| `GET /telemetry/extended_state` | `{ landed_state, vtol_state }` |
| `GET /telemetry/battery` | `{ voltage, current, percentage }` |
| `GET /telemetry/gps` | `{ fix_type, latitude, longitude, altitude, satellites, hdop }` |
| `GET /telemetry/local_position` | `{ x, y, z }` |
| `GET /telemetry/orientation` | `{ roll, pitch, yaw }` (degrees) |
| `GET /telemetry/velocity` | `{ vx, vy, vz }` (m/s) |
| `GET /telemetry/imu` | `{ ax, ay, az, gx, gy, gz }` (accel m/s², gyro rad/s) |
| `GET /telemetry/mag` | `{ x, y, z }` (Tesla) |
| `GET /telemetry/baro` | `{ pressure, temperature }` (Pa, °C) |
| `GET /telemetry/vfr_hud` | `{ airspeed, groundspeed, heading, throttle, altitude, climb_rate }` |
| `GET /telemetry/rc_in` | `{ channels: [int, ...] }` (raw RC input PWM) |
| `GET /telemetry/rc_out` | `{ channels: [int, ...] }` (motor output PWM) |
| `GET /telemetry/home` | `{ latitude, longitude, altitude }` |
| `GET /telemetry/messages` | `["status text 1", "status text 2", ...]` (last 50 FCU messages) |

**Recommended polling strategy for mobile app:**
- `/telemetry/state` — poll every 500ms (arm/mode state)
- `/telemetry/battery` — poll every 2s
- `/telemetry/gps` — poll every 1s
- `/telemetry/orientation` + `/telemetry/vfr_hud` — poll every 200ms for attitude display
- `/telemetry` (full) — on app open, then switch to individual endpoints

---

## 3. Camera & Streaming

All on **`:8080`**. Two physical cameras: `landing` (QR/precision land) and `tracking` (vision/YOLO).

### 3.1 List Cameras

**`GET /camera/list`**

**Response:**
```json
{
  "landing": {
    "name": "landing",
    "active": true,
    "has_frame": true,
    "frame_count": 1523,
    "fps": 29.8,
    "device": "/dev/v4l/by-id/usb-Sonix...-video-index0",
    "width": 1280,
    "height": 720,
    "target_fps": 30
  },
  "tracking": {
    "name": "tracking",
    "active": true,
    "has_frame": true,
    "frame_count": 892,
    "fps": 16.7,
    "device": "/dev/v4l/by-id/usb-Jieli...-video-index0",
    "width": 1280,
    "height": 720,
    "target_fps": 30
  }
}
```

### 3.2 Camera MJPEG Stream

**`GET /camera/{name}/stream`**

Returns `multipart/x-mixed-replace` MJPEG stream. Use in an `<img>` tag or a streaming HTTP client.

- `name`: `landing` or `tracking`
- Content-Type: `multipart/x-mixed-replace; boundary=frame`

**For mobile:** Use a streaming image library. Each frame is a complete JPEG with `--frame` boundary.

### 3.3 Camera Snapshot

**`GET /camera/{name}/snapshot`**

Returns a single JPEG frame. Lighter than streaming for infrequent updates.

- Content-Type: `image/jpeg`
- `name`: `landing` or `tracking`

**Error:** 503 if no frame available, 404 if unknown camera name.

### 3.4 Camera Status

**`GET /camera/{name}/status`**

Returns the same status object as the corresponding entry in `/camera/list`.

### 3.5 Legacy Aliases (default to `landing`)

| Endpoint | Equivalent |
|---|---|
| `GET /camera/stream` | `GET /camera/landing/stream` |
| `GET /camera/snapshot` | `GET /camera/landing/snapshot` |
| `GET /camera/status` | `GET /camera/landing/status` |

---

## 4. Mission

All on **`:8080`**. Upload and execute autonomous waypoint missions.

### 4.1 Upload & Execute Mission

**`POST /mission`**

Uploads waypoints in QGC WPL 110 format, sets AUTO mode, and optionally arms.

**Request:**
```json
{
  "wpl": "QGC WPL 110\n0\t1\t0\t16\t0\t0\t0\t0\t17.385\t78.486\t100.0\t1\n...",
  "auto_arm": true
}
```

| Field | Type | Default | Description |
|---|---|---|---|
| `wpl` | string | required | Full QGC WPL 110 waypoint file content |
| `auto_arm` | bool | `true` | Arm after setting AUTO mode |

**Response:**
```json
{ "success": true, "message": "Mission started with 5 waypoints", "waypoints_uploaded": 5 }
```

**Failure cascade** — the endpoint rolls through: clear → push → set AUTO → arm. If any step fails, it returns partial success info:
```json
{ "success": false, "message": "Mission uploaded, AUTO set, but arm failed: ...", "waypoints_uploaded": 5 }
```

### 4.2 Clear Mission

**`DELETE /mission`**

Clears all waypoints from the flight controller.

**Response:**
```json
{ "success": true, "message": "Mission cleared", "waypoints_uploaded": 0 }
```

---

## 5. Precision Landing

All on **`:8080`**. GPS-free QR-code-based autonomous landing using the landing camera.

### 5.1 Start Precision Landing

**`POST /land/precision`**

Switches to ALT_HOLD mode, enables virtual TX, begins searching for QR code. The drone will:
1. Search for QR code in landing camera
2. Center itself over the QR using PID control
3. Descend when centered
4. Switch to LAND mode when close enough

**Prerequisites:** drone must be connected and armed.

**Request:** no body

**Response:**
```json
{ "success": true, "message": "Precision landing started — searching for QR" }
```

### 5.2 Abort Landing

**`POST /land/abort`**

Aborts precision landing. Drone hovers in place.

**Response:**
```json
{ "success": true, "message": "Aborted — hovering" }
```

### 5.3 Landing Status

**`GET /land/status`**

**Response:**
```json
{
  "state": "descending",
  "qr_detected": true,
  "qr": {
    "ex": 0.05,
    "ey": -0.02,
    "raw_ex": 0.06,
    "raw_ey": -0.03,
    "ratio": 0.15,
    "box": [320, 200, 80, 80]
  }
}
```

| Field | Description |
|---|---|
| `state` | One of: `idle`, `searching`, `descending`, `complete`, `aborted` |
| `qr_detected` | Whether QR is currently visible |
| `qr.ex`, `qr.ey` | Filtered position error, -1.0 to 1.0 (0 = centered) |
| `qr.raw_ex`, `qr.raw_ey` | Unfiltered position error |
| `qr.ratio` | QR width / frame width (larger = closer to ground) |
| `qr.box` | `[x, y, w, h]` pixel bounding box of QR in frame |

### 5.4 Get Landing Config

**`GET /land/config`**

**Response:**
```json
{
  "KP": { "value": 0.35, "default": 0.35 },
  "KI": { "value": 0.08, "default": 0.08 },
  "KD": { "value": 0.15, "default": 0.15 },
  "I_LIMIT": { "value": 0.3, "default": 0.3 },
  "EMA_ALPHA": { "value": 0.4, "default": 0.4 },
  "CENTER_THRESHOLD": { "value": 0.08, "default": 0.08 },
  "LAND_BOX_RATIO": { "value": 0.4, "default": 0.4 },
  "SEARCH_TIMEOUT": { "value": 10.0, "default": 10.0 },
  "DESCENT_STEP": { "value": 0.06, "default": 0.06 },
  "MIN_DESCENT_STEP": { "value": 0.02, "default": 0.02 }
}
```

### 5.5 Update Landing Config

**`POST /land/config`**

**Request:** (only include fields you want to change)
```json
{ "KP": 0.4, "SEARCH_TIMEOUT": 15.0 }
```

**Response:**
```json
{ "success": true, "message": "Config updated" }
```

---

## 6. Vision / Object Tracking

All on **`:8081`** (vision-detect container). YOLOv8s on Hailo-8 NPU + KCF tracker.

### 6.1 Get Annotated Frame

**`GET /frame`**

Returns the latest JPEG frame with HUD overlays (bounding boxes, reticle, FPS, labels).

- Content-Type: `image/jpeg`
- 503 if no frame available

**For mobile:** Poll this at 5–10 fps for a live view. Full speed is ~53 fps but the mobile app doesn't need that.

### 6.2 Get Tracker State

**`GET /state`**

**Response:**
```json
{
  "dets": [
    { "cls": "person", "conf": 0.87, "box": [120, 50, 200, 400] },
    { "cls": "car", "conf": 0.72, "box": [400, 200, 150, 100] }
  ],
  "tracking": true,
  "lost": false,
  "locked_box": [120, 50, 200, 400],
  "seq": 4523
}
```

| Field | Type | Description |
|---|---|---|
| `dets` | array | All YOLO detections in current frame. Each: `{ cls, conf, box: [x,y,w,h] }` |
| `tracking` | bool | Whether a target is currently locked |
| `lost` | bool | Target was locked but is currently not visible |
| `locked_box` | array\|null | `[x, y, w, h]` of the locked target, or null |
| `seq` | int | Frame sequence number (monotonically increasing) |

### 6.3 Lock / Unlock Target

**`POST /lock`**

Lock onto a target by class name (from YOLO detection) or by bounding box (drag-select in UI).

**Lock by bounding box (drag-select):**
```json
{ "box": [120, 50, 200, 400] }
```
`box` is `[x, y, w, h]` in pixel coordinates. Uses KCF correlation tracker.

**Lock by class (click detection in panel):**
```json
{ "box": [120, 50, 200, 400], "cls": "person" }
```
When `cls` is set, YOLO handles frame-to-frame tracking (no KCF).

**Unlock:**
```json
{ "unlock": true }
```

**Response:**
```json
{ "ok": true }
```

### 6.4 COCO Classes

The detector recognizes all 80 COCO classes. Common ones for the police/safety drone use case:
`person`, `bicycle`, `car`, `motorcycle`, `bus`, `truck`, `boat`, `dog`, `backpack`, `handbag`, `suitcase`, `knife`, `cell phone`

Full list: person, bicycle, car, motorcycle, airplane, bus, train, truck, boat, traffic light, fire hydrant, stop sign, parking meter, bench, bird, cat, dog, horse, sheep, cow, elephant, bear, zebra, giraffe, backpack, umbrella, handbag, tie, suitcase, frisbee, skis, snowboard, sports ball, kite, baseball bat, baseball glove, skateboard, surfboard, tennis racket, bottle, wine glass, cup, fork, knife, spoon, bowl, banana, apple, sandwich, orange, broccoli, carrot, hot dog, pizza, donut, cake, chair, couch, potted plant, bed, dining table, toilet, tv, laptop, mouse, remote, keyboard, cell phone, microwave, oven, toaster, sink, refrigerator, book, clock, vase, scissors, teddy bear, hair drier, toothbrush.

---

## 7. FC Parameters (Settings)

All on **`:8080`**. Read/write ArduPilot flight controller parameters (persisted on the FC).

### 7.1 Get Single Parameter

**`GET /params/{param_id}`**

**Example:** `GET /params/MOT_THST_EXPO`

**Response:**
```json
{ "param_id": "MOT_THST_EXPO", "value": 0.65 }
```

**Error:** 404 if parameter doesn't exist.

### 7.2 Set Single Parameter

**`POST /params/{param_id}`**

**Request:**
```json
{ "value": 0.7 }
```

**Response:**
```json
{ "param_id": "MOT_THST_EXPO", "value": 0.7, "success": true }
```

**Error:** 500 if FC rejects the value.

### 7.3 Batch Get Parameters

**`POST /params/batch/get`**

Read multiple parameters in one call. Saves round trips for settings screens.

**Request:**
```json
{ "params": ["MOT_THST_EXPO", "MOT_SPIN_MIN", "MOT_SPIN_ARM", "MOT_BAT_VOLT_MAX"] }
```

**Response:**
```json
{
  "MOT_THST_EXPO": 0.65,
  "MOT_SPIN_MIN": 0.15,
  "MOT_SPIN_ARM": 0.1,
  "MOT_BAT_VOLT_MAX": 16.8
}
```

Values are `null` if the parameter wasn't found.

**Common parameter groups:**

| Category | Parameters |
|---|---|
| Motor tuning | `MOT_THST_EXPO`, `MOT_SPIN_MIN`, `MOT_SPIN_ARM`, `MOT_BAT_VOLT_MAX`, `MOT_BAT_VOLT_MIN` |
| RC trim | `RC1_TRIM`, `RC2_TRIM`, `RC3_TRIM`, `RC4_TRIM` |
| AHRS trim | `AHRS_TRIM_X`, `AHRS_TRIM_Y`, `AHRS_TRIM_Z` |
| Arming | `ARMING_CHECK`, `BRD_SAFETY_DEFLT` |
| Failsafe | `FS_THR_ENABLE`, `FS_THR_VALUE` |

---

## 8. Calibration

All on **`:8080`**. All calibration endpoints require the drone to be **disarmed**. Returns 409 if armed, 503 if not connected.

### 8.1 Calibration Status

**`GET /calibration/status`**

Check what's available before showing calibration UI.

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

---

### 8.2 Motor Test

**`POST /calibration/motor_test`**

Spin individual motors for verification. **REMOVE PROPELLERS FIRST.**

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
| `motor` | int | 1–12 | 1 | Motor number (1-based) |
| `throttle_pct` | float | 0–100 | 10.0 | Throttle percentage |
| `duration_s` | float | 0.1–10.0 | 2.0 | Duration per motor |
| `mode` | string | `"single"` / `"sequence"` | `"single"` | Single motor or sequential |
| `motor_count` | int | 1–12 | 1 | Motors to test in sequence mode |

**Response:**
```json
{ "success": true, "message": "Motor test started: single m1 @ 10% for 2.0s", "result_code": 0 }
```

### 8.3 Emergency Stop Motor Test

**`POST /calibration/motor_test/stop`**

Immediately stops any running motor test and force-disarms.

**Request:** no body

**Response:**
```json
{ "success": true, "message": "Motor test stopped" }
```

---

### 8.4 Gyro Calibration

**`POST /calibration/gyro`**

Calibrate gyroscopes. Drone must be completely still on a flat surface.

**Request:** no body  
**Duration:** ~5 seconds

**Response:**
```json
{ "success": true, "message": "Gyro calibration complete", "result_code": 0 }
```

---

### 8.5 Level Horizon

**`POST /calibration/level_horizon`**

One-click AHRS trim — sets level reference from current attitude. Place drone flat on a known-level surface.

**Request:** no body  
**Duration:** ~2 seconds

**Response:**
```json
{ "success": true, "message": "Level horizon captured", "result_code": 0 }
```

---

### 8.6 Barometer

**`POST /calibration/baro`**

Reset barometer ground pressure. Sets current altitude as zero reference.

**Request:** no body

**Response:**
```json
{ "success": true, "message": "Barometer zeroed", "result_code": 0 }
```

---

### 8.7 Compass Calibration

Multi-step process: start → rotate drone through all axes → accept/cancel.

#### Start

**`POST /calibration/compass/start`**

**Request:** (optional body)
```json
{ "autosave": true, "retry_on_fail": false }
```

| Field | Type | Default | Description |
|---|---|---|---|
| `autosave` | bool | `true` | Auto-save offsets when done |
| `retry_on_fail` | bool | `false` | Auto-retry if calibration fails |

**Response:**
```json
{ "success": true, "message": "Compass calibration started — rotate drone through all axes" }
```

#### Progress

**`GET /calibration/compass/progress`**

Poll this every ~500ms during calibration. Decoded from raw MAVLink messages.

**Response:**
```json
{
  "active": true,
  "progress": {
    "0": { "pct": 45, "cal_status": 0, "attempt": 1, "direction": 2 }
  },
  "report": null
}
```

When complete:
```json
{
  "active": false,
  "progress": { "0": { "pct": 100, ... } },
  "report": {
    "0": { "fitness": 12.5, "ofs_x": -15.2, "ofs_y": 8.7, "ofs_z": -3.1, "autosaved": true }
  }
}
```

#### Accept (if autosave was off)

**`POST /calibration/compass/accept`**

**Response:**
```json
{ "success": true, "message": "Compass cal accepted" }
```

#### Cancel

**`POST /calibration/compass/cancel`**

**Response:**
```json
{ "success": true, "message": "Compass cal cancelled" }
```

---

### 8.8 Accelerometer Calibration (6-position)

Multi-step: start → position drone in 6 orientations → confirm each.

#### Get Position Descriptions

**`GET /calibration/accel/positions`**

**Response:**
```json
[
  { "position": 1, "label": "LEVEL",     "instruction": "Place the drone flat on a level surface, right-side up." },
  { "position": 2, "label": "LEFT SIDE", "instruction": "Roll the drone onto its left side." },
  { "position": 3, "label": "RIGHT SIDE","instruction": "Roll the drone onto its right side." },
  { "position": 4, "label": "NOSE DOWN", "instruction": "Tip the drone nose-down (pointing at the floor)." },
  { "position": 5, "label": "NOSE UP",   "instruction": "Tip the drone nose-up (pointing at the ceiling)." },
  { "position": 6, "label": "ON BACK",   "instruction": "Flip the drone upside-down, on its back." }
]
```

#### Start

**`POST /calibration/accel/start`**

**Request:** no body

**Response:**
```json
{ "success": true, "message": "Accel cal started — place drone LEVEL" }
```

#### Confirm Position

**`POST /calibration/accel/position`**

Call after physically placing the drone in the requested orientation.

**Request:**
```json
{ "position": 1 }
```

**Response:**
```json
{ "success": true, "message": "Position 1 (LEVEL) captured" }
```

Repeat for positions 1 through 6. Watch `/telemetry/messages` for FC prompts and completion.

#### Simple Accel Cal (level only)

**`POST /calibration/accel/simple`**

Quick single-position calibration. Drone must be flat and level.

**Response:**
```json
{ "success": true, "message": "Simple accel cal sent" }
```

---

### 8.9 Reboot Flight Controller

**`POST /calibration/reboot_fc`**

Reboot the autopilot. Required after compass calibration to load new offsets. Service clients auto-rebuild ~12s after reboot.

**Request:** no body

**Response:**
```json
{ "success": true, "message": "FC reboot sent" }
```

**Note:** All telemetry and services will be unavailable for ~10–15 seconds after reboot. Poll `/telemetry/state` → `connected: true` to know when the FC is back.

---

## 9. System

### 9.1 Reconnect MAVROS

**`POST /system/reconnect_mavros`**

Manually rebuild all MAVROS service clients. Use if service calls return "unavailable" after an FC reboot that wasn't auto-detected.

**Request:** no body

**Response:**
```json
{ "success": true, "message": "MAVROS service clients rebuilt" }
```

---

## Error Handling

### HTTP Status Codes

| Code | Meaning |
|---|---|
| 200 | Success |
| 400 | Bad request (missing fields, invalid values) |
| 404 | Resource not found (bad param name, unknown camera) |
| 409 | Conflict (e.g., calibrating while armed) |
| 422 | Validation error (out-of-range values) |
| 500 | FC rejected the command |
| 502 | Upstream service unavailable (proxy error) |
| 503 | Not connected to FCU / no camera frame |

### Common Failure Patterns

1. **"Not connected to FCU"** — MAVROS hasn't connected. Check USB cable to CubeOrange+.
2. **"Command rejected by FCU"** — FC denied the action. Check arming checks, mode prerequisites.
3. **"Command service unavailable"** — MAVROS service dead. Try `POST /system/reconnect_mavros`.
4. **503 on camera** — Camera not plugged in or ffmpeg not running.

---

## Versioning & Future APIs

### API Namespace Convention

All new feature modules should follow this pattern:

```
/<module>/<action>          — primary action
/<module>/status            — current state
/<module>/config            — get tunable params
POST /<module>/config       — update params
```

### Planned Future Modules

| Module | Namespace | Description |
|---|---|---|
| Follower | `/follower/*` | Auto-follow locked target using vision tracker + virtual TX |
| Gas sensors | `/sensors/*` | MQ-135 + MQ-3 readings via Teensy serial |
| Geofence | `/geofence/*` | Virtual boundary enforcement |
| Flight recorder | `/recorder/*` | Onboard flight data logging |
| Alerts | `/alerts/*` | Configurable event notifications |

### Integration Notes for Mobile App

1. **Connection discovery:** The drone runs on Tailscale. The mobile app should connect to the Tailscale IP (`100.81.21.121`) or use mDNS/service discovery.

2. **Polling vs WebSocket:** Currently all APIs are request/response. For real-time telemetry on mobile, poll `/telemetry/state` at 2–5 Hz. A WebSocket telemetry stream is a planned addition.

3. **Timestamps:** The drone uses server-side timestamps for control commands. If calling `:8080` directly (not through the Flask proxy), set `timestamp` to the current epoch time from the mobile device. Clock sync isn't critical — the field is used for staleness rejection, not precision timing.

4. **Camera streaming on mobile:** Use `/camera/{name}/snapshot` polled at 5–10 fps rather than the MJPEG stream. MJPEG multipart streams can be tricky on mobile HTTP clients.

5. **Vision frame on mobile:** Poll `GET :8081/frame` at 5 fps. For state-only (no image), poll `GET :8081/state` at 10 fps.

6. **Idempotency:** All POST endpoints are safe to retry. Arm when already armed returns success. Enable when already enabled returns success.
