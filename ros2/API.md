# NovaROS Drone Control API Reference

**Base URL:** `http://<drone-ip>:8080`

All endpoints return JSON. All POST endpoints accept `Content-Type: application/json`.

---

## State & Commands

### GET /telemetry/state

Get drone connection, arm, and mode status. **Poll this first before anything else.**

```json
// Response
{
  "connected": true,
  "armed": false,
  "mode": "STABILIZE",
  "system_status": 3
}
```

| Field | Type | Description |
|-------|------|-------------|
| connected | bool | FCU connection alive |
| armed | bool | Motors armed |
| mode | string | Current flight mode |
| system_status | int | MAVLink system status (3=standby, 4=active) |

---

### POST /arm

Arm the drone. Requires STABILIZE mode and throttle at minimum.

```json
// Response
{ "success": true, "message": "Armed" }

// Failure examples
{ "success": false, "message": "Not connected to FCU" }
{ "success": false, "message": "Command rejected by FCU" }
```

---

### POST /disarm

Normal disarm. May be rejected if drone is in flight.

```json
// Response
{ "success": true, "message": "Disarmed" }
```

---

### POST /disarm/force

Force disarm. **Bypasses safety checks. Motors stop immediately. Use only in emergencies.**

```json
// Response
{ "success": true, "message": "Force disarmed" }
```

---

### POST /mode

Set flight mode.

```json
// Request
{
  "mode": "loiter",
  "platform": "ardupilot"   // optional, default "ardupilot"
}

// Response
{ "success": true, "message": "Mode set to LOITER", "mode_sent": "LOITER" }
```

**Available modes:** `stabilize`, `loiter`, `alt_hold`, `land`, `rtl`, `guided`, `auto`, `brake`

**Recommended for manual flying:** `alt_hold` (FC handles altitude, pilot handles direction). `loiter` adds GPS position hold but requires GPS fix. `stabilize` gives direct motor power control — harder to fly but no sensor dependencies.

---

### GET /modes

List available modes.

```json
// Response
{
  "modes": ["stabilize", "loiter", "alt_hold", "land", "rtl", "guided", "auto", "brake"],
  "platforms": ["ardupilot", "px4"]
}
```

---

## Manual Control (Virtual TX)

The virtual TX is designed for internet-based control. Commands include a **duration** — how long the command should be held even if no new command arrives. This handles network latency and drops gracefully.

### Typical flow:

```
1. POST /control/enable       → Enable virtual transmitter
2. POST /arm                   → Arm drone
3. POST /control/command       → Send stick inputs (repeat as needed)
4. POST /control/disable       → Release control
5. POST /disarm/force          → Disarm
```

---

### POST /control/enable

Enable virtual transmitter. Must be called before sending commands. Requires FCU connection.

```json
// Response
{ "success": true, "message": "Enabled" }
```

---

### POST /control/disable

Disable virtual transmitter and release all RC channels back to hardware.

```json
// Response
{ "success": true, "message": "Disabled" }
```

---

### POST /control/command

**Send a control command.** This is the main control endpoint.

```json
// Request
{
  "timestamp": 1700000000.123,    // Date.now() / 1000 in JavaScript
  "throttle": 0.5,                // 0.0 (idle) to 1.0 (full)
  "roll": 0.0,                    // -1.0 (left) to 1.0 (right)
  "pitch": 0.0,                   // -1.0 (back) to 1.0 (forward)
  "yaw": 0.0,                     // -1.0 (left) to 1.0 (right)
  "duration": 1.0                 // optional, default 1.0 seconds
}
```

```json
// Response - new command accepted
{ "success": true, "message": "OK" }

// Response - same values as current (dedup), hold extended
{ "success": true, "message": "Hold" }

// Failure responses
{ "success": false, "message": "Virtual TX not enabled" }
{ "success": false, "message": "Command too old: 2.50s > 1.5s" }
{ "success": false, "message": "Command from future: -3.00s" }
{ "success": false, "message": "Out of order: newer command already accepted" }
{ "success": false, "message": "Invalid duration: 0.0s (must be > 0)" }
```

**Field details:**

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| timestamp | float | now-1.5s to now+1s | Unix timestamp when command was created. Use `Date.now() / 1000` in JS. Commands older than 1.5s are rejected. |
| throttle | float | 0.0 to 1.0 | 0.0 = motors idle, 1.0 = full throttle. Values outside range are clamped. |
| roll | float | -1.0 to 1.0 | -1.0 = full left, 1.0 = full right. 0.0 = center. |
| pitch | float | -1.0 to 1.0 | -1.0 = full back, 1.0 = full forward. 0.0 = center. |
| yaw | float | -1.0 to 1.0 | -1.0 = rotate left, 1.0 = rotate right. 0.0 = center. |
| duration | float | > 0 | How long to hold this command in seconds. Default 1.0. For stable flight over internet, use 1.0-2.0. |

**Behavior notes:**
- The command holds for `duration` seconds even if no new command arrives
- When hold expires with no new command, all channels smoothly auto-center (failsafe)
- **Failsafe is mode-aware:** In ALT_HOLD/LOITER, throttle centers to mid-stick (hover in place). In STABILIZE, throttle goes to idle.
- If you send the same values again, it returns `"Hold"` and extends the timer (no jerk)
- If an older command arrives after a newer one, it is rejected (out-of-order protection)
- All transitions are smoothed — no sudden jumps even on large input changes
- Rate limiters cap how fast values can change (~400ms for full reversal)

**Throttle meaning by mode:**
- **STABILIZE:** 0.0 = idle, 1.0 = full power (direct motor control)
- **ALT_HOLD/LOITER:** ~0.44 (mid-stick) = hold altitude, below = descend, above = climb

---

### GET /control/status

Get virtual TX state, remaining hold time, current outputs.

```json
// Response
{
  "enabled": true,
  "in_failsafe": false,
  "centering": false,
  "hold_remaining": 0.73,
  "last_command_age": 0.27,
  "outputs": {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "throttle": 0.5
  },
  "pwm": {
    "roll": 1500,
    "pitch": 1500,
    "yaw": 1500,
    "throttle": 1540
  }
}
```

| Field | Description |
|-------|-------------|
| enabled | Whether virtual TX is active |
| in_failsafe | True when hold expired and no new command |
| centering | True when auto-centering channels |
| hold_remaining | Seconds until current command expires (0.0 if expired) |
| last_command_age | Seconds since last accepted command |
| outputs | Normalized output values after processing pipeline |
| pwm | Actual PWM values being sent to flight controller |

---

### GET /control/config

Get all tunable parameters.

```json
// Response
{
  "command_timeout": 1.5,
  "failsafe_timeout": 0.5,
  "publish_rate": 50.0,
  "center_rate": 2.0,
  "roll_expo": 0.3,
  "pitch_expo": 0.3,
  "yaw_expo": 0.2,
  "throttle_expo": 0.0,
  "roll_smoothing": 0.7,
  "pitch_smoothing": 0.7,
  "yaw_smoothing": 0.5,
  "throttle_smoothing": 0.8,
  "roll_deadzone": 0.05,
  "pitch_deadzone": 0.05,
  "yaw_deadzone": 0.05,
  "throttle_deadzone": 0.02,
  "roll_rate_limit": 3.0,
  "pitch_rate_limit": 3.0,
  "yaw_rate_limit": 2.0,
  "throttle_rate_limit": 1.5,
  "throttle_pwm_idle": 1100,
  "throttle_pwm_max": 2000
}
```

---

### POST /control/config

Update tunable parameters. Only send the fields you want to change.

```json
// Request - only include fields to update
{
  "roll_expo": 0.5,
  "throttle_rate_limit": 2.0
}

// Response
{ "success": true, "message": "Config updated" }
```

---

## Missions

### POST /mission

Upload a mission in QGC WPL 110 format, set AUTO mode, and optionally arm.

```json
// Request
{
  "wpl": "QGC WPL 110\n0\t1\t0\t16\t0\t0\t0\t0\t28.6139\t77.2090\t10\t1\n1\t0\t3\t16\t0\t0\t0\t0\t28.6150\t77.2100\t15\t1",
  "auto_arm": true
}

// Response
{
  "success": true,
  "message": "Mission started with 2 waypoints",
  "waypoints_uploaded": 2
}
```

---

### DELETE /mission

Clear current mission from flight controller.

```json
// Response
{ "success": true, "message": "Mission cleared" }
```

---

## Telemetry

### GET /telemetry

Get all telemetry data in one call.

```json
// Response
{
  "state": { "connected": true, "armed": false, "mode": "STABILIZE", "system_status": 3 },
  "extended_state": { "landed_state": 0, "vtol_state": 0 },
  "battery": { "voltage": 16.2, "current": 1.5, "percentage": 0.85 },
  "gps": { "fix_type": 3, "latitude": 28.6139, "longitude": 77.2090, "altitude": 25.0, "satellites": 12, "hdop": 0.9 },
  "local_position": { "x": 0.0, "y": 0.0, "z": -2.5 },
  "orientation": { "roll": 0.01, "pitch": -0.02, "yaw": 1.57 },
  "velocity": { "vx": 0.0, "vy": 0.0, "vz": 0.0 },
  "imu": { "ax": 0.1, "ay": 0.0, "az": 9.8, "gx": 0.0, "gy": 0.0, "gz": 0.0 },
  "mag": { "x": 0.2, "y": 0.1, "z": 0.4 },
  "baro": { "pressure": 101325.0, "temperature": 25.0 },
  "vfr_hud": { "airspeed": 0.0, "groundspeed": 0.0, "heading": 90, "throttle": 0.0, "altitude": 25.0, "climb_rate": 0.0 },
  "rc_in": { "channels": [1500, 1500, 1100, 1500] },
  "rc_out": { "channels": [1000, 1000, 1000, 1000] },
  "home": { "latitude": 28.6139, "longitude": 77.2090, "altitude": 0.0 },
  "relative_alt": 25.0,
  "heading": 90.0,
  "status_text": "GPS: 3D Fix"
}
```

### Individual telemetry endpoints

For lighter polling, use individual endpoints. Each returns only its section.

| Endpoint | Response fields |
|----------|----------------|
| GET `/telemetry/state` | connected, armed, mode, system_status |
| GET `/telemetry/battery` | voltage, current, percentage |
| GET `/telemetry/gps` | fix_type, latitude, longitude, altitude, satellites, hdop |
| GET `/telemetry/local_position` | x, y, z |
| GET `/telemetry/orientation` | roll, pitch, yaw (radians) |
| GET `/telemetry/velocity` | vx, vy, vz (m/s) |
| GET `/telemetry/imu` | ax, ay, az (m/s2), gx, gy, gz (rad/s) |
| GET `/telemetry/mag` | x, y, z (Tesla) |
| GET `/telemetry/baro` | pressure (Pa), temperature (C) |
| GET `/telemetry/vfr_hud` | airspeed, groundspeed, heading, throttle, altitude, climb_rate |
| GET `/telemetry/rc_in` | channels (array of PWM values) |
| GET `/telemetry/rc_out` | channels (array of motor/servo PWM outputs) |
| GET `/telemetry/home` | latitude, longitude, altitude |
| GET `/telemetry/extended_state` | landed_state, vtol_state |

---

## JavaScript Examples

### Check connection

```javascript
const API = 'http://drone-ip:8080';

const res = await fetch(`${API}/telemetry/state`);
const state = await res.json();
// state.connected, state.armed, state.mode
```

### Arm and fly

```javascript
// Enable virtual TX
await fetch(`${API}/control/enable`, { method: 'POST' });

// Arm
await fetch(`${API}/arm`, { method: 'POST' });

// Send throttle (hold for 2 seconds)
await fetch(`${API}/control/command`, {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    timestamp: Date.now() / 1000,
    throttle: 0.5,
    roll: 0.0,
    pitch: 0.0,
    yaw: 0.0,
    duration: 2.0
  })
});
```

### Continuous control loop

```javascript
// Send commands at ~200ms intervals for smooth control
// Each command holds for 1s as a safety net
let controlInterval = setInterval(() => {
  fetch(`${API}/control/command`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      timestamp: Date.now() / 1000,
      throttle: currentThrottle,   // from UI state
      roll: currentRoll,
      pitch: currentPitch,
      yaw: currentYaw,
      duration: 1.0
    })
  });
}, 200);

// Stop
clearInterval(controlInterval);
```

### Emergency stop

```javascript
// Disable TX + force disarm
await fetch(`${API}/control/disable`, { method: 'POST' });
await fetch(`${API}/disarm/force`, { method: 'POST' });
```

### Poll telemetry

```javascript
// Poll state at 2Hz for UI updates
setInterval(async () => {
  const res = await fetch(`${API}/telemetry/state`);
  const state = await res.json();
  updateUI(state);
}, 500);

// Poll battery at 0.5Hz
setInterval(async () => {
  const res = await fetch(`${API}/telemetry/battery`);
  const battery = await res.json();
  updateBatteryUI(battery);
}, 2000);
```

### Monitor control status

```javascript
// Check hold_remaining to know when to send next command
const res = await fetch(`${API}/control/status`);
const status = await res.json();

if (status.hold_remaining < 0.3) {
  // Less than 300ms left, send next command soon
  sendNextCommand();
}

if (status.in_failsafe) {
  // Connection lost or commands stopped
  showWarning('Failsafe active - auto-centering');
}
```

---

## Error Handling

All endpoints return `{ "success": bool, "message": string }` for command operations.

**Common error patterns:**

| Message | Cause | Action |
|---------|-------|--------|
| "Not connected to FCU" | Serial connection lost | Check USB cable, restart container |
| "Command rejected by FCU" | Pre-arm checks failing | Check GPS, battery, calibration |
| "Virtual TX not enabled" | Forgot to enable | Call POST /control/enable first |
| "Command too old" | Clock drift or network delay > 1.5s | Sync clocks, check latency |
| "Out of order" | Network reordered packets | Safe to ignore, newer command already active |
| "Invalid duration" | duration <= 0 | Use positive duration (default 1.0) |

---

## Notes for Web App Integration

1. **Always check `/telemetry/state` before operations.** Don't arm if not connected.
2. **Use `Date.now() / 1000` for timestamps** — the API expects Unix seconds, not milliseconds.
3. **Set `duration: 1.0` or higher** for commands over internet. This gives a 1-second safety net if a packet is lost.
4. **Poll `/control/status`** to monitor `hold_remaining`. Send next command before it hits 0.
5. **`"Hold"` response is normal** — it means your command matches the current state and the timer was extended. No action needed.
6. **Stream rate must be enabled** after each container restart for telemetry to work. The web app should call `/telemetry/state` on startup — if it returns empty/stale data, the stream rate needs to be set.
7. **Force disarm is the emergency stop.** Wire it to a prominent red button in the UI.
8. **Use ALT_HOLD for manual flying.** In ALT_HOLD, throttle mid-stick (~0.44) = hold altitude. The pilot controls direction, the FC handles altitude. Much easier than STABILIZE.
9. **LOITER requires GPS.** If flying indoors or without GPS fix, use ALT_HOLD instead.
10. **Arming takes ~1s to confirm** in ALT_HOLD mode. After POST `/arm` returns success, poll `/telemetry/state` to verify `armed: true` before sending commands.
11. **Failsafe is mode-aware.** If internet drops in ALT_HOLD/LOITER, the drone hovers in place (throttle mid-stick). In STABILIZE, throttle goes to idle.
