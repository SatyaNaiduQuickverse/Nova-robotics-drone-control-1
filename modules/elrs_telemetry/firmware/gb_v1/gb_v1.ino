// ground_bridge.ino — ESP32-C6 USB-CDC ↔ UART1 wire bridge (ground side).
//
//   Pi 5  <-- USB-CDC -->  ESP32-C6  <-- UART1 -->  Ranger Micro 2G4 (Tx module)
//
// Functional twin of the drone-side firmware (byte pump). Adds one
// architecturally significant difference vs. drone side: a host-loss
// failsafe that injects a safe CRSF channel frame on UART1 when the
// Pi stops feeding USB-CDC. Without this, a daemon crash / USB hang
// leaves the Tx broadcasting whatever frame was last sent — possibly
// full-stick + active button bits.
//
// Flash:
//   arduino-cli compile --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc ground_bridge
//   arduino-cli upload  -p /dev/ttyACM0 \
//                       --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc ground_bridge
//
// Wiring (EDIT FOR YOUR PCB):
//   ESP32 GPIO_UART_RX  <- Ranger Micro Tx-side TX pad
//   ESP32 GPIO_UART_TX  -> Ranger Micro Tx-side RX pad
//   Power Tx module DIRECTLY from Pi 5V/GND, NOT through ESP VBUS
//   (drone team's hard-won lesson — Tx draws bursty current).
//   Common ground between Pi/ESP/Tx is mandatory.
//
// Continuity-check the wiring with a multimeter BEFORE powering the
// Tx. The drone-side post-mortem on this is in
// memory:elrs-wiring-diagnosis.md — saves a multi-day debug session.

// ── PIN CONFIGURATION ────────────────────────────────────────────────────
// Set per the ground-side PCB: GPIO 16 + 17 wired to the Ranger Micro
// Tx UART pads. Convention used here:
//   PIN_UART_RX (ESP input)  = GPIO 16 — wire to Tx-side TX pad (telem coming back)
//   PIN_UART_TX (ESP output) = GPIO 17 — wire to Tx-side RX pad (CRSF frames going out)
//
// If after flashing you see `uart_to_usb` stays at 0 in the # stats
// line while `usb_to_uart` rises, the Tx is receiving frames but its
// telemetry isn't reaching us — most likely RX/TX are crossed. Swap
// the two values below and re-flash.

static const int  PIN_UART_RX = 16;         // <- Ranger Micro Tx TX pad  (matches AlfredoCRSF firmware's RX_OUT)
static const int  PIN_UART_TX = 17;         // -> Ranger Micro Tx RX pad  (matches AlfredoCRSF firmware's TX_OUT)

// ── Protocol constants ───────────────────────────────────────────────────

static const long UART_BAUD = 420000;        // CRSF baud (mandatory)
static const long USB_BAUD  = 1000000;       // symbolic on USB-CDC

// ── Failsafe configuration ───────────────────────────────────────────────
// After 1 s of USB-CDC silence, fire failsafe CRSF frames at exactly
// the Tx's configured Packet Rate. The Ranger Micro 2G4's MCU stops
// transmitting on RF if host frames arrive much faster than the
// packet rate (interprets it as host malfunction on its half-duplex
// bus). 100 Hz matches "100Hz Full"; anything ≥200 Hz is risky.
// Use micros() rather than millis() so the gate is precise to 10 µs.

static const unsigned long FAILSAFE_AFTER_MS = 1000;
static const unsigned long INJECT_INTERVAL_US = 10000;   // 100 Hz (= packet rate)

// 16 channels in CRSF units (172=min, 992=center, 1811=max).
// Per drone-team contract:
//   CH1 throttle MIN — prevents runaway thrust on host loss
//   CH8 ops field 0 — clears all bitfield operations (no spurious
//                     vision-lock engage / follow / abort / cancel)
//   all others CENTER — sticks neutral, mode rotary at index 0
static const uint16_t FS_CHANNELS[16] = {
    172,   // CH1  throttle MIN
    992,   // CH2  roll center
    992,   // CH3  pitch center
    992,   // CH4  yaw center
    992,   // CH5  arm center (= disarmed)
    992,   // CH6  force-disarm center (no rising edge)
    172,   // CH7  mode rotary at index 0 (= STABILIZE)
    172,   // CH8  ops field = 0 (all bits clear)
    992,   // CH9  box coords centered (irrelevant when CH8=0)
    992,   // CH10
    992,   // CH11
    992,   // CH12
    992,   // CH13–16 not carried over RF in 12ch Mixed mode
    992,
    992,
    992,
};

// ── State ────────────────────────────────────────────────────────────────

HardwareSerial CRSF(1);

// tx_sent: total bytes written to Ranger UART (Pi-forwarded + failsafe-injected).
// tx_rx:   total bytes read from Ranger UART (telemetry uplink to Pi).
// Names match the drone team's tx_bridge.ino diagnostics so logs read identically.
static unsigned long tx_sent = 0;
static unsigned long tx_rx = 0;
static unsigned long failsafe_frames_sent = 0;
static unsigned long last_stats_ms = 0;
static unsigned long last_usb_data_ms = 0;
static unsigned long last_inject_us = 0;  // micros() of last failsafe emission

// ── CRSF encoder (failsafe injection only) ──────────────────────────────

static uint8_t crc8_dvbs2(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
        }
    }
    return crc;
}

static void send_failsafe_frame() {
    uint8_t frame[26];
    frame[0] = 0xC8;          // sync/addr (FC destination)
    frame[1] = 24;            // length: type(1) + payload(22) + CRC(1)
    frame[2] = 0x16;          // RC_CHANNELS_PACKED

    // Pack 16 × 11-bit channels LSB-first into bytes 3..24.
    uint32_t bits = 0;
    int nbits = 0;
    int out = 3;
    for (int i = 0; i < 16; i++) {
        bits |= ((uint32_t)(FS_CHANNELS[i] & 0x7FF)) << nbits;
        nbits += 11;
        while (nbits >= 8) {
            frame[out++] = bits & 0xFF;
            bits >>= 8;
            nbits -= 8;
        }
    }
    frame[25] = crc8_dvbs2(&frame[2], 23);

    CRSF.write(frame, 26);
    tx_sent += 26;
    failsafe_frames_sent++;
}

// ── Setup / loop ─────────────────────────────────────────────────────────

void setup() {
    Serial.begin(USB_BAUD);
    CRSF.begin(UART_BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
    Serial.printf("# ground_bridge ready uart_rx=%d uart_tx=%d baud=%ld\n",
                  PIN_UART_RX, PIN_UART_TX, UART_BAUD);
    last_usb_data_ms = millis();   // start in non-failsafe state
}

void loop() {
    uint8_t buf[256];

    // UART -> USB  (Tx telemetry → Pi)
    int n = CRSF.available();
    if (n > 0) {
        if (n > (int)sizeof(buf)) n = sizeof(buf);
        int read = CRSF.readBytes(buf, n);
        if (read > 0) {
            Serial.write(buf, read);
            tx_rx += read;
        }
    }

    // USB -> UART  (Pi's CRSF → Tx → RF uplink)
    int m = Serial.available();
    if (m > 0) {
        if (m > (int)sizeof(buf)) m = sizeof(buf);
        int read = Serial.readBytes(buf, m);
        if (read > 0) {
            CRSF.write(buf, read);
            tx_sent += read;
            last_usb_data_ms = millis();
        }
    }

    // Failsafe injection: when Pi is silent past the threshold, fire
    // CRSF frames at exactly 100 Hz to match the Tx's configured
    // packet rate. Re-read millis() HERE (not at top of loop) so the
    // value is monotonically >= last_usb_data_ms even if the USB-CDC
    // read above just updated it. Earlier version had a race where a
    // stale `now` from loop start could be < last_usb_data_ms, the
    // unsigned subtraction would wrap, and failsafe would spuriously
    // fire on every iteration with USB-CDC activity.
    unsigned long now = millis();
    if ((now - last_usb_data_ms) > FAILSAFE_AFTER_MS) {
        unsigned long now_us = micros();
        if ((now_us - last_inject_us) >= INJECT_INTERVAL_US) {
            last_inject_us = now_us;
            send_failsafe_frame();
        }
    }

    // Periodic stats. '#' is not a CRSF sync byte, so the Pi's parser
    // drops these without affecting host_ok / bad_sync counters.
    if ((now - last_stats_ms) >= 2000) {
        last_stats_ms = now;
        bool fs_active = (now - last_usb_data_ms) > FAILSAFE_AFTER_MS;
        Serial.printf("# stats tx_sent=%lu tx_rx=%lu fs=%d fs_frames=%lu uptime_s=%lu\n",
                      tx_sent, tx_rx,
                      fs_active ? 1 : 0, failsafe_frames_sent,
                      now / 1000);
    }
}
