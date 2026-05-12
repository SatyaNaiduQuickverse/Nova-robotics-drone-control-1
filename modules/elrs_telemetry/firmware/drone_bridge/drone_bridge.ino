// drone_bridge.ino — v2 (ELRS bidirectional bridge)
//
// One ESP32-C6, two UARTs, one USB-CDC link to the drone-pi. Multiplexes
// both UARTs onto the USB-CDC stream using a 5-byte framed protocol so
// the daemon can read/write both streams over a single device file.
//
// Pi 5  <-- USB-CDC (framed) -->  ESP32-C6  <-- UART1 (channel 1) -->  NEW Ranger Tx 2G4 (downlink Tx)
//                                            <-- UART0 (channel 2) -->  EXISTING RP4-class Rx (uplink Rx)
//
// Wiring (per DRONE_TEAM_ELRS_DOWNLINK_PROMPT.md):
//   ESP GPIO 16  (UART1 RX, input)   <-- NEW Tx     TX pad
//   ESP GPIO 17  (UART1 TX, output)  --> NEW Tx     RX pad
//   ESP GPIO  5  (UART0 RX, input)   <-- Existing Rx TX pad
//   ESP GPIO  7  (UART0 TX, output)  --> Existing Rx RX pad
//   ESP GND      --> both modules GND
//
// Power: both modules from drone-pi 5V/GND DIRECTLY, NOT from ESP VBUS.
// The Tx pulls bursty current and brownouts the ESP regulator. We learned
// this the hard way on the uplink-only build.
//
// Failsafe: NONE on drone side. Per the prompt: if drone-pi goes silent,
// the downlink Tx must NOT keep transmitting stale frames — ground side
// reads RF silence as "drone went down". Compare ground_bridge.ino which
// DOES inject neutral failsafe on its UART1 (uplink) for the same reason
// in reverse.
//
// Wire format on USB-CDC (both directions, byte-for-byte symmetric with
// ground_bridge.ino):
//
//   +------+------+--------+----------+----------+
//   | 0xFE | 0xCA | chan u8| len_lo u8| len_hi u8|  (5-byte header)
//   +------+------+--------+----------+----------+
//   | payload (len bytes, raw module bytes) ...   |
//   +----------------------------------------------+
//
//   Magic = 0xCAFE little-endian (on wire: 0xFE then 0xCA).
//   chan = 1 → UART1 (NEW Tx, downlink)
//   chan = 2 → UART0 (existing Rx, uplink)
//   len    = u16 LE, max 256 bytes/frame.
//
// Diagnostic '#'-prefixed text lines are emitted OUTSIDE the framing.
// The Pi-side parser strips '#...\n' lines before demuxing.
//
// Build:  arduino-cli compile --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc drone_bridge
// Flash:  arduino-cli upload  -p /dev/ttyACM0 \
//                             --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc drone_bridge

#include <stdint.h>

// --- Pins / config -------------------------------------------------------
static const int  PIN_TX_UART_RX = 16;   // UART1: ESP RX  <-  NEW Tx     TX pad
static const int  PIN_TX_UART_TX = 17;   // UART1: ESP TX  ->  NEW Tx     RX pad
static const int  PIN_RX_UART_RX =  5;   // UART0: ESP RX  <-  Existing Rx TX pad
static const int  PIN_RX_UART_TX =  7;   // UART0: ESP TX  ->  Existing Rx RX pad
static const long UART_BAUD            = 420000;
static const long USB_BAUD             = 1000000;     // symbolic; CDC ignores baud
static const unsigned long STATS_INTERVAL_MS = 2000;

static const uint8_t  MAGIC_BYTE_1 = 0xFE;
static const uint8_t  MAGIC_BYTE_2 = 0xCA;
static const uint8_t  CHAN_TX      = 1;     // UART1: NEW downlink Tx
static const uint8_t  CHAN_RX      = 2;     // UART0: existing uplink Rx
static const size_t   MAX_PAYLOAD  = 256;

HardwareSerial TxUart(1);   // UART1 hardware peripheral
HardwareSerial RxUart(0);   // UART0 hardware peripheral (note: not USB-CDC — that's `Serial` when CDCOnBoot=cdc)

// --- Counters (cumulative, surfaced via stats line) ---------------------
static unsigned long up_to_pi = 0;   // UART1 (Tx module) → USB
static unsigned long pi_to_up = 0;   // USB → UART1 (Tx module)
static unsigned long dn_to_pi = 0;   // UART0 (Rx module) → USB
static unsigned long pi_to_dn = 0;   // USB → UART0 (Rx module)
static unsigned long bad_sync = 0;   // demux state-machine desync recoveries (ESP-side)
static unsigned long last_stats_ms = 0;

// --- Demux state machine (Pi → ESP → UART) ------------------------------
enum DemuxState {
    DMX_WAIT_MAGIC1,
    DMX_WAIT_MAGIC2,
    DMX_WAIT_CHAN,
    DMX_WAIT_LEN_LO,
    DMX_WAIT_LEN_HI,
    DMX_READ_PAYLOAD,
};

static DemuxState dmx_state    = DMX_WAIT_MAGIC1;
static uint8_t    dmx_chan     = 0;
static uint16_t   dmx_len      = 0;
static uint16_t   dmx_read     = 0;
static uint8_t    dmx_buf[MAX_PAYLOAD];

static inline void demux_byte(uint8_t b) {
    switch (dmx_state) {
        case DMX_WAIT_MAGIC1:
            if (b == MAGIC_BYTE_1) dmx_state = DMX_WAIT_MAGIC2;
            // else: garbage byte, stay in WAIT_MAGIC1 (don't count — we're
            // hunting for a boundary, not desynced from a known frame).
            break;
        case DMX_WAIT_MAGIC2:
            if (b == MAGIC_BYTE_2) {
                dmx_state = DMX_WAIT_CHAN;
            } else if (b == MAGIC_BYTE_1) {
                // 0xFE 0xFE — stay in MAGIC2, treat new 0xFE as the start.
            } else {
                dmx_state = DMX_WAIT_MAGIC1;
                bad_sync++;
            }
            break;
        case DMX_WAIT_CHAN:
            dmx_chan = b;
            dmx_state = DMX_WAIT_LEN_LO;
            break;
        case DMX_WAIT_LEN_LO:
            dmx_len = b;
            dmx_state = DMX_WAIT_LEN_HI;
            break;
        case DMX_WAIT_LEN_HI:
            dmx_len |= ((uint16_t)b << 8);
            if (dmx_len == 0 || dmx_len > MAX_PAYLOAD ||
                (dmx_chan != CHAN_TX && dmx_chan != CHAN_RX)) {
                // Invalid header — desync. Resync from next 0xFE.
                dmx_state = DMX_WAIT_MAGIC1;
                bad_sync++;
            } else {
                dmx_read = 0;
                dmx_state = DMX_READ_PAYLOAD;
            }
            break;
        case DMX_READ_PAYLOAD:
            dmx_buf[dmx_read++] = b;
            if (dmx_read >= dmx_len) {
                if (dmx_chan == CHAN_TX) {
                    TxUart.write(dmx_buf, dmx_len);
                    pi_to_up += dmx_len;
                } else if (dmx_chan == CHAN_RX) {
                    RxUart.write(dmx_buf, dmx_len);
                    pi_to_dn += dmx_len;
                }
                // unknown chan already filtered in WAIT_LEN_HI
                dmx_state = DMX_WAIT_MAGIC1;
            }
            break;
    }
}

// --- Mux (UART → ESP → Pi USB-CDC) --------------------------------------
//
// Drain whatever's available on a UART (up to MAX_PAYLOAD bytes), wrap in
// the 5-byte header, write to USB. One header per drain cycle, so under
// idle conditions we don't spam tiny framed packets — but we DO emit as
// soon as data lands (loop runs ~ instant) so latency is one loop tick.
static inline void mux_uart_to_usb(uint8_t channel, HardwareSerial& uart,
                                   unsigned long& counter) {
    int avail = uart.available();
    if (avail <= 0) return;
    if (avail > (int)MAX_PAYLOAD) avail = MAX_PAYLOAD;

    uint8_t buf[MAX_PAYLOAD];
    int n = uart.readBytes(buf, avail);
    if (n <= 0) return;

    uint8_t hdr[5];
    hdr[0] = MAGIC_BYTE_1;
    hdr[1] = MAGIC_BYTE_2;
    hdr[2] = channel;
    hdr[3] = (uint8_t)(n & 0xFF);
    hdr[4] = (uint8_t)((n >> 8) & 0xFF);
    Serial.write(hdr, 5);
    Serial.write(buf, n);
    counter += n;
}

// --- Boot + loop --------------------------------------------------------

void setup() {
    Serial.begin(USB_BAUD);
    TxUart.begin(UART_BAUD, SERIAL_8N1, PIN_TX_UART_RX, PIN_TX_UART_TX);
    RxUart.begin(UART_BAUD, SERIAL_8N1, PIN_RX_UART_RX, PIN_RX_UART_TX);
    Serial.printf("# drone_bridge v2 ready up=GPIO%d/%d dn=GPIO%d/%d baud=%ld\n",
                  PIN_TX_UART_RX, PIN_TX_UART_TX,
                  PIN_RX_UART_RX, PIN_RX_UART_TX, UART_BAUD);
}

void loop() {
    // 1. UART1 (NEW Tx module) → USB framed (channel 1 = up)
    mux_uart_to_usb(CHAN_TX, TxUart, up_to_pi);

    // 2. UART0 (existing Rx module) → USB framed (channel 2 = dn)
    mux_uart_to_usb(CHAN_RX, RxUart, dn_to_pi);

    // 3. USB → demuxer → appropriate UART
    while (Serial.available() > 0) {
        int b = Serial.read();
        if (b >= 0) demux_byte((uint8_t)b);
    }

    // 4. Stats every 2 s — emitted OUTSIDE the framing
    unsigned long now_ms = millis();
    if (now_ms - last_stats_ms >= STATS_INTERVAL_MS) {
        last_stats_ms = now_ms;
        Serial.printf("# stats up_to_pi=%lu pi_to_up=%lu dn_to_pi=%lu pi_to_dn=%lu "
                      "fs=0 fs_frames=0 bad_sync=%lu uptime_s=%lu\n",
                      up_to_pi, pi_to_up, dn_to_pi, pi_to_dn,
                      bad_sync, now_ms / 1000);
    }
}
