// tx_bridge.ino v3 — dual-UART CRSF rig with content-aware diagnostics
//
// Same wiring as v2:
//   UART1 (GPIO 16/17) ↔ Ranger Micro 2G4 Tx
//   UART0 (GPIO  7/ 5) ↔ RP4TD RX
//   USB-CDC                → Pi /dev/ttyACM0
//
// Improvements over v2:
//   - Tracks separate counters for "raw bytes" vs "CRSF sync bytes seen".
//     Boot ROM residue and noise has no 0xC8 / 0xEA / 0xEC / 0xEE bytes
//     so the sync count differentiates real CRSF from garbage.
//   - Stats every 1 s for finer resolution.
//   - Discards UART0 RX bytes for first 500 ms after boot (ignores boot
//     ROM FIFO residue which masquerades as RP4TD activity).
//   - Captures first 32 bytes per UART AFTER the post-boot window for
//     visibility on what the bus is actually carrying.
//   - Periodically dumps a "live sample" of the most recent 16 bytes
//     from each UART, so byte content is visible across the window.

#include <stdint.h>

// Forward declarations — Arduino preprocessor auto-generates function
// prototypes at the top of the file before user code, so types those
// prototypes reference must be visible at the very top.
struct UartMetrics;

static const int PIN_TX_UART_RX = 16;
static const int PIN_TX_UART_TX = 17;
static const int PIN_RX_UART_RX =  7;
static const int PIN_RX_UART_TX =  5;
static const long UART_BAUD            = 420000;
static const long USB_BAUD             = 1000000;
static const unsigned long FS_INTERVAL_US      = 10000;   // 100 Hz
static const unsigned long BOOT_DRAIN_MS       = 500;     // ignore UART0 noise after boot
static const unsigned long STATS_INTERVAL_MS   = 1000;
static const unsigned long SAMPLE_INTERVAL_MS  = 5000;    // dump live samples every 5 s

HardwareSerial RangerUart(1);
HardwareSerial Rp4tdUart(0);

static uint16_t channels[16] = {
    172, 992, 992, 992, 992, 992, 172, 172,
    992, 992, 992, 992, 992, 992, 992, 992,
};

static uint8_t crc8_dvbs2(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
    }
    return crc;
}

// CRSF sync bytes (any of these starting a frame indicates real CRSF activity).
static inline bool is_crsf_sync(uint8_t b) {
    return b == 0xC8 || b == 0xEA || b == 0xEC || b == 0xEE || b == 0x00;
}

// Per-UART metrics — declared before functions that use it (Arduino
// preprocessor auto-prototypes all functions at the top, so types they
// reference must be declared above).
struct UartMetrics {
    unsigned long bytes_total = 0;     // raw bytes seen
    unsigned long sync_bytes  = 0;     // count of bytes matching CRSF sync
    unsigned long c8_bytes    = 0;     // 0xC8 specifically (FC origin)
    unsigned long ea_bytes    = 0;     // 0xEA specifically (RX origin)
    unsigned long ec_bytes    = 0;     // 0xEC specifically (handset)
    unsigned long ee_bytes    = 0;     // 0xEE specifically (TX module)
    uint8_t last_bytes[16]    = {0};   // ring of most recent 16 bytes
    int      last_idx         = 0;
};

static UartMetrics tx_m;   // UART1 RX (Ranger Tx echo)
static UartMetrics rx_m;   // UART0 RX (RP4TD output)

static void build_rc_frame(uint8_t* out) {
    out[0] = 0xC8;
    out[1] = 24;
    out[2] = 0x16;
    uint32_t bits = 0;
    int nbits = 0;
    int o = 3;
    for (int i = 0; i < 16; i++) {
        bits |= ((uint32_t)(channels[i] & 0x7FF)) << nbits;
        nbits += 11;
        while (nbits >= 8) {
            out[o++] = bits & 0xFF;
            bits >>= 8;
            nbits -= 8;
        }
    }
    out[25] = crc8_dvbs2(&out[2], 23);
}

static unsigned long bytes_tx_sent = 0;
static unsigned long fs_frames     = 0;
static unsigned long last_fs_us    = 0;
static unsigned long last_stats_ms = 0;
static unsigned long last_sample_ms = 0;

static void tally(UartMetrics& m, const uint8_t* buf, int n) {
    m.bytes_total += n;
    for (int i = 0; i < n; i++) {
        uint8_t b = buf[i];
        if (is_crsf_sync(b)) m.sync_bytes++;
        if (b == 0xC8) m.c8_bytes++;
        else if (b == 0xEA) m.ea_bytes++;
        else if (b == 0xEC) m.ec_bytes++;
        else if (b == 0xEE) m.ee_bytes++;
        m.last_bytes[m.last_idx] = b;
        m.last_idx = (m.last_idx + 1) % 16;
    }
}

static void dump_sample(const char* label, const UartMetrics& m) {
    if (m.bytes_total == 0) {
        Serial.printf("# sample %s: NO BYTES\n", label);
        return;
    }
    Serial.printf("# sample %s last16:", label);
    // Print in time order (last_idx is the next-write position)
    for (int i = 0; i < 16; i++) {
        int idx = (m.last_idx + i) % 16;
        Serial.printf(" %02X", m.last_bytes[idx]);
    }
    Serial.printf(" | sync=%lu C8=%lu EA=%lu EC=%lu EE=%lu\n",
                  m.sync_bytes, m.c8_bytes, m.ea_bytes, m.ec_bytes, m.ee_bytes);
}

void setup() {
    Serial.begin(USB_BAUD);
    RangerUart.begin(UART_BAUD, SERIAL_8N1, PIN_TX_UART_RX, PIN_TX_UART_TX);
    Rp4tdUart.begin(UART_BAUD, SERIAL_8N1, PIN_RX_UART_RX, PIN_RX_UART_TX);
    Serial.printf("# tx_bridge_v3 ready  Tx=UART1(rx=%d tx=%d)  RX=UART0(rx=%d tx=%d) baud=%ld fs_hz=100 boot_drain_ms=%lu\n",
                  PIN_TX_UART_RX, PIN_TX_UART_TX,
                  PIN_RX_UART_RX, PIN_RX_UART_TX,
                  UART_BAUD, BOOT_DRAIN_MS);
}

void loop() {
    uint8_t frame[26];
    uint8_t buf[256];
    unsigned long now_ms = millis();

    // 1. Drain UART1 RX (Ranger Tx)
    int n = RangerUart.available();
    if (n > 0) {
        if (n > (int)sizeof(buf)) n = sizeof(buf);
        int read = RangerUart.readBytes(buf, n);
        if (read > 0) tally(tx_m, buf, read);
    }

    // 2. Drain UART0 RX (RP4TD), but discard during BOOT_DRAIN_MS
    n = Rp4tdUart.available();
    if (n > 0) {
        if (n > (int)sizeof(buf)) n = sizeof(buf);
        int read = Rp4tdUart.readBytes(buf, n);
        if (read > 0 && now_ms >= BOOT_DRAIN_MS) {
            tally(rx_m, buf, read);
        }
        // bytes during boot drain are silently dropped to avoid polluting metrics
    }

    // 3. 100 Hz CRSF inject
    unsigned long now_us = micros();
    if (now_us - last_fs_us >= FS_INTERVAL_US) {
        last_fs_us = now_us;
        build_rc_frame(frame);
        RangerUart.write(frame, sizeof(frame));
        bytes_tx_sent += sizeof(frame);
        fs_frames++;
    }

    // 3b. Send a CRSF DEVICE_PING every 2 s to the Tx module.
    // Frame: [0xC8][len=4][type=0x28][dest=0xEE TX][orig=0xC8 FC][CRC]
    // CRC over [type, dest, orig] = 3 bytes.
    // Any alive CRSF device should respond with DEVICE_INFO (0x29).
    // If Tx is wired bi-directionally and powered, we'll see a response
    // on UART1 RX even if the RF link is dead. This isolates "wire to
    // Tx" from "Tx → RX RF link" failure modes.
    static unsigned long last_ping_ms = 0;
    if (now_ms - last_ping_ms >= 2000) {
        last_ping_ms = now_ms;
        uint8_t ping[6];
        ping[0] = 0xC8;
        ping[1] = 0x04;
        ping[2] = 0x28;
        ping[3] = 0xEE;  // dest = TX module
        ping[4] = 0xC8;  // origin = FC
        ping[5] = crc8_dvbs2(&ping[2], 3);
        RangerUart.write(ping, sizeof(ping));
    }

    // 4. Stats line every 1 s
    if (now_ms - last_stats_ms >= STATS_INTERVAL_MS) {
        last_stats_ms = now_ms;
        Serial.printf("# stats t=%lus tx_sent=%lu fs_frames=%lu | "
                      "Tx_UART1: total=%lu sync=%lu C8=%lu EA=%lu | "
                      "RX_UART0: total=%lu sync=%lu C8=%lu EA=%lu\n",
                      now_ms / 1000, bytes_tx_sent, fs_frames,
                      tx_m.bytes_total, tx_m.sync_bytes, tx_m.c8_bytes, tx_m.ea_bytes,
                      rx_m.bytes_total, rx_m.sync_bytes, rx_m.c8_bytes, rx_m.ea_bytes);
    }

    // 5. Live byte sample every 5 s (so we can see content drift)
    if (now_ms - last_sample_ms >= SAMPLE_INTERVAL_MS) {
        last_sample_ms = now_ms;
        dump_sample("Tx_UART1", tx_m);
        dump_sample("RX_UART0", rx_m);
    }
}
