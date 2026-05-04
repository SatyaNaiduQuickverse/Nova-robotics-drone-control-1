// drone_bridge.ino
// Dumb USB-CDC <-> UART1 bridge for an ELRS RP4TD RX.
//
//   Pi 5  <-- USB-CDC -->  ESP32-C6  <-- UART1 -->  RP4TD
//
// Wiring:
//   ESP32 GPIO 7 (UART1 RX)  <-  RP4TD TX pad   (data OUT from RX module)
//   ESP32 GPIO 5 (UART1 TX)  ->  RP4TD RX pad   (data IN to RX module — telemetry uplink)
//   ESP32 GND                ->  RP4TD GND
//   Pi GPIO Pin 2 (5V)       ->  RP4TD 5V       (powered direct from Pi, NOT ESP32 VBUS;
//                                                avoids brownout under RP4TD TX-burst load)
//   Pi GPIO Pin 6 (GND)      ->  RP4TD GND      (shared)
//
// No state, no framing, no failsafe. The Pi handles all CRSF parsing
// and frame validation; this firmware is purely a wire bridge.
//
// Build:  arduino-cli compile --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc drone_bridge
// Flash:  arduino-cli upload  -p /dev/ttyACM0 \
//                             --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc drone_bridge

static const int  PIN_UART_RX = 7;
static const int  PIN_UART_TX = 5;
static const long UART_BAUD   = 420000;
static const long USB_BAUD    = 1000000;   // symbolic on USB-CDC; not enforced

HardwareSerial CRSF(1);

static unsigned long bytes_uart_to_usb = 0;
static unsigned long bytes_usb_to_uart = 0;
static unsigned long last_stats_ms     = 0;

void setup() {
  Serial.begin(USB_BAUD);
  CRSF.begin(UART_BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
  // Print one banner so 'cat /dev/ttyACM0' confirms the firmware is alive.
  Serial.printf("# drone_bridge ready uart_rx=%d uart_tx=%d baud=%ld\n",
                PIN_UART_RX, PIN_UART_TX, UART_BAUD);
}

void loop() {
  uint8_t buf[256];

  // UART -> USB  (uplink: RX -> Pi)
  int n = CRSF.available();
  if (n > 0) {
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    int read = CRSF.readBytes(buf, n);
    if (read > 0) {
      Serial.write(buf, read);
      bytes_uart_to_usb += read;
    }
  }

  // USB -> UART  (downlink: Pi -> RX -> ground)
  int m = Serial.available();
  if (m > 0) {
    if (m > (int)sizeof(buf)) m = sizeof(buf);
    int read = Serial.readBytes(buf, m);
    if (read > 0) {
      CRSF.write(buf, read);
      bytes_usb_to_uart += read;
    }
  }

  // Periodic stats line, prefixed with '#' so the Pi parser drops it.
  unsigned long now = millis();
  if (now - last_stats_ms >= 2000) {
    last_stats_ms = now;
    Serial.printf("# stats uart_to_usb=%lu usb_to_uart=%lu uptime_s=%lu\n",
                  bytes_uart_to_usb, bytes_usb_to_uart, now / 1000);
  }
}
