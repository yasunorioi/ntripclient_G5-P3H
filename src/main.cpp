/*
 *  ntripclient_G5-P3H — M5Atom Lite NTRIP client for a Septentrio mosaic-go G5 P3H rover.
 *
 *  Data flow (rover):
 *    caster (rtk.toiso.fit:2101/eniwa-bd982, anonymous)
 *        │  RTCM3 over WiFi (NTRIP)
 *        ▼
 *    M5Atom Lite  ── Grove UART G26/G32 (3.3V TTL) ──►  mosaic-go COM1  (RTCM3 IN → RTK)
 *
 *    mosaic-go COM2 ──► NMEA GGA @38400 ──► tractor guidance   (mosaic emits directly;
 *                                                               the Atom is not on this leg)
 *
 *  Wiring (Grove HY2.0 on the Atom, 3 wires to mosaic COM1 — both sides are 3.3V LVTTL,
 *  no RS232/level shifter on this leg):
 *    Atom G26 (TX) ──► mosaic COM1 RX
 *    Atom G32 (RX) ◄── mosaic COM1 TX        (optional; unused by this pure forwarder)
 *    GND ── GND
 *  If no RTK fix appears, swap G26/G32 (TX/RX crossed wrong).
 *
 *  mosaic-go one-time configuration (do ONCE via the receiver's web UI / RxTools,
 *  then SAVE to boot config — the mosaic only applies output config in the quiet
 *  window before it starts streaming, so leaving it to boot config is the robust path):
 *    - COM1: accept RTCM3 corrections as the differential source (setRTCMInput / auto).
 *    - COM2: baud 38400, output NMEA GGA (e.g. setNMEAOutput, Stream, COM2, GGA, sec1).
 *  With that saved, this Atom stays a dumb, robust RTCM3 forwarder.
 *
 *  WiFi: WiFiManager (hold the Atom button at boot for the config portal).
 *  NTRIP auth: anonymous (single-base mount, raw — no GGA upload).
 *  LED: red=start/fail, yellow=connecting, blue=portal, green=forwarding RTCM3,
 *       rainbow=stalled (no RTCM3 for >5s; auto-reboot after 30s).
 */
#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <FastLED.h>
#include "NTRIPClient.h"

NTRIPClient ntrip_c;

// M5Atom Lite onboard WS2812 LED
#define LED_PIN 27
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// ---- NTRIP caster config ----
const char* casterHost = "rtk.toiso.fit";
const uint16_t casterPort = 2101;
const char* mountpoint = "eniwa-bd982";
const char* ntripUser = "";  // anonymous
const char* ntripPass = "";  // anonymous

// ---- mosaic COM1 link on the Atom's Grove UART (3.3V TTL) ----
// Serial2.begin(baud, cfg, rx_pin, tx_pin). Atom TX(G26) -> mosaic COM1 RX.
#define MOSAIC_TX_PIN 26   // Atom G26 = Serial2 TX -> mosaic COM1 RX
#define MOSAIC_RX_PIN 32   // Atom G32 = Serial2 RX <- mosaic COM1 TX (unused here)
#define MOSAIC_COM1_BAUD 115200   // mosaic COM1 default; match the receiver's COM1 baud

// ---- State ----
uint64_t totalBytes = 0;
uint64_t lastBytes  = 0;
unsigned long lastDataTime = 0;
const unsigned long STALL_TIMEOUT_MS = 5000;
uint8_t rainbowHue = 0;

// HSV to RGB (hue: 0-255, sat/val: 0-255)
void hsvToRgb(uint8_t h, uint8_t s, uint8_t v, uint8_t &r, uint8_t &g, uint8_t &b) {
  uint8_t region = h / 43;
  uint8_t remainder = (h - region * 43) * 6;
  uint8_t p = (v * (255 - s)) >> 8;
  uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
  uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;
  switch (region) {
    case 0:  r=v; g=t; b=p; break;
    case 1:  r=q; g=v; b=p; break;
    case 2:  r=p; g=v; b=t; break;
    case 3:  r=p; g=q; b=v; break;
    case 4:  r=t; g=p; b=v; break;
    default: r=v; g=p; b=q; break;
  }
}

void setLed(uint8_t r, uint8_t g, uint8_t b) {
  leds[0] = CRGB(r, g, b);
  FastLED.show();
}

void setLedRainbow() {
  uint8_t r, g, b;
  hsvToRgb(rainbowHue, 255, 64, r, g, b);
  setLed(r, g, b);
  rainbowHue += 4;
}

// ---- WiFi Setup (WiFiManager) ----
void setupWiFi() {
  WiFiManager wm;

  bool doManualConfig = false;
  Serial.println("Hold button for WiFi config...");
  for (int i = 0; i < 200; i++) {
    M5.update();
    if (M5.BtnA.isPressed()) {
      doManualConfig = true;
      break;
    }
    delay(10);
  }

  if (doManualConfig) {
    Serial.println("Starting WiFi config portal");
    setLed(0, 0, 0x40);  // Blue = config mode
    wm.startConfigPortal("NTRIP-Client");
  } else {
    Serial.println("WiFi connecting...");
    setLed(0x40, 0x40, 0);  // Yellow = connecting
    wm.autoConnect("NTRIP-Client");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi failed, restarting...");
    setLed(0x40, 0, 0);
    delay(3000);
    ESP.restart();
  }
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  // mosaic COM1 link (RTCM3 out to the receiver) on the Atom's Grove UART.
  Serial2.begin(MOSAIC_COM1_BAUD, SERIAL_8N1, MOSAIC_RX_PIN, MOSAIC_TX_PIN);  // (rx=32, tx=26)

  setLed(0x40, 0, 0);  // Red = starting
  setupWiFi();

  Serial.print("Connecting to NTRIP: ");
  Serial.print(casterHost);
  Serial.print(":");
  Serial.print(casterPort);
  Serial.print("/");
  Serial.println(mountpoint);

  int port = casterPort;
  if (!ntrip_c.reqRaw((char*)casterHost, port, (char*)mountpoint, (char*)ntripUser, (char*)ntripPass)) {
    Serial.println("NTRIP connection failed, restarting...");
    setLed(0x40, 0, 0);
    delay(15000);
    ESP.restart();
  }

  Serial.println("NTRIP connected!");
  setLed(0, 0x40, 0);  // Green = connected
  lastDataTime = millis();
}

void loop() {
  M5.update();

  // Read RTCM3 from NTRIP and forward to mosaic COM1 (Serial2).
  while (ntrip_c.available()) {
    char ch = ntrip_c.read();
    Serial2.print(ch);
    totalBytes++;
  }

  unsigned long now = millis();

  if (totalBytes > lastBytes) {
    lastBytes = totalBytes;
    lastDataTime = now;
    setLed(0, 0x40, 0);  // Green = forwarding RTCM3
  }

  // Stall detection
  if (now - lastDataTime > STALL_TIMEOUT_MS) {
    setLedRainbow();  // Rainbow = stalled
    if (now - lastDataTime > 30000) {
      Serial.println("Stalled 30s, restarting...");
      ntrip_c.stop();
      delay(1000);
      ESP.restart();
    }
  }

  if (!ntrip_c.connected()) {
    Serial.println("NTRIP disconnected, restarting...");
    setLed(0x40, 0, 0);  // Red
    delay(5000);
    ESP.restart();
  }

  // Print stats every second
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 1000) {
    Serial.printf("RTCM bytes: %llu\n", totalBytes);
    lastPrint = now;
  }

  delay(10);
}
