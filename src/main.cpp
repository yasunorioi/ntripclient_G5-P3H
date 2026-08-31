/*
 *  ntripclient_G5-P3H — M5Atom Lite NTRIP client for a Septentrio mosaic-go G5 P3H rover.
 *
 *  Data flow (rover):
 *    caster (NTRIP, configurable — default rtk.toiso.fit:2101/eniwa-bd982, anon)
 *        │  RTCM3 over WiFi
 *        ▼
 *    M5Atom Lite  ── Grove UART G26/G32 (3.3V TTL) ──►  mosaic-go COM1  (RTCM3 IN → RTK)
 *                 ◄── NMEA GGA (fix status) ───────────  mosaic-go COM1  (for the LED)
 *
 *    mosaic-go COM2 ──► NMEA GGA @38400 ──► tractor guidance   (mosaic emits directly)
 *
 *  Three finished behaviours:
 *    1. Fix-status LED  — reads GGA back on COM1 and colours the LED by RTK quality.
 *    2. Field config    — NTRIP host/port/mount/user/pass set in the WiFiManager
 *                         portal, saved to NVS (Preferences). No reflash to move sites.
 *    3. No-reboot reconnect — WiFi / NTRIP drops retry in place; a reboot is only the
 *                         last resort after a long outage.
 *
 *  Wiring (Grove HY2.0, both sides 3.3V LVTTL — no level shifter on this leg):
 *    Atom G26 (TX) ──► mosaic COM1 RX      Atom G32 (RX) ◄── mosaic COM1 TX      GND──GND
 *  If no fix / no GGA, suspect G26/G32 swapped.
 *
 *  mosaic-go setup: NONE by hand. The Atom self-provisions the receiver on boot
 *  (setNMEAOutput on COM1 for the LED + COM2 @38400 for the tractor), re-applied
 *  every power-up (current config, like tab5-caster) — a factory receiver works
 *  out of the box. Assumes COM1 is at its 115200 default. RTCM3 fed to COM1 is
 *  auto-used as corrections. Set PROVISION_MOSAIC 0 to disable.
 *
 *  LED: blue=portal, yellow=WiFi connecting, magenta=NTRIP connecting,
 *       GREEN=RTK fixed, CYAN=RTK float, ORANGE=corrections flowing but not fixed,
 *       rainbow=corrections stalled, red=offline (pre-reboot).
 */
#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <FastLED.h>
#include <Preferences.h>
#include "NTRIPClient.h"

NTRIPClient ntrip_c;
Preferences prefs;

// M5Atom Lite onboard WS2812 LED
#define LED_PIN 27
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// ---- mosaic COM1 link on the Atom's Grove UART (3.3V TTL) ----
#define MOSAIC_TX_PIN 26          // Atom G26 = Serial2 TX -> mosaic COM1 RX
#define MOSAIC_RX_PIN 32          // Atom G32 = Serial2 RX <- mosaic COM1 TX (GGA back)
#define MOSAIC_COM1_BAUD 115200   // mosaic COM1 default; the command channel too

// Self-provision the mosaic on boot so a factory receiver works out of the box
// (no manual RxTools/web-UI step). Set 0 if you pre-configured + saved boot config.
#define PROVISION_MOSAIC 1
#define TRACTOR_NMEA_BAUD 38400   // mosaic COM2 -> tractor guidance

// ---- Runtime config (NVS-backed, editable in the portal) ----
struct Config {
  char host[64];
  int  port;
  char mount[48];
  char user[32];
  char pass[32];
} cfg;

// Factory defaults (used until the portal overwrites them).
static const char* DEF_HOST  = "rtk.toiso.fit";
static const int   DEF_PORT  = 2101;
static const char* DEF_MOUNT = "eniwa-bd982";
static const char* DEF_USER  = "";      // anonymous
static const char* DEF_PASS  = "";

void loadConfig() {
  prefs.begin("ntrip", true);           // read-only
  String h = prefs.getString("host", DEF_HOST);
  cfg.port = prefs.getInt("port", DEF_PORT);
  String m = prefs.getString("mount", DEF_MOUNT);
  String u = prefs.getString("user", DEF_USER);
  String p = prefs.getString("pass", DEF_PASS);
  prefs.end();
  strlcpy(cfg.host, h.c_str(), sizeof(cfg.host));
  strlcpy(cfg.mount, m.c_str(), sizeof(cfg.mount));
  strlcpy(cfg.user, u.c_str(), sizeof(cfg.user));
  strlcpy(cfg.pass, p.c_str(), sizeof(cfg.pass));
}

void saveConfig() {
  prefs.begin("ntrip", false);          // read-write
  prefs.putString("host", cfg.host);
  prefs.putInt("port", cfg.port);
  prefs.putString("mount", cfg.mount);
  prefs.putString("user", cfg.user);
  prefs.putString("pass", cfg.pass);
  prefs.end();
}

// ---- Fix state (from GGA readback) ----
enum FixQ { FIX_NONE = 0, FIX_GPS = 1, FIX_DGPS = 2, FIX_RTK_FIXED = 4, FIX_RTK_FLOAT = 5 };
volatile int  g_fixQ = FIX_NONE;
unsigned long g_lastGga = 0;
char nmea[100];
uint8_t nmeaLen = 0;

// ---- Link state ----
uint64_t totalBytes = 0;
uint64_t lastBytes  = 0;
unsigned long lastRtcm = 0;
const unsigned long RTCM_STALL_MS = 5000;   // corrections silent -> "stalled"
const unsigned long GGA_STALE_MS  = 5000;   // GGA silent -> fix unknown
const unsigned long NTRIP_RETRY_MS = 5000;  // backoff between NTRIP reconnects
const unsigned long WIFI_LASTRESORT_MS = 180000;  // WiFi down this long -> reboot
uint8_t rainbowHue = 0;
unsigned long wifiDownSince = 0;
unsigned long lastNtripTry  = 0;

// ---------- LED ----------
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
void setLed(uint8_t r, uint8_t g, uint8_t b) { leds[0] = CRGB(r, g, b); FastLED.show(); }
void setLedRainbow() {
  uint8_t r, g, b; hsvToRgb(rainbowHue, 255, 64, r, g, b); setLed(r, g, b); rainbowHue += 4;
}

// Colour the LED by the most important thing happening right now.
void updateLed(bool wifiUp, bool ntripUp, bool rtcmStalled) {
  unsigned long now = millis();
  if (!wifiUp)  { setLed(0x40, 0x40, 0); return; }          // yellow = WiFi connecting
  if (!ntripUp) { setLed(0x30, 0, 0x30); return; }          // magenta = NTRIP connecting
  if (rtcmStalled) { setLedRainbow(); return; }             // rainbow = corrections stalled
  bool ggaFresh = (now - g_lastGga) < GGA_STALE_MS;
  if (!ggaFresh) { setLed(0x20, 0x20, 0x20); return; }      // dim white = no GGA yet
  switch (g_fixQ) {
    case FIX_RTK_FIXED: setLed(0, 0x40, 0);    break;       // green
    case FIX_RTK_FLOAT: setLed(0, 0x30, 0x30); break;       // cyan
    default:            setLed(0x40, 0x18, 0); break;       // orange = corrections, not fixed
  }
}

// ---------- GGA parse ----------
// Pull the fix-quality field (index 6) out of a completed $..GGA sentence.
int ggaQuality(const char* line) {
  int commas = 0;
  const char* p = line;
  while (*p) {
    if (*p == ',') {
      if (++commas == 6) {
        p++;
        if (*p == ',' || *p == 0) return FIX_NONE;   // empty field = no fix
        return atoi(p);
      }
    }
    p++;
  }
  return FIX_NONE;
}

// Feed one byte from COM1; on a complete GGA line, update the fix state.
void feedNmea(char c) {
  if (c == '$') { nmeaLen = 0; nmea[nmeaLen++] = c; return; }
  if (c == '\r' || c == '\n') {
    if (nmeaLen > 6) {
      nmea[nmeaLen] = 0;
      if (strstr(nmea, "GGA,")) { g_fixQ = ggaQuality(nmea); g_lastGga = millis(); }
    }
    nmeaLen = 0;
    return;
  }
  if (nmeaLen < sizeof(nmea) - 1) nmea[nmeaLen++] = c;
  else nmeaLen = 0;   // overrun -> resync on next '$'
}

// ---------- WiFi (WiFiManager portal + custom NTRIP params) ----------
bool g_shouldSave = false;
void onSaveParams() { g_shouldSave = true; }

void setupWiFi() {
  WiFiManager wm;

  char portStr[8]; snprintf(portStr, sizeof(portStr), "%d", cfg.port);
  WiFiManagerParameter p_host ("host",  "NTRIP host",  cfg.host,  sizeof(cfg.host) - 1);
  WiFiManagerParameter p_port ("port",  "NTRIP port",  portStr,   7);
  WiFiManagerParameter p_mount("mount", "Mountpoint",  cfg.mount, sizeof(cfg.mount) - 1);
  WiFiManagerParameter p_user ("user",  "User (opt)",  cfg.user,  sizeof(cfg.user) - 1);
  WiFiManagerParameter p_pass ("pass",  "Pass (opt)",  cfg.pass,  sizeof(cfg.pass) - 1);
  wm.addParameter(&p_host); wm.addParameter(&p_port); wm.addParameter(&p_mount);
  wm.addParameter(&p_user); wm.addParameter(&p_pass);
  wm.setSaveParamsCallback(onSaveParams);
  wm.setConfigPortalTimeout(180);   // don't sit in the portal forever in the field

  bool hold = false;
  Serial.println("Hold button for WiFi/NTRIP config portal...");
  for (int i = 0; i < 200; i++) { M5.update(); if (M5.BtnA.isPressed()) { hold = true; break; } delay(10); }

  if (hold) { setLed(0, 0, 0x40); wm.startConfigPortal("NTRIP-Client"); }   // blue
  else      { setLed(0x40, 0x40, 0); wm.autoConnect("NTRIP-Client"); }      // yellow

  if (g_shouldSave) {
    strlcpy(cfg.host,  p_host.getValue(),  sizeof(cfg.host));
    cfg.port = atoi(p_port.getValue());
    if (cfg.port <= 0) cfg.port = DEF_PORT;
    strlcpy(cfg.mount, p_mount.getValue(), sizeof(cfg.mount));
    strlcpy(cfg.user,  p_user.getValue(),  sizeof(cfg.user));
    strlcpy(cfg.pass,  p_pass.getValue(),  sizeof(cfg.pass));
    saveConfig();
    Serial.printf("Saved NTRIP config: %s:%d/%s\n", cfg.host, cfg.port, cfg.mount);
  }
}

// Try to (re)open the NTRIP stream. Non-fatal on failure — caller retries.
bool ntripConnect() {
  int port = cfg.port;
  Serial.printf("NTRIP connect %s:%d/%s ...\n", cfg.host, cfg.port, cfg.mount);
  bool ok = ntrip_c.reqRaw(cfg.host, port, cfg.mount, cfg.user, cfg.pass);
  Serial.println(ok ? "NTRIP connected" : "NTRIP connect failed");
  if (ok) lastRtcm = millis();
  return ok;
}

// ---------- mosaic self-provisioning (Septentrio command channel on COM1) ----------
// Send one ASCII command (CR-terminated) and scan the reply for the ack. The
// receiver prefixes a good reply with "$R:" and a rejected one with "$R?".
bool sendMosaicCmd(const char* cmd, uint32_t timeout_ms) {
  while (Serial2.available()) Serial2.read();     // flush stale bytes
  Serial2.print(cmd); Serial2.print("\r\n");
  char buf[256]; size_t n = 0; unsigned long t0 = millis();
  while (millis() - t0 < timeout_ms) {
    while (Serial2.available()) { char ch = Serial2.read(); if (n < sizeof(buf) - 1) buf[n++] = ch; }
    buf[n] = 0;
    if (strstr(buf, "$R:")) return true;          // acked
    if (strstr(buf, "$R?")) return false;         // rejected
    delay(5);
  }
  return false;                                   // silent (timeout)
}

// Retry a command until it acks or the shared deadline passes. Best-effort: a
// no-ack is logged and we press on — the RTCM3 forwarder still runs.
bool provisionOne(const char* cmd, unsigned long deadline) {
  while (millis() < deadline) {
    if (sendMosaicCmd(cmd, 800)) { Serial.printf("  OK  %s\n", cmd); return true; }
    delay(300);
  }
  Serial.printf("  no-ack %s (continuing)\n", cmd);
  return false;
}

void provisionMosaic() {
#if PROVISION_MOSAIC
  setLed(0x30, 0x10, 0);                          // amber = provisioning
  unsigned long deadline = millis() + 25000;      // the receiver may still be cold-booting
  Serial.println("Provisioning mosaic (COM1)...");
  bool alive = false;
  while (millis() < deadline) { if (sendMosaicCmd("getReceiverCapabilities", 800)) { alive = true; break; } delay(400); }
  if (!alive) { Serial.println("mosaic silent — skipping (check wiring/baud; forwarder still runs)"); return; }

  char cmd[80];
  // COM2 -> tractor: set baud, then output NMEA GGA at 1 Hz.
  snprintf(cmd, sizeof(cmd), "setCOMSettings, COM2, baud%d", TRACTOR_NMEA_BAUD);
  provisionOne(cmd, deadline);
  provisionOne("setNMEAOutput, Stream1, COM2, GGA, sec1", deadline);
  // COM1 -> Atom: NMEA GGA for the fix-status LED. Last, since it starts GGA on
  // our command channel. RTCM3 corrections we feed to COM1 are auto-used by
  // default (no explicit input command needed — verified on this P3H over USB).
  provisionOne("setNMEAOutput, Stream2, COM1, GGA, sec1", deadline);
  Serial.println("mosaic provisioning done");
#endif
}

void setup() {
  auto c = M5.config();
  M5.begin(c);
  Serial.begin(115200);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  Serial2.begin(MOSAIC_COM1_BAUD, SERIAL_8N1, MOSAIC_RX_PIN, MOSAIC_TX_PIN);

  loadConfig();
  setLed(0x40, 0, 0);        // red = starting
  provisionMosaic();         // self-configure the receiver (out-of-the-box)
  setupWiFi();               // blocks until WiFi up (or portal)
  lastRtcm = millis();
}

void loop() {
  M5.update();
  unsigned long now = millis();

  // --- WiFi: reconnect in place; reboot only as a last resort ---
  bool wifiUp = (WiFi.status() == WL_CONNECTED);
  if (!wifiUp) {
    if (wifiDownSince == 0) { wifiDownSince = now; WiFi.reconnect(); Serial.println("WiFi down, reconnecting..."); }
    if (now - wifiDownSince > WIFI_LASTRESORT_MS) { Serial.println("WiFi down too long, reboot"); ESP.restart(); }
    updateLed(false, false, false);
    delay(50);
    return;
  }
  wifiDownSince = 0;

  // --- NTRIP: reconnect in place with backoff ---
  bool ntripUp = ntrip_c.connected();
  if (!ntripUp) {
    if (now - lastNtripTry > NTRIP_RETRY_MS) { lastNtripTry = now; ntrip_c.stop(); ntripConnect(); }
    updateLed(true, false, false);
    delay(50);
    return;
  }

  // --- Forward RTCM3 to mosaic COM1 ---
  while (ntrip_c.available()) { Serial2.write((uint8_t)ntrip_c.read()); totalBytes++; }
  if (totalBytes > lastBytes) { lastBytes = totalBytes; lastRtcm = now; }

  // --- Read GGA back from COM1 for the fix-status LED ---
  while (Serial2.available()) feedNmea((char)Serial2.read());

  bool rtcmStalled = (now - lastRtcm > RTCM_STALL_MS);
  updateLed(true, true, rtcmStalled);

  // --- 1 Hz status line ---
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 1000) {
    const char* q = (g_fixQ==FIX_RTK_FIXED?"RTK-FIX":g_fixQ==FIX_RTK_FLOAT?"RTK-FLT":
                     g_fixQ==FIX_DGPS?"DGPS":g_fixQ==FIX_GPS?"GPS":"NONE");
    bool ggaFresh = (now - g_lastGga) < GGA_STALE_MS;
    Serial.printf("RTCM %llu B | fix=%s%s\n", totalBytes, q, ggaFresh ? "" : " (stale)");
    lastPrint = now;
  }

  delay(5);
}
