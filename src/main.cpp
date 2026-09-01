/*
 *  ntripclient_G5-P3H — M5Atom Lite NTRIP client for a Septentrio mosaic-go G5 P3H rover.
 *
 *  Data flow (rover):
 *    caster (NTRIP, configurable — default rtk.toiso.fit:2101/eniwa-bd982, anon)
 *        │  RTCM3 over WiFi
 *        ▼
 *    M5Atom Lite  ── Grove UART G26/G32 (3.3V TTL) ──►  mosaic-go COM1  (RTCM3 IN → RTK)
 *                 ◄── NMEA GGA(+GSA+GSV) ─────────────  mosaic-go COM1  (LED + web monitor)
 *
 *    mosaic-go COM2 ──► NMEA GGA @38400 ──► tractor guidance   (mosaic emits directly)
 *
 *  Finished behaviours:
 *    1. Fix-status LED  — reads GGA back on COM1 and colours the LED by RTK quality.
 *    2. Field config    — NTRIP host/port/mount/user/pass set in the WiFiManager
 *                         portal, saved to NVS (Preferences). No reflash to move sites.
 *    3. No-reboot reconnect — WiFi / NTRIP drops retry in place; a reboot is only the
 *                         last resort after a long outage.
 *    4. Web monitor     — LCD-less status + skyplot + C/N0 bars at ntrip-rover.local,
 *                         plus session stats (throughput, DOP, drops, TTFF, fix %).
 *                         Skyplot/DOP need GGA+GSA+GSV on COM1 (see README).
 *
 *  Wiring (Grove HY2.0, both sides 3.3V LVTTL — no level shifter on this leg):
 *    Atom G26 (TX) ──► mosaic COM1 RX      Atom G32 (RX) ◄── mosaic COM1 TX      GND──GND
 *  If no fix / no GGA, suspect G26/G32 swapped.
 *
 *  mosaic-go setup: best-effort self-provisioning on boot. If the receiver is
 *  already streaming GGA (saved boot config), the Atom just forwards. Otherwise it
 *  tries the Septentrio command channel on COM1 (setNMEAOutput COM1 GGA+GSA+GSV for
 *  the LED + skyplot, and COM2 @ configured baud for the tractor) in the quiet window.
 *  NOTE(hardware): the user's P3H is RTCMv3-only on COM1 and IGNORES ASCII commands
 *  on the shared RTCM+GGA UART — so on that unit self-provisioning no-acks and it
 *  runs off a boot config saved via USB/RxTools (see README). Assumes COM1 @115200.
 *  RTCM3 fed to COM1 is auto-used as corrections. Set PROVISION_MOSAIC 0 to disable.
 *
 *  LED: red=starting, blue=portal, yellow=WiFi (re)connecting, amber=provisioning,
 *       magenta=NTRIP connecting, rainbow=corrections stalled, dim-white=no GGA yet,
 *       ORANGE=corrections flowing but not fixed, CYAN=RTK float, GREEN=RTK fixed.
 */
#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <FastLED.h>
#include <Preferences.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "NTRIPClient.h"

NTRIPClient ntrip_c;
Preferences prefs;
WebServer server(80);
#define MDNS_NAME "ntrip-rover"   // reachable as http://ntrip-rover.local/

// M5Atom Lite onboard WS2812 LED
#define LED_PIN 27
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// ---- mosaic COM1 link on the Atom's Grove UART (3.3V TTL) ----
#define MOSAIC_TX_PIN 26          // Atom G26 = Serial2 TX -> mosaic COM1 RX
#define MOSAIC_RX_PIN 32          // Atom G32 = Serial2 RX <- mosaic COM1 TX (GGA back)
#define MOSAIC_COM1_BAUD 115200   // mosaic COM1 default; the command channel too

// Self-provision the mosaic so a factory receiver works out of the box (no
// manual RxTools/web-UI step). Set 0 if you pre-configured + saved boot config.
#define PROVISION_MOSAIC 1

// ---- Runtime config (NVS-backed, editable in the portal) ----
struct Config {
  char host[64];
  int  port;
  char mount[48];
  char user[32];
  char pass[32];
  int  com2_baud;       // mosaic COM2 -> tractor: baud
  char com2_nmea[32];   // mosaic COM2 -> tractor: NMEA sentences, e.g. "GGA" / "GGA+VTG"
} cfg;

// Factory defaults (used until the portal/web overwrites them).
static const char* DEF_HOST  = "rtk.toiso.fit";
static const int   DEF_PORT  = 2101;
static const char* DEF_MOUNT = "eniwa-bd982";
static const char* DEF_USER  = "";      // anonymous
static const char* DEF_PASS  = "";
static const int   DEF_COM2_BAUD = 38400;
static const char* DEF_COM2_NMEA = "GGA";

void loadConfig() {
  // Open read-write and seed the defaults on a fresh device, so the keys exist.
  // Otherwise the Arduino Preferences layer logs an [E] for every missing key
  // (it still returns the default, but the noise looks like a fault on boot).
  prefs.begin("ntrip", false);
  // Seed any missing key individually (so keys added in later firmware versions
  // get created on an already-provisioned device too — avoids [E] NOT_FOUND logs).
  if (!prefs.isKey("host"))   prefs.putString("host", DEF_HOST);
  if (!prefs.isKey("port"))   prefs.putInt("port", DEF_PORT);
  if (!prefs.isKey("mount"))  prefs.putString("mount", DEF_MOUNT);
  if (!prefs.isKey("user"))   prefs.putString("user", DEF_USER);
  if (!prefs.isKey("pass"))   prefs.putString("pass", DEF_PASS);
  if (!prefs.isKey("c2baud")) prefs.putInt("c2baud", DEF_COM2_BAUD);
  if (!prefs.isKey("c2nmea")) prefs.putString("c2nmea", DEF_COM2_NMEA);
  String h = prefs.getString("host", DEF_HOST);
  cfg.port = prefs.getInt("port", DEF_PORT);
  String m = prefs.getString("mount", DEF_MOUNT);
  String u = prefs.getString("user", DEF_USER);
  String p = prefs.getString("pass", DEF_PASS);
  cfg.com2_baud = prefs.getInt("c2baud", DEF_COM2_BAUD);
  String n = prefs.getString("c2nmea", DEF_COM2_NMEA);
  prefs.end();
  strlcpy(cfg.host, h.c_str(), sizeof(cfg.host));
  strlcpy(cfg.mount, m.c_str(), sizeof(cfg.mount));
  strlcpy(cfg.user, u.c_str(), sizeof(cfg.user));
  strlcpy(cfg.pass, p.c_str(), sizeof(cfg.pass));
  strlcpy(cfg.com2_nmea, n.c_str(), sizeof(cfg.com2_nmea));
}

void saveConfig() {
  prefs.begin("ntrip", false);          // read-write
  prefs.putString("host", cfg.host);
  prefs.putInt("port", cfg.port);
  prefs.putString("mount", cfg.mount);
  prefs.putString("user", cfg.user);
  prefs.putString("pass", cfg.pass);
  prefs.putInt("c2baud", cfg.com2_baud);
  prefs.putString("c2nmea", cfg.com2_nmea);
  prefs.end();
}

// ---- Fix state (from GGA readback) ----
enum FixQ { FIX_NONE = 0, FIX_GPS = 1, FIX_DGPS = 2, FIX_RTK_FIXED = 4, FIX_RTK_FLOAT = 5 };
const char* fixStr(int q) {
  switch (q) { case FIX_RTK_FIXED: return "RTK-FIX"; case FIX_RTK_FLOAT: return "RTK-FLT";
               case FIX_DGPS: return "DGPS";      case FIX_GPS: return "GPS";      default: return "NONE"; }
}
volatile int  g_fixQ = FIX_NONE;
unsigned long g_lastGga = 0;
char nmea[100];
uint8_t nmeaLen = 0;

// ---- Extra GGA fields (all single-threaded, read/written only in loop context) ----
int    g_sats    = 0;     // GGA field 7: satellites used in the solution
float  g_hdop    = 0;     // GGA field 8: horizontal dilution of precision
float  g_alt     = 0;     // GGA field 9: antenna altitude (m)
float  g_corrAge = -1;    // GGA field 13: age of differential corrections (s); -1 = none
double g_lat     = 0;     // GGA field 2/3: latitude (decimal deg, +N)
double g_lon     = 0;     // GGA field 4/5: longitude (decimal deg, +E)
char   g_utc[12] = "";    // GGA field 1: UTC "hh:mm:ss"
float  g_pdop    = 0;     // GSA field 15: position DOP
float  g_vdop    = 0;     // GSA field 17: vertical DOP

// ---- Satellites in view (from GSV) — for the skyplot + C/N0 bars ----
// GGA gives no per-sat data, so the skyplot needs GSV enabled on COM1. GSV arrives
// as a once-a-second burst of sentences (one group per constellation). We accumulate
// into a temp buffer and commit it to the display buffer when the burst goes quiet.
struct Sat {
  char    sys;    // constellation: G R E J C I S (from the GSV talker id)
  uint8_t prn;    // satellite number within the constellation
  int16_t elev;   // elevation, deg (0..90)
  int16_t azim;   // azimuth,   deg (0..359, from true north)
  uint8_t snr;    // C/N0, dB-Hz (0 = tracked but no signal reported)
};
#define MAX_SATS 72   // open-sky multi-constellation (G+R+E+C+J+I+S) can top 60 in view
Sat g_satView[MAX_SATS]; int g_satCount = 0;   // committed (displayed) set
Sat g_satTmp [MAX_SATS]; int g_satTmpN = 0;    // accumulating the current epoch

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

// ---- Session stats (for field monitoring — how healthy has the link been?) ----
uint32_t g_rtcmBps       = 0;   // rolling RTCM throughput, bytes/sec
uint64_t g_rateLastBytes = 0;   // totalBytes at last rate sample
uint32_t g_ntripDrops    = 0;   // NTRIP connection lost (connected -> down) count
uint32_t g_wifiDrops     = 0;   // WiFi connection lost count
unsigned long g_firstFixMs  = 0;   // millis() of first RTK-FIX (0 = never fixed yet)
unsigned long g_fixAccumMs  = 0;   // cumulative time held in RTK-FIX
unsigned long g_lastFixSample = 0; // last loop timestamp, for fix-time accounting

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
  bool ggaFresh = g_lastGga && (now - g_lastGga) < GGA_STALE_MS;
  if (!ggaFresh) { setLed(0x20, 0x20, 0x20); return; }      // dim white = no GGA yet
  switch (g_fixQ) {
    case FIX_RTK_FIXED: setLed(0, 0x40, 0);    break;       // green
    case FIX_RTK_FLOAT: setLed(0, 0x30, 0x30); break;       // cyan
    default:            setLed(0x40, 0x18, 0); break;       // orange = corrections, not fixed
  }
}

// ---------- GGA parse ----------
// Copy comma-separated field n (0-based) of an NMEA line into out (empty if absent).
// Stops at the next comma or the '*' checksum delimiter.
static void ggaField(const char* line, int n, char* out, size_t outsz) {
  const char* p = line;
  for (int f = 0; f < n && *p; p++) if (*p == ',') f++;   // skip past n commas
  size_t j = 0;
  while (*p && *p != ',' && *p != '*' && j < outsz - 1) out[j++] = *p++;
  out[j] = 0;
}

// ddmm.mmmm (NMEA lat/lon) + hemisphere -> signed decimal degrees.
static double nmeaToDeg(const char* dm, char hemi) {
  if (!dm[0]) return 0;
  double v = atof(dm);
  int    d = (int)(v / 100);
  double deg = d + (v - d * 100) / 60.0;
  return (hemi == 'S' || hemi == 'W') ? -deg : deg;
}

// Pull fix quality + the useful diagnostic fields out of a completed $..GGA line.
//   $..GGA,time,lat,N,lon,E,quality,numSV,HDOP,alt,M,sep,M,age,refID*cs
//     field  1   2  3  4  5    6      7    8    9              13
void parseGga(const char* line) {
  char f[16], h[4], g[16];
  ggaField(line, 6,  f, sizeof f); g_fixQ    = f[0] ? atoi(f) : FIX_NONE;
  ggaField(line, 7,  f, sizeof f); g_sats    = f[0] ? atoi(f) : 0;
  ggaField(line, 8,  f, sizeof f); g_hdop    = f[0] ? atof(f) : 0;
  ggaField(line, 9,  f, sizeof f); g_alt     = f[0] ? atof(f) : 0;
  ggaField(line, 13, f, sizeof f); g_corrAge = f[0] ? atof(f) : -1;   // -1 = no corrections
  ggaField(line, 2, f, sizeof f); ggaField(line, 3, h, sizeof h); g_lat = nmeaToDeg(f, h[0]);
  ggaField(line, 4, f, sizeof f); ggaField(line, 5, h, sizeof h); g_lon = nmeaToDeg(f, h[0]);
  ggaField(line, 1, g, sizeof g);   // UTC hhmmss(.ss) -> hh:mm:ss
  if (strlen(g) >= 6) snprintf(g_utc, sizeof g_utc, "%c%c:%c%c:%c%c", g[0],g[1],g[2],g[3],g[4],g[5]);
}

// GSA: dilution of precision. $..GSA,mode,fix,sv1..sv12,PDOP,HDOP,VDOP*cs
void parseGsa(const char* line) {
  char f[16];
  ggaField(line, 15, f, sizeof f); if (f[0]) g_pdop = atof(f);
  ggaField(line, 17, f, sizeof f); if (f[0]) g_vdop = atof(f);
}

// GSV: satellites in view. $xxGSV,numMsg,msgNum,numSats,{prn,elev,azim,snr}x(<=4)*cs
// The talker (xx) names the constellation. Accumulate into the temp buffer; loop()
// commits it to the display buffer once the once-a-second burst goes quiet.
void parseGsv(const char* line) {
  char sys = 'G';                                   // GP=G GL=R GA=E GB=C GQ=J GI=I
  if (line[1] == 'G') switch (line[2]) {
    case 'L': sys='R'; break; case 'A': sys='E'; break; case 'B': sys='C'; break;
    case 'Q': sys='J'; break; case 'I': sys='I'; break; case 'S': sys='S'; break;
    default:  sys='G'; break;
  }
  char f[16];
  for (int grp = 0; grp < 4; grp++) {               // NMEA caps GSV at 4 sats/sentence
    int base = 4 + grp * 4;
    ggaField(line, base, f, sizeof f);
    if (!f[0] || g_satTmpN >= MAX_SATS) break;
    Sat s; s.sys = sys; s.prn = atoi(f);
    ggaField(line, base + 1, f, sizeof f); s.elev = f[0] ? atoi(f) : 0;
    ggaField(line, base + 2, f, sizeof f); s.azim = f[0] ? atoi(f) : 0;
    ggaField(line, base + 3, f, sizeof f); s.snr  = f[0] ? atoi(f) : 0;
    if (s.prn) g_satTmp[g_satTmpN++] = s;
  }
}

// Publish the accumulated GSV set to the display buffer. Called on each GGA (the
// once-a-second epoch marker) — robust whether the receiver bursts its GSV
// sentences or spreads them across the second (this mosaic spreads them, so a
// gap-based "burst end" mis-fires between groups and keeps only a fragment).
void commitSats() {
  if (!g_satTmpN) return;               // no GSV this epoch — keep the last view
  memcpy(g_satView, g_satTmp, g_satTmpN * sizeof(Sat));
  g_satCount = g_satTmpN;
  g_satTmpN  = 0;
}

// Feed one byte from COM1; on a complete GGA line, update the fix state.
void feedNmea(char c) {
  if (c == '$') { nmeaLen = 0; nmea[nmeaLen++] = c; return; }
  if (c == '\r' || c == '\n') {
    if (nmeaLen > 6) {
      nmea[nmeaLen] = 0;
      if      (strstr(nmea, "GGA,")) { parseGga(nmea); g_lastGga = millis(); commitSats(); }
      else if (strstr(nmea, "GSV,")) parseGsv(nmea);
      else if (strstr(nmea, "GSA,")) parseGsa(nmea);
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
  WiFi.persistent(false);           // avoid NVS thrash / connect-state races (0x3014)
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
  wm.setConfigPortalTimeout(180);      // don't sit in the portal forever in the field
  wm.setCaptivePortalEnable(true);     // captive portal: DNS wildcard + redirect so the
                                       // phone's OS auto-opens the page (iOS CNA / Android)
  wm.setEnableConfigPortal(false);     // autoConnect tries saved creds ONLY; we open the
                                       // portal ourselves below, on a clean Wi-Fi state
  wm.setAPCallback([](WiFiManager*){
    Serial.printf("config portal up — join SSID 'NTRIP-Client', page auto-opens (http://%s/)\n",
                  WiFi.softAPIP().toString().c_str());
  });

  bool hold = false;
  Serial.println("Hold button for WiFi/NTRIP config portal...");
  for (int i = 0; i < 200; i++) { M5.update(); if (M5.BtnA.isPressed()) { hold = true; break; } delay(10); }

  bool connected = false;
  if (!hold) {
    setLed(0x40, 0x40, 0);                        // yellow = trying saved WiFi
    connected = wm.autoConnect("NTRIP-Client");   // saved creds only (no auto-portal)
  }
  if (!connected) {
    // Button held, or no/unreachable saved WiFi. Open the portal on a CLEAN Wi-Fi
    // state: autoConnect's failed STA attempt can wedge the stack (0x3014
    // ESP_ERR_WIFI_STOP_STATE), which breaks the softAP's DNS — then the captive
    // redirect never fires and the phone never auto-opens the page. Full off->on
    // cycle clears it so DNS + the captive portal come up reliably.
    WiFi.disconnect(false, false);
    WiFi.mode(WIFI_OFF); delay(200);
    WiFi.mode(WIFI_STA); delay(150);
    setLed(0, 0, 0x40);                           // blue = config portal
    Serial.println("opening captive config portal — join SSID 'NTRIP-Client'");
    connected = wm.startConfigPortal("NTRIP-Client");
  }

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
char g_dbgReply[160];   // last raw reply (printable), for provisioning diagnosis
static void snapReply(const char* buf, size_t n) {
  size_t j = 0;
  for (size_t i = 0; i < n && j < sizeof(g_dbgReply) - 1; i++) {
    char c = buf[i];
    g_dbgReply[j++] = (c >= 32 && c < 127) ? c : (c == '\r' || c == '\n') ? '/' : '.';
  }
  g_dbgReply[j] = 0;
}

bool sendMosaicCmd(const char* cmd, uint32_t timeout_ms) {
  while (Serial2.available()) Serial2.read();     // flush stale bytes
  Serial2.print(cmd); Serial2.print("\r\n");
  char buf[256]; size_t n = 0; unsigned long t0 = millis();
  bool ret = false;
  while (millis() - t0 < timeout_ms) {
    while (Serial2.available()) { char ch = Serial2.read(); if (n < sizeof(buf) - 1) buf[n++] = ch; }
    buf[n] = 0;
    if (strstr(buf, "$R:")) { ret = true;  break; }   // acked
    if (strstr(buf, "$R?")) { ret = false; break; }   // rejected
    delay(5);
  }
  snapReply(buf, n);                                  // stash for diagnosis
  return ret;
}

// Non-blocking provisioning state machine. The tricky part (learned on hardware):
//  - Provision TOO EARLY (0-25 s) and the receiver is still cold-booting -> no ack.
//  - Provision TOO LATE, after RTK is streaming, and setNMEAOutput is IGNORED
//    (the mosaic only applies output config in the "quiet window" before it streams).
// So we HOLD RTCM3 forwarding until provisioning resolves (keeping the receiver out
// of RTK = quiet window open), and keep polling for it to come alive in the loop.
// WiFi/web/NTRIP still come up immediately; only the byte-forward waits.
enum ProvState { PROV_WAIT, PROV_DONE, PROV_GAVEUP };
ProvState     g_prov = PROV_WAIT;
unsigned long g_provDeadline = 0;   // set in setup()
unsigned long g_lastProvTry  = 0;
bool          g_forceProv    = false;   // web COM2 change: push commands even if GGA present

// Retry one command until it acks or a short deadline passes (only runs once the
// receiver is confirmed alive, so this blocks for at most a couple of seconds).
bool provisionOne(const char* cmd, unsigned long deadline) {
  while (millis() < deadline) {
    if (sendMosaicCmd(cmd, 800)) { Serial.printf("  OK  %s\n", cmd); return true; }
    delay(200);
  }
  Serial.printf("  no-ack %s (continuing)\n", cmd);
  return false;
}

// Apply the receiver config from cfg (COM2 -> tractor, COM1 GGA -> our LED).
void applyProvision() {
  char cmd[96];
  snprintf(cmd, sizeof(cmd), "setCOMSettings, COM2, baud%d", cfg.com2_baud);
  provisionOne(cmd, millis() + 6000);
  snprintf(cmd, sizeof(cmd), "setNMEAOutput, Stream1, COM2, %s, sec1", cfg.com2_nmea);
  provisionOne(cmd, millis() + 6000);
  // COM1 GGA (fix-status LED) + GSA/GSV (DOP + skyplot). Last, since it starts NMEA
  // on our command channel. RTCM3 fed to COM1 is auto-used as corrections (no cmd).
  provisionOne("setNMEAOutput, Stream2, COM1, GGA+GSA+GSV, sec1", millis() + 6000);
}

// Called every loop while PROV_WAIT. Polls the receiver; on the first ack it
// applies the config and transitions to PROV_DONE. Times out to PROV_GAVEUP.
void provisionTick(unsigned long now) {
#if PROVISION_MOSAIC
  if (g_prov != PROV_WAIT) return;

  // Shortcut: if the receiver is already streaming NMEA (GGA fresh) it's already
  // configured (saved boot config) — no provisioning needed, start forwarding.
  // This is also the ONLY path for a receiver whose COM1 input is set to
  // RTCMv3-only and therefore silently ignores ASCII commands (observed on the
  // user's P3H): commands can't reconfigure it, but its saved config already works.
  if (!g_forceProv && g_lastGga && (now - g_lastGga) < GGA_STALE_MS) {
    g_prov = PROV_DONE;   // g_lastGga!=0 -> a real GGA arrived (not the 0-init in the
    Serial.println("provision: GGA present — receiver already configured, forwarding");
    return;               // first GGA_STALE_MS ms, which used to false-positive here)
  }

  if (now > g_provDeadline) {
    g_prov = PROV_GAVEUP; g_forceProv = false;
    Serial.printf("provision: gave up (COM1 not answering commands; last=[%s]) — forwarding\n", g_dbgReply);
    return;
  }
  if (now - g_lastProvTry < 1500) return;         // poll ~every 1.5 s
  g_lastProvTry = now;
  // Try the command interface (a factory receiver's COM1 accepts commands; we're
  // in the quiet window since forwarding is held while PROV_WAIT).
  if (!sendMosaicCmd("getReceiverCapabilities", 600)) return;
  Serial.println("provision: command interface alive — applying config");
  applyProvision();
  g_prov = PROV_DONE; g_forceProv = false;
#else
  g_prov = PROV_GAVEUP;
#endif
}

// ---------- runtime web UI (STA) — configure + monitor without an LCD ----------
// Reachable at http://ntrip-rover.local/ (or the device IP) once on WiFi.
// WiFi creds stay with WiFiManager (the boot portal); this only edits NTRIP +
// shows live status. Config changes apply hot (NTRIP restarts, no reboot).
static const char PAGE[] PROGMEM = R"HTML(
<!doctype html><meta name=viewport content="width=device-width,initial-scale=1">
<title>NTRIP rover</title>
<style>body{font-family:system-ui,sans-serif;margin:1.2rem;max-width:30rem}
h1{font-size:1.2rem}h2{font-size:.95rem;margin:.9rem 0 .2rem;color:#555}
label{display:block;margin:.5rem 0 .1rem}
input{width:100%;padding:.4rem;box-sizing:border-box}
button{margin-top:.8rem;padding:.5rem 1rem}
#s{background:#f4f4f4;padding:.6rem;border-radius:.4rem;margin:.6rem 0;font-family:monospace;white-space:pre-wrap}
.r{color:#b00}.g{color:#080}
#sky{width:100%;max-width:20rem;display:block;margin:0 auto}
#cnowrap{overflow-x:auto}#cno{height:150px;min-width:100%}
#leg{font-size:.75rem;margin:.2rem 0}#leg b{padding:0 .25rem;border-radius:.2rem;color:#fff}
.hint{color:#999;font-size:.8rem;text-align:center;padding:.5rem}</style>
<h1>NTRIP rover (G5-P3H)</h1>
<div id=s>loading…</div>
<h2>Skyplot</h2>
<svg id=sky viewBox="0 0 200 200"></svg>
<h2>Signal C/N0 (dB-Hz)</h2>
<div id=cnowrap><svg id=cno viewBox="0 0 100 150" preserveAspectRatio=xMinYMin></svg></div>
<div id=leg></div>
<h2>NTRIP config</h2>
<form method=POST action=/save>
<label>NTRIP host<input name=host></label>
<label>Port<input name=port></label>
<label>Mountpoint<input name=mount></label>
<label>User (optional)<input name=user></label>
<label>Pass (optional)<input name=pass></label>
<button>Save &amp; apply</button></form>
<!-- COM2 (tractor) baud/NMEA are set on the receiver via RxTools/USB and saved to
     boot config — not editable here (this P3H doesn't answer commands on the shared
     RTCM+GGA UART). See README "受信機を USB で設定する". -->
<form method=POST action=/reboot style=display:inline><button>Reboot</button></form>
<form method=POST action=/wifireset style=display:inline><button>Forget WiFi</button></form>
<script>
const COL={G:'#2a2',R:'#f90',E:'#c2c',J:'#26f',C:'#d22',I:'#088',S:'#888'};
const NAME={G:'GPS',R:'GLO',E:'GAL',J:'QZS',C:'BDS',I:'NAV',S:'SBAS'};
async function u(){let d=await(await fetch('/status.json')).json();
document.getElementById('s').innerHTML=
'WiFi  '+d.wifi.ssid+'  '+d.wifi.ip+'  '+d.wifi.rssi+'dBm\n'+
'NTRIP '+(d.ntrip.connected?'<span class=g>connected</span>':'<span class=r>down</span>')+
'  '+d.ntrip.host+':'+d.ntrip.port+'/'+d.ntrip.mount+'\n'+
'RTCM  '+d.ntrip.bytes+' B  '+d.ntrip.bps+' B/s'+(d.ntrip.stalled?'  <span class=r>STALLED</span>':'')+'\n'+
'Fix   '+(d.fix.q=='RTK-FIX'?'<span class=g>':'')+d.fix.q+(d.fix.q=='RTK-FIX'?'</span>':'')+
  (d.fix.fresh?'':'  (stale)')+'  sats '+d.fix.sats+'/'+d.fix.nview+
  '  age '+(d.fix.age<0?'—':d.fix.age+'s')+'\n'+
'DOP   H'+d.fix.hdop+'  P'+d.fix.pdop+'  V'+d.fix.vdop+'  alt '+d.fix.alt+'m\n'+
'Pos   '+d.pos.lat.toFixed(7)+', '+d.pos.lon.toFixed(7)+'  '+(d.pos.utc||'--:--:--')+' UTC\n'+
'Stats drops N'+d.stats.ntrip_drops+'/W'+d.stats.wifi_drops+
  '  fix '+d.stats.fix_pct+'%'+(d.stats.ttff<0?'  (no fix yet)':'  TTFF '+d.stats.ttff+'s')+'\n'+
'Prov  '+(d.prov=='done'?'<span class=g>done</span>':d.prov=='wait'?'<span class=r>waiting…</span>':'gave up')+
'\nUp    '+d.up+' s';
for(let k of ['host','port','mount','user','pass'])
  {let e=document.querySelector('[name='+k+']');if(e&&!e.dataset.t)e.value=d.cfg[k];}}
function skyplot(sats){let s='';
for(let el of [0,30,60]){let r=90*(90-el)/90;
  s+='<circle cx=100 cy=100 r='+r+' fill=none stroke=#ddd/>';}
s+='<line x1=100 y1=10 x2=100 y2=190 stroke=#eee/><line x1=10 y1=100 x2=190 y2=100 stroke=#eee/>';
s+='<text x=100 y=8 font-size=9 text-anchor=middle fill=#999>N</text>';
s+='<text x=195 y=104 font-size=9 fill=#999>E</text>';
s+='<text x=100 y=200 font-size=9 text-anchor=middle fill=#999>S</text>';
s+='<text x=1 y=104 font-size=9 fill=#999>W</text>';
for(let t of sats){let r=90*(90-t.el)/90,a=t.az*Math.PI/180;
  let x=100+r*Math.sin(a),y=100-r*Math.cos(a),c=COL[t.sys]||'#888';
  s+='<circle cx='+x.toFixed(1)+' cy='+y.toFixed(1)+' r=7.5 fill="'+c+'"/>';
  s+='<text x='+x.toFixed(1)+' y='+(y+2.5).toFixed(1)+' font-size=6 fill=#fff text-anchor=middle>'+t.prn+'</text>';}
document.getElementById('sky').innerHTML=s;}
function cno(sats){let bs=sats.slice().sort((a,b)=>a.sys==b.sys?a.prn-b.prn:a.sys<b.sys?-1:1);
let W=Math.max(bs.length*14,100),H=150,base=120,cn=document.getElementById('cno');let b='';
for(let v of [20,30,40,50]){let y=base-v/55*base;
  b+='<line x1=16 y1='+y+' x2='+W+' y2='+y+' stroke=#eee/><text x=0 y='+(y+2)+' font-size=8 fill=#999>'+v+'</text>';}
bs.forEach((t,i)=>{let x=18+i*14,h=t.snr/55*base,y=base-h,c=COL[t.sys]||'#888';
  b+='<rect x='+x+' y='+y.toFixed(1)+' width=10 height='+Math.max(0,h).toFixed(1)+' fill="'+c+'"/>';
  b+='<text x='+(x+5)+' y='+(base+10)+' font-size=6 text-anchor=middle fill="'+c+'">'+t.prn+'</text>';
  if(t.snr)b+='<text x='+(x+5)+' y='+(y-1).toFixed(1)+' font-size=6 text-anchor=middle fill=#444>'+t.snr+'</text>';});
cn.setAttribute('viewBox','0 0 '+W+' '+H);cn.innerHTML=b;cn.style.minWidth=(W<300?'100%':W+'px');}
async function sky(){let d=await(await fetch('/sats.json')).json();let sats=d.sats||[];
if(!sats.length){document.getElementById('sky').innerHTML=
  '<text x=100 y=100 font-size=9 text-anchor=middle fill=#999>no GSV — enable GGA+GSV on COM1</text>';
  document.getElementById('cno').innerHTML='';document.getElementById('leg').innerHTML='';return;}
skyplot(sats);cno(sats);
let sysN={};for(let t of sats)sysN[t.sys]=(sysN[t.sys]||0)+1;
document.getElementById('leg').innerHTML=Object.keys(sysN).sort().map(k=>
  '<b style="background:'+(COL[k]||'#888')+'">'+(NAME[k]||k)+'</b> '+sysN[k]).join('  ');}
u();sky();setInterval(u,2000);setInterval(sky,2000);
document.querySelectorAll('input').forEach(e=>e.addEventListener('input',()=>e.dataset.t=1));
</script>)HTML";

void handleRoot()   { server.send_P(200, "text/html", PAGE); }

void handleStatus() {
  unsigned long now = millis();
  bool ggaFresh = g_lastGga && (now - g_lastGga) < GGA_STALE_MS;
  bool stalled  = ntrip_c.connected() && (now - lastRtcm > RTCM_STALL_MS);
  // Cumulative RTK-fix time as a percent of uptime (how solid has the link been).
  int fixPct = now ? (int)((g_fixAccumMs * 100) / now) : 0;
  int ttff   = g_firstFixMs ? (int)(g_firstFixMs / 1000) : -1;   // -1 = never fixed
  char buf[1024];
  snprintf(buf, sizeof(buf),
    "{\"wifi\":{\"ssid\":\"%s\",\"ip\":\"%s\",\"rssi\":%d},"
    "\"ntrip\":{\"host\":\"%s\",\"port\":%d,\"mount\":\"%s\",\"connected\":%s,"
    "\"bytes\":%llu,\"bps\":%lu,\"stalled\":%s},"
    "\"fix\":{\"q\":\"%s\",\"fresh\":%s,\"sats\":%d,\"nview\":%d,"
    "\"hdop\":%.1f,\"pdop\":%.1f,\"vdop\":%.1f,\"alt\":%.1f,\"age\":%.1f},"
    "\"pos\":{\"lat\":%.7f,\"lon\":%.7f,\"utc\":\"%s\"},"
    "\"stats\":{\"ntrip_drops\":%lu,\"wifi_drops\":%lu,\"ttff\":%d,\"fix_pct\":%d},"
    "\"cfg\":{\"host\":\"%s\",\"port\":%d,\"mount\":\"%s\",\"user\":\"%s\",\"pass\":\"%s\","
    "\"com2_baud\":%d,\"com2_nmea\":\"%s\"},"
    "\"prov\":\"%s\",\"up\":%lu}",
    WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI(),
    cfg.host, cfg.port, cfg.mount, ntrip_c.connected() ? "true" : "false",
    (unsigned long long)totalBytes, (unsigned long)g_rtcmBps, stalled ? "true" : "false",
    fixStr(g_fixQ), ggaFresh ? "true" : "false", g_sats, g_satCount,
    g_hdop, g_pdop, g_vdop, g_alt, g_corrAge,
    g_lat, g_lon, g_utc,
    (unsigned long)g_ntripDrops, (unsigned long)g_wifiDrops, ttff, fixPct,
    cfg.host, cfg.port, cfg.mount, cfg.user, cfg.pass, cfg.com2_baud, cfg.com2_nmea,
    g_prov==PROV_DONE?"done":g_prov==PROV_GAVEUP?"gaveup":"wait", now / 1000);
  server.send(200, "application/json", buf);
}

// Satellite list for the skyplot + C/N0 bars. Kept separate from /status.json so
// the (larger, once-a-second) sat array doesn't bloat the frequent status poll.
void handleSats() {
  static char buf[3072];               // 72 sats * ~34 chars < 3072
  int n = snprintf(buf, sizeof(buf), "{\"sats\":[");
  for (int i = 0; i < g_satCount && n < (int)sizeof(buf) - 64; i++) {
    const Sat& s = g_satView[i];
    n += snprintf(buf + n, sizeof(buf) - n, "%s{\"sys\":\"%c\",\"prn\":%d,\"el\":%d,\"az\":%d,\"snr\":%d}",
                  i ? "," : "", s.sys, s.prn, s.elev, s.azim, s.snr);
  }
  snprintf(buf + n, sizeof(buf) - n, "]}");
  server.send(200, "application/json", buf);
}

void handleSave() {
  int  oldBaud = cfg.com2_baud;
  char oldNmea[32]; strlcpy(oldNmea, cfg.com2_nmea, sizeof(oldNmea));

  if (server.hasArg("host"))  strlcpy(cfg.host,  server.arg("host").c_str(),  sizeof(cfg.host));
  if (server.hasArg("port"))  { int p = server.arg("port").toInt(); if (p > 0) cfg.port = p; }
  if (server.hasArg("mount")) strlcpy(cfg.mount, server.arg("mount").c_str(), sizeof(cfg.mount));
  if (server.hasArg("user"))  strlcpy(cfg.user,  server.arg("user").c_str(),  sizeof(cfg.user));
  if (server.hasArg("pass"))  strlcpy(cfg.pass,  server.arg("pass").c_str(),  sizeof(cfg.pass));
  if (server.hasArg("com2_baud")) { int b = server.arg("com2_baud").toInt(); if (b > 0) cfg.com2_baud = b; }
  if (server.hasArg("com2_nmea")) strlcpy(cfg.com2_nmea, server.arg("com2_nmea").c_str(), sizeof(cfg.com2_nmea));
  saveConfig();
  Serial.printf("web: saved NTRIP %s:%d/%s  COM2 %d/%s — applying\n",
                cfg.host, cfg.port, cfg.mount, cfg.com2_baud, cfg.com2_nmea);
  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "saved");

  ntrip_c.stop();          // loop reconnects NTRIP with the new config (hot apply)
  // COM2 change -> re-provision the receiver. Going back to PROV_WAIT pauses
  // forwarding, so the mosaic drops RTK and the quiet window reopens; provisionTick
  // then re-applies setCOMSettings/setNMEAOutput. Brief RTK blip during reconfig.
  if (cfg.com2_baud != oldBaud || strcmp(cfg.com2_nmea, oldNmea) != 0) {
    g_prov = PROV_WAIT; g_forceProv = true; g_provDeadline = millis() + 10000; g_lastProvTry = 0;
    Serial.println("web: COM2 changed — re-provisioning (needs a command-capable receiver)");
  }
}

void handleReboot()   { server.send(200, "text/plain", "rebooting"); delay(200); ESP.restart(); }
void handleWifiReset() {
  server.send(200, "text/plain", "forgetting WiFi, rebooting to portal");
  delay(200); WiFi.disconnect(true, true); delay(200); ESP.restart();
}

void startWebServer() {
  if (MDNS.begin(MDNS_NAME)) { MDNS.addService("http", "tcp", 80);
    Serial.printf("web UI: http://%s.local/  (or http://%s/)\n", MDNS_NAME, WiFi.localIP().toString().c_str()); }
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status.json", HTTP_GET, handleStatus);
  server.on("/sats.json", HTTP_GET, handleSats);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/reboot", HTTP_POST, handleReboot);
  server.on("/wifireset", HTTP_POST, handleWifiReset);
  server.begin();
}

void setup() {
  auto c = M5.config();
  M5.begin(c);
  Serial.begin(115200);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  // The once-a-second GGA+GSA+GSV burst from a full multi-constellation sky is
  // ~1.5 KB; the default 256-byte UART RX FIFO overflows mid-burst and the skyplot
  // loses most sats. Enlarge the RX buffer so a whole burst survives a busy loop.
  Serial2.setRxBufferSize(2048);
  Serial2.begin(MOSAIC_COM1_BAUD, SERIAL_8N1, MOSAIC_RX_PIN, MOSAIC_TX_PIN);

  loadConfig();
  setLed(0x40, 0, 0);        // red = starting
  setupWiFi();               // blocks until WiFi up (or portal)
  startWebServer();          // http://ntrip-rover.local/ — config + status (no LCD)
  g_provDeadline = millis() + 30000;   // provisioning runs in loop() until this
  lastRtcm = millis();
}

void loop() {
  M5.update();
  unsigned long now = millis();

  // --- WiFi: reconnect in place; reboot only as a last resort ---
  bool wifiUp = (WiFi.status() == WL_CONNECTED);
  if (!wifiUp) {
    if (wifiDownSince == 0) { wifiDownSince = now; g_wifiDrops++; WiFi.reconnect(); Serial.println("WiFi down, reconnecting..."); }
    if (now - wifiDownSince > WIFI_LASTRESORT_MS) { Serial.println("WiFi down too long, reboot"); ESP.restart(); }
    updateLed(false, false, false);
    delay(50);
    return;
  }
  wifiDownSince = 0;
  server.handleClient();       // web UI (works whether or not NTRIP is up)
  provisionTick(now);          // self-provision the receiver (non-blocking)
  bool provResolved = (g_prov != PROV_WAIT);

  // --- NTRIP: reconnect in place with backoff ---
  bool ntripUp = ntrip_c.connected();
  static bool prevNtripUp = false;
  if (prevNtripUp && !ntripUp) g_ntripDrops++;   // count connected -> down transitions
  prevNtripUp = ntripUp;
  if (!ntripUp && (now - lastNtripTry > NTRIP_RETRY_MS)) {
    lastNtripTry = now; ntrip_c.stop(); ntripConnect();
  }

  // --- RTCM3 -> mosaic COM1, but only once provisioning has resolved. Holding
  //     it keeps the receiver out of RTK so setNMEAOutput lands in the quiet
  //     window; meanwhile drain+discard so the NTRIP socket doesn't back up. ---
  if (ntripUp) {
    if (provResolved) {
      while (ntrip_c.available()) { Serial2.write((uint8_t)ntrip_c.read()); totalBytes++; }
      if (totalBytes > lastBytes) { lastBytes = totalBytes; lastRtcm = now; }
    } else {
      while (ntrip_c.available()) ntrip_c.read();   // discard while provisioning
    }
  }

  // --- Read GGA back from COM1 for the fix-status LED ---
  while (Serial2.available()) feedNmea((char)Serial2.read());

  // (GSV is committed on the GGA epoch boundary in feedNmea/commitSats.)

  // --- Fix-time accounting: accumulate wall time held in a fresh RTK-FIX ---
  bool fixed = (g_fixQ == FIX_RTK_FIXED) && g_lastGga && ((now - g_lastGga) < GGA_STALE_MS);
  if (fixed) {
    if (g_firstFixMs == 0) g_firstFixMs = now;              // time-to-first-fix
    if (g_lastFixSample) g_fixAccumMs += (now - g_lastFixSample);
  }
  g_lastFixSample = now;

  // --- LED ---
  if (!provResolved)   setLed(0x30, 0x10, 0);            // amber = provisioning
  else if (!ntripUp)   updateLed(true, false, false);    // magenta = NTRIP connecting
  else                 updateLed(true, true, (now - lastRtcm > RTCM_STALL_MS));

  // --- 1 Hz status line + RTCM throughput sample ---
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 1000) {
    unsigned long dt = now - lastPrint;                    // ~1000 ms
    g_rtcmBps = (uint32_t)((totalBytes - g_rateLastBytes) * 1000 / dt);
    g_rateLastBytes = totalBytes;
    bool ggaFresh = g_lastGga && (now - g_lastGga) < GGA_STALE_MS;
    Serial.printf("RTCM %llu B %lu B/s | fix=%s%s sats=%d hdop=%.1f age=%.1f | drops N%lu/W%lu | prov=%s\n",
                  totalBytes, (unsigned long)g_rtcmBps, fixStr(g_fixQ),
                  ggaFresh ? "" : " (stale)", g_sats, g_hdop, g_corrAge,
                  (unsigned long)g_ntripDrops, (unsigned long)g_wifiDrops,
                  g_prov==PROV_DONE?"done":g_prov==PROV_GAVEUP?"gaveup":"wait");
    lastPrint = now;
  }

  delay(5);
}
