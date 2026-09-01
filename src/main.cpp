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
 *  mosaic-go setup: best-effort self-provisioning on boot. If the receiver is
 *  already streaming GGA (saved boot config), the Atom just forwards. Otherwise it
 *  tries the Septentrio command channel on COM1 (setNMEAOutput COM1 GGA for the LED
 *  + COM2 @ configured baud for the tractor) during the pre-RTK quiet window.
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
  // COM1 GGA for the fix-status LED. Last, since it starts GGA on our command
  // channel. RTCM3 fed to COM1 is auto-used as corrections (no explicit command).
  provisionOne("setNMEAOutput, Stream2, COM1, GGA, sec1", millis() + 6000);
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
  if (!g_forceProv && (now - g_lastGga) < GGA_STALE_MS) {
    g_prov = PROV_DONE;
    Serial.println("provision: GGA present — receiver already configured, forwarding");
    return;
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
h1{font-size:1.2rem}label{display:block;margin:.5rem 0 .1rem}
input{width:100%;padding:.4rem;box-sizing:border-box}
button{margin-top:.8rem;padding:.5rem 1rem}
#s{background:#f4f4f4;padding:.6rem;border-radius:.4rem;margin:.6rem 0;font-family:monospace;white-space:pre-wrap}
.r{color:#b00}.g{color:#080}</style>
<h1>NTRIP rover (G5-P3H)</h1>
<div id=s>loading…</div>
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
async function u(){let r=await fetch('/status.json');let d=await r.json();
document.getElementById('s').innerHTML=
'WiFi  '+d.wifi.ssid+'  '+d.wifi.ip+'  '+d.wifi.rssi+'dBm\n'+
'NTRIP '+(d.ntrip.connected?'<span class=g>connected</span>':'<span class=r>down</span>')+
'  '+d.ntrip.host+':'+d.ntrip.port+'/'+d.ntrip.mount+'\n'+
'RTCM  '+d.ntrip.bytes+' B'+(d.ntrip.stalled?'  <span class=r>STALLED</span>':'')+'\n'+
'Fix   '+(d.fix.q=='RTK-FIX'?'<span class=g>':'')+d.fix.q+(d.fix.q=='RTK-FIX'?'</span>':'')+
  (d.fix.fresh?'':'  (stale)')+'\n'+
'Prov  '+(d.prov=='done'?'<span class=g>done</span>':d.prov=='wait'?'<span class=r>waiting…</span>':'gave up')+
'\nUp    '+d.up+' s';
for(let k of ['host','port','mount','user','pass'])
  {let e=document.querySelector('[name='+k+']');if(e&&!e.dataset.t)e.value=d.cfg[k];}}
u();setInterval(u,2000);
document.querySelectorAll('input').forEach(e=>e.addEventListener('input',()=>e.dataset.t=1));
</script>)HTML";

void handleRoot()   { server.send_P(200, "text/html", PAGE); }

void handleStatus() {
  unsigned long now = millis();
  bool ggaFresh = (now - g_lastGga) < GGA_STALE_MS;
  bool stalled  = ntrip_c.connected() && (now - lastRtcm > RTCM_STALL_MS);
  char buf[640];
  snprintf(buf, sizeof(buf),
    "{\"wifi\":{\"ssid\":\"%s\",\"ip\":\"%s\",\"rssi\":%d},"
    "\"ntrip\":{\"host\":\"%s\",\"port\":%d,\"mount\":\"%s\",\"connected\":%s,"
    "\"bytes\":%llu,\"stalled\":%s},"
    "\"fix\":{\"q\":\"%s\",\"fresh\":%s},"
    "\"cfg\":{\"host\":\"%s\",\"port\":%d,\"mount\":\"%s\",\"user\":\"%s\",\"pass\":\"%s\","
    "\"com2_baud\":%d,\"com2_nmea\":\"%s\"},"
    "\"prov\":\"%s\",\"up\":%lu}",
    WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI(),
    cfg.host, cfg.port, cfg.mount, ntrip_c.connected() ? "true" : "false",
    (unsigned long long)totalBytes, stalled ? "true" : "false",
    fixStr(g_fixQ), ggaFresh ? "true" : "false",
    cfg.host, cfg.port, cfg.mount, cfg.user, cfg.pass, cfg.com2_baud, cfg.com2_nmea,
    g_prov==PROV_DONE?"done":g_prov==PROV_GAVEUP?"gaveup":"wait", now / 1000);
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
    if (wifiDownSince == 0) { wifiDownSince = now; WiFi.reconnect(); Serial.println("WiFi down, reconnecting..."); }
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

  // --- LED ---
  if (!provResolved)   setLed(0x30, 0x10, 0);            // amber = provisioning
  else if (!ntripUp)   updateLed(true, false, false);    // magenta = NTRIP connecting
  else                 updateLed(true, true, (now - lastRtcm > RTCM_STALL_MS));

  // --- 1 Hz status line ---
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 1000) {
    bool ggaFresh = (now - g_lastGga) < GGA_STALE_MS;
    Serial.printf("RTCM %llu B | fix=%s%s | prov=%s\n", totalBytes, fixStr(g_fixQ),
                  ggaFresh ? "" : " (stale)",
                  g_prov==PROV_DONE?"done":g_prov==PROV_GAVEUP?"gaveup":"wait");
    lastPrint = now;
  }

  delay(5);
}
