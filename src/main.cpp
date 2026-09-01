/*
 *  ntripclient_G5-P3H — M5Atom Lite NTRIP client for a Septentrio mosaic-go G5 P3H rover.
 *
 *  Thin wrapper: all behaviour lives in the receiver-agnostic core library
 *  ntripclient-core (NtripRover.h). This sketch only sets the P3H-specific bits.
 *
 *  Wiring (Grove HY2.0, both sides 3.3V LVTTL — no level shifter):
 *    Atom G26 (TX) ──► mosaic COM1 RX      Atom G32 (RX) ◄── mosaic COM1 TX      GND──GND
 *    RTCM3 over WiFi ──► Atom ──► COM1 (RTK)  ;  COM1 GGA(+GSA+GSV) ──► Atom (LED + web)
 *    mosaic COM2 ──► NMEA @38400 ──► tractor (mosaic emits directly; Atom not involved)
 *  If no fix / no GGA, suspect G26/G32 swapped.
 *
 *  NOTE(hardware): this P3H is RTCMv3-only on COM1 and IGNORES ASCII commands on the
 *  shared RTCM+GGA UART, so self-provisioning no-acks — it runs off a boot config saved
 *  via USB/RxTools (COM1 = GGA+GSA+GSV, see README). The core still works: it detects the
 *  streaming GGA and forwards. Set o.provision=false to skip the self-provisioning probe.
 */
#include <NtripRover.h>

void setup() {
  NtripRover::Options o;
  o.board   = "G5-P3H";
  o.atomCom = "COM1";        // Atom Grove is wired to mosaic COM1 on the P3H
  // Pins 26/32 @115200 and the NTRIP defaults (rtk.toiso.fit:2101/eniwa-bd982) are
  // already the core defaults, so nothing else to set.
  NtripRover::begin(o);
}

void loop() { NtripRover::loop(); }
