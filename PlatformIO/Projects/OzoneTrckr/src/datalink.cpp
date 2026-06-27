#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <FFat.h>
#include <time.h>          // NTP -> system time
#include "datalink.h"
#include "secrets.h"   // WiFi/PC config — gitignored (see secrets.example.h)

static const char*    WIFI_SSID = SECRET_WIFI_SSID;
static const char*    WIFI_PASS = SECRET_WIFI_PASS;
static const char*    PC_HOST   = SECRET_PC_HOST;
static const uint16_t PC_PORT   = SECRET_PC_PORT;

static const uint32_t WIFI_TIMEOUT_MS = 15000;   // give up connecting after this
static const uint32_t TCP_TIMEOUT_MS  = 5000;

#define CTRL_PORT 5050                       // device listens here for 'send'/'stop'
static WiFiServer ctrlServer(CTRL_PORT);
static WiFiClient ctrlClient;
static bool       ctrlUp = false;

// ============================================================================
//  CSV FORMAT (single source of truth)
//  4 header rows + a column header + one row per measurement. Written to either a
//  FFat File (flash backup) or a WiFiClient (network upload) -- both are Print.
// ============================================================================
// Write the 4-row metadata header + the CSV column header (no data rows -- the rows
// are streamed in one at a time during the run, see runFileAppendRow).
static void writeCsvHeader(Print& out, const MeasurementMeta &meta) {
  char hdr[112];
  snprintf(hdr, sizeof(hdr), "TIME: %s, DATE: %s", meta.timeStr, meta.dateStr);
  out.println(hdr);
  snprintf(hdr, sizeof(hdr), "LATITUDE: %.6f, LONGITUDE: %.6f", meta.lat, meta.lon);
  out.println(hdr);
  if (meta.envValid)
    snprintf(hdr, sizeof(hdr), "TEMP: %.2f degC, HUMIDITY: %.2f %%, PRESSURE: %.2f hPa",
             meta.tempC, meta.humidity, meta.pressureHPa);
  else
    snprintf(hdr, sizeof(hdr), "TEMP: NA, HUMIDITY: NA, PRESSURE: NA");
  out.println(hdr);
  snprintf(hdr, sizeof(hdr), "OFFSET CAL: %.4f +/- %.4f mV, %.4f +/- %.4f mV",
           meta.offset0, meta.offset0Std, meta.offset1, meta.offset1Std);
  out.println(hdr);
  out.println("elevation,mean_ch0,stddev_ch0,mean_ch1,stddev_ch1,temp_c");
}

// ============================================================================
//  FLASH STORAGE (FFat) -- single rolling file /lastrun.csv
//  The run streams to one fixed file (no directory enumeration). It is recovered over
//  serial (+WiFi) at the END of the run and re-dumped over serial at the NEXT boot;
//  runFileBegin() then overwrites it. NOTE: the 960 KB partition holds only ~one large
//  run, so a new run reclaims the space -- the previous run must be delivered or serial-
//  recovered before the next run starts (see the ozone_trckr boot sequence).
// ============================================================================
#define LASTRUN_PATH "/lastrun.csv"   // single rolling run file -> no enumeration to get wrong

// Quick write-test: a FAT can MOUNT yet be corrupt/unwritable -- writes then silently
// fail and you get 0-byte files (exactly what bit us). Write a tiny file, read it back.
static bool flashWriteTest() {
  File f = FFat.open("/.wtest", "w");
  if (!f) return false;
  f.println("ok");
  f.close();
  File c = FFat.open("/.wtest", "r");
  size_t sz = c ? c.size() : 0;
  if (c) c.close();
  FFat.remove("/.wtest");
  return sz > 0;
}

// Mount ONLY -- do NOT reformat here. A corrupt-but-readable FS must stay readable long
// enough for boot recovery (dumpLastRunToSerial) to run first; storageEnsureWritable()
// does the destructive reformat afterwards.
bool storageBegin() {
  if (FFat.begin(false)) { Serial.println("FLASH: FFat mounted"); return true; }
  Serial.println("FLASH: mount FAILED -> formatting");
  if (FFat.begin(true)) { Serial.println("FLASH: formatted + mounted"); return true; }
  Serial.println("FLASH: unavailable (no storage)");
  return false;
}

// Ensure the FS is actually writable; reformat a mounted-but-corrupt FS. Call this AFTER
// boot recovery (it can wipe /lastrun.csv). Returns false only if even a reformat won't
// help -> a hardware erase is then needed (pio run -t erase).
bool storageEnsureWritable() {
  if (flashWriteTest()) return true;
  Serial.println("FLASH: WRITE TEST FAILED (corrupt FS) -> reformatting");
  FFat.end();
  bool fmt = FFat.format();
  Serial.printf("FLASH: format %s\n", fmt ? "ok" : "returned false");
  FFat.begin(true);
  if (!flashWriteTest()) {
    Serial.println("FLASH: STILL unwritable -> do a hardware erase: pio run -t erase");
    return false;
  }
  Serial.println("FLASH: writable");
  return true;
}

// Clean unmount before deep sleep -> avoids leaving the FAT unmountable.
void storageEnd() { FFat.end(); }

// ---- Streaming run log: open-append-close PER BLOCK (no long-lived handle) ----
// On FFat, holding one handle open + flush() produced 0-byte files; closing after each
// block reliably commits and keeps the file size correct. RAM is flat regardless of run
// length, and a crash loses at most the last block (each prior close is a clean commit).
bool runFileBegin(const MeasurementMeta &meta) {
  File f = FFat.open(LASTRUN_PATH, "w");   // overwrite previous run (already dumped at boot)
  if (!f) { Serial.println("FLASH: open " LASTRUN_PATH " (w) failed"); return false; }
  writeCsvHeader(f, meta);
  f.close();
  // Verify the header actually landed -> catch an unwritable FS at the START, not after a run.
  File c = FFat.open(LASTRUN_PATH, "r");
  size_t sz = c ? c.size() : 0;
  if (c) c.close();
  if (sz == 0) { Serial.println("FLASH: header wrote 0 bytes -> flash NOT writable"); return false; }
  Serial.printf("FLASH: streaming run to %s (header %u bytes)\n", LASTRUN_PATH, (unsigned)sz);
  return true;
}

bool runFileAppendRow(float el, float m0, float sd0, float m1, float sd1, float t) {
  File f = FFat.open(LASTRUN_PATH, "a");
  if (!f) return false;
  size_t before = f.size();                  // committed size before this row
  char line[112];
  snprintf(line, sizeof(line), "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f", el, m0, sd0, m1, sd1, t);
  f.println(line);
  f.close();
  // Confirm the row actually committed (file grew). println()'s return can be nonzero
  // even when the underlying write silently failed; the size read-back catches that, so
  // the flashHealthy/"F" dot is truthful if flash goes bad mid-run.
  File c = FFat.open(LASTRUN_PATH, "r");
  size_t after = c ? c.size() : 0;
  if (c) c.close();
  return after > before;
}

void runFileEnd() { /* nothing -- each block was already committed by its own close */ }

// True if a non-empty last run is stored on flash.
bool lastRunExists() {
  File f = FFat.open(LASTRUN_PATH, "r");
  if (!f) return false;
  bool ok = (f.size() > 0);
  f.close();
  return ok;
}

// Copy an open file to any Print sink (Serial or a WiFiClient). Returns bytes written.
static size_t streamFile(File& f, Print& out) {
  uint8_t buf[512];
  size_t total = 0;
  while (f.available()) { size_t n = f.read(buf, sizeof(buf)); total += out.write(buf, n); }
  return total;
}

// Re-emit the last run to Serial verbatim (the same CSV the WiFi upload sends), bracketed
// so it's easy to lift out of a PuTTY log. No-op if nothing is stored.
void dumpLastRunToSerial() {
  if (!lastRunExists()) { Serial.println("FLASH: no last run to dump"); return; }
  File f = FFat.open(LASTRUN_PATH, "r");
  Serial.println("---- DATA START ----");
  streamFile(f, Serial);
  f.close();
  Serial.println("---- DATA END ----");
}

// ============================================================================
//  WIFI / TCP HELPERS
// ============================================================================
// Apply a static IP if secrets.h defines one -> no DHCP lease to expire (the prime
// suspect for the ~1 h drop-offs). No-op (plain DHCP) if SECRET_STATIC_IP is undefined.
static void applyNetConfig() {
#ifdef SECRET_STATIC_IP
  IPAddress ip, gw, sn, dns;
  if (ip.fromString(SECRET_STATIC_IP) && gw.fromString(SECRET_GATEWAY) &&
      sn.fromString(SECRET_SUBNET)   && dns.fromString(SECRET_DNS)) {
    if (!WiFi.config(ip, gw, sn, dns))
      Serial.println("WiFi: static IP config failed (falling back to DHCP)");
  }
#endif
}

// Connect only if not already associated (commandLinkBegin may have done it). Leaves
// WiFi UP for the caller -- we never disconnect here; deep sleep cuts power anyway.
static bool wifiEnsure() {
  if (WiFi.status() == WL_CONNECTED) return true;
  Serial.print("WiFi: connecting to "); Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  applyNetConfig();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > WIFI_TIMEOUT_MS) { Serial.println("WiFi: connect timeout"); return false; }
    delay(200);
  }
  Serial.print("WiFi: connected, IP "); Serial.println(WiFi.localIP());
  return true;
}

// One TCP connection, stream one backup file verbatim. Single attempt -- the caller's
// time-boxed loop handles the retry cadence. Returns true ONLY if a non-empty file was
// fully sent, so an empty/short transfer is never mistaken for "delivered" (and deleted).
static bool uploadOneFile(const char* path) {
  // Open + size-check FIRST: never "deliver" (hence delete) an empty file.
  File f = FFat.open(path, "r");
  if (!f) return false;
  size_t fsize = f.size();
  if (fsize == 0) {
    f.close();
    Serial.printf("TCP: %s is empty -> NOT uploading (kept on flash)\n", path);
    return false;
  }
  if (!wifiEnsure()) { f.close(); return false; }
  WiFiClient client;
  client.setTimeout(TCP_TIMEOUT_MS / 1000);
  if (!client.connect(PC_HOST, PC_PORT)) {
    Serial.printf("TCP: %s:%u not reachable\n", PC_HOST, PC_PORT);
    f.close();
    return false;
  }
  size_t sent = streamFile(f, client);
  f.close();
  client.flush();
  client.stop();
  if (sent < fsize) {   // short write -> treat as failure so the file is kept & retried
    Serial.printf("TCP: %s short write (%u/%u bytes) -> kept on flash\n",
                  path, (unsigned)sent, (unsigned)fsize);
    return false;
  }
  Serial.printf("TCP: uploaded %s (%u bytes)\n", path, (unsigned)sent);
  return true;
}

// Send the last run over WiFi to the PC's ncat listener. One attempt; true only if the
// whole non-empty file went out. NOT deleted on success -- it stays as the "last run"
// (re-dumpable over serial) until the next run overwrites it.
bool uploadLastRun() {
  return uploadOneFile(LASTRUN_PATH);
}

// ============================================================================
//  REMOTE CONTROL LINK (end a run early over WiFi)
// ============================================================================
bool commandLinkBegin() {
  Serial.print("CTRL: connecting WiFi for remote stop (");
  Serial.print(WIFI_SSID); Serial.println(")");
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("ozone");       // DHCP hostname + basis for ozone.local
  applyNetConfig();                // static IP (if configured) -> no lease to expire
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > WIFI_TIMEOUT_MS) {
      Serial.println("CTRL: WiFi timeout -> no remote stop (run ends at MAX_SUBLISTS)");
      return false;
    }
    delay(200);
  }
  WiFi.setSleep(false);            // radio always on -> deauth is seen immediately, so
                                   // WiFi.status() stays truthful and reconnect actually fires
                                   // (modem sleep masked deauths -> "ghost" connections)
  ctrlServer.begin();
  ctrlServer.setNoDelay(true);
  ctrlUp = true;

  // mDNS: reachable as ozone.local regardless of the DHCP-assigned IP.
  if (MDNS.begin("ozone")) {
    MDNS.addService("ozone", "tcp", CTRL_PORT);
    Serial.println("CTRL: mDNS up -> ozone.local");
  } else {
    Serial.println("CTRL: mDNS failed (use the IP)");
  }

  Serial.print("CTRL: ready. End run early:  ncat ozone.local ");
  Serial.print(CTRL_PORT);
  Serial.print("   (IP "); Serial.print(WiFi.localIP()); Serial.println(")");
  return true;
}

// "x.x.x.x" once associated, "offline" otherwise. Used by the main to show the
// address on the TFT.
String deviceAddress() {
  if (WiFi.status() != WL_CONNECTED) return String("offline");
  return WiFi.localIP().toString();
}

// Get exact UTC from NTP over WiFi. Returns true + the UTC epoch on success. Requires
// WiFi associated AND internet reachable; the caller sets the RTC from epochOut.
bool ntpSyncUTC(uint32_t &epochOut) {
  if (WiFi.status() != WL_CONNECTED) return false;
  configTime(0, 0, "pool.ntp.org", "time.google.com");   // UTC, no TZ/DST offsets
  struct tm tmv;
  if (!getLocalTime(&tmv, 8000)) return false;           // wait up to 8 s for the SNTP reply
  time_t now = time(nullptr);
  if (now < 1700000000) return false;                    // sanity: must be after 2023-11
  epochOut = (uint32_t)now;
  return true;
}

// Non-blocking. Accepts a waiting client, reads one line, and returns true if it is
// 'send'/'stop'/'q'. Safe to call every loop iteration.
bool stopCommandReceived() {
  if (!ctrlUp) return false;

  if (!ctrlClient || !ctrlClient.connected()) {
    ctrlClient = ctrlServer.available();    // accept a new client if one is waiting
    if (ctrlClient) ctrlClient.println("ozone: connected. send 'send' to end run + upload.");
  }
  if (ctrlClient && ctrlClient.available()) {
    String cmd = ctrlClient.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();
    if (cmd == "send" || cmd == "stop" || cmd == "q") {
      ctrlClient.println("ozone: sending + shutting down.");
      ctrlClient.flush();
      ctrlClient.stop();
      Serial.println("CTRL: stop command received");
      return true;
    }
    if (cmd.length()) ctrlClient.println("ozone: unknown command; send 'send'.");
  }
  return false;
}

// Call frequently from the measurement loop. Keeps the link *genuinely* alive:
//   - connected  -> every 60 s send a tiny UDP packet to the gateway so the AP never
//     deauthenticates us as an "idle" client (the device otherwise only listens and
//     transmits nothing for the whole run -> looks dead to the AP);
//   - disconnected -> reconnect (one attempt per 30 s), re-listen, re-announce mDNS.
// Modem sleep is OFF (commandLinkBegin) so a deauth is seen immediately and
// WiFi.status() reflects reality -- a missed deauth otherwise leaves a "ghost"
// association that looks connected but isn't reachable, and this would never fire.
void wifiKeepalive() {
  if (!ctrlUp) return;
  uint32_t now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastTry = 0;
    if (lastTry != 0 && now - lastTry < 30000) return;   // throttle reconnect attempts
    lastTry = now;
    Serial.println("CTRL: WiFi dropped -> reconnecting (keepalive)");
    WiFi.disconnect();
    applyNetConfig();                // re-assert static IP on reconnect
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
      if (millis() - t0 > 8000) { Serial.println("CTRL: reconnect timed out (will retry)"); return; }
      delay(100);
    }
    WiFi.setSleep(false);
    Serial.print("CTRL: reconnected, IP "); Serial.println(WiFi.localIP());
    ctrlServer.begin();                       // re-listen for 'send' after the reconnect
    MDNS.end();                               // re-announce ozone.local on the new link
    if (MDNS.begin("ozone")) MDNS.addService("ozone", "tcp", CTRL_PORT);
    return;
  }

  // Connected: keep uplink traffic flowing so the AP never idle-deauths us.
  static uint32_t lastTouch = 0;
  if (lastTouch == 0 || now - lastTouch >= 60000) {
    lastTouch = now;
    WiFiUDP udp;
    if (udp.beginPacket(WiFi.gatewayIP(), 9)) {   // port 9 = discard; fire-and-forget
      uint8_t b = 0;
      udp.write(b);
      udp.endPacket();
    }
  }
}
