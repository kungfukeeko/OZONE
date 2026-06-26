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
//  FLASH BACKUP (FFat)
//  Each finished run is saved to /run_<stamp>.csv BEFORE any upload, so the data is
//  durable the instant measuring ends. A unique per-run name means a new run never
//  overwrites an earlier one that hasn't been delivered yet; files are removed only
//  once successfully uploaded.
// ============================================================================
static bool isBackup(const String& nameIn) {
  String name = nameIn;
  name.toLowerCase();   // FAT may report 8.3 names in upper case
  return name.indexOf("run_") >= 0 && name.endsWith(".csv");
}

bool storageBegin() {
  if (!FFat.begin(true)) {        // true = format if the FS partition is unformatted
    Serial.println("FLASH: FFat mount FAILED (no backup this session)");
    return false;
  }
  Serial.println("FLASH: FFat mounted");
  return true;
}

// ---- Streaming run log ----
// The run file stays open for the whole run: header at start, one row appended per
// block with a flush at each block boundary (durable within ~1 s), closed at the end.
// RAM use is flat regardless of run length, and a crash/power-loss loses at most the
// last block -- the partial file is still valid CSV and is delivered on the next boot.
static File runFile;

bool runFileBegin(const MeasurementMeta &meta) {
  // /run_YYYYMMDD_HHMMSS.csv  (strip the separators from the start date/time strings)
  char d[11] = "0000-00-00", t[9] = "00:00:00";
  if (meta.dateStr && meta.dateStr[0]) { strncpy(d, meta.dateStr, 10); d[10] = 0; }
  if (meta.timeStr && meta.timeStr[0]) { strncpy(t, meta.timeStr, 8);  t[8]  = 0; }
  char path[40];
  snprintf(path, sizeof(path), "/run_%c%c%c%c%c%c%c%c_%c%c%c%c%c%c.csv",
           d[0],d[1],d[2],d[3],d[5],d[6],d[8],d[9], t[0],t[1],t[3],t[4],t[6],t[7]);

  runFile = FFat.open(path, "w");
  if (!runFile) { Serial.printf("FLASH: open %s failed -> NOT logging to flash\n", path); return false; }
  writeCsvHeader(runFile, meta);
  runFile.flush();
  Serial.printf("FLASH: streaming run to %s\n", path);
  return true;
}

bool runFileAppendRow(float el, float m0, float sd0, float m1, float sd1, float t) {
  if (!runFile) return false;
  char line[112];
  snprintf(line, sizeof(line), "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f", el, m0, sd0, m1, sd1, t);
  runFile.println(line);
  runFile.flush();            // commit at the block boundary -> durable within ~1 s
  return true;
}

void runFileEnd() {
  if (runFile) { runFile.flush(); runFile.close(); }
}

int pendingBackupCount() {
  int n = 0;
  File root = FFat.open("/");
  if (!root) return 0;
  for (File f = root.openNextFile(); f; f = root.openNextFile())
    if (isBackup(String(f.name()))) n++;
  root.close();
  return n;
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
  size_t sent = 0;
  uint8_t buf[512];
  while (f.available()) {
    size_t n = f.read(buf, sizeof(buf));
    sent += client.write(buf, n);
  }
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

// Upload every pending /run_*.csv and delete each on success. Stops at the first
// failure (PC unreachable) so the caller can wait and retry. Returns how many landed.
// NOTE: a plain `ncat -l 5000` (no -k) accepts only ONE connection, so if several runs
// are pending, restart the listener between them (or use `ncat -k`, which concatenates).
int uploadPendingBackups() {
  String names[24];
  int cnt = 0;
  File root = FFat.open("/");
  if (!root) return 0;
  for (File f = root.openNextFile(); f && cnt < 24; f = root.openNextFile()) {
    String name = f.name();
    if (isBackup(name)) {
      if (!name.startsWith("/")) name = "/" + name;
      names[cnt++] = name;
    }
  }
  root.close();
  if (cnt == 0) return 0;
  if (!wifiEnsure()) return 0;

  int sent = 0;
  for (int i = 0; i < cnt; i++) {
    if (uploadOneFile(names[i].c_str())) {
      FFat.remove(names[i]);
      sent++;
    } else {
      break;   // PC unreachable -> stop; the caller will retry next round
    }
  }
  return sent;
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
