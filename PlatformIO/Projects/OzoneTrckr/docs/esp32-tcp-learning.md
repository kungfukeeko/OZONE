# ESP32 + TCP + `datalink` — learning references

A structured "syllabus" to understand the networking in `src/datalink.cpp`
(WiFi station mode, TCP client upload, TCP command server, mDNS, modem sleep).
Goes fundamentals -> platform -> the exact things in our code.

## 1. The foundation: TCP/IP & sockets
Everything in `datalink` is the Berkeley sockets model wrapped in Arduino classes.
Learn the model once and the rest is just naming.

- **Beej's Guide to Network Programming** — https://beej.us/guide/bgnet/
  The classic free book on sockets (client `connect()`, server `bind/listen/accept`,
  `send/recv`). Read "What is a socket?", "client-server background", and the
  "system calls" chapters. This *is* what `WiFiClient`/`WiFiServer` are underneath.

- **TCP on Wikipedia** — https://en.wikipedia.org/wiki/Transmission_Control_Protocol
  Three-way handshake, ports, reliable stream, teardown. Skim for vocabulary.

- *Book (deep, optional):* W. Richard Stevens, *TCP/IP Illustrated, Vol. 1* — the
  reference for wire-level detail.

## 2. The platform: ESP32 + Arduino WiFi

- **arduino-esp32 documentation** — https://docs.espressif.com/projects/arduino-esp32/en/latest/
  Official. See the **WiFi** API section (`WiFi.mode`, `begin`, `status`,
  `setHostname`, `setSleep`).

- **arduino-esp32 WiFi examples (GitHub)** —
  https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFi/examples
  `WiFiClient`, `WiFiClientBasic`, and **`SimpleWiFiServer`** are almost exactly the
  two halves of what we wrote.

- **Random Nerd Tutorials – ESP32** — https://randomnerdtutorials.com/projects-esp32/
  The most beginner-friendly, project-based walkthroughs (WiFi client, server, TCP).

## 3. Exactly what `datalink.cpp` does — concept -> reference

WiFi.mode(WIFI_STA); WiFi.begin(ssid,pass)
Station mode joining an AP; DHCP gives you an IP 
arduino-esp32 WiFi docs

WiFiClient client; client.connect(host,port)
**TCP client** — outbound connection (device -> PC's `ncat -l`)
Beej "A Simple Stream Client"; `WiFiClient` example

WiFiServer ctrlServer(5050); server.available()
**TCP server** — inbound listener (PC -> device:5050)
Beej "A Simple Stream Server"; SimpleWiFiServer example

ctrlServer.setNoDelay(true)
**Nagle's algorithm / TCP_NODELAY** — why small packets get delayed and how to disable 
https://en.wikipedia.org/wiki/Nagle%27s_algorithm

connect-retry loop + client.setTimeout
timeouts, connection refused, retry/backoff 
Beej (error handling)

MDNS.begin("ozone"); MDNS.addService(...)
**mDNS / DNS-SD / Zeroconf** — `.local` name resolution & service discovery 
RFC 6762 (https://www.rfc-editor.org/rfc/rfc6762), RFC 6763 (https://www.rfc-editor.org/rfc/rfc6763); ESPmDNS examples (https://github.com/espressif/arduino-esp32/tree/master/libraries/ESPmDNS/examples)

WiFi.setSleep(true) (the ~100 ms ping) 
**WiFi modem sleep / DTIM** — power vs latency trade-off 
ESP-IDF WiFi guide (https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html), power-save section)

WiFi.setHostname("ozone")
DHCP hostname option 
DHCP basics (Wikipedia)

## 4. The tool we use to receive/trigger
- **Ncat Users' Guide** — https://nmap.org/ncat/guide/index.html
  Listen mode (`-l`, `-k` keep-open), connect mode, redirection. Explains why
  `ncat -l 5000 > data.csv` (server) and `ncat ozone.local 5050` (client) are
  opposite roles.

## 5. Going deeper (optional)
- **lwIP** — https://www.nongnu.org/lwip/2_1_x/index.html
  The lightweight TCP/IP stack the ESP32 actually runs; `WiFiClient` ends up calling
  lwIP sockets.
- **ESP-IDF networking docs** —
  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/index.html
  The layer beneath Arduino; useful for direct sockets/TLS/control.
- **ESP32-S2 datasheet** —
  https://www.espressif.com/sites/default/files/documentation/esp32-s2_datasheet_en.pdf
  Our chip: single core, native USB, WiFi-only (no BT) — why the radio shares time
  with the measurement loop.

## Suggested order
Beej (ch. 1-6) -> arduino-esp32 WiFi examples (client + server) -> mDNS RFC intro +
ESPmDNS example -> modem-sleep / deep-sleep guides. After Beej, re-read `datalink.cpp`
and it reads like plain sockets code.

## 6. NTP time sync (configTime / getLocalTime)
We pull exact UTC over WiFi and set the DS3231 from it (`ntpSyncUTC`).

- **arduino-esp32 `SimpleTime` example** —
  https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Time/SimpleTime/SimpleTime.ino
  Uses `configTime(gmtOffset, dstOffset, server)` + `getLocalTime(&tm)` — *exactly* our two calls.
- **Random Nerd – ESP32 NTP clock** — https://randomnerdtutorials.com/esp32-ntp-client-date-time-arduino-ide/
  Beginner walkthrough of the same pattern.
- **ESP-IDF system time / SNTP** —
  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system_time.html
  What `configTime` drives underneath (lwIP SNTP); how the sync notification works.
- **NTP / SNTP protocol** — https://en.wikipedia.org/wiki/Network_Time_Protocol ·
  pool servers: https://www.pool.ntp.org/en/use.html  (we use `pool.ntp.org` + `time.google.com`).
- **C time API** — https://en.cppreference.com/w/c/chrono  (`time()`, `struct tm`, `localtime`).
  Note: we `configTime(0,0,...)` so the epoch is **UTC**; the RTC stores UTC and the display adds the offset.

## 7. Flash storage (FFat / FAT, single rolling file)
The run streams to `/lastrun.csv` on the **FFat** (FAT) partition — open→append→close per block.

| In our code | Concept to study | Reference |
|---|---|---|
| `FFat.begin() / open() / read / write / close` | the Arduino **FS / File** API (shared by FFat, LittleFS, SPIFFS) | arduino-esp32 FS docs (https://docs.espressif.com/projects/arduino-esp32/en/latest/api/fs.html); FFat examples (https://github.com/espressif/arduino-esp32/tree/master/libraries/FFat/examples) |
| `FFat` on the `ffat` (subtype 0x81) partition, not LittleFS | **partition tables** — why this Adafruit board uses a FAT data partition | https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html |
| open-append-close **per block**; 0-byte files; reformat-to-recover | **FAT has no journaling** -> a write/power-cut can corrupt it | FAT: https://en.wikipedia.org/wiki/File_Allocation_Table ; ESP-IDF FATFS: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/fatfs.html |
| the FAT impl under FFat | **FatFs** (ELM-ChaN) — `f_open/f_write/f_sync/f_close`, options | http://elm-chan.org/fsw/ff/00index_e.html |
| flash endurance across many small writes | **wear levelling** (under FFat) | https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/wear-levelling.html |
| `Preferences` (NVS) for small key/values (coords, build stamp) | **NVS** — different store, for settings not bulk data | https://docs.espressif.com/projects/arduino-esp32/en/latest/api/preferences.html |

Hard-won lesson (see project memory): on this board FFat can **mount yet be unwritable** when
corrupt — writes silently fail to 0 bytes. Per-block open/append/close commits reliably; a boot
write-test reformats a corrupt FS; a true wipe needs a **hardware erase** (`pio run -t erase`).
