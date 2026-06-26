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
