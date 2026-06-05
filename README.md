------  MJERENJE STUPCA OZONA  ------

Ovdje nam se nalaze svi bitni file-ovi za proizvesti i programirati instrument ☀️

Prioritet je:

1. Ugladiti i kalibrirati suncokret 🌻
2. Programirati ESP32 feature u ozone_v2.ino: izračun pozicije sunca -DONE-, slanje mjerenja preko weba -DONE- i prikaz na ekranu -DONE-, sleep-wakeup kontrola -TODO-💫💽💤
3. Spojiti sunflower.ino i ozone_v2.ino 👯
4. Dizajnirati poklopac za platformu ⛺

Kako bi primili podatke preko WiFi-a na kraju mjerenja
- Create secrets.h , copy secrets.example.h and add your SECRET_WIFI_SSID, SECRET_WIFI_PASS, SECRET_PC_HOST
- Install nmap and run command before measurement end: ncat -l 5000 > data.csv

File će biti spremljen pod data.csv, preimenuj ga prije idućeg mjerenja.
Ovo ću update-at po potrebi.
