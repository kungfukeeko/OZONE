#include <Arduino.h>
#include <WiFi.h>
#include "datalink.h"
#include "secrets.h"   // WiFi/PC config — gitignored (see secrets.example.h)

static const char*    WIFI_SSID = SECRET_WIFI_SSID;
static const char*    WIFI_PASS = SECRET_WIFI_PASS;
static const char*    PC_HOST   = SECRET_PC_HOST;
static const uint16_t PC_PORT   = SECRET_PC_PORT;

static const uint32_t WIFI_TIMEOUT_MS = 15000;   // give up connecting after this
static const uint32_t TCP_TIMEOUT_MS  = 5000;

bool uploadMeasurementsCSV(const float (*rows)[5], size_t count) {
  Serial.print("WiFi: connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > WIFI_TIMEOUT_MS) {
      Serial.println("WiFi: connect timeout");
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      return false;
    }
    delay(200);
  }
  Serial.print("WiFi: connected, IP ");
  Serial.println(WiFi.localIP());

  WiFiClient client;
  client.setTimeout(TCP_TIMEOUT_MS / 1000);   // seconds
  if (!client.connect(PC_HOST, PC_PORT)) {
    Serial.print("TCP: could not reach ");
    Serial.print(PC_HOST); Serial.print(":"); Serial.println(PC_PORT);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return false;
  }

  // CSV header + one row per line
  client.println("elevation,mean_ch0,stddev_ch0,mean_ch1,stddev_ch1");
  char line[96];
  for (size_t i = 0; i < count; i++) {
    snprintf(line, sizeof(line), "%.4f,%.4f,%.4f,%.4f,%.4f",
             rows[i][0], rows[i][1], rows[i][2], rows[i][3], rows[i][4]);
    client.println(line);
  }
  client.flush();
  client.stop();
  Serial.print("TCP: sent ");
  Serial.print((unsigned)count);
  Serial.println(" rows");

  // radio off to save battery
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  return true;
}
