#pragma once
#include <Arduino.h>

// Metadata captured at the measurement-start moment, written as a 3-row header
// above the data: (time, date), (latitude, longitude), (temp, humidity, pressure).
struct MeasurementMeta {
  const char* timeStr;      // "HH:MM:SS"   (local)
  const char* dateStr;      // "YYYY-MM-DD" (local)
  double      lat;
  double      lon;
  bool        envValid;     // false if no BME280 was found -> env row prints NA
  float       tempC;
  float       humidity;     // %
  float       pressureHPa;  // hPa
  float       offset0;      // offset-cal result ch0 (mV)
  float       offset0Std;   // its stddev (mV)
  float       offset1;      // offset-cal result ch1 (mV)
  float       offset1Std;   // its stddev (mV)
};

// ---- Flash storage (FFat) ----
// storageBegin(): mount the filesystem (call once in setup). Returns false if no FS.
// pendingBackupCount(): how many undelivered /run_*.csv files remain on flash.
// uploadPendingBackups(): upload + delete pending files; returns how many were sent.
bool storageBegin();
int  pendingBackupCount();
int  uploadPendingBackups();

// ---- Streaming run log (write each block to flash as it's measured) ----
// runFileBegin(): open /run_<stamp>.csv and write the header. Returns false if no FS.
// runFileAppendRow(): append one measurement row and flush (durable at the block boundary).
// runFileEnd(): flush + close at run end -> the file is the complete, durable record.
bool runFileBegin(const MeasurementMeta &meta);
bool runFileAppendRow(float el, float m0, float sd0, float m1, float sd1, float t);
void runFileEnd();

// ---- Optional remote-control link (end a run early over WiFi) ----
// commandLinkBegin(): connect WiFi at startup and listen on CTRL_PORT
// stopCommandReceived(): non-blocking poll; returns true once 'dump'/'stop' has arrived.
bool commandLinkBegin();
bool stopCommandReceived();
void wifiKeepalive();            // call each loop: reconnects WiFi if it dropped (keeps dump reachable)
String deviceAddress();          // "x.x.x.x" once connected, else "offline"
bool   ntpSyncUTC(uint32_t &epochOut);  // exact UTC from NTP over WiFi (true on success)
