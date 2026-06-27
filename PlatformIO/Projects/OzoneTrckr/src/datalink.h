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

// ---- Flash storage (FFat) -- single rolling run file /lastrun.csv ----
// storageBegin(): mount (no silent auto-format). storageEnd(): clean unmount before sleep.
// lastRunExists(): is a non-empty last run stored?
// dumpLastRunToSerial(): re-emit the last run over Serial (CSV, bracketed) -- WiFi-free recovery.
// uploadLastRun(): send the last run over WiFi (true only if fully sent).
bool storageBegin();              // mount only (no reformat -- keeps a corrupt FS readable for recovery)
bool storageEnsureWritable();     // write-test; reformat if corrupt. Call AFTER boot recovery.
void storageEnd();
bool lastRunExists();
void dumpLastRunToSerial();
bool uploadLastRun();

// ---- Streaming run log (write each block to /lastrun.csv as it's measured) ----
// runFileBegin(): open the file (overwrites the previous run) + write header.
// runFileAppendRow(): append one row and flush (durable at the block boundary).
// runFileEnd(): flush + close at run end.
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
