#pragma once
#include <Arduino.h>

// Metadata captured at the measurement-start moment, written as a 2-row header
// above the data: (time, date) and (latitude, longitude).
// (BME280 env + hardware offset-cal fields were removed -- the offset is now stored as a
//  PHASE-0 data row with flag 0; env sensors are not wired on this board yet.)
struct MeasurementMeta {
  const char* timeStr;      // "HH:MM:SS"   (local)
  const char* dateStr;      // "YYYY-MM-DD" (local)
  double      lat;
  double      lon;
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
// flag: 0 = offset block (PHASE 0), 1 = measurement block (PHASE 1/2)
bool runFileAppendRow(int flag, float el, float m0, float sd0, float m1, float sd1);
void runFileEnd();

// ---- Optional remote-control link (end a run early over WiFi) ----
// commandLinkBegin(): connect WiFi at startup and listen on CTRL_PORT
// stopCommandReceived(): non-blocking poll; returns true once 'dump'/'stop' has arrived.
bool commandLinkBegin();
bool stopCommandReceived();
void wifiKeepalive();            // call each loop: reconnects WiFi if it dropped (keeps dump reachable)
String deviceAddress();          // "x.x.x.x" once connected, else "offline"
bool   ntpSyncUTC(uint32_t &epochOut);  // exact UTC from NTP over WiFi (true on success)
