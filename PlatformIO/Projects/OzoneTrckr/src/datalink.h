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
};

bool uploadMeasurementsCSV(const float (*rows)[5], size_t count, const MeasurementMeta &meta);
