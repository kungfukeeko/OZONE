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

// Connect to WiFi, open a TCP connection to the configured computer, stream the
// 3-row start-metadata header followed by the measurement rows as CSV, then
// disconnect and power the radio down again. Returns true if all rows were sent.
// Each data row is {elevation, mean_ch0, stddev_ch0, mean_ch1, stddev_ch1}.
bool uploadMeasurementsCSV(const float (*rows)[5], size_t count, const MeasurementMeta &meta);
