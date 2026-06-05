#pragma once
#include <Arduino.h>

// Connect to WiFi, open a TCP connection to the configured computer, stream the
// measurement rows as CSV, then disconnect and power the radio down again.
// Returns true if all rows were sent. Each row is {elevation, mean_ch0,
// stddev_ch0, mean_ch1, stddev_ch1}.
bool uploadMeasurementsCSV(const float (*rows)[5], size_t count);
