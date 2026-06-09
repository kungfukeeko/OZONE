#pragma once
#include <Arduino.h>
#include <RTClib.h>          // DateTime
#include "SolarCalculator.h" // SunPos

// App state owned by the main sketch, read by the display module.
extern double gpsLat, gpsLon;
extern bool   hasFix, hasStoredPos, rtcSyncedGPS, gpsAwake;

// ---------------- Display API ----------------
void displayInit();                  // power TFT/I2C rail, init panel, clear screen
void displayError(const char* msg);  // full-screen error message
void splashSunrise();                // boot animation (sun rises over the sea)
void splashSunset();                 // shutdown animation (sun sets)
void displayOff();                   // blank the panel and switch the backlight off
void displayMessage(const char* msg, bool ok = true);  // full-screen status (green ok / red fail)
void drawScreen(const DateTime& now, const SunPos& sun,
                float m0, float sd0, float m1, float sd1);  // live measurement dashboard
