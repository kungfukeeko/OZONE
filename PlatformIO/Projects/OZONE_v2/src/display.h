#pragma once
#include <Arduino.h>
#include <RTClib.h>   // DateTime

// Sun position result — shared between the solar math (main sketch) and the display.
struct SunPos { double elevation; double azimuth; };

// App state owned by the main sketch, read by the display module.
extern double gpsLat, gpsLon;
extern bool   hasFix, hasStoredPos, rtcSyncedGPS;

// ---------------- Display API ----------------
void displayInit();                  // power TFT/I2C rail, init panel, clear screen
void displayError(const char* msg);  // full-screen error message
void splashSunrise();                // boot animation (sun rises over the sea)
void splashSunset();                 // shutdown animation (sun sets)
void displayOff();                   // blank the panel and switch the backlight off
void drawScreen(const DateTime& now, const SunPos& sun,
                float m0, float sd0, float m1, float sd1);  // live measurement dashboard
