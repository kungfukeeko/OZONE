#pragma once

// Result of a sun-position calculation (degrees).
struct SunPos { double elevation; double azimuth; };

// NOAA Solar Calculator. Angles in degrees; the Julian Day must be in UTC.
double toRad(double d);
double toDeg(double r);
double julianDay(int yr, int mo, int dy, int hr, int mn, int sc);
SunPos sunPosition(double latDeg, double lonDeg, double JD);
