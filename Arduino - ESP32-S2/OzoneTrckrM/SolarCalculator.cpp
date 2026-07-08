#include <Arduino.h>   // constrain()
#include <math.h>
#include "SolarCalculator.h"

double toRad(double d) { return d * M_PI / 180.0; }
double toDeg(double r) { return r * 180.0 / M_PI; }

// NOAA Solar Calculator. JD must be in UTC.
double julianDay(int yr, int mo, int dy, int hr, int mn, int sc) {
  if (mo <= 2) { yr--; mo += 12; }
  int A = yr / 100;
  int B = 2 - A + A / 4;
  double jd = (int)(365.25 * (yr + 4716)) + (int)(30.6001 * (mo + 1))
              + dy + B - 1524.5;
  return jd + (hr + mn / 60.0 + sc / 3600.0) / 24.0;
}

SunPos sunPosition(double latDeg, double lonDeg, double JD) {
  double JC = (JD - 2451545.0) / 36525.0;

  double L0 = fmod(280.46646 + JC * (36000.76983 + JC * 0.0003032), 360.0);
  if (L0 < 0) L0 += 360.0;
  double M = 357.52911 + JC * (35999.05029 - 0.0001537 * JC);
  double e = 0.016708634 - JC * (0.000042037 + 0.0000001267 * JC);

  double C = sin(toRad(M))   * (1.914602 - JC * (0.004817 + 0.000014 * JC))
           + sin(toRad(2*M)) * (0.019993 - 0.000101 * JC)
           + sin(toRad(3*M)) * 0.000289;

  double sunLon = L0 + C;
  double omega  = 125.04 - 1934.136 * JC;
  double lambda = sunLon - 0.00569 - 0.00478 * sin(toRad(omega));

  double eps0 = 23.0 + (26.0 + (21.448 - JC * (46.815 + JC * (0.00059 - JC * 0.001813))) / 60.0) / 60.0;
  double eps  = eps0 + 0.00256 * cos(toRad(omega));

  double sinDec = sin(toRad(eps)) * sin(toRad(lambda));
  double dec    = toDeg(asin(sinDec));

  double y   = tan(toRad(eps / 2)) * tan(toRad(eps / 2));
  double EqT = 4.0 * toDeg(y * sin(2 * toRad(L0))
             - 2 * e * sin(toRad(M))
             + 4 * e * y * sin(toRad(M)) * cos(2 * toRad(L0))
             - 0.5 * y * y * sin(4 * toRad(L0))
             - 1.25 * e * e * sin(2 * toRad(M)));

  // fraction of day past midnight UTC (0.0–1.0)
  double dayFrac = JD - floor(JD) - 0.5;
  if (dayFrac < 0) dayFrac += 1.0;

  double TST = fmod(dayFrac * 1440.0 + EqT + 4.0 * lonDeg, 1440.0);
  if (TST < 0) TST += 1440.0;

  double HA = (TST / 4.0 < 0) ? TST / 4.0 + 180.0 : TST / 4.0 - 180.0;

  double cosZ = constrain(
      sin(toRad(latDeg)) * sin(toRad(dec)) +
      cos(toRad(latDeg)) * cos(toRad(dec)) * cos(toRad(HA)),
      -1.0, 1.0);
  double zenith = toDeg(acos(cosZ));
  double elev   = 90.0 - zenith;

  // atmospheric refraction correction
  double refr;
  if      (elev > 85.0)    refr = 0.0;
  else if (elev > 5.0)     refr = ( 58.1 / tan(toRad(elev))
                                  -  0.07 / pow(tan(toRad(elev)), 3)
                                  + 0.000086 / pow(tan(toRad(elev)), 5)) / 3600.0;
  else if (elev > -0.575)  refr = (1735 + elev * (-518.2 + elev * (103.4 + elev * (-12.79 + elev * 0.711)))) / 3600.0;
  else                     refr = (-20.772 / tan(toRad(elev))) / 3600.0;
  elev += refr;

  // azimuth (clockwise from north)
  double sinZ = sin(toRad(zenith));
  double az   = 0.0;
  if (sinZ > 1e-10) {
    double cosAz = constrain(
        (sin(toRad(latDeg)) * cosZ - sin(toRad(dec))) / (cos(toRad(latDeg)) * sinZ),
        -1.0, 1.0);
    az = (HA > 0) ? fmod(toDeg(acos(cosAz)) + 180.0, 360.0)
                  : fmod(540.0 - toDeg(acos(cosAz)), 360.0);
  }

  return {elev, az};
}
