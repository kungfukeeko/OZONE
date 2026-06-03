#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <TinyGPSPlus.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <math.h>

// Adafruit Feather ESP32-S2 TFT built-in display pins
// (defined in board variant, listed here for clarity)
#ifndef TFT_CS
  #define TFT_CS        42
  #define TFT_DC        40
  #define TFT_RST       41
  #define TFT_BACKLITE  45
  #define TFT_I2C_POWER  7
#endif

// GPS on Serial1 — connect NEO-M8N TX → GPIO2 (Feather RX), NEO-M8N RX ← GPIO1 (Feather TX)
// A0-A3 (GPIO15-18) are reserved for LDRs
#define GPS_RX    2
#define GPS_TX    1
#define GPS_BAUD  9600

// ------------------------------------------------------------------ hardware
Adafruit_ST7789  tft(&SPI, TFT_CS, TFT_DC, TFT_RST);
RTC_DS3231       rtc;
TinyGPSPlus      gps;
HardwareSerial   gpsSerial(1);

// ------------------------------------------------------------------ colors
#define C_BG      0x0000  // black
#define C_TITLE   0x07FF  // cyan
#define C_TIME    0x07E0  // green
#define C_DATE    0x001F  // blue
#define C_COORD   0xFFE0  // yellow
#define C_AZ      0xF81F  // magenta
#define C_LABEL   0x7BEF  // light gray
#define C_SEP     0x2945  // dark divider
#define C_GPS_OK  0x07E0
#define C_GPS_BAD 0xF800

// ------------------------------------------------------------------ state
double  gpsLat = 0.0, gpsLon = 0.0;
bool    hasFix        = false;
bool    rtcSyncedGPS  = false;  // true after RTC has been set from GPS time

struct SunPos { double elevation, azimuth; };

// ------------------------------------------------------------------ NOAA solar algorithm
// Follows NOAA Solar Calculator spreadsheet formulas exactly.
// JD must be in UTC.

static double toRad(double d) { return d * M_PI / 180.0; }
static double toDeg(double r) { return r * 180.0 / M_PI; }

static double julianDay(int yr, int mo, int dy, int hr, int mn, int sc) {
    if (mo <= 2) { yr--; mo += 12; }
    int A = yr / 100;
    int B = 2 - A + A / 4;
    double jd = (int)(365.25 * (yr + 4716)) + (int)(30.6001 * (mo + 1))
                + dy + B - 1524.5;
    return jd + (hr + mn / 60.0 + sc / 3600.0) / 24.0;
}

static SunPos sunPosition(double latDeg, double lonDeg, double JD) {
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

// ------------------------------------------------------------------ helpers
static uint16_t elevColor(double elev) {
    if (elev <  0.0) return 0x001F;  // blue  — below horizon
    if (elev < 10.0) return 0xFC00;  // red-orange — near horizon
    if (elev < 30.0) return 0xFD20;  // orange
    if (elev < 60.0) return 0xFFE0;  // yellow
    return 0xFFFF;                   // white — high sun
}

static const char* azCardinal(double az) {
    static const char* dirs[8] = {"N","NE","E","SE","S","SW","W","NW"};
    return dirs[(int)((az + 22.5) / 45.0) % 8];
}

// ------------------------------------------------------------------ display
// Layout (240×135 landscape):
//  y=4   ZENITH (size2)                [GPS dot]
//  y=22  ─ separator ─
//  y=26  HH:MM:SS (size3)
//  y=51  ─ separator ─
//  y=55  DD/MM/YYYY (size2)
//  y=73  ─ separator ─
//  y=77  LAT: ... LON: ... (size1)
//  y=88  ─ separator ─
//  y=91  ELEVATION label   AZIMUTH label (size1)
//  y=100 +XX.X deg         XXX.X NE     (size2)

static void drawScreen(const DateTime& now, const SunPos& sun) {
    tft.fillScreen(C_BG);

    // separator lines
    tft.drawFastHLine(0,  22, 240, C_SEP);
    tft.drawFastHLine(0,  51, 240, C_SEP);
    tft.drawFastHLine(0,  73, 240, C_SEP);
    tft.drawFastHLine(0,  88, 240, C_SEP);

    // title
    tft.setTextWrap(false);
    tft.setTextColor(C_TITLE);
    tft.setTextSize(2);
    tft.setCursor(4, 4);
    tft.print("SUN POSITION");

    // GPS status dot: red=no fix, orange=fix/no sync, green=fix+RTC synced
    uint16_t dotColor = !hasFix ? C_GPS_BAD : (rtcSyncedGPS ? C_GPS_OK : 0xFD20);
    tft.fillCircle(229, 11, 7, dotColor);
    tft.setTextColor(0x0000);
    tft.setTextSize(1);
    tft.setCursor(226, 7);  // center 6x8 char inside circle at (229,11)
    tft.print(!hasFix ? "?" : (rtcSyncedGPS ? "G" : "g"));

    // time
    char buf[40];
    tft.setTextColor(C_TIME);
    tft.setTextSize(3);
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    tft.setCursor(4, 26);
    tft.print(buf);

    // date
    tft.setTextColor(C_DATE);
    tft.setTextSize(2);
    snprintf(buf, sizeof(buf), "%02d/%02d/%04d", now.day(), now.month(), now.year());
    tft.setCursor(4, 55);
    tft.print(buf);

    // coordinates
    tft.setTextColor(C_COORD);
    tft.setTextSize(1);
    if (hasFix) {
        snprintf(buf, sizeof(buf), "LAT %+09.4f   LON %+010.4f", gpsLat, gpsLon);
    } else {
        snprintf(buf, sizeof(buf), "LAT  ---.----   LON  ----.----");
    }
    tft.setCursor(4, 77);
    tft.print(buf);

    // elevation label + value
    tft.setTextColor(C_LABEL);
    tft.setTextSize(1);
    tft.setCursor(4, 91);
    tft.print("ELEVATION");

    tft.setTextSize(2);
    if (hasFix) {
        tft.setTextColor(elevColor(sun.elevation));
        snprintf(buf, sizeof(buf), "%+6.1f", sun.elevation);
        tft.setCursor(4, 101);
        tft.print(buf);
        tft.setTextSize(1);
        tft.setTextColor(C_LABEL);
        tft.print(" deg");
    } else {
        tft.setTextColor(C_LABEL);
        tft.setCursor(4, 101);
        tft.print("  ---.-");
    }

    // azimuth label + value
    tft.setTextColor(C_LABEL);
    tft.setTextSize(1);
    tft.setCursor(132, 91);
    tft.print("AZIMUTH");

    tft.setTextSize(2);
    if (hasFix) {
        tft.setTextColor(C_AZ);
        snprintf(buf, sizeof(buf), "%6.1f", sun.azimuth);
        tft.setCursor(132, 101);
        tft.print(buf);
        tft.print(" ");
        tft.print(azCardinal(sun.azimuth));
    } else {
        tft.setTextColor(C_LABEL);
        tft.setCursor(132, 101);
        tft.print("  ---.-");
    }
}

// ------------------------------------------------------------------ setup / loop
void setup() {
    Serial.begin(115200);
    delay(2000);  // wait for USB CDC to enumerate on ESP32-S2

    // power up I2C bus (Feather ESP32-S2 TFT has a power switch on pin 7)
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
    delay(10);

    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);

    tft.init(135, 240);
    tft.setRotation(3);
    tft.fillScreen(C_BG);
    tft.setTextWrap(false);

    // boot message
    tft.setTextColor(C_TITLE);
    tft.setTextSize(2);
    tft.setCursor(4, 4);
    tft.print("ZENITH  init...");

    Wire.begin();
    if (!rtc.begin()) {
        tft.fillScreen(C_BG);
        tft.setTextColor(0xF800);
        tft.setTextSize(2);
        tft.setCursor(4, 4);
        tft.print("DS3231 ERROR");
        while (1) delay(1000);
    }

    // Set RTC from compile time only when it has lost power (first use or battery dead).
    // DS3231 stores UTC — ensure your build machine is set to UTC when flashing.
    if (rtc.lostPower()) {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    Serial.println("GPS serial started, waiting for NMEA...");

    tft.fillScreen(C_BG);
}

unsigned long lastDraw = 0;

void loop() {
    // keep feeding GPS parser continuously
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gps.encode(c);
        Serial.write(c);  // TODO: remove after GPS confirmed working
    }

    if (gps.location.isValid()) {
        gpsLat = gps.location.lat();
        gpsLon = gps.location.lng();
        hasFix = true;
    }

    // Sync RTC from GPS UTC time once on first valid fix
    if (!rtcSyncedGPS && gps.date.isValid() && gps.time.isValid()) {
        rtc.adjust(DateTime(gps.date.year(), gps.date.month(), gps.date.day(),
                            gps.time.hour(), gps.time.minute(), gps.time.second()));
        rtcSyncedGPS = true;
    }

    if (millis() - lastDraw >= 1000UL || lastDraw == 0) {
        lastDraw = millis();

        DateTime now = rtc.now();
        // DS3231 time must be UTC for the NOAA algorithm to give correct results
        double JD   = julianDay(now.year(), now.month(), now.day(),
                                now.hour(), now.minute(), now.second());
        SunPos sun  = sunPosition(gpsLat, gpsLon, JD);
        drawScreen(now, sun);
    }
}
