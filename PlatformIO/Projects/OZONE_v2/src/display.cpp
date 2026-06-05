#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <RTClib.h>
#include <math.h>
#include "display.h"

// ---------------- TFT (built-in ST7789, Feather ESP32-S2 TFT) ----------------
#ifndef TFT_CS
  #define TFT_CS        42
  #define TFT_DC        40
  #define TFT_RST       41
  #define TFT_BACKLITE  45
  #define TFT_I2C_POWER  7
#endif

// ---------------- DISPLAY COLORS ----------------
#define C_BG      0x0000  // black
#define C_TIME    0x07E0  // green
#define C_DATE    0xFFE0  // yellow
#define C_COORD   0x3C7F  // blue
#define C_AZ      0xF81F  // magenta
#define C_LABEL   0x7BEF  // light gray
#define C_SEP     0x2945  // dark divider
#define C_CH1     0x07FF  // cyan
#define C_CH2     0xFD20  // orange
#define C_GPS_OK  0x07E0
#define C_GPS_BAD 0xF800

static Adafruit_ST7789 tft(&SPI, TFT_CS, TFT_DC, TFT_RST);

// ================================================================== COLOR HELPERS
// blend two RGB colors (0-255 components) by t (0..1) -> RGB565
static uint16_t lerp565(int r1, int g1, int b1, int r2, int g2, int b2, float t) {
  int r = r1 + (int)((r2 - r1) * t);
  int g = g1 + (int)((g2 - g1) * t);
  int b = b1 + (int)((b2 - b1) * t);
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static inline int      iclamp(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }
static inline int      ilerp(int a, int b, float t) { return a + (int)((b - a) * t); }
static inline uint16_t rgb565(int r, int g, int b) {
  r = iclamp(r); g = iclamp(g); b = iclamp(b);
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// elevation -> colour (blue below horizon ... white high sun)
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

// ================================================================== BOOT/SHUTDOWN SCENE
// Sun over the sea: sky deepens to daytime BLUE as the sun rises (night -> blue),
// over "Kauai" teal water. The sun stays centred and only moves UP/DOWN.
#define SKY_W       240
#define SKY_H       135
#define SEA_HORIZON 100    // y where sea meets sky (~3/4 of the screen is sky)
#define ANIM_SUN_R  13

// Daytime blue sky: deeper blue up top, paler blue toward the horizon, fading to
// night as d -> 0.  fy: 0 = top of screen, 1 = horizon.
static uint16_t skyColor(float fy, float d) {
  int r = ilerp( 56, 150, fy);
  int g = ilerp(120, 200, fy);
  int b = ilerp(214, 236, fy);
  return rgb565(ilerp(8, r, d), ilerp(10, g, d), ilerp(30, b, d));
}

// teal water: lighter at the surface, fading to night.  fy: 0 = waterline, 1 = foreground.
static uint16_t seaColor(float fy, float d) {
  int r = ilerp( 44, 14, fy), g = ilerp(176, 84, fy), b = ilerp(178, 108, fy);
  return rgb565(ilerp(6, r, d), ilerp(14, g, d), ilerp(28, b, d));
}

// sun disc / glow by height fraction h: coral low -> warm pale gold high
static uint16_t sunDiscColor(float h) {
  if (h < 0.5f) return lerp565(232,  88, 60, 255, 170,  90, h / 0.5f);
  return              lerp565(255, 170, 90, 255, 250, 225, (h - 0.5f) / 0.5f);
}
static uint16_t sunGlowColor(float h) {
  if (h < 0.5f) return lerp565(150,  45, 50, 255, 150, 110, h / 0.5f);
  return              lerp565(255, 150, 110, 255, 220, 180, (h - 0.5f) / 0.5f);
}

// filled circle clipped to rows ABOVE clipY (y < clipY only). Used for the sun so
// its underwater half is never drawn -> no per-frame flicker below the horizon.
static void fillCircleTop(int cx, int cy, int r, uint16_t color, int clipY) {
  for (int dy = -r; dy <= r; dy++) {
    int y = cy + dy;
    if (y < 0 || y >= clipY) continue;
    int dx = (int)(sqrtf((float)(r * r - dy * dy)) + 0.5f);
    tft.drawFastHLine(cx - dx, y, 2 * dx + 1, color);
  }
}

// little puffy cloud from a few overlapping blobs
static void drawCloud(int cx, int cy, int s, uint16_t c) {
  tft.fillCircle(cx,         cy,          s,               c);
  tft.fillCircle(cx - s,     cy + s / 3,  (int)(s * 0.7f), c);
  tft.fillCircle(cx + s,     cy + s / 3,  (int)(s * 0.7f), c);
  tft.fillCircle(cx - s / 2, cy - s / 3,  (int)(s * 0.6f), c);
  tft.fillCircle(cx + s / 2, cy - s / 4,  (int)(s * 0.7f), c);
  tft.fillRect(cx - s - 2,   cy + s / 3,  2 * s + 4, s / 2 + 1, c);   // flat base
}

// Render one full frame. d = dayness / sun-height: 0 = night & sun below the sea
// horizon, 1 = bright day & sun high. The sun is centred and moves only up/down.
static void drawSkyScene(float d) {
  const int W = SKY_W, H = SKY_H, HOR = SEA_HORIZON;
  if (d < 0) d = 0; if (d > 1) d = 1;

  // --- sky gradient (night -> blue) ---
  for (int y = 0; y < HOR; y++)
    tft.drawFastHLine(0, y, W, skyColor((float)y / HOR, d));

  // --- stars (night only, fade out as it brightens) ---
  float starV = 1.0f - d / 0.5f;
  if (starV > 0.0f) {
    uint8_t sb = (uint8_t)(220 * starV);
    uint16_t star = rgb565(sb, sb, sb);
    for (int i = 0; i < 34; i++) {
      int sx = (i * 53 + 17) % W;
      int sy = (i * 29 + 7)  % (HOR - 8);
      tft.drawPixel(sx, sy, star);
      if (i % 4 == 0) tft.drawPixel(sx + 1, sy, star);
    }
  }

  // --- sun: centred, moves only vertically with d. yBase sits far enough below
  //     the horizon that the whole disc + halo are hidden under the water at d=0. ---
  int   sunX  = W / 2;
  int   yBase = HOR + 24;
  int   sunY  = (int)(yBase - sinf(d * (float)M_PI / 2.0f) * (yBase - 24));
  float hf = (float)(HOR - sunY) / (HOR - 24); if (hf < 0) hf = 0; if (hf > 1) hf = 1;
  uint16_t disc = sunDiscColor(hf);
  uint16_t glow = sunGlowColor(hf);
  fillCircleTop(sunX, sunY, ANIM_SUN_R + 5, glow, HOR);                                   // halo
  fillCircleTop(sunX, sunY, ANIM_SUN_R,     disc, HOR);                                   // disc
  fillCircleTop(sunX - 4, sunY - 4, ANIM_SUN_R / 3, lerp565(255, 200, 130, 255, 255, 235, hf), HOR);

  // --- little drifting clouds (faint at night, white by day) ---
  uint16_t cloud = lerp565(44, 46, 64, 250, 252, 255, d);
  int cdrift = (int)(d * 14);
  drawCloud( 58 + cdrift, 42, 5, cloud);    // kept off-centre so they don't sit on the sun
  drawCloud(150 + cdrift, 30, 6, cloud);
  drawCloud(206 + cdrift, 20, 5, cloud);

  // --- sea (full width) ---
  int beachTop = H - 5;
  for (int y = HOR; y < beachTop; y++)
    tft.drawFastHLine(0, y, W, seaColor((float)(y - HOR) / (beachTop - HOR), d));

  // --- soft bright band right along the horizon ---
  tft.drawFastHLine(0, HOR, W, lerp565(22, 24, 36, 210, 232, 240, d));

  // --- centred sun glitter straight down the water (shimmering, tapering) ---
  if (hf > 0.0f) {
    int dr = (disc >> 8) & 0xF8, dg = (disc >> 3) & 0xFC, db = (disc << 3) & 0xF8;
    for (int y = HOR + 1; y < beachTop; y++) {
      float dist = (float)(y - HOR) / (beachTop - HOR);
      int jit = (int)(4.0f * sinf(y * 0.7f + d * 8.0f));
      int hw  = 2 + (int)(5.0f * (1.0f - dist));
      float mix = (0.2f + 0.6f * (1.0f - dist)) * hf;
      uint16_t rc = lerp565(40, 150, 150, dr, dg, db, mix);
      if (((y + (int)(d * 12)) & 1) == 0)            // sparkle: skip alternate rows
        tft.drawFastHLine(sunX - hw + jit, y, hw * 2, rc);
    }
  }

  // --- thin sandy foreground ---
  tft.fillRect(0, beachTop, W, H - beachTop, rgb565(ilerp(16, 210, d), ilerp(14, 176, d), ilerp(12, 150, d)));
}

// boot: night warms into day while the sun rises straight up out of the sea
void splashSunrise() {
  const int FRAMES = 300;
  for (int f = 0; f < FRAMES; f++) {
    float t = (float)f / (FRAMES - 1);
    float e = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);  // smootherstep 0 -> 1
    drawSkyScene(e);

    tft.setTextSize(1);
    tft.setCursor(78, SKY_H - 10);
    tft.setTextColor(lerp565(60, 70, 90, 235, 245, 255, e));
    tft.print("initializing...");
    delay(1);
  }
}

// shutdown: day fades to night while the sun sinks straight down into the sea
void splashSunset() {
  const int FRAMES = 300;
  for (int f = 0; f < FRAMES; f++) {
    float t = (float)f / (FRAMES - 1);
    float s = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);  // smootherstep 0 -> 1
    float d = 1.0f - s;                                      // day -> night
    drawSkyScene(d);

    tft.setTextSize(1);
    tft.setCursor(74, SKY_H - 10);
    tft.setTextColor(lerp565(235, 245, 255, 40, 50, 70, s)); // fade the label out
    tft.print("shutting down...");
    delay(1);
  }
}

// ================================================================== DRAW SCREEN
// Layout (240×135 landscape), everything size 2, three groups:
//  y=5    HH:MM:SS (green)  DD/MM/YY (yellow)            [GPS dot]
//  y=25   N44.5328 E14.4690 (blue)
//  y=46   ─ separator ─
//  y=52   EL +XX.XX deg (elev-colored)
//  y=72   AZ XXX.XX NN  (magenta)
//  y=92   ─ separator ─
//  y=98   Ch1: mean  stddev  (cyan)
//  y=118  Ch2: mean  stddev  (orange)
void drawScreen(const DateTime& now, const SunPos& sun,
                float m0, float sd0, float m1, float sd1) {
  char buf[40];
  bool havePos = hasFix || hasStoredPos;
  tft.fillScreen(C_BG);
  tft.setTextWrap(false);
  tft.setTextSize(2);

  // separators
  tft.drawFastHLine(0, 46, 240, C_SEP);
  tft.drawFastHLine(0, 92, 240, C_SEP);

  // ----- time + date -----
  tft.setTextColor(C_TIME);
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  tft.setCursor(4, 5);
  tft.print(buf);

  tft.setTextColor(C_DATE);
  snprintf(buf, sizeof(buf), "%02d/%02d/%02d", now.day(), now.month(), now.year() % 100);
  tft.setCursor(112, 5);
  tft.print(buf);

  // GPS status dot: red=no fix, orange=fix/no sync, green=fix+RTC synced
  uint16_t dotColor = !hasFix ? C_GPS_BAD : (rtcSyncedGPS ? C_GPS_OK : 0xFD20);
  tft.fillCircle(229, 12, 9, dotColor);
  tft.setTextSize(1);
  tft.setTextColor(0x0000);
  tft.setCursor(226, 8);
  tft.print(!hasFix ? "?" : (rtcSyncedGPS ? "G" : "g"));
  tft.setTextSize(2);

  // ----- coordinates -----
  tft.setTextColor(C_COORD);
  if (havePos) {
    snprintf(buf, sizeof(buf), "%c%.4f %c%.4f",
             gpsLat >= 0 ? 'N' : 'S', fabs(gpsLat),
             gpsLon >= 0 ? 'E' : 'W', fabs(gpsLon));
  } else {
    snprintf(buf, sizeof(buf), "--.----  ---.----");
  }
  tft.setCursor(4, 25);
  tft.print(buf);

  // ----- elevation -----
  tft.setTextColor(C_LABEL);
  tft.setCursor(4, 52);
  tft.print("EL ");
  if (havePos) {
    tft.setTextColor(elevColor(sun.elevation));
    snprintf(buf, sizeof(buf), "%+6.2f", sun.elevation);
    tft.print(buf);
    tft.setTextColor(C_LABEL);
    tft.print(" deg");
  } else {
    tft.print("  ---.--");
  }

  // ----- azimuth -----
  tft.setTextColor(C_LABEL);
  tft.setCursor(4, 72);
  tft.print("AZ ");
  if (havePos) {
    tft.setTextColor(C_AZ);
    snprintf(buf, sizeof(buf), "%6.2f", sun.azimuth);
    tft.print(buf);
    tft.print(" ");
    tft.print(azCardinal(sun.azimuth));
  } else {
    tft.print("  ---.--");
  }

  // ----- channel readings -----
  tft.setTextColor(C_CH1);
  snprintf(buf, sizeof(buf), "Ch1:%8.2f %6.2f", m0, sd0);
  tft.setCursor(4, 98);
  tft.print(buf);

  tft.setTextColor(C_CH2);
  snprintf(buf, sizeof(buf), "Ch2:%8.2f %6.2f", m1, sd1);
  tft.setCursor(4, 118);
  tft.print(buf);
}

// ================================================================== INIT / ERROR
void displayInit() {
  // power the TFT / I2C rail (also feeds the DS3231) before using the panel
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(50);
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  tft.init(135, 240);
  tft.setRotation(3);
  tft.fillScreen(C_BG);
  tft.setTextWrap(false);
}

void displayError(const char* msg) {
  tft.fillScreen(C_BG);
  tft.setTextColor(C_GPS_BAD);
  tft.setTextSize(2);
  tft.setCursor(4, 4);
  tft.print(msg);
}

void displayOff() {
  tft.fillScreen(C_BG);              // clear to black
  tft.enableDisplay(false);          // ST7789 display off
  digitalWrite(TFT_BACKLITE, LOW);   // backlight off -> screen dark
}
