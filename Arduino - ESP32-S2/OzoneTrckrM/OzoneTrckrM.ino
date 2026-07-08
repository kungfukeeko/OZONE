#include <math.h>
#include <time.h>                  // system clock (set by NTP)
#include <sys/time.h>              // settimeofday() for the build-time fallback
#include <string.h>
#include "Arduino.h"
#include <WiFi.h>                  // WiFi.status() -> live NeoPixel status during the run
#include <Adafruit_NeoPixel.h>    // onboard status LED (this Feather variant has NO TFT)
#include "ADS131M04.h"            // ADC object
#include "ESP32Servo.h"
#include "ESP32PWM.h"
#include "datalink.h"             // WiFi upload + FFat flash log
#include "SolarCalculator.h"      // SunPos + NOAA sun-position math
#include "sunflower.h"            // two-axis LDR sun tracker (steers the mount onto the sun)
#include "driver/gpio.h"          // gpio_hold_en() -- latch servo pins across deep sleep
#include "esp_sleep.h"            // esp_deep_sleep_start()

// NOTE: GPS (TinyGPSPlus), RTC (DS3231) and BME280 are intentionally NOT included here.
//       They are not wired on this board yet -- to be added back later. The clock comes
//       from NTP (datalink ntpSyncUTC -> ESP32 system time) and the coordinates are
//       hardcoded below. There is no TFT on this Feather variant -> status is shown on
//       the onboard NeoPixel instead.

// ---------------- ANALOG BOARD / ADC PINS ----------------
#define EN 11
#define FILTER 5
#define ADC_DRDY 6
#define ADC_CS 9

#define ADC_SCK 36
#define ADC_MISO 37
#define ADC_MOSI 35

// onboard BOOT button (GPIO0): press during a run to end it early WITHOUT WiFi
#define STOP_BTN 0

// ---------------- SUN TRACKER (servos + quadrant LDRs) ----------------
#define TRK_H_PIN 12   // horizontal (azimuth) servo
#define TRK_V_PIN 13   // vertical (elevation) servo
#define LDR_LD    A0   // LDR bottom-left
#define LDR_RT    A1   // LDR top-right
#define LDR_RD    A2   // LDR bottom-right
#define LDR_LT    A3   // LDR top-left

// ---------------- STATUS LED (onboard NeoPixel) ----------------
// Feather ESP32-S2 (non-TFT): one WS2812 on PIN_NEOPIXEL, powered by NEOPIXEL_POWER.
#ifndef PIN_NEOPIXEL
#define PIN_NEOPIXEL 33
#endif
#ifndef NEOPIXEL_POWER
#define NEOPIXEL_POWER 21
#endif
#define LED_BRIGHTNESS 40          // 0-255; keep modest (battery + eye comfort)

#define UTC_OFFSET_HOURS 2

// ---------------- MEASUREMENT CONFIG ----------------
// The run length is set by LOOPS (there is no MAX_SUBLISTS anymore). Each loop() does:
//   PHASE 0  -> 1 offset block (SAMPLES_FOR_CAL sub-samples, filter at pos_cal)
//   PHASE 1  -> BLOCKS_PER_PHASE measurement blocks (SAMPLES_PER_BLOCK each, filter pos1)
//   PHASE 2  -> BLOCKS_PER_PHASE measurement blocks (SAMPLES_PER_BLOCK each, filter pos2)
// => rows per loop      = 1 + 2 * BLOCKS_PER_PHASE
//    total stored rows  = LOOPS * (1 + 2 * BLOCKS_PER_PHASE)
#define LOOPS               250   // <-- dictates how long the measurement lasts (see estimateRunSeconds)
#define BLOCKS_PER_PHASE     10   // measurement blocks (rows) per filter phase
#define SAMPLES_PER_BLOCK    20   // sub-samples averaged into one measurement row's mean/stddev
#define OVERSAMPLE           15   // ADC conversions averaged into each sub-sample
#define SAMPLES_FOR_CAL     100   // sub-samples in the single PHASE-0 offset block
#define ADC_ODR_SPS         250   // ADC output data rate (High-Res, OSR 16384)

#define SERVO_SETTLE_TIME   400   // ms mechanical settle after a filter move
#define FILTER_EASE_MS      700   // ms ease-in-out duration for a filter servo move
#define ADC_DISCARD_SAMPLES  25   // conversions dropped after CMD_WAKEUP (settling spike)

// ---------------- DATA DELIVERY (flash backup + WiFi upload) ----------------
#define UPLOAD_WINDOW_MS     600000UL  // grind WiFi delivery up to this long at end of run
#define UPLOAD_RETRY_GAP_MS   15000UL  // wait between WiFi delivery attempts
#define PUTTY_GRACE_MS         6000UL  // at boot: wait so PuTTY can attach before the serial dump

// ---------------- HARDWARE OBJECTS ----------------
ADS131M04 adc1;
Servo     filter_servo;
Adafruit_NeoPixel statusLED(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// -------- FILTER SERVO POSITIONS (servo pulse width, microseconds) --------
int pos1    = 550;
int pos2    = 2440;
int pos_cal = 1500;   // "dark"/offset position used by PHASE 0

// SKALA
float FS = 8388608.0;
float PREF_ADC = 1200.0;

double Vch0;
double Vch1;

// hardcoded measurement site (GPS to be added back later)
double gpsLat = 44.5302;
double gpsLon = 14.4706;

// ---------------- INTERRUPT ----------------
volatile bool drdy_fall = false;   // data ready, set by interrupt on every conversion
void IRAM_ATTR adc_ready_interrupt() { drdy_fall = true; }

// ---------------- RUN STATE ----------------
size_t measurement_index = 0;      // rows written this run
bool   loggingToFlash    = false;  // true once the run file is open
bool   flashHealthy      = false;  // did the last block's flash write succeed? (status LED)
bool   stopRequested     = false;  // set by the BOOT button -> end the run early

// measurement-start metadata (written into the CSV header)
char   startDate[11] = "";   // YYYY-MM-DD (local)
char   startTime[9]  = "";   // HH:MM:SS  (local)

// ---------------- FUNCTION PROTOTYPES ----------------
void setup_ADC_CARD();
void filter_rotation(int pos);
void offset_measure(float &mean_ch0, float &sd_ch0, float &mean_ch1, float &sd_ch1);
void measurement(float &mean_ch0, float &sd_ch0, float &mean_ch1, float &sd_ch1);
void storeMeasurement(int flag, float el, float m0, float sd0, float m1, float sd1);

// ============================================================================
//  STATUS LED (NeoPixel)  -- replaces the TFT status dots / messages
//  Colours (r,g,b): green = ok, red = fail, blue = WiFi connecting, cyan = IP/link,
//  green-ish = NTP, orange = warning/flash-only, magenta = sending data.
// ============================================================================
// packed 0xRRGGBB colours (single value -> safe inside ternaries)
#define COL_OK    0x006E00
#define COL_FAIL  0x960000
#define COL_WIFI  0x000096
#define COL_NET   0x006E6E
#define COL_NTP   0x009628
#define COL_WARN  0xA04600
#define COL_SEND  0x960096
#define COL_BOOT  0x282828

void statusBegin() {
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);   // power the onboard NeoPixel rail
  statusLED.begin();
  statusLED.setBrightness(LED_BRIGHTNESS);
  statusLED.clear();
  statusLED.show();
}
void statusSet(uint32_t rgb) { statusLED.setPixelColor(0, rgb); statusLED.show(); }
void statusOff() { statusSet(0); }
void statusBlink(uint32_t rgb, uint8_t times, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < times; i++) { statusSet(rgb); delay(onMs); statusOff(); delay(offMs); }
}
void statusPowerOff() { statusOff(); digitalWrite(NEOPIXEL_POWER, LOW); }

// One short blink per measurement block -> live "flash + WiFi" health during the run:
//   green  = flash write OK and WiFi up | orange = flash OK but WiFi down | red = flash write failed.
void statusBlock() {
  if (!flashHealthy)                      statusBlink(COL_FAIL, 1, 45, 0);
  else if (WiFi.status() != WL_CONNECTED) statusBlink(COL_WARN, 1, 45, 0);
  else                                    statusBlink(COL_OK,   1, 45, 0);
}

// ---------------- RUN-LENGTH ESTIMATE ----------------
// seconds(one block) = SAMPLES * OVERSAMPLE / ADC_ODR_SPS   (the ADC integration time)
// seconds(one filter move) = ease + settle + discard-window
// seconds(one loop) = offset block + 2*BLOCKS_PER_PHASE measurement blocks + 3 filter moves
// total run seconds = LOOPS * seconds(one loop)
static float secondsPerBlock(int subsamples) { return (float)subsamples * OVERSAMPLE / (float)ADC_ODR_SPS; }
static float secondsPerFilterMove() {
  return (FILTER_EASE_MS + SERVO_SETTLE_TIME + ADC_DISCARD_SAMPLES * 1000.0f / ADC_ODR_SPS) / 1000.0f;
}
static uint32_t estimateRunSeconds() {
  float perLoop = secondsPerBlock(SAMPLES_FOR_CAL)                       // PHASE 0 (1 offset block)
                + 2 * BLOCKS_PER_PHASE * secondsPerBlock(SAMPLES_PER_BLOCK)  // PHASE 1 + PHASE 2
                + 3 * secondsPerFilterMove();                            // 3 filter moves per loop
  return (uint32_t)(LOOPS * perLoop);
}

// ---------------- CLOCK (NTP, with build-time fallback) ----------------
// NTP is the only clock source (no RTC). If NTP is unreachable, seed the ESP32 system
// clock from the compile time so the sun-position calc and CSV timestamps are at least
// approximately right (flagged with an orange status blink).
static void seedClockFromBuildTime() {
  static const char* mns = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char mon[4] = {0}; int d = 1, y = 2025, hh = 0, mm = 0, ss = 0;
  sscanf(__DATE__, "%3s %d %d", mon, &d, &y);
  sscanf(__TIME__, "%d:%d:%d", &hh, &mm, &ss);
  int mo = 0; for (int i = 0; i < 12; i++) if (strncmp(mon, mns + 3 * i, 3) == 0) { mo = i; break; }
  setenv("TZ", "UTC0", 1); tzset();
  struct tm t = {0};
  t.tm_year = y - 1900; t.tm_mon = mo; t.tm_mday = d;
  t.tm_hour = hh; t.tm_min = mm; t.tm_sec = ss;
  time_t asUTC = mktime(&t);                          // TZ=UTC0 -> tm treated as UTC
  time_t utc   = asUTC - (time_t)UTC_OFFSET_HOURS * 3600;  // build time is local -> to UTC
  struct timeval tv = { utc, 0 };
  settimeofday(&tv, nullptr);
}

// ---------------- SETUP ----------------
void setup() {
  setCpuFrequencyMhz(80);

  // Hold the analog board OFF long enough to discharge/cold-boot the ADC
  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);

  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // never block on USB writes when no serial host is draining them

  statusBegin();
  statusSet(COL_BOOT);       // dim white = booting

  delay(2500);               // USB CDC enumeration + discharge the analog rail (cold-boot the ADC)

  digitalWrite(EN, HIGH);    // power the analog board up (cold) before touching the shared SPI bus
  delay(100);

  // Deselect the ADC on the shared SPI bus before anything else uses it.
  pinMode(ADC_CS, OUTPUT);
  digitalWrite(ADC_CS, HIGH);

  pinMode(STOP_BTN, INPUT_PULLUP);   // BOOT button = WiFi-free manual early-stop

  filter_servo.attach(FILTER, 400, 2600);
  filter_servo.writeMicroseconds(pos_cal);

  // ------- FIND THE SUN -------
  sunflowerBegin(TRK_H_PIN, TRK_V_PIN, LDR_LT, LDR_RT, LDR_LD, LDR_RD);
  sunflowerFindSun(30000);

  // ------- FLASH (mount only; recovery gets first crack before any reformat) -------
  bool flashOK = storageBegin();
  Serial.println(flashOK ? "FLASH: ready" : "FLASH: MOUNT FAILED");
  statusBlink(flashOK ? COL_OK : COL_FAIL, 2, 150, 150);   // checkpoint: FLASH

  // ------- WIFI + COMMAND LINK -------
  statusSet(COL_WIFI);                                     // blue = connecting
  bool wifiOK = commandLinkBegin();
  if (wifiOK) {
    Serial.print("IP: "); Serial.println(deviceAddress());
    statusBlink(COL_NET, 2, 150, 150);                    // checkpoint: WiFi + IP
  } else {
    Serial.println("WiFi: unavailable (no remote stop / NTP / upload)");
    statusBlink(COL_FAIL, 3, 150, 150);
  }

  // ------- CLOCK: NTP (system time), build-time fallback -------
  uint32_t ntpEpoch = 0;
  if (ntpSyncUTC(ntpEpoch)) {
    Serial.println("CLOCK: NTP synced (UTC)");
    statusBlink(COL_NTP, 2, 150, 150);                    // checkpoint: NTP ok
  } else {
    seedClockFromBuildTime();
    Serial.println("CLOCK: NTP failed -> seeded from build time");
    statusBlink(COL_WARN, 3, 150, 150);                   // checkpoint: NTP fallback
  }

  // ------- RECOVER THE PREVIOUS RUN (serial + WiFi) before it is overwritten -------
  if (lastRunExists()) {
    Serial.println("==== LAST RUN: recovering over serial + WiFi ====");
    statusSet(COL_SEND);
    delay(PUTTY_GRACE_MS);                  // time to attach PuTTY / start `ncat -l 5000`
    if (Serial) dumpLastRunToSerial();
    bool sentWifi = uploadLastRun();
    Serial.printf("Last run recovery: wifi=%s\n", sentWifi ? "sent" : "not sent");
    statusBlink(sentWifi ? COL_OK : COL_WARN, 2, 200, 150);
  }

  // Only NOW is it safe to reformat a corrupt FS -- recovery has had its chance.
  if (flashOK && !storageEnsureWritable()) {
    flashOK = false;
    Serial.println("FLASH: UNWRITABLE");
    statusBlink(COL_FAIL, 3, 200, 150);
  }

  // ------- ADC -------
  setup_ADC_CARD();
  attachInterrupt(ADC_DRDY, adc_ready_interrupt, FALLING);
  drdy_fall = false;

  Serial.println("---- MEASUREMENTS START ----");

  // ------- MEASUREMENT-START METADATA (local time + coordinates) -------
  time_t nowUTC   = time(nullptr);
  time_t nowLocal = nowUTC + (time_t)UTC_OFFSET_HOURS * 3600;
  struct tm lt;   gmtime_r(&nowLocal, &lt);   // gmtime on the shifted epoch = local wall clock
  snprintf(startDate, sizeof(startDate), "%04d-%02d-%02d", lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday);
  snprintf(startTime, sizeof(startTime), "%02d:%02d:%02d", lt.tm_hour, lt.tm_min, lt.tm_sec);
  Serial.printf("Start: %s %s | lat %.6f lon %.6f\n", startDate, startTime, gpsLat, gpsLon);

  uint32_t est = estimateRunSeconds();
  Serial.printf("Plan: %d loops x (1 + 2*%d) = %lu rows;  estimated %lu s (~%lu min)\n",
                LOOPS, BLOCKS_PER_PHASE,
                (unsigned long)((uint32_t)LOOPS * (1 + 2 * BLOCKS_PER_PHASE)),
                (unsigned long)est, (unsigned long)(est / 60));

  // ------- OPEN THE RUN FILE (stream each block to flash from here on) -------
  MeasurementMeta meta = { startTime, startDate, gpsLat, gpsLon };
  loggingToFlash = runFileBegin(meta);
  flashHealthy   = loggingToFlash;
  Serial.println(loggingToFlash ? "FLASH: logging to /lastrun.csv" : "FLASH: NO -- serial only");
  statusBlink(loggingToFlash ? COL_OK : COL_FAIL, 2, 150, 150);   // checkpoint: logging
}

// ---------------- LOOP ----------------
void loop() {
  static uint32_t loopCount = 0;
  loopCount++;

  // ------- RE-AIM AT THE SUN (every 5th loop) -------
  if (loopCount % 5 == 0) sunflowerFindSun(5000);

  // ------- SUN POSITION (from the NTP-set system clock, UTC) -------
  time_t nowUTC = time(nullptr);
  struct tm u;  gmtime_r(&nowUTC, &u);
  double JD  = julianDay(u.tm_year + 1900, u.tm_mon + 1, u.tm_mday, u.tm_hour, u.tm_min, u.tm_sec);
  SunPos sun = sunPosition(gpsLat, gpsLon, JD);
  float  el  = (float)sun.elevation;

  // ------- PHASE 0 : OFFSET (one block, filter at pos_cal, flag = 0) -------
  filter_rotation(pos_cal);
  {
    float m0, sd0, m1, sd1;
    offset_measure(m0, sd0, m1, sd1);
    storeMeasurement(0, el, m0, sd0, m1, sd1);   // flag 0 = offset
    statusBlock();
    if (digitalRead(STOP_BTN) == LOW) { delay(30); if (digitalRead(STOP_BTN) == LOW) stopRequested = true; }
    wifiKeepalive();
    if (stopCommandReceived()) stopRequested = true;
  }

  // ------- PHASE 1 : filter pos1 (flag = 1) -------
  filter_rotation(pos1);
  for (int i = 0; i < BLOCKS_PER_PHASE; i++) {
    float m0, sd0, m1, sd1;
    measurement(m0, sd0, m1, sd1);
    storeMeasurement(1, el, m0, sd0, m1, sd1);
    statusBlock();
    if (digitalRead(STOP_BTN) == LOW) { delay(30); if (digitalRead(STOP_BTN) == LOW) stopRequested = true; }
    wifiKeepalive();
    if (stopCommandReceived()) stopRequested = true;
  }

  // ------- PHASE 2 : filter pos2 (flag = 1) -------
  filter_rotation(pos2);
  for (int i = 0; i < BLOCKS_PER_PHASE; i++) {
    float m0, sd0, m1, sd1;
    measurement(m0, sd0, m1, sd1);
    storeMeasurement(1, el, m0, sd0, m1, sd1);
    statusBlock();
    if (digitalRead(STOP_BTN) == LOW) { delay(30); if (digitalRead(STOP_BTN) == LOW) stopRequested = true; }
    wifiKeepalive();
    if (stopCommandReceived()) stopRequested = true;
    // NOTE: a stop request lets the current phase finish -> a run always ends with
    // complete phases (never a partial 10-block phase).
  }

  // ------- END OF RUN -------
  // Ends when LOOPS is reached, OR the BOOT button was pressed, OR a 'stop' arrived over WiFi.
  if (loopCount >= LOOPS || stopRequested || stopCommandReceived()) {
    filter_rotation(pos_cal);            // park at the dark position

    runFileEnd();
    Serial.printf("---- RUN COMPLETE: %u rows on flash (%lu loops) ----\n",
                  (unsigned)measurement_index, (unsigned long)loopCount);

    if (loggingToFlash) {
      if (Serial) dumpLastRunToSerial();          // 1) serial (WiFi-free)

      statusSet(COL_SEND);                        // magenta = sending data
      Serial.println("Sending data...");
      bool sent = uploadLastRun();                // 2) WiFi -> PC listener
      uint32_t t0 = millis();
      while (!sent && millis() - t0 < UPLOAD_WINDOW_MS) {
        if (digitalRead(STOP_BTN) == LOW) { Serial.println("WiFi retry skipped (BOOT)"); break; }
        statusBlink(COL_SEND, 1, 80, 0);
        uint32_t leftS = (UPLOAD_WINDOW_MS - (millis() - t0)) / 1000;
        Serial.printf("WiFi upload failed -- on flash, retrying (%lus left, BOOT=skip). "
                      "PC listener up?  ncat -l 5000 > data.csv\n", (unsigned long)leftS);
        delay(UPLOAD_RETRY_GAP_MS);
        sent = uploadLastRun();
      }
      statusBlink(sent ? COL_OK : COL_WARN, 3, 200, 150);
      Serial.println(sent ? "Upload done" : "On flash + serial");
    } else {
      statusBlink(COL_FAIL, 3, 200, 150);
      Serial.println("NO FLASH -- check serial");
    }
    delay(800);

    // ------- POWER DOWN -------
    Serial.println("Shutting down system...");
    adc1.sendcmd(CMD_STANDBY);
    delay(5);
    adc1.end();
    digitalWrite(ADC_CS,   LOW);
    digitalWrite(ADC_SCK,  LOW);
    digitalWrite(ADC_MOSI, LOW);
    digitalWrite(EN, LOW);
    storageEnd();                        // clean unmount -> next boot mounts cleanly
    statusPowerOff();                    // LED off + cut its rail

    // Latch the RTC-capable tracker servo pins so they hold level (no twitch at shutdown).
    gpio_hold_en((gpio_num_t)TRK_H_PIN);
    gpio_hold_en((gpio_num_t)TRK_V_PIN);
    gpio_deep_sleep_hold_en();

    Serial.println("Going into deep sleep...");
    // NO wake source by design: one run per manual placement; power-cycle / RESET to restart.
    esp_deep_sleep_start();
  }
}

// ---------------- ADC INIT ----------------
// 2kHz 0b0000001100010010 , 4kHz 0b0000001100001110 , 8kHz 0b0000001100001010
void setup_ADC_CARD() {
  uint32_t ADC_CLOCK_REG = 0b0000011100011110; // High-res, OSR 16384 (~250 SPS); CH0/CH1/CH2 enabled
  uint32_t ADC_CFG_REG   = 0b0000011000000000; // delay before measurement begins
  adc1.begin(ADC_SCK, ADC_MISO, ADC_MOSI, ADC_CS);
  adc1.sendcmd(CMD_RESET);
  delay(10);
  adc1.sendcmd(CMD_STANDBY);
  delay(10);
  adc1.writeRegister(REG_CLOCK, ADC_CLOCK_REG);
  adc1.writeRegister(REG_CFG, ADC_CFG_REG);
  adc1.writeRegister(THRSHLD_LSB, 0b0000000000000000);

  adc1.setChannelPGA(0, 0); // GAIN 1
  adc1.setChannelPGA(1, 0);
  adc1.setChannelPGA(2, 0);

  adc1.setInputChannelSelection(0, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc1.setInputChannelSelection(1, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc1.setInputChannelSelection(2, INPUT_CHANNEL_MUX_AIN0P_AIN0N);

  // No hardware offset register is programmed: the offset is measured as PHASE 0 (flag 0)
  // and subtracted in post-processing, so both offset and signal rows are raw voltages.

  delay(10);
  adc1.sendcmd(CMD_WAKEUP);
  delay(10);
  adc1.readADC();   // flush any latched DRDY so the first interrupt edge is genuine
}

// ---------------- FILTER SERVO EASING ----------------
static int filterCurrentUs = 1500;   // last commanded filter position (= pos_cal at boot)

// Smoothly move the filter servo to `target` over durationMs (smoothstep ease-in-out).
static void easeServoTo(int target, uint32_t durationMs) {
  int start = filterCurrentUs;
  if (target != start) {
    const uint32_t STEP_MS = 15;                 // ~66 position updates per second
    uint32_t steps = durationMs / STEP_MS;
    if (steps < 1) steps = 1;
    for (uint32_t i = 1; i <= steps; i++) {
      float t = (float)i / (float)steps;
      float e = t * t * (3.0f - 2.0f * t);       // smoothstep
      filter_servo.writeMicroseconds(start + (int)lround((target - start) * e));
      delay(STEP_MS);
    }
    filter_servo.writeMicroseconds(target);
  }
  filterCurrentUs = target;
}

// ---------------- FILTER ROTATION ----------------
void filter_rotation(int pos) {
  adc1.sendcmd(CMD_STANDBY);       // ADC quiet while the servo moves (no noise in the data)
  delay(10);

  Serial.print("Filter to "); Serial.print(pos); Serial.println(" us");

  easeServoTo(pos, FILTER_EASE_MS);
  delay(SERVO_SETTLE_TIME);        // mechanical settle before measuring

  adc1.sendcmd(CMD_WAKEUP);
  delay(10);

  // discard the wake settling spike so it never lands in a measurement
  drdy_fall = false;
  int discarded = 0;
  unsigned long t0 = millis();
  while (discarded < ADC_DISCARD_SAMPLES) {
    if (drdy_fall) { drdy_fall = false; adc1.readADC(); discarded++; }
    if (millis() - t0 > 2000) break;   // ADC not responding
  }
}

// ---------------- OFFSET MEASURE (PHASE 0) ----------------
// Identical scheme to measurement() but with SAMPLES_FOR_CAL sub-samples -> a longer,
// lower-noise "dark" block. Stored as a normal row with flag 0.
void offset_measure(float &mean_v0, float &stddev_v0, float &mean_v1, float &stddev_v1) {
  double m0 = 0, m1 = 0, s0 = 0, s1 = 0;   // Welford running mean / M2 over sub-samples
  int collected = 0;

  while (collected < SAMPLES_FOR_CAL) {
    double acc0 = 0, acc1 = 0;
    int got = 0;
    unsigned long t0 = millis();
    while (got < OVERSAMPLE) {
      if (drdy_fall) {
        drdy_fall = false;
        adcOutput temp = adc1.readADC();
        acc0 += temp.ch0;
        acc1 += temp.ch1;
        got++;
      }
      if (millis() - t0 > 2000) { Serial.println("ADC timeout during offset block"); break; }
    }
    if (got == 0) break;

    Vch0 = (acc0 / got) / FS * PREF_ADC;   // averaged counts -> mV
    Vch1 = (acc1 / got) / FS * PREF_ADC;

    double d0 = Vch0 - m0; m0 += d0 / (collected + 1); s0 += d0 * (Vch0 - m0);
    double d1 = Vch1 - m1; m1 += d1 / (collected + 1); s1 += d1 * (Vch1 - m1);
    collected++;
  }
  int denom = (collected > 1) ? (collected - 1) : 1;
  mean_v0 = m0; mean_v1 = m1;
  float var0 = s0 / denom; if (var0 < 0) var0 = 0; stddev_v0 = sqrt(var0);
  float var1 = s1 / denom; if (var1 < 0) var1 = 0; stddev_v1 = sqrt(var1);

  Serial.print("OFFSET  ch0: "); Serial.print(mean_v0); Serial.print(" +/- "); Serial.print(stddev_v0);
  Serial.print(" mV   ch1: ");   Serial.print(mean_v1); Serial.print(" +/- "); Serial.print(stddev_v1);
  Serial.println(" mV");
}

// ---------------- MEASUREMENT ----------------
void measurement(float &mean_v0, float &stddev_v0, float &mean_v1, float &stddev_v1) {
  double m0 = 0, m1 = 0, s0 = 0, s1 = 0;
  int collected = 0;

  while (collected < SAMPLES_PER_BLOCK) {
    double acc0 = 0, acc1 = 0;
    int got = 0;
    unsigned long t0 = millis();
    while (got < OVERSAMPLE) {
      if (drdy_fall) {
        drdy_fall = false;
        adcOutput temp = adc1.readADC();
        acc0 += temp.ch0;
        acc1 += temp.ch1;
        got++;
      }
      if (millis() - t0 > 2000) { Serial.println("ADC timeout: no DRDY (analog board disconnected?)"); break; }
    }
    if (got == 0) break;

    Vch0 = (acc0 / got) / FS * PREF_ADC;   // averaged counts -> mV
    Vch1 = (acc1 / got) / FS * PREF_ADC;

    double d0 = Vch0 - m0; m0 += d0 / (collected + 1); s0 += d0 * (Vch0 - m0);
    double d1 = Vch1 - m1; m1 += d1 / (collected + 1); s1 += d1 * (Vch1 - m1);
    collected++;
  }
  int denom = (collected > 1) ? (collected - 1) : 1;
  mean_v0 = m0; mean_v1 = m1;
  float var0 = s0 / denom; if (var0 < 0) var0 = 0; stddev_v0 = sqrt(var0);
  float var1 = s1 / denom; if (var1 < 0) var1 = 0; stddev_v1 = sqrt(var1);

  Serial.print("Prvi kanal: ");   Serial.print(mean_v0); Serial.print(" mV   "); Serial.print(stddev_v0);
  Serial.print("   Drugi kanal: "); Serial.print(mean_v1); Serial.print(" mV   "); Serial.println(stddev_v1);
}

// ---------------- STORE ----------------
// Row format: flag,elevation,mean_ch0,stddev_ch0,mean_ch1,stddev_ch1
//   flag = 0 -> offset block (PHASE 0) | flag = 1 -> measurement block (PHASE 1/2)
void storeMeasurement(int flag, float el, float m0, float sd0, float m1, float sd1) {
  if (loggingToFlash) flashHealthy = runFileAppendRow(flag, el, m0, sd0, m1, sd1);

  // Live serial copy in the parser's bracketed format (independent backup via PuTTY).
  Serial.printf("[%d,%.4f,%.4f,%.4f,%.4f,%.4f]\n", flag, el, m0, sd0, m1, sd1);

  measurement_index++;
}
