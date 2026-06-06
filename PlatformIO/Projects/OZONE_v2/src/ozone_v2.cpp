#include "Arduino.h"
#include "ADS131M04.h"  // ADC object
#include <ESP32Servo.h>
#include <Wire.h>
#include <RTClib.h>
#include <TinyGPSPlus.h>
#include <Preferences.h>
#include <math.h>
#include "display.h"    // TFT object, colours, animations and drawScreen live here
#include "datalink.h"   // WiFi upload of the measurement list to a computer

// ---------------- ANALOG BOARD / ADC PINS ----------------
#define EN 11
#define S0 5
#define ADC_DRDY 6
#define ADC_CS 9

#define ADC_SCK 36
#define ADC_MISO 37
#define ADC_MOSI 35

// ---------------- GPS (Serial1) ----------------
// NEO-M8N TX -> GPIO2 (Feather RX), NEO-M8N RX <- GPIO1 (Feather TX)
#define GPS_RX    2
#define GPS_TX    1
#define GPS_BAUD  9600

#define UTC_OFFSET_HOURS 2

// ---------------- MEASUREMENT CONFIG ----------------
// 2000 sublists, rotate filters every 10 sublists, calibrate every 200
#define MAX_SUBLISTS 100 //2000
//#define MAX_CAL 200     // Kalibrira se samo jednom na početku mjerenja
#define BLOCKS_PER_PHASE 10   // Koliko mjerenja prije okretanja filtera
#define SAMPLES_PER_BLOCK 20  // Koliko sample-ova za jedno mjerenje
#define SAMPLES_FOR_CAL 100
#define SERVO_SETTLE_TIME 1000
#define ADC_DISCARD_SAMPLES 5

// ---------------- HARDWARE OBJECTS ----------------
ADS131M04 adc1;        //objekt ADC-a
adcOutput adc_podatak; //output tip, pristup kanalima 0-3, status
Servo filter_servo;

RTC_DS3231      rtc;
TinyGPSPlus     gps;
HardwareSerial  gpsSerial(1);
Preferences     prefs;

// ---------------- STATE ----------------
uint8_t cal_cycles = 0;
// -------- FILTER SERVO POSITIONS --------
int pos1 = 10;
int pos2 = 180;
int pos_cal = 100;

// SKALA
int32_t neg_scale = -8388608;
int32_t pos_scale = 8388607;
float FS = 8388608.0;

float NREF_ADC = -1200.0;
float PREF_ADC = 1200.0;

double Vch0;
double Vch1;
double Vch2;
float Vref = 101.75; //izmjereno stolnim DMM-mom dok je uređaj napajan USB-C kabelom, 101.13 mV kad je napajan baterijom

// GPS / sun-position state
double gpsLat = 0.0, gpsLon = 0.0;
bool   hasFix       = false;
bool   hasStoredPos = false;  // true if NVS holds coordinates from a previous GPS fix
bool   rtcSyncedGPS = false;

// ---------------- INTERRUPT ----------------
volatile uint16_t drdy_div = 0;
volatile bool drdy_fall = false; //data ready, seta se interuptom
void IRAM_ATTR adc_ready_interrupt() {
    drdy_div++;
    if (drdy_div >= 100) {
        drdy_div = 0;
        drdy_fall = true;
    }
}

// ---------------- DYNAMIC STORAGE ----------------
// each row: elevation, mean_ch0, stddev_ch0, mean_ch1, stddev_ch1
float (*measurements)[5] = NULL;
size_t measurement_index = 0;

// ---------------- FUNCTION PROTOTYPES ----------------
void   setup_ADC_CARD();
void   filter_rotation(int pos);
void   offsetCalibration(float &offset_v0, float &offset_v1);
void   measurement(float &mean_ch0, float &sttdev_ch0, float &mean_ch1, float &sttdev_ch1);
void   storeMeasurement(float el, float a, float b, float c, float d);

void   pollGPS();
double toRad(double d);
double toDeg(double r);

double julianDay(int yr, int mo, int dy, int hr, int mn, int sc);
SunPos sunPosition(double latDeg, double lonDeg, double JD);

// ---------------- SETUP ----------------
void setup() {
  // The ADS131M04 only starts converting reliably after a *cold* boot, but it
  // also shares the SPI bus with the built-in TFT (SCK/MOSI/MISO). An unpowered
  // ADC loads those lines and the TFT goes blank. 
  // So: hold the analog board OFF long enough to discharge/cold-boot the ADC 
  // (the screen is blank anyway during USB enumeration), THEN power it up BEFORE any TFT/SPI activity.
  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);

  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // never block on USB writes when no serial host is draining them
  delay(2500);               // USB CDC enumeration + discharge the analog rail (cold-boot the ADC)

  digitalWrite(EN, HIGH);    // power the analog board up (cold) before touching the shared SPI bus
  delay(100);

  // Deselect the ADC on the shared SPI bus BEFORE the TFT uses it. Otherwise the
  // powered ADC sees the TFT's clock/data on SCK/MOSI, desyncs its SPI frame and
  // stops producing DRDY -> measurement times out with zero readings.
  pinMode(ADC_CS, OUTPUT);
  digitalWrite(ADC_CS, HIGH);

  // init the TFT (also powers the shared I2C rail the DS3231 needs) + boot animation
  displayInit();
  splashSunrise();

  Wire.begin();
  if (!rtc.begin()) {
    Serial.println("DS3231 ERROR");
    displayError("DS3231 ERROR");
    while (1) delay(1000);
  }
  if (rtc.lostPower() || rtc.now().year() < 2024 || rtc.now().year() > 2035) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // load last known coordinates from flash so sun position shows before a fix
  prefs.begin("ozone", true);  // read-only
  gpsLat = prefs.getDouble("lat", 0.0);
  gpsLon = prefs.getDouble("lon", 0.0);
  hasStoredPos = prefs.getBool("valid", false);
  prefs.end();

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS serial started, waiting for NMEA...");

  setup_ADC_CARD();
  attachInterrupt(ADC_DRDY, adc_ready_interrupt, FALLING);
  drdy_div  = 0;
  drdy_fall = false;

  measurements = (float (*)[5])malloc(MAX_SUBLISTS * sizeof(*measurements));
  if (!measurements) {
    Serial.println("Memory allocation failed!");
    while (1);
  }
  delay(800);
  Serial.println("---- MEASUREMENTS START ----");
  // ------- OFFSET CALIBRATION -------
  filter_rotation(pos_cal);
  float offset_v0, offset_v1;
  offsetCalibration(offset_v0, offset_v1);
}

// ---------------- LOOP ----------------
void loop() {

  // ------- SUN POSITION -------
  pollGPS();
  DateTime utcNow   = rtc.now();
  double   JD       = julianDay(utcNow.year(), utcNow.month(), utcNow.day(),
                                utcNow.hour(), utcNow.minute(), utcNow.second());
  SunPos   sun      = sunPosition(gpsLat, gpsLon, JD);
  float    el = (float)sun.elevation;

  // ------- PHASE  1 -------
  filter_rotation(pos1);
  for (int i =0; i<BLOCKS_PER_PHASE; i++) {
    float mean_ch0, stddev_ch0, mean_ch1, stddev_ch1;
    measurement(mean_ch0, stddev_ch0, mean_ch1, stddev_ch1);
    storeMeasurement(el, mean_ch0, stddev_ch0, mean_ch1, stddev_ch1);
    DateTime nowLocal = DateTime(rtc.now().unixtime() + UTC_OFFSET_HOURS * 3600UL);
    drawScreen(nowLocal, sun, mean_ch0, stddev_ch0, mean_ch1, stddev_ch1);
  }
  // ------- PHASE  2 -------
  filter_rotation(pos2);
  for (int i =0; i<BLOCKS_PER_PHASE; i++) {
    float mean_ch0, stddev_ch0, mean_ch1, stddev_ch1;
    measurement(mean_ch0, stddev_ch0, mean_ch1, stddev_ch1);
    storeMeasurement(el, mean_ch0, stddev_ch0, mean_ch1, stddev_ch1);
    DateTime nowLocal = DateTime(rtc.now().unixtime() + UTC_OFFSET_HOURS * 3600UL);
    drawScreen(nowLocal, sun, mean_ch0, stddev_ch0, mean_ch1, stddev_ch1);
  }
/*
  // ------- CALIBRATION -------
  if (cal_cycles >= MAX_CAL) {
    kalibracija tokom mjerenja
  }
*/
  // ------- PRINT AND SHUTDOWN -------
  if (measurement_index >= MAX_SUBLISTS) {
    Serial.println("---- DATA START ----");
    for (size_t i = 0; i < MAX_SUBLISTS; i++) {
      Serial.print("[");
      Serial.print(measurements[i][0], 4); Serial.print(",");
      Serial.print(measurements[i][1], 4); Serial.print(",");
      Serial.print(measurements[i][2], 4); Serial.print(",");
      Serial.print(measurements[i][3], 4); Serial.print(",");
      Serial.print(measurements[i][4], 4);
      Serial.println("]");
    }
    Serial.println("---- DATA END ----");

    // ------- SEND DATA OVER WIFI (before freeing the buffer) -------
    displayMessage("Sending data...");
    bool sent = uploadMeasurementsCSV(measurements, MAX_SUBLISTS);
    displayMessage(sent ? "Upload done" : "Upload failed", sent);
    delay(1500);

    Serial.println("Shutting down system...");
    if (measurements != NULL) {
      free(measurements);
      measurements = NULL;
    }

    splashSunset();
    displayOff();

    adc1.sendcmd(CMD_STANDBY);
    delay(5);

    adc1.end();
    // Tri-state SPI pins
    digitalWrite(ADC_CS, LOW);
    digitalWrite(ADC_SCK, LOW);
    digitalWrite(ADC_MOSI, LOW);

    filter_rotation(pos1);

    digitalWrite(EN, LOW);
    Serial.println("Going into deep sleep...");
    esp_deep_sleep_start();
  }
}

// ---------------- ADC INIT ----------------
// 2kHz 0b0000001100010010 , 4kHz 0b0000001100001110 , 8kHz 0b0000001100001010
void setup_ADC_CARD() {

  uint32_t ADC_CLOCK_REG = 0b0000011100010010; //High res, Oversampling ratio 4:2 512 oko 2khz, enable samo 0 i 1 ch 001 je 256
  uint32_t ADC_CFG_REG = 0b0000011000000000; //Delay before measurment begins, za kasnije mozda enable
  adc1.begin(ADC_SCK, ADC_MISO, ADC_MOSI, ADC_CS);
  adc1.sendcmd(CMD_RESET);
  delay(10);
  adc1.sendcmd(CMD_STANDBY);
  delay(10);
  adc1.writeRegister(REG_CLOCK, ADC_CLOCK_REG);
  adc1.writeRegister(REG_CFG, ADC_CFG_REG); //
  adc1.writeRegister(THRSHLD_LSB, 0b0000000000000000);

  adc1.setChannelPGA(0, 0); // GAIN 1
  adc1.setChannelPGA(1, 0);
  adc1.setChannelPGA(2, 0);

  adc1.setInputChannelSelection(0, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc1.setInputChannelSelection(1, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc1.setInputChannelSelection(2, INPUT_CHANNEL_MUX_AIN0P_AIN0N);

  delay(10);
  adc1.sendcmd(CMD_WAKEUP);
  delay(10);
  adc1.readADC();   // flush any latched DRDY so the first interrupt edge is genuine
}

// ---------------- FILTER ROTATION ----------------
void filter_rotation(int pos) {

  adc1.sendcmd(CMD_STANDBY);
  delay(10);

  Serial.print("Filter in position ");
  Serial.println(pos);

  filter_servo.attach(S0);
  filter_servo.write(pos);
  delay(SERVO_SETTLE_TIME);
  filter_servo.detach();

  adc1.sendcmd(CMD_WAKEUP);
  delay(10);
}

// ---------------- OFFSET CALIBRATION ----------------
void offsetCalibration(float &offset_v0, float &offset_v1) {
  int32_t m0 = 0;
  int32_t m1 = 0;
  int collected = 0;

  while (collected < SAMPLES_FOR_CAL) {
    if (drdy_fall) {
      drdy_fall = false;

      adcOutput temp = adc1.readADC();

      int32_t delta0 = temp.ch0 - m0;
      m0 += delta0 / (collected + 1);

      int32_t delta1 = temp.ch1 - m1;
      m1 += delta1 / (collected + 1);
      collected++;
    }
  }
  adc1.setChannelOffsetCalibration(0, m0);
  adc1.setChannelOffsetCalibration(1, m1);

  offset_v0 = m0 / FS * PREF_ADC;
  offset_v1 = m1 / FS * PREF_ADC;

  Serial.print("Reference voltage: ");
  Serial.print(Vref);
  Serial.println(" mV");
  Serial.print("Offset Calibration: ");
  Serial.print(offset_v0);
  Serial.print(" mV   ");
  Serial.print(offset_v1);
  Serial.println(" mV");
}

// ---------------- READ AND COMPUTE ----------------
void measurement(float &mean_v0, float &stddev_v0, float &mean_v1, float &stddev_v1) {
  double m0 = 0;
  double m1 = 0;
  double s0 = 0;
  double s1 = 0;
  int collected = 0;

  unsigned long t0 = millis();
  while (collected < SAMPLES_PER_BLOCK) {

    if (drdy_fall) {
      drdy_fall = false;

      adcOutput temp = adc1.readADC();
      Vch0 = temp.ch0 / FS * PREF_ADC;
      Vch1 = temp.ch1 / FS * PREF_ADC;

      double delta0 = Vch0 - m0;
      m0 += delta0 / (collected + 1);
      s0 += delta0 * (Vch0 - m0);

      double delta1 = Vch1 - m1;
      m1 += delta1 / (collected + 1);
      s1 += delta1 * (Vch1 - m1);

      collected++;
    }
    if (millis() - t0 > 2000) {   // ADC not responding
      Serial.println("ADC timeout: no DRDY (analog board disconnected?)");
      break;
    }
  }
  int denom = (collected > 1) ? (collected - 1) : 1;
  mean_v0 = m0;
  mean_v1 = m1;
  float variance0 = s0 / denom;
  float variance1 = s1 / denom;
  if (variance0 < 0) variance0 = 0;
  stddev_v0 = sqrt(variance0);
  if (variance1 < 0) variance1 = 0;
  stddev_v1 = sqrt(variance1);

  Serial.print("Prvi kanal: " );
  Serial.print(mean_v0);
  Serial.print(" mV   ");
  Serial.print(stddev_v0);

  Serial.print("   Drugi kanal: " );
  Serial.print(mean_v1);
  Serial.print(" mV   ");
  Serial.println(stddev_v1);
}

// ---------------- STORE ----------------
void storeMeasurement(float el, float a, float b, float c, float d) {
  if (measurement_index >= MAX_SUBLISTS)
    return;

  measurements[measurement_index][0] = el;
  measurements[measurement_index][1] = a;
  measurements[measurement_index][2] = b;
  measurements[measurement_index][3] = c;
  measurements[measurement_index][4] = d;

  measurement_index++;
  cal_cycles++;
}

// ---------------- GPS POLL + RTC SYNC ----------------
void pollGPS() {
  while (gpsSerial.available())
    gps.encode(gpsSerial.read());

  if (gps.location.isValid()) {
    gpsLat = gps.location.lat();
    gpsLon = gps.location.lng();
    if (!hasFix) {
      prefs.begin("ozone", false);
      prefs.putDouble("lat", gpsLat);
      prefs.putDouble("lon", gpsLon);
      prefs.putBool("valid", true);
      prefs.end();
      hasStoredPos = true;
    }
    hasFix = true;
  }

  // Sync RTC from GPS UTC once on first valid fix.
  // location.isValid() guards against pre-fix garbage time (year 2043).
  if (!rtcSyncedGPS && gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
    rtc.adjust(DateTime(gps.date.year(), gps.date.month(), gps.date.day(),
                        gps.time.hour(), gps.time.minute(), gps.time.second()));
    rtcSyncedGPS = true;
  }
}

// ---------------- MATH HELPERS ----------------
double toRad(double d) { return d * M_PI / 180.0; }
double toDeg(double r) { return r * 180.0 / M_PI; }

// ================================================================== JULIAN DAY
// NOAA Solar Calculator. JD must be in UTC.
double julianDay(int yr, int mo, int dy, int hr, int mn, int sc) {
  if (mo <= 2) { yr--; mo += 12; }
  int A = yr / 100;
  int B = 2 - A + A / 4;
  double jd = (int)(365.25 * (yr + 4716)) + (int)(30.6001 * (mo + 1))
              + dy + B - 1524.5;
  return jd + (hr + mn / 60.0 + sc / 3600.0) / 24.0;
}

// ================================================================== SUN POSITION
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

