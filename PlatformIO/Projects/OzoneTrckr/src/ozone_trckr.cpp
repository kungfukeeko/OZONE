#include "Arduino.h"
#include "ADS131M04.h"  // ADC object
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_BME280.h>  // temp/humidity/pressure sensor (I2C)
#include <RTClib.h>
#include <TinyGPSPlus.h>
#include <Preferences.h>
#include <math.h>
#include "display.h"          // TFT object, colours, animations and drawScreen live here
#include "datalink.h"         // WiFi upload of the measurement list to a computer
#include "SolarCalculator.h"  // SunPos + NOAA sun-position math
#include "sunflower.h"        // two-axis LDR sun tracker (steers the mount onto the sun)

// ---------------- ANALOG BOARD / ADC PINS ----------------
#define EN 11
#define FILTER 5
#define ADC_DRDY 6
#define ADC_CS 9

#define ADC_SCK 36
#define ADC_MISO 37
#define ADC_MOSI 35

// ---------------- SUN TRACKER (servos + quadrant LDRs) ----------------
#define TRK_H_PIN 12   // horizontal (azimuth) servo
#define TRK_V_PIN 13   // vertical (elevation) servo
#define LDR_LT    A3   // LDR top-left
#define LDR_RT    A1   // LDR top-right
#define LDR_LD    A0   // LDR bottom-left
#define LDR_RD    A2   // LDR bottom-right

// ---------------- GPS (Serial1) ----------------
// NEO-M8N TX -> GPIO2 (Feather RX), NEO-M8N RX <- GPIO1 (Feather TX)
#define GPS_RX    2
#define GPS_TX    1
#define GPS_BAUD  9600

#define UTC_OFFSET_HOURS 2

// ---------------- MEASUREMENT CONFIG ----------------
// 2000 sublists, rotate filters every 10 sublists, calibrate every 200
#define MAX_SUBLISTS 2000
//#define MAX_CAL 200     // Kalibrira se samo jednom na početku mjerenja
#define BLOCKS_PER_PHASE 10   // Koliko mjerenja prije okretanja filtera
#define SAMPLES_PER_BLOCK 20  // reported sub-samples per measurement block (sets the stored stddev)
#define OVERSAMPLE 15         // ADC conversions averaged into each reported sub-sample (extra integration on top of hardware OSR)
#define SAMPLES_FOR_CAL 100
#define SERVO_SETTLE_TIME 2000
#define ADC_DISCARD_SAMPLES 5

// ---------------- HARDWARE OBJECTS ----------------
ADS131M04 adc1;        //objekt ADC-a
Servo filter_servo;

RTC_DS3231      rtc;
Adafruit_BME280 bme;          // shares the I2C bus with the DS3231
bool            bmeOK = false;
TinyGPSPlus     gps;
HardwareSerial  gpsSerial(1);
Preferences     prefs;

// ---------------- STATE ----------------
uint8_t cal_cycles = 0;
// -------- FILTER SERVO POSITIONS (servo pulse width, microseconds) --------
// 400 / 2600 are the ends of the widened SG90 range; pos_cal sits halfway
int pos1    = 650;
int pos2    = 2700;
int pos_cal = 1500;

// SKALA
float FS = 8388608.0;
float PREF_ADC = 1200.0;

double Vch0;
double Vch1;
float Vref = 101.75; //izmjereno stolnim DMM-mom dok je uređaj napajan USB-C kabelom, 101.13 mV kad je napajan baterijom

// GPS / sun-position state
double gpsLat = 0.0, gpsLon = 0.0;
bool   hasFix       = false;
bool   hasStoredPos = false;  // true if NVS holds coordinates from a previous GPS fix
bool   rtcSyncedGPS = false;
bool   gpsAwake     = true;   // false once we've fixed and put the GPS into backup

// ---------------- INTERRUPT ----------------
// Flag EVERY conversion (no ÷100 decimation). measurement() reads each DRDY and
// averages OVERSAMPLE of them per reported sub-sample, so no ADC data is wasted.
volatile bool drdy_fall = false; //data ready, set by interrupt on every conversion
void IRAM_ATTR adc_ready_interrupt() {
    drdy_fall = true;
}

// ---------------- DYNAMIC STORAGE ----------------
// each row: elevation, mean_ch0, stddev_ch0, mean_ch1, stddev_ch1
float (*measurements)[5] = NULL;
size_t measurement_index = 0;

// ---------------- MEASUREMENT-START METADATA (emitted as CSV header) ----------------
char   startDate[11] = "";   // YYYY-MM-DD (local)
char   startTime[9]  = "";   // HH:MM:SS  (local)
double startLat = 0.0, startLon = 0.0;
float  startTemp = 0, startHum = 0, startPress = 0;
bool   startEnvValid = false;

// ---------------- FUNCTION PROTOTYPES ----------------
void   setup_ADC_CARD();
void   filter_rotation(int pos);
void   offsetCalibration(float &offset_v0, float &offset_v1);
void   measurement(float &mean_ch0, float &sttdev_ch0, float &mean_ch1, float &sttdev_ch1);
void   storeMeasurement(float el, float a, float b, float c, float d);

static void gpsEnterBackup();
void   pollGPS();

// ---------------- SETUP ----------------
void setup() {
  // Drop the core clock for the long measurement phase.
  // APB stays at 80 MHz, so SPI/ADC, servo PWM, UART/GPS and WiFi are unaffected.
  setCpuFrequencyMhz(80);

  // Hold the analog board OFF long enough to discharge/cold-boot the ADC 
  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);

  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // never block on USB writes when no serial host is draining them
  delay(2500);               // USB CDC enumeration + discharge the analog rail (cold-boot the ADC)

  digitalWrite(EN, HIGH);    // power the analog board up (cold) before touching the shared SPI bus
  delay(100);

  // Deselect the ADC on the shared SPI bus BEFORE the TFT uses it.
  pinMode(ADC_CS, OUTPUT);
  digitalWrite(ADC_CS, HIGH);

  displayInit();

  filter_servo.attach(FILTER, 400, 2600);
  filter_servo.writeMicroseconds(pos_cal);

  sunflowerBegin(TRK_H_PIN, TRK_V_PIN, LDR_LT, LDR_RT, LDR_LD, LDR_RD);
  sunflowerFindSun(20000);

  Wire.begin();
  if (!rtc.begin()) {
    Serial.println("DS3231 ERROR");
    displayError("DS3231 ERROR");
    while (1) delay(1000);
  }
  if (rtc.lostPower() || rtc.now().year() < 2024 || rtc.now().year() > 2035) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // BME280 on the same I2C bus (modules are usually 0x76, Adafruit boards 0x77).
  bmeOK = bme.begin(0x76) || bme.begin(0x77);
  if (!bmeOK) Serial.println("BME280 not found (skipping env print)");

  // load last known coordinates from flash so sun position shows before a fix
  prefs.begin("ozone", true);  // read-only
  gpsLat = prefs.getDouble("lat", 0.0);
  gpsLon = prefs.getDouble("lon", 0.0);
  hasStoredPos = prefs.getBool("valid", false);
  prefs.end();

  // GPS is NOT polled this run -- coordinates were provisioned to flash and loaded above.
  // Open the UART only to send the backup command, then leave the module asleep all run.
  //gpsSerial.setRxBufferSize(4096);   // not needed: no NMEA is read here
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  //Serial.println("GPS serial started, waiting for NMEA...");
  Serial.println("GPS: using stored coordinates from flash; putting module to sleep");
  gpsEnterBackup();   // <- load-from-flash + GPS asleep for the whole run

  setup_ADC_CARD();
  attachInterrupt(ADC_DRDY, adc_ready_interrupt, FALLING);
  drdy_fall = false;

  measurements = (float (*)[5])malloc(MAX_SUBLISTS * sizeof(*measurements));
  if (!measurements) {
    Serial.println("Memory allocation failed!");
    while (1);
  }

  Serial.println("---- MEASUREMENTS START ----");

  // ------- MEASUREMENT-START METADATA (time / coordinates / environment) -------
  DateTime startLocal = DateTime(rtc.now().unixtime() + UTC_OFFSET_HOURS * 3600UL);
  snprintf(startDate, sizeof(startDate), "%04d-%02d-%02d",
           startLocal.year(), startLocal.month(), startLocal.day());
  snprintf(startTime, sizeof(startTime), "%02d:%02d:%02d",
           startLocal.hour(), startLocal.minute(), startLocal.second());
  startLat = gpsLat;   // fallback = last known position (flash); overwritten by the first live GPS fix in pollGPS()
  startLon = gpsLon;

  if (bmeOK) {
    startTemp     = bme.readTemperature();
    startHum      = bme.readHumidity();
    startPress    = bme.readPressure() / 100.0F;
    startEnvValid = true;
  }

  Serial.printf("Start: %s %s | lat %.6f lon %.6f\n", startDate, startTime, startLat, startLon);
  if (startEnvValid) {
    Serial.print("Temp: ");     Serial.print(startTemp);  Serial.print(" C  ; ");
    Serial.print("Humidity: "); Serial.print(startHum);   Serial.print(" %  ; ");
    Serial.print("Pressure: "); Serial.print(startPress); Serial.println(" hPa");
  }


  // ------- OFFSET CALIBRATION -------
  float offset_v0, offset_v1;
  offsetCalibration(offset_v0, offset_v1);
}

// ---------------- LOOP ----------------
void loop() {
  // ------- RE-AIM AT THE SUN -------
  static uint32_t loopCount = 0;
  if (++loopCount % 10 == 0) sunflowerFindSun(5000);

  // ------- SUN POSITION -------
  //pollGPS();
  DateTime utcNow   = rtc.now();
  double   JD       = julianDay(utcNow.year(), utcNow.month(), utcNow.day(),
                                utcNow.hour(), utcNow.minute(), utcNow.second());
  SunPos   sun      = sunPosition(gpsLat, gpsLon, JD);
  float    el = (float)sun.elevation;

  // ------- PHASE  1 -------
  filter_rotation(pos1);
  for (int i =0; i<BLOCKS_PER_PHASE; i++) {
    //pollGPS();   // service the NMEA stream every block (~1 s) so a fix is actually recognised
    float mean_ch0, stddev_ch0, mean_ch1, stddev_ch1;
    measurement(mean_ch0, stddev_ch0, mean_ch1, stddev_ch1);
    storeMeasurement(el, mean_ch0, stddev_ch0, mean_ch1, stddev_ch1);
    DateTime nowLocal = DateTime(rtc.now().unixtime() + UTC_OFFSET_HOURS * 3600UL);
    drawScreen(nowLocal, sun, mean_ch0, stddev_ch0, mean_ch1, stddev_ch1);
  }
  // ------- PHASE  2 -------
  filter_rotation(pos2);
  for (int i =0; i<BLOCKS_PER_PHASE; i++) {
    //pollGPS();   // service the NMEA stream every block (~1 s) so a fix is actually recognised
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
    MeasurementMeta meta = {
      startTime, startDate, startLat, startLon,
      startEnvValid, startTemp, startHum, startPress
    };
    bool sent = uploadMeasurementsCSV(measurements, MAX_SUBLISTS, meta);
    displayMessage(sent ? "Upload done" : "Upload failed", sent);
    delay(1500);

    Serial.println("Shutting down system...");
    if (measurements != NULL) {
      free(measurements);
      measurements = NULL;
    }
    filter_rotation(pos1);
    displayOff();

    adc1.sendcmd(CMD_STANDBY);
    delay(5);

    adc1.end();
    // Tri-state SPI pins
    digitalWrite(ADC_CS, LOW);
    digitalWrite(ADC_SCK, LOW);
    digitalWrite(ADC_MOSI, LOW);

    digitalWrite(EN, LOW);
    Serial.println("Going into deep sleep...");
    esp_deep_sleep_start();
  }
}

// ---------------- ADC INIT ----------------
// 2kHz 0b0000001100010010 , 4kHz 0b0000001100001110 , 8kHz 0b0000001100001010
void setup_ADC_CARD() {

  uint32_t ADC_CLOCK_REG = 0b0000011100011110; //High-res, OSR 16384 (~250 SPS, lowest noise); CH0/CH1/CH2 enabled (OSR field 100->111)
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

  Serial.print("Filter to ");
  Serial.print(pos);
  Serial.println(" us");

  filter_servo.writeMicroseconds(pos);
  delay(SERVO_SETTLE_TIME);

  adc1.sendcmd(CMD_WAKEUP);
  delay(10);
}

// ---------------- OFFSET CALIBRATION ----------------
void offsetCalibration(float &offset_v0, float &offset_v1) {
  int32_t m0 = 0;
  int32_t m1 = 0;
  int collected = 0;

  // Watchdog on the GAP since the last sample
  unsigned long tLast = millis();
  while (collected < SAMPLES_FOR_CAL) {
    if (drdy_fall) {
      drdy_fall = false;

      adcOutput temp = adc1.readADC();

      int32_t delta0 = temp.ch0 - m0;
      m0 += delta0 / (collected + 1);

      int32_t delta1 = temp.ch1 - m1;
      m1 += delta1 / (collected + 1);
      collected++;
      tLast = millis();           // got data -> pet the watchdog
    }
    if (millis() - tLast > 2000) {   // no DRDY for 2 s -> ADC really not responding
      Serial.println("ADC timeout during offset calibration");
      break;
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

  // show the measured offsets on the TFT (same Ch1/Ch2 look as measurements),
  // then hold them long enough to read before the first measurement screen
  drawOffsetScreen(offset_v0, offset_v1);
  delay(3000);
}

// ---------------- READ AND COMPUTE ----------------
// Each reported sub-sample is the average of OVERSAMPLE consecutive conversions
// (the ADC already oversamples in hardware at OSR 16384). The block mean is thus
// built from SAMPLES_PER_BLOCK*OVERSAMPLE conversions, while the reported stddev is
// the spread of the SAMPLES_PER_BLOCK averaged sub-samples (how steady the signal
// was over the block, not single-conversion jitter).
void measurement(float &mean_v0, float &stddev_v0, float &mean_v1, float &stddev_v1) {
  double m0 = 0;   // running mean  (Welford) over averaged sub-samples
  double m1 = 0;
  double s0 = 0;   // running M2     (Welford)
  double s1 = 0;
  int collected = 0;

  while (collected < SAMPLES_PER_BLOCK) {

    // ---- build one low-noise sub-sample by averaging OVERSAMPLE conversions ----
    double acc0 = 0, acc1 = 0;
    int got = 0;
    unsigned long t0 = millis();
    while (got < OVERSAMPLE) {
      if (drdy_fall) {
        drdy_fall = false;
        adcOutput temp = adc1.readADC();
        acc0 += temp.ch0;          // accumulate raw counts (exact in double)
        acc1 += temp.ch1;
        got++;
      }
      if (millis() - t0 > 2000) {  // ADC not responding
        Serial.println("ADC timeout: no DRDY (analog board disconnected?)");
        break;
      }
    }
    if (got == 0) break;           // ADC dead -> abandon the block

    Vch0 = (acc0 / got) / FS * PREF_ADC;   // averaged counts -> mV
    Vch1 = (acc1 / got) / FS * PREF_ADC;

    double delta0 = Vch0 - m0;
    m0 += delta0 / (collected + 1);
    s0 += delta0 * (Vch0 - m0);

    double delta1 = Vch1 - m1;
    m1 += delta1 / (collected + 1);
    s1 += delta1 * (Vch1 - m1);

    collected++;
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
  if (!gpsAwake) return;   // already fixed + in backup -> nothing to do

  while (gpsSerial.available())
    gps.encode(gpsSerial.read());

  // A real fix carries position AND UTC time together. Save both, then sleep the
  // GPS for the rest of the run. (location.isValid() guards against the pre-fix
  // garbage time the module spits out, e.g. year 2043.)
  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
    gpsLat = gps.location.lat();
    gpsLon = gps.location.lng();
    startLat = gpsLat;   // first live fix -> this is what the CSV header reports
    startLon = gpsLon;
    prefs.begin("ozone", false);
    prefs.putDouble("lat", gpsLat);
    prefs.putDouble("lon", gpsLon);
    prefs.putBool("valid", true);
    prefs.end();
    hasStoredPos = true;
    hasFix = true;

    rtc.adjust(DateTime(gps.date.year(), gps.date.month(), gps.date.day(),
                        gps.time.hour(), gps.time.minute(), gps.time.second()));
    rtcSyncedGPS = true;

    gpsEnterBackup();   // location fixed -> put GPS to sleep
  }
}

static void gpsEnterBackup() {
  const uint8_t pmreq[] = {
    0xB5, 0x62, 0x02, 0x41, 0x08, 0x00,   // header: RXM-PMREQ, 8-byte payload
    0x00, 0x00, 0x00, 0x00,               // duration = 0 (infinite)
    0x02, 0x00, 0x00, 0x00,               // flags = backup
    0x4D, 0x3B                            // checksum
  };
  gpsSerial.write(pmreq, sizeof(pmreq));
  gpsSerial.flush();        // ensure it's transmitted before we stop polling
  gpsAwake = false;
  Serial.println("GPS: fix done -> backup (sleep) command sent");
}