#include "Arduino.h"
#include "ADS131M04.h"
#include <ESP32Servo.h>
#include <TimeLib.h>
//#include <Adafruit_NeoPixel.h>
//#include "Adafruit_TestBed.h"
//#include <Adafruit_ST7789.h> 
//#include <Fonts/FreeSans12pt7b.h>

#define EN 11
#define S0 12
#define ADC_DRDY 6
#define ADC_CS 9

#define ADC_SCK 36
#define ADC_MISO 37
#define ADC_MOSI 35

// 2000 sublists, rotate filters every 10 sublists, calibrate every 200
#define MAX_SUBLISTS 2000 //2000
//#define MAX_CAL 200     //kalibrira se samo jednom na početku mjerenja
#define BLOCKS_PER_PHASE 10   //koliko mjerenja prije okretanja filtera
#define SAMPLES_PER_BLOCK 20   //koliko sample-ova za jedno mjerenje
#define SAMPLES_FOR_CAL 100
#define SERVO_SETTLE_TIME 1000
#define ADC_DISCARD_SAMPLES 5

uint8_t cal_cycles = 0;
int utc_offset = +2;
time_t startTime, endTime;

// #define BUFFER_TIME 60 
// BUFFER_TIME/2(Hz) su sekunde

// INTERRUPT
volatile uint16_t drdy_div = 0;
volatile bool drdy_fall = false; //data ready, seta se interuptom
void IRAM_ATTR adc_ready_interrupt() {
    drdy_div++;
    if (drdy_div >= 100) {
        drdy_div = 0;
        drdy_fall = true;
    }
}

// SVE ADC
ADS131M04 adc1;        //objekt ADC-a
adcOutput adc_podatak; //output tip, pristup kanalima 0-3, status

Servo filter_servo;

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

void printTime();
time_t toUtc(time_t local);
time_t compileTime();
void printElapsed(uint64_t start, uint64_t end);

void setup_ADC_CARD();
void offsetCalibration();
void filter_rotation(int pos);
void measurement(float &mean_ch2, float &sttdev_ch2);
void storeMeasurement(float a, float b, float c, float d);

// ---------------- DYNAMIC STORAGE ----------------
float (*measurements)[4] = NULL;
size_t measurement_index = 0;

// ---------------- SETUP ----------------
void setup() {
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);

  Serial.begin(115200);
  delay(5000);
  setTime(toUtc(compileTime()));
  startTime = millis();

  filter_servo.attach(S0);

  setup_ADC_CARD();
  attachInterrupt(ADC_DRDY, adc_ready_interrupt, FALLING);//interupt na DRDY pin  

  measurements = (float (*)[4])malloc(MAX_SUBLISTS * sizeof(*measurements));
  if (!measurements) {
    Serial.println("Memory allocation failed!");
    while (1);
  }
  delay(3000);
  Serial.println("---- MEASUREMENTS START ----");
  // ------- OFFSET CALIBRATION -------
  filter_rotation(pos_cal);
  offsetCalibration();
}

// ---------------- LOOP ----------------
void loop() {
  
  // ------- PHASE  1 -------
  filter_rotation(pos1);
  for (int i =0; i<BLOCKS_PER_PHASE; i++) {
    float mean_ch0, stddev_ch0, mean_ch1, stddev_ch1;
    measurement(mean_ch0, stddev_ch0, mean_ch1, stddev_ch1);
    storeMeasurement(mean_ch0, stddev_ch0, mean_ch1, stddev_ch1);
  }
  // ------- PHASE  2 -------
  filter_rotation(pos2);
  for (int i =0; i<BLOCKS_PER_PHASE; i++) {
    float mean_ch0, stddev_ch0, mean_ch1, stddev_ch1;
    measurement(mean_ch0, stddev_ch0, mean_ch1, stddev_ch1);
    storeMeasurement(mean_ch0, stddev_ch0, mean_ch1, stddev_ch1);
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
      Serial.print(measurements[i][3], 4);
      Serial.println("]");
    }
    Serial.println("---- DATA END ----");
    endTime = millis();
    printElapsed(startTime, endTime);
    Serial.println("Shutting down system...");
    if (measurements != NULL) {
      free(measurements);
      measurements = NULL;
    }
    adc1.sendcmd(CMD_STANDBY);
    delay(5);
    // Disable SPI peripheral
    adc1.end();
    // Tri-state SPI pins
    digitalWrite(ADC_CS, LOW);
    digitalWrite(ADC_SCK, LOW);
    digitalWrite(ADC_MOSI, LOW);

    filter_rotation(pos1);
    // Power down analog board
    digitalWrite(EN, LOW);
    Serial.println("System powered down.");
    while (1) {
      Serial.println("Go to sleep!");
      delay(1000);
    }
  }
}

// ADC INIT
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
}  

// ---------------- FILTER ROTATION ----------------
void filter_rotation(int pos) {

  adc1.sendcmd(CMD_STANDBY);
  delay(10);

  Serial.print("Filter in position ");
  Serial.println(pos);

  filter_servo.write(pos);
  delay(SERVO_SETTLE_TIME);

  adc1.sendcmd(CMD_WAKEUP);
  delay(10);
}

// --------------- OFFSET CALIBRATION ---------------
void offsetCalibration() {
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

  float offset_v0 = m0 / FS * PREF_ADC;
  float offset_v1 = m1 / FS * PREF_ADC;

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
  }
  mean_v0 = m0;
  mean_v1 = m1;
  float variance0 = s0 / (SAMPLES_PER_BLOCK - 1);
  float variance1 = s1 / (SAMPLES_PER_BLOCK - 1);
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
void storeMeasurement(float a, float b, float c, float d) {
  if (measurement_index >= MAX_SUBLISTS)
    return;

  measurements[measurement_index][0] = a;
  measurements[measurement_index][1] = b;
  measurements[measurement_index][2] = c;
  measurements[measurement_index][3] = d;

  measurement_index++;
  cal_cycles++;
}

void storeOffset(float a, float b, float c, float d) {
  if (measurement_index >= MAX_SUBLISTS)
    return;

  measurements[measurement_index][0] = a;
  measurements[measurement_index][1] = b;
  measurements[measurement_index][2] = c;
  measurements[measurement_index][3] = d;
}

time_t toUtc(time_t local) {
  return local - utc_offset * 3600L;
}

// Code from JChristensen/Timezone Clock example
time_t compileTime() {
  const uint8_t COMPILE_TIME_DELAY = 8;
  const char *compDate = __DATE__, *compTime = __TIME__, *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char chMon[4], *m;
  tmElements_t tm;

  strncpy(chMon, compDate, 3);
  chMon[3] = '\0';
  m = strstr(months, chMon);
  tm.Month = ((m - months) / 3 + 1);

  tm.Day = atoi(compDate + 4);
  tm.Year = atoi(compDate + 7) - 1970;
  tm.Hour = atoi(compTime);
  tm.Minute = atoi(compTime + 3);
  tm.Second = atoi(compTime + 6);
  time_t t = makeTime(tm);
  return t + COMPILE_TIME_DELAY;
}
void printTime(time_t t) {
  char buffer[25];
  sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d",
          year(t), month(t), day(t),
          hour(t), minute(t), second(t));
  Serial.println(buffer);
}

void printElapsed(uint64_t start, uint64_t end) {

  uint64_t elapsed = end - start;

  uint32_t seconds = elapsed / 1000;
  uint32_t minutes = seconds / 60;
  uint32_t hours = minutes / 60;

  seconds %= 60;
  minutes %= 60;

  Serial.print("Measurement duration: ");
  Serial.print(hours);
  Serial.print("h ");
  Serial.print(minutes);
  Serial.print("m ");
  Serial.print(seconds);
  Serial.println("s");
}