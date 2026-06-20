#include <Arduino.h>
#include <ESP32Servo.h>

// Standalone LDR sun tracker — same algorithm as OzoneTrckr's sunflower module:
// capped incremental-P (no integral to wind up), oversampled LDR reads, a deadband,
// and a fixed settle interval between moves. Runs continuously here in loop().

// ---- servo + LDR pins ----
const int H_PIN = 12;   // horizontal servo
const int V_PIN = 13;   // vertical servo
const int FILTER = 5;   // filter servo
const int ldrlt = A3;   // top-left
const int ldrrt = A1;   // top-right
const int ldrld = A0;   // bottom-left
const int ldrrd = A2;   // bottom-right

// ---- pulse range: narrow on purpose so the servo can't ram the gear stops ----
const int US_MIN = 1000, US_MAX = 2000;   // 90 deg -> 1500us (centre)

// ---- travel limits (deg) — adjust to the real rig ----
const float H_MIN = 10, H_MAX = 170;
const float V_MIN = 50, V_MAX = 140;

// ---- rest / start position ----
const float START_H = 90, START_V = 140;

// ---- proportional, momentum-limited step ----
const float KP       = 0.006f; // deg of step per ADC count of imbalance
const float MIN_STEP = 0.15f;  // deg: smallest move -> keeps inching the last bit
const float MAX_STEP = 1.0f;   // deg: HARD CAP -> momentum limit (no lurch on the heavy mount)
const int   TOL      = 60;     // deadband (counts): hold when imbalance is below this
const int   STEP_MS  = 30;     // min interval between moves (settle time -> anti-hunt)
const int   LDR_AVG  = 8;      // analogRead samples averaged per LDR (noise reduction, no lag)

Servo horizontal;
Servo vertical;
Servo filter;
float servoh = START_H, servov = START_V;

// map an angle (0..180 deg) to a servo pulse width (sub-degree -> smooth motion)
int angleToUs(float deg) {
  return (int)(US_MIN + (US_MAX - US_MIN) * deg / 180.0f + 0.5f);
}

// proportional step magnitude: big when far off, small when close, capped both ends
float stepFor(int err) {
  return constrain(KP * abs(err), MIN_STEP, MAX_STEP);
}

// average several back-to-back reads -> beats down ADC noise with zero added lag
int readLDR(int pin) {
  long s = 0;
  for (int i = 0; i < LDR_AVG; i++) s += analogRead(pin);
  return (int)(s / LDR_AVG);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  filter.attach(FILTER, 400, 2600);
  filter.writeMicroseconds(2600);
  analogReadResolution(12);          // TOL / KP are in 12-bit counts
  horizontal.attach(H_PIN, US_MIN, US_MAX);
  vertical.attach(V_PIN, US_MIN, US_MAX);
  servoh = START_H;  servov = START_V;
  horizontal.writeMicroseconds(angleToUs(servoh));
  vertical.writeMicroseconds(angleToUs(servov));
}

void loop() {
  int lt = readLDR(ldrlt), rt = readLDR(ldrrt);
  int ld = readLDR(ldrld), rd = readLDR(ldrrd);
  int avt = (lt + rt) / 2, avd = (ld + rd) / 2;   // top / bottom
  int avl = (lt + ld) / 2, avr = (rt + rd) / 2;   // left / right
  int dV = avt - avd;   // >0 = top brighter
  int dH = avl - avr;   // >0 = left brighter

  if (abs(dV) > TOL) {
    float s = stepFor(dV);
    if (avd > avt) servov += s; else servov -= s;   // bottom brighter -> raise
    servov = constrain(servov, V_MIN, V_MAX);
    vertical.writeMicroseconds(angleToUs(servov));
  }
  if (abs(dH) > TOL) {
    float s = stepFor(dH);
    if (avl > avr) servoh -= s; else servoh += s;    // left brighter -> swing left
    servoh = constrain(servoh, H_MIN, H_MAX);
    horizontal.writeMicroseconds(angleToUs(servoh));
  }

  delay(STEP_MS);   // settle between moves -> anti-hunt
}
