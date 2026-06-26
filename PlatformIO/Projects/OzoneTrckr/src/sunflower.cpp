#include "sunflower.h"
#include "ESP32Servo.h"
#include "ESP32PWM.h"

static int H_PIN, V_PIN;                  // horizontal / vertical servo
static int ldrlt, ldrrt, ldrld, ldrrd;    // top-left / top-right / bottom-left / bottom-right

// ---- pulse range: narrow on purpose so the servo can't ram the gear stops ----
static const int US_MIN = 1000, US_MAX = 2000;   // 90 deg -> 1500us (centre)

// ---- travel limits (deg) — adjust to the real rig ----
static const float H_MIN = 10,  H_MAX = 170;
static const float V_MIN = 40,  V_MAX = 150;

// ---- rest / start position (where it begins each seek) ----
static const float START_H = 90, START_V = 150;

// ---- proportional, momentum-limited step ----
static const float KP       = 0.006f; // deg of step per ADC count of imbalance
static const float MIN_STEP  = 0.15f;  // deg: smallest move -> keeps inching the last bit
static const float MAX_STEP  = 0.6f;   // deg: HARD CAP -> momentum limit (no lurch on the heavy mount)
static const int   TOL       = 60;     // deadband (counts): hold when imbalance is below this
static const int   STEP_MS   = 20;     // min interval between moves (settle time -> anti-hunt)
static const int   SETTLE_N  = 12;     // consecutive in-band cycles before we call it "locked"
static const int   LDR_AVG   = 8;      // analogRead samples averaged per LDR (noise reduction)

static Servo horizontal;
static Servo vertical;
static float servoh = START_H, servov = START_V;

// map an angle (0..180 deg) to a servo pulse width (sub-degree -> smooth motion)
static int angleToUs(float deg) {
  return (int)(US_MIN + (US_MAX - US_MIN) * deg / 180.0f + 0.5f);
}

// proportional step magnitude: big when far off, small when close, capped both ends
static float stepFor(int err) {
  return constrain(KP * abs(err), MIN_STEP, MAX_STEP);
}

static int readLDR(int pin) {
  long s = 0;
  for (int i = 0; i < LDR_AVG; i++) s += analogRead(pin);
  return (int)(s / LDR_AVG);
}

static bool trackOnce() {
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
  return (abs(dV) <= TOL) && (abs(dH) <= TOL);
}

void sunflowerBegin(int hPin, int vPin, int ldrLT, int ldrRT, int ldrLD, int ldrRD) {
  H_PIN = hPin;  V_PIN = vPin;
  ldrlt = ldrLT; ldrrt = ldrRT; ldrld = ldrLD; ldrrd = ldrRD;

  analogReadResolution(12);          // TOL / KP are in 12-bit counts
  horizontal.attach(H_PIN, US_MIN, US_MAX);
  vertical.attach(V_PIN, US_MIN, US_MAX);
  servoh = START_H;  servov = START_V;
  horizontal.writeMicroseconds(angleToUs(servoh));
  vertical.writeMicroseconds(angleToUs(servov));
}

void sunflowerFindSun(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  int settled = 0;
  while (millis() - t0 < timeoutMs) {
    if (trackOnce()) {
      if (++settled >= SETTLE_N) return;   // locked -> hold here
    } else {
      settled = 0;
    }
    delay(STEP_MS);
  }
  // timed out: leave the servos where they are (still attached -> holds position)
}
