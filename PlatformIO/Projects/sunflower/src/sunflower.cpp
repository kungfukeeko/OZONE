#include <Arduino.h>
#include <ESP32Servo.h>

// ----------------------------------------------------------------------------
// LDR sun tracker — proportional control.
// Four LDRs in a 2x2 grid behind a cross/shade so the four quadrants get
// unequal light unless the sun is centred. We steer two servos to equalise them.
//
// Smoothness comes from: (1) step size proportional to how far off-centre the
// sun is (big move when far, tiny when close), (2) a slew-rate cap so a large
// error can't snap the servo, (3) EMA-filtered LDR readings to kill jitter, and
// (4) a deadband so it stops hunting once centred.
// ----------------------------------------------------------------------------

Servo horizontal;   // azimuth servo
Servo vertical;     // elevation servo

// LDR pins
const int ldrlt = A3;   // top-left
const int ldrrt = A1;   // top-right
const int ldrld = A0;   // down-left
const int ldrrd = A2;   // down-right

// servo wiring
const int H_PIN = 12;
const int V_PIN = 13;

// ---------------- tuning (start here) ----------------
const float KP        = 0.012f;  // deg of correction per unit of LDR difference
const float MAX_STEP  = 0.9f;    // max deg moved per update -> caps speed (smoothness)
const int   DEADBAND  = 60;      // hold position if the imbalance is smaller than this
const float EMA_ALPHA = 0.20f;   // LDR smoothing: lower = smoother but slower (0..1)
const int   DT_MS     = 20;      // control update period (ms)

// servo travel limits (degrees)
const float H_MIN = 10.0f, H_MAX = 170.0f;
const float V_MIN = 50.0f, V_MAX = 130.0f;

// pulse range used for the servos (matches the widened SG90 range)
const int US_MIN = 500, US_MAX = 2500;
// ------------------------------------------------------

float servoh = 90.0f, servov = 90.0f;     // current commanded angles (kept as float)
float fLT = 0, fRT = 0, fLD = 0, fRD = 0;  // EMA-filtered LDR values
bool  filterInit = false;

// map an angle (0..180 deg) to a servo pulse width
static int angleToUs(float deg) {
  return (int)(US_MIN + (US_MAX - US_MIN) * (deg / 180.0f) + 0.5f);
}

// move a float servo position toward its target by a proportional, slew-limited
// step, but only if the imbalance is outside the deadband. Returns new position.
static float track(float pos, float err, float lo, float hi) {
  if (fabsf(err) <= DEADBAND) return pos;            // centred enough -> hold (no hunting)
  float step = constrain(KP * err, -MAX_STEP, MAX_STEP);
  pos = constrain(pos + step, lo, hi);
  return pos;
}

void setup() {
  Serial.begin(115200);
  horizontal.attach(H_PIN, US_MIN, US_MAX);
  vertical.attach(V_PIN, US_MIN, US_MAX);
  horizontal.writeMicroseconds(angleToUs(servoh));
  vertical.writeMicroseconds(angleToUs(servov));
}

void loop() {
  int lt = analogRead(ldrlt);
  int rt = analogRead(ldrrt);
  int ld = analogRead(ldrld);
  int rd = analogRead(ldrrd);

  // exponential moving average smooths sensor noise (seed on first pass)
  if (!filterInit) { fLT = lt; fRT = rt; fLD = ld; fRD = rd; filterInit = true; }
  fLT += EMA_ALPHA * (lt - fLT);
  fRT += EMA_ALPHA * (rt - fRT);
  fLD += EMA_ALPHA * (ld - fLD);
  fRD += EMA_ALPHA * (rd - fRD);

  float avt = (fLT + fRT) * 0.5f;   // top
  float avd = (fLD + fRD) * 0.5f;   // bottom
  float avl = (fLT + fLD) * 0.5f;   // left
  float avr = (fRT + fRD) * 0.5f;   // right

  // errors keep the original tracking direction:
  //   brighter bottom (avd>avt) -> raise servov ; brighter right (avr>avl) -> raise servoh
  float eVert  = avd - avt;
  float eHoriz = avr - avl;

  servov = track(servov, eVert,  V_MIN, V_MAX);
  servoh = track(servoh, eHoriz, H_MIN, H_MAX);

  vertical.writeMicroseconds(angleToUs(servov));
  horizontal.writeMicroseconds(angleToUs(servoh));

  delay(DT_MS);
}
