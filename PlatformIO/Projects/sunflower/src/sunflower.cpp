#include <Arduino.h>
#include <ESP32Servo.h> // include Servo library

Servo horizontal;   // horizontal servo
float servoh;  // stand horizontal servo (float -> allows sub-degree, smooth moves)

Servo vertical;     // vertical servo (MG90 metal gear)
float servov;  // stand vertical servo

// LDR pin connections
int ldrld = A0; //LDR down left
int ldrrt = A1; //LDR top rigt
int ldrrd = A2; //ldr down rigt
int ldrlt = A3; //LDR top left

// ---- proportional, momentum-limited motion (heavy mount, uncontrollable servo speed) ----
// Step grows with how far off the sun is (fast search), shrinks to a creep near the sun
// (smooth + precise), and is HARD-CAPPED so no single move builds dangerous momentum.
// NOTE: step sizes below are in DEGREES, and were scaled x(1000/1800) when the us range
// widened from 1000-2000 to 600-2400, so each step moves the mount the SAME physical
// distance as before (us-per-deg went 5.6 -> 10). Re-scale again if you change the us range.
float KP       = 0.003; // deg of step per ADC count of error. Bigger = quicker search.
float MIN_STEP = 0.05;  // deg: smallest move -> keeps it inching the last bit (stays smooth/precise)
float MAX_STEP = 0.55;   // deg: HARD CAP per step. Servo speed can't be set, so this is the momentum limit.
const int US_MIN = 600, US_MAX = 2400; // full-ish travel; the 50-130 / 10-170 deg constrains are now
                                        // the real limits. 90 deg -> 1500us (centre).

// map an angle (0..180 deg) to a servo pulse width. Same idea as write(), but in
// microseconds so we can command fractions of a degree -> smooth motion.
int angleToUs(float deg) {
  return (int)(US_MIN + (US_MAX - US_MIN) * deg / 180.0 + 0.5);
}

// proportional step magnitude: big when far off, small when close, capped both ends.
float stepFor(int err) {
  return constrain(KP * abs(err), MIN_STEP, MAX_STEP);
}

float START_H = 90;
float START_V = 140;

void setup()  {
  Serial.begin(115200);
  delay(1000);                       // power the servos during this window
  horizontal.attach(12, US_MIN, US_MAX);
  vertical.attach(13, US_MIN, US_MAX);

  servoh = START_H;  servov = START_V;
  horizontal.writeMicroseconds(angleToUs(servoh));
  vertical.writeMicroseconds(angleToUs(servov));
}

void loop() {
  int lt = analogRead(ldrlt); // top left
  int rt = analogRead(ldrrt); // top right
  int ld = analogRead(ldrld); // down left
  int rd = analogRead(ldrrd); // down rigt

int dtime = 40; // loop period (ms): smaller = more responsive/smoother, but must stay >= one move's settle time or it overshoots & hunts
                // analogRead(4)/20; 40
int tol = 90;   // deadband (ADC counts): smaller = more precise on the sun, but too small makes it nod/hunt. Raise toward 80-100 if it nods.
                // analogRead(5)/4;  80

int avt = (lt + rt) / 2; // average value top
int avd = (ld + rd) / 2; // average value down
int avl = (lt + ld) / 2; // average value left
int avr = (rt + rd) / 2; // average value right

int dvert = avt - avd; // check the diffirence of up and down
int dhoriz = avl - avr;// check the diffirence og left and rigt

// check if the diffirence is in the tolerance else change vertical angle (proportional, capped step)
if (abs(dvert) > tol)  {
  float s = stepFor(dvert);
  if (avd > avt)  { servov += s; }
  else if (avd < avt) { servov -= s; }
  servov = constrain(servov, 50, 130);
  vertical.writeMicroseconds(angleToUs(servov));
}

// check if the diffirence is in the tolerance else change horizontal angle (proportional, capped step)
if (abs(dhoriz) > tol)  {
  float s = stepFor(dhoriz);
  if (avl > avr)  { servoh -= s; }
  else if (avl < avr) { servoh += s; }
  servoh = constrain(servoh, 10, 170);
  horizontal.writeMicroseconds(angleToUs(servoh));
}
delay(dtime);
}
