#include <Arduino.h>
#include <ESP32Servo.h> // include Servo library

Servo horizontal;   // horizontal servo
float servoh = 90;  // stand horizontal servo (float -> allows sub-degree, smooth moves)

Servo vertical;     // vertical servo (MG90 metal gear)
float servov = 90;  // stand vertical servo

// LDR pin connections
int ldrld = A0; //LDR down left
int ldrrt = A1; //LDR top rigt
int ldrrd = A2; //ldr down rigt
int ldrlt = A3; //LDR top left

// ---- gentle motion (heavy mount: small constant moves, no sudden jumps) ----
float stepDeg = 0.25; // deg moved per loop. SMALL & constant = smooth, low momentum, no sudden moves.
                      // smaller = gentler/less shake (slower); larger = faster but more momentum. Keep it small.
const int US_MIN = 1000, US_MAX = 2000; // pulse range. 90 deg -> 1500us (centre). Narrow on purpose so it
                                        // CANNOT over-travel and ram the gear stops.

// map an angle (0..180 deg) to a servo pulse width. Same idea as write(), but in
// microseconds so we can command fractions of a degree -> smooth motion.
int angleToUs(float deg) {
  return (int)(US_MIN + (US_MAX - US_MIN) * deg / 180.0 + 0.5);
}

void setup()  {
  Serial.begin(115200);
  horizontal.attach(12, US_MIN, US_MAX);
  vertical.attach(13, US_MIN, US_MAX);
  horizontal.writeMicroseconds(angleToUs(servoh));
  vertical.writeMicroseconds(angleToUs(servov));
}

void loop() {
  int lt = analogRead(ldrlt); // top left
  int rt = analogRead(ldrrt); // top right
  int ld = analogRead(ldrld); // down left
  int rd = analogRead(ldrrd); // down rigt

int dtime = 30; // loop period (ms): smaller = more responsive/smoother, but must stay >= one move's settle time or it overshoots & hunts
                // analogRead(4)/20; 40
int tol = 60;   // deadband (ADC counts): smaller = more precise on the sun, but too small makes it nod/hunt. Raise toward 80-100 if it nods.
                // analogRead(5)/4;  80

int avt = (lt + rt) / 2; // average value top
int avd = (ld + rd) / 2; // average value down
int avl = (lt + ld) / 2; // average value left
int avr = (rt + rd) / 2; // average value right

int dvert = avt - avd; // check the diffirence of up and down
int dhoriz = avl - avr;// check the diffirence og left and rigt

// check if the diffirence is in the tolerance else change vertical angle (one small step)
if (abs(dvert) > tol)  {
  if (avd > avt)  {
    servov += stepDeg;
    if (servov > 130) { servov = 130; }
  }
  else if (avd < avt) {
    servov -= stepDeg;
    if (servov < 50) { servov = 50; }
  }
  vertical.writeMicroseconds(angleToUs(servov));
}

// check if the diffirence is in the tolerance else change horizontal angle (one small step)
if (abs(dhoriz) > tol)  {
  if (avl > avr)  {
    servoh -= stepDeg;
    if (servoh < 10) { servoh = 10; }
  }
  else if (avl < avr) {
    servoh += stepDeg;
    if (servoh > 170) { servoh = 170; }
  }
  horizontal.writeMicroseconds(angleToUs(servoh));
}
delay(dtime);
}
