#include <Arduino.h>
#include <ESP32Servo.h> // include Servo library
#include <PID_v1.h>

Servo horizontal;   // horizontal servo
float servoh;  // stand horizontal servo (float -> allows sub-degree, smooth moves)

Servo vertical;     // vertical servo (MG90 metal gear)
float servov;  // stand vertical servo

// LDR pin connections
int ldrld = A0; //LDR down left
int ldrrt = A1; //LDR top rigt
int ldrrd = A2; //ldr down rigt
int ldrlt = A3; //LDR top left

const int US_MIN = 600, US_MAX = 2400; // full-ish travel; the axis limits below are the real
                                        // bounds. 90 deg -> 1500us (centre).

// map an angle (0..180 deg) to a servo pulse width (microseconds -> sub-degree commands).
int angleToUs(float deg) {
  return (int)(US_MIN + (US_MAX - US_MIN) * deg / 180.0 + 0.5);
}

// ---- startup position (roughly where the mount rests at power-on) ----
float START_H = 90;
float START_V = 140;

// ---- PID control (one per axis) ----
double vIn, vOut, vSet = 0;
double hIn, hOut, hSet = 0;

// START GENTLE on this heavy mount: Kp small so a big error can't lunge it, Ki slowly
// removes the standing offset (so it locks onto the moving sun), Kd damps hunting.
// Tune on the bench -- see notes at the bottom of the file.
double KHp = 0.001, KHi = 0.002, KHd = 0.0;
double KVp = 0.001, KVi = 0.001, KVd = 0.0;

PID vPID(&vIn, &vOut, &vSet, KVp, KVi, KVd, DIRECT);
PID hPID(&hIn, &hOut, &hSet, KHp, KHi, KHd, DIRECT);

// axis travel limits (deg) -> also the PID output limits (gives anti-windup for free)
const float V_MIN = 50,  V_MAX = 140;
const float H_MIN = 10,  H_MAX = 170;

const int TOLh = 120;
const int TOLv = 120;
const int SAMPLE_MS = 100;   // PID update period (>= the mount's settle time -> anti-hunt)

void setup()  {
  Serial.begin(115200);
  delay(1000);                       // power the servos during this window
  horizontal.attach(12, US_MIN, US_MAX);
  vertical.attach(13, US_MIN, US_MAX);

  servoh = START_H;  servov = START_V;
  horizontal.writeMicroseconds(angleToUs(servoh));
  vertical.writeMicroseconds(angleToUs(servov));

  // hand the servos over to the PIDs bumplessly (output starts at the current angle)
  vOut = servov;  hOut = servoh;
  vPID.SetOutputLimits(V_MIN, V_MAX);
  hPID.SetOutputLimits(H_MIN, H_MAX);
  vPID.SetSampleTime(SAMPLE_MS);
  hPID.SetSampleTime(SAMPLE_MS);
  vPID.SetMode(AUTOMATIC);
  hPID.SetMode(AUTOMATIC);
}

void loop() {
  int lt = analogRead(ldrlt); // top left
  int rt = analogRead(ldrrt); // top right
  int ld = analogRead(ldrld); // down left
  int rd = analogRead(ldrrd); // down right

  int avt = (lt + rt) / 2;    // top
  int avd = (ld + rd) / 2;    // bottom
  int avl = (lt + ld) / 2;    // left
  int avr = (rt + rd) / 2;    // right

  double dV = avt - avd;      // vertical imbalance   (>0 = top brighter)
  double dH = avl - avr;      // horizontal imbalance (>0 = left brighter)

  // deadband: treat a tiny imbalance as zero so it holds at lock instead of nodding
  vIn = (fabs(dV) < TOLv) ? 0.0 : dV;
  hIn = (fabs(dH) < TOLh) ? 0.0 : dH;

  // PID recomputes only every SAMPLE_MS; write the servo on the steps it updates
  if (vPID.Compute()) { servov = vOut; vertical.writeMicroseconds(angleToUs(servov)); }
  if (hPID.Compute()) { servoh = hOut; horizontal.writeMicroseconds(angleToUs(servoh)); }

  delay(10);
}