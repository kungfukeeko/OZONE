#include <Arduino.h>
#include <ESP32Servo.h> // include Servo library 

Servo horizontal; // horizontal servo
int servoh = 90; // stand horizontal servo

Servo vertical; // vertical servo 
int servov = 90; // stand vertical servo

// LDR pin connections
int ldrld = A0; //LDR down left
int ldrrt = A1; //LDR top rigt
int ldrrd = A2; //ldr down rigt
int ldrlt = A3; //LDR top left

void setup()  {
  Serial.begin(115200);
  horizontal.attach(12); 
  vertical.attach(13);
}

void loop() {
  int lt = analogRead(ldrlt); // top left
  int rt = analogRead(ldrrt); // top right
  int ld = analogRead(ldrld); // down left
  int rd = analogRead(ldrrd); // down rigt

int dtime = 40; // analogRead(4)/20; 40
int tol = 80;   // analogRead(5)/4;  80

int avt = (lt + rt) / 2; // average value top
int avd = (ld + rd) / 2; // average value down
int avl = (lt + ld) / 2; // average value left
int avr = (rt + rd) / 2; // average value right

int dvert = avt - avd; // check the diffirence of up and down
int dhoriz = avl - avr;// check the diffirence og left and rigt

// check if the diffirence is in the tolerance else change vertical angle
if (abs(dvert) > tol)  {
  if (avd > avt)  {
    servov = ++servov;
    if (servov > 130) { servov = 130; }
  }
  else if (avd < avt) {
    servov= --servov;
    if (servov < 50) { servov = 50; }
  }
  else if (avl == avr)  {}  // do nothing
  vertical.write(servov);
}

// check if the diffirence is in the tolerance else change horizontal angle
if (abs(dhoriz) > tol)  {
  if (avl > avr)  {
    servoh = --servoh;
    if (servoh < 10) { servoh = 10; }
  }
  else if (avl < avr) {
    servoh = ++servoh;
    if (servoh > 170) { servoh = 170; }
  }
  else if (avl == avr)  {}  // do nothing
  horizontal.write(servoh);
}
delay(dtime);
}
