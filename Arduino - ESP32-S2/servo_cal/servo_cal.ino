#include <ESP32Servo.h>
#define S0 13

Servo myservo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  myservo.attach(S0);
}

void loop() {
  myservo.write(5);
  delay(2000);
  myservo.write(90);
  delay(2000);
  myservo.write(180);
  delay(2000);
  

}
