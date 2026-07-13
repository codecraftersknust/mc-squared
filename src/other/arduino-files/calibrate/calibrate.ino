#include <Servo.h>

Servo esc;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  esc.attach(7);
  Serial.println("Starting calibration...");
  Serial.println("send 'c' to begin.");

  while(!Serial.available());
  while(Serial.available()) Serial.read();

}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()){
    char input = Serial.read();

    if (input == 'c') {
      Serial.println("Calibration started!");

      Serial.println("Setting max throttle (2000)...");
      esc.writeMicroseconds(2000);
      delay(5000);

      Serial.println("Setting min throttle (1000)...");
      esc.writeMicroseconds(1000);
      delay(5000);

      Serial.println("Setting Neutral throttle (1500)...");
      esc.writeMicroseconds(1500);
      delay(5000);

      Serial.println("Calibration complete! Disconnect power");
      }
    }

}
