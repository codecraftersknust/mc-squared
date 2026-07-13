#include <Servo.h>

Servo esc;

void setup() {
  esc.attach(7);

  // Arm ESC
  esc.writeMicroseconds(1000);
  delay(2000);

  Serial.begin(115200);
  Serial.println("ESC Armed");
}

void loop() {

  // Slow speed
  esc.writeMicroseconds(1300);
  delay(5000);

  // Stop
  esc.writeMicroseconds(1000);
  delay(3000);
}
