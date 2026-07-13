
#include <Servo.h>

Servo steeringServo;

const int SERVO_PIN = 3;

void setup() {
  // put your setup code here, to run once:
  steeringServo.attach(SERVO_PIN);

  // Start centered
  steeringServo.write(90);
}

void loop() {
  // put your main code here, to run repeatedly:
  steeringServo.write(45);
  delay(2000);
  steeringServo.write(90);
  delay(2000);
  steeringServo.write(135);
  delay(2000);

}
