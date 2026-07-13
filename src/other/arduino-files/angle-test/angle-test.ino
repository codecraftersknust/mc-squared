#include <Servo.h>

Servo steeringServo;

const int SERVO_PIN = 3;

void setup() {
  Serial.begin(115200);

  steeringServo.attach(SERVO_PIN);

  // Start centered
  steeringServo.write(90);

  Serial.println("Servo Ready");
}

void loop() {
  if (Serial.available()) {

    String angleString = Serial.readStringUntil('\n');

    int angle = angleString.toInt();

    // Clamp to allowed range
    angle = constrain(angle, 45, 135);

    steeringServo.write(angle);

    Serial.print("Angle set to: ");
    Serial.println(angle);
  }
}
