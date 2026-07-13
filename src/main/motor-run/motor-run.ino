#include <Servo.h>

Servo steeringServo;
Servo escMotor;

const int SERVO_PIN = 3;
const int ESC_PIN = 7;

// Limits
const int STEERING_MIN = 45;
const int STEERING_MAX = 135;
const int STEERING_CENTER = 90;

const int ESC_MIN = 1500;   // stop
const int ESC_MAX = 2000;

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(115200);

  steeringServo.attach(SERVO_PIN);
  escMotor.attach(ESC_PIN);

  // Center steering
  steeringServo.write(STEERING_CENTER);

  // Arm ESC (IMPORTANT)
  escMotor.writeMicroseconds(1000);
  delay(5000);

  // Start motor slowly (optional safety)
  escMotor.writeMicroseconds(1600);

  inputString.reserve(50);

  Serial.println("Ready");
}

void loop() {

  if (stringComplete) {

    int commaIndex = inputString.indexOf(',');

    if (commaIndex > 0) {

      String angleStr = inputString.substring(0, commaIndex);
      String speedStr = inputString.substring(commaIndex + 1);

      int angle = angleStr.toInt();
      int speed = speedStr.toInt();

      // Clamp values for safety
      angle = constrain(angle, STEERING_MIN, STEERING_MAX);
      speed = constrain(speed, ESC_MIN, ESC_MAX);

      // Apply to hardware
      steeringServo.write(angle);
      escMotor.writeMicroseconds(speed);

      Serial.print("Angle: ");
      Serial.print(angle);
      Serial.print(" Speed: ");
      Serial.println(speed);
    }

    inputString = "";
    stringComplete = false;
  }
}

// Serial event handler
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();

    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}
