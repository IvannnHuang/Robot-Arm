#include <ESP32Servo.h>

const int servoPin = 26;   // change if needed
Servo testServo;

int angle = 0;            // starting angle
const int maxAngle = 180; // sweep to this angle
const int stepSize = 2;   // degrees per step
const int delayMs = 50;   // speed of sweep

void setup() {
  Serial.begin(115200);

  testServo.setPeriodHertz(50);
  testServo.attach(servoPin);  // normal ESP32 servo range

  Serial.println("Starting servo sweep test...");
}

void loop() {
  // Sweep forward: 0 → 180
  if (angle == 0){
    delay(3000);
    angle += stepSize;
  }
  if (angle > 0 && angle <= maxAngle) {
    testServo.write(angle);
    Serial.print("Servo angle = ");
    Serial.println(angle);
    angle += stepSize;
  } else {
    // When finished, hold position forever
    Serial.println("Reached max angle. Stopping sweep.");
    while (true) { delay(1000); }
  }

  delay(delayMs);
}
