#include <AccelStepper.h>

#define STEP_PIN 17
#define DIR_PIN 16
#define ENABLE_PIN 4

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  Serial.begin(115200);
  Serial.println("Moving to 45 degrees and holding...");

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // LOW = enabled (for A4988/DRV8825)

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  // --- Calculate steps for 45 degrees ---
  // 1.8° per step = 200 steps/rev = 360°
  // 45° = (200 * 45 / 360) = 25 steps
  long stepsFor45deg = 25;

  // Move to 45 degrees (25 full steps)
  stepper.moveTo(stepsFor45deg);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  Serial.println("Reached 45 degrees — holding position.");
}

void loop() {
  // Keep coils energized to hold torque
  digitalWrite(ENABLE_PIN, LOW);
  delay(1000);
}

