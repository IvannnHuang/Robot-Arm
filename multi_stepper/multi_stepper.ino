#include <ESP32Servo.h>

// =====================
// Pin configuration
// =====================

// Stepper A (Base)
const int STP_A = 13;
const int DIR_A = 12;

// Stepper B (Joint 1)
const int STP_B = 27;
const int DIR_B = 33;

// Stepper C (Joint 2)
const int STP_C = 15;
const int DIR_C = 32;

// Gripper servo
const int GRIP_PIN = 4;

// Joystick analog inputs
const int JOY_A_PIN = 34;
const int JOY_B_PIN = 39;
const int JOY_C_PIN = 36;
const int JOY_D_PIN = 25;

// =====================
// Stepper timing / geometry
// =====================

// Stepper interval time
unsigned long STEP_INTERVAL_US_A = 80000;  // safe default
unsigned long STEP_INTERVAL_US_B = 4000;  // safe default
unsigned long STEP_INTERVAL_US_C = 40000;  // safe default

// Global pulse width (HIGH time)
unsigned long STEP_PULSE_US = 5;  // safe default

// Assume 200 full steps per revolution (no microstepping)
const float STEPS_PER_REV = 200.0f;
const float DEG_PER_STEP  = 360.0f / STEPS_PER_REV;

// =====================
// Serial input buffer
// =====================
char serialBuffer[32];
int  serialIndex = 0;

// =====================
// Motor state with angle limits
// =====================
struct Motor {
  int stepPin;
  int dirPin;
  int dir;                 // -1, 0, +1 from joystick
  bool stepState;          // LOW/HIGH state of step pin
  unsigned long lastToggle;
  unsigned long intervalUs; // per-motor step interval
  float angleDeg;          // current logical angle
  float minDeg;
  float maxDeg;
};

// Base: 0..360 deg
Motor motorA = { STP_A, DIR_A, 0, false, 0, STEP_INTERVAL_US_A, 0.0f, -180.0f, 180.0f };
// Joint1: 0..120 deg
Motor motorB = { STP_B, DIR_B, 0, false, 0,  STEP_INTERVAL_US_B, 0.0f, -12000.0f, 0.0f };
// Joint2: 0..120 deg
Motor motorC = { STP_C, DIR_C, 0, false, 0, STEP_INTERVAL_US_C, 0.0f, -1200.0f, 500.0f };

// =====================
// Servo state
// =====================
Servo gripServo;
int   servoAngle = 90;
int   lastPrintedServoAngle = 90;

const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;
const unsigned long SERVO_INTERVAL_MS = 50;
unsigned long lastServoUpdate = 0;

// =====================
// Helper functions
// =====================

// Joystick mapping -1 / 0 / +1
int joystickToDir(int raw) {
  if (raw <= 1)          return -1;
  else if (raw >= 4095)  return +1;
  else                   return 0;
}

// Non-blocking motor update with angle limits
void updateMotor(Motor &m) {
  // No command -> keep step LOW and do nothing
  if (m.dir == 0) {
    m.stepState = false;
    digitalWrite(m.stepPin, LOW);
    return;
  }

  // At angle limit in this direction? Then refuse to step.
  if ((m.dir > 0 && m.angleDeg >= m.maxDeg) ||
      (m.dir < 0 && m.angleDeg <= m.minDeg)) {
    // Keep output LOW; just update lastToggle to avoid tight loop
    m.stepState = false;
    digitalWrite(m.stepPin, LOW);
    m.lastToggle = micros();
    return;
  }

  // Set direction pin
  digitalWrite(m.dirPin, (m.dir > 0) ? HIGH : LOW);

  unsigned long now = micros();

  if (!m.stepState) {
    // Currently LOW -> see if it's time to start a pulse (rising edge)
    if (now - m.lastToggle >= (m.intervalUs - STEP_PULSE_US)) {
      m.stepState = true;
      m.lastToggle = now;
      digitalWrite(m.stepPin, HIGH);

      // Rising edge = one step → update angle
      if (m.dir > 0) m.angleDeg += DEG_PER_STEP;
      else           m.angleDeg -= DEG_PER_STEP;

      // Clamp to [minDeg, maxDeg] to avoid drift
      if (m.angleDeg < m.minDeg) m.angleDeg = m.minDeg;
      if (m.angleDeg > m.maxDeg) m.angleDeg = m.maxDeg;
    }
  } else {
    // Currently HIGH -> see if pulse width elapsed to go LOW
    if (now - m.lastToggle >= STEP_PULSE_US) {
      m.stepState = false;
      m.lastToggle = now;
      digitalWrite(m.stepPin, LOW);
    }
  }
}

// Servo update with angle limit
void updateServo(int dir) {
  unsigned long now = millis();

  if (dir != 0 && now - lastServoUpdate >= SERVO_INTERVAL_MS) {
    lastServoUpdate = now;

    servoAngle += dir;
    if (servoAngle < SERVO_MIN_ANGLE) servoAngle = SERVO_MIN_ANGLE;
    if (servoAngle > SERVO_MAX_ANGLE) servoAngle = SERVO_MAX_ANGLE;

    gripServo.write(servoAngle);

    if (servoAngle != lastPrintedServoAngle) {
      lastPrintedServoAngle = servoAngle;
    }
  }
}

void handleSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();

    // End of command line
    if (c == '\n' || c == '\r') {
      serialBuffer[serialIndex] = '\0';

      if (serialIndex > 0) {
        unsigned long newInterval1, newInterval2, newInterval3;
        // int count = sscanf(serialBuffer, "%lu %lu", &newInterval1, &newInterval2, &newInterval3);
        int count = sscanf(serialBuffer, "%lu %lu", &newInterval1);

        if (count == 1) {
          motorB.intervalUs = newInterval1;

          Serial.print("Updated speed: ");
          Serial.println(motorB.intervalUs);
        }
        else if (count == 3) {
          motorA.intervalUs = newInterval1;
          motorB.intervalUs = newInterval2;
          motorC.intervalUs = newInterval3;

          Serial.println("Updated speed");
        } else {
          Serial.println("Invalid format. Use: <interval> <pulse>");
        }
      }

      serialIndex = 0;  // reset buffer
    }

    // Store characters
    else if (serialIndex < (int)sizeof(serialBuffer) - 1) {
      serialBuffer[serialIndex++] = c;
    }
  }
}


// =====================
// Setup / Loop
// =====================

void setup() {
  pinMode(STP_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(STP_B, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(STP_C, OUTPUT);
  pinMode(DIR_C, OUTPUT);

  Serial.begin(115200);
  Serial.println("Multi-stepper + Servo with angle-limited joints ready.");

  // Servo init
  gripServo.setPeriodHertz(50);
  gripServo.attach(GRIP_PIN);
  gripServo.write(servoAngle);

  // Initial angles (all zero by design)
  Serial.print("Initial Base angle: ");  Serial.println(motorA.angleDeg);
  Serial.print("Initial Joint1 angle: ");Serial.println(motorB.angleDeg);
  Serial.print("Initial Joint2 angle: ");Serial.println(motorC.angleDeg);
}

void loop() {
  handleSerialInput();

  // Read joystick directions
  motorA.dir = joystickToDir(analogRead(JOY_A_PIN)); // Base
  motorB.dir = joystickToDir(analogRead(JOY_B_PIN)); // Joint1
  motorC.dir = joystickToDir(analogRead(JOY_C_PIN)); // Joint2
  int servoDir = joystickToDir(analogRead(JOY_D_PIN)); // Gripper

  // Update steppers with angle limits
  updateMotor(motorA);
  updateMotor(motorB);
  updateMotor(motorC);

  // Update gripper servo
  updateServo(servoDir);
  
  // Serial.print("Initial Base angle: ");  Serial.println(motorA.angleDeg);
  // Serial.print("Initial Joint1 angle: ");Serial.println(motorB.angleDeg);
  // Serial.print("Initial Joint2 angle: ");Serial.println(motorC.angleDeg);
}
