#include <ESP32Servo.h>

// =====================
// Pin configuration
// =====================

// Stepper A
const int STP_A = 13;
const int DIR_A = 12;

// Stepper B
const int STP_B = 27;
const int DIR_B = 33;

// Stepper C
const int STP_C = 15;
const int DIR_C = 32;

// Gripper servo
const int GRIP_PIN = 4;

// Joystick analog inputs (CHANGE TO YOUR PINS)
const int JOY_A_PIN = 34;
const int JOY_B_PIN = 39;
const int JOY_C_PIN = 36;
const int JOY_D_PIN = 25;

// =====================
// Timing parameters (modifiable at runtime)
// =====================
unsigned long STEP_INTERVAL_US = 5000;  // default
unsigned long STEP_PULSE_US    = 5;     // default

// Buffer for reading serial commands
char serialBuffer[32];
int serialIndex = 0;

// =====================
// Motor state struct
// =====================
struct Motor {
  int stepPin;
  int dirPin;
  int dir;
  bool stepState;
  unsigned long lastToggle;
};

Motor motorA = { STP_A, DIR_A, 0, false, 0 };
Motor motorB = { STP_B, DIR_B, 0, false, 0 };
Motor motorC = { STP_C, DIR_C, 0, false, 0 };

// =====================
// Servo state
// =====================
Servo gripServo;                        // ESP32Servo object
int   servoAngle = 180;                  // start at middle (0–180)
int   lastPrintedServoAngle = servoAngle;

const int SERVO_MIN_ANGLE = 120;
const int SERVO_MAX_ANGLE = 180;

// how often we nudge the servo when joystick is held
const unsigned long SERVO_INTERVAL_MS = 10;  // 20 Hz
unsigned long lastServoUpdate = 0;

// =====================
// Helper functions
// =====================

// joystick mapping
int joystickToDir(int raw) {
  if (raw <= 1)          return -1;
  else if (raw >= 4095)  return +1;
  else                   return 0;
}

// Non-blocking motor update
void updateMotor(Motor &m) {
  if (m.dir == 0) {
    m.stepState = false;
    digitalWrite(m.stepPin, LOW);
    return;
  }

  digitalWrite(m.dirPin, (m.dir > 0) ? HIGH : LOW);

  unsigned long now = micros();

  // LOW → HIGH (start pulse)
  if (!m.stepState) {
    if (now - m.lastToggle >= (STEP_INTERVAL_US - STEP_PULSE_US)) {
      m.stepState = true;
      m.lastToggle = now;
      digitalWrite(m.stepPin, HIGH);
    }
  }
  // HIGH → LOW (end pulse)
  else {
    if (now - m.lastToggle >= STEP_PULSE_US) {
      m.stepState = false;
      m.lastToggle = now;
      digitalWrite(m.stepPin, LOW);
    }
  }
}

// Non-blocking servo update using same -1/0/+1 joystick logic
void updateServo(int dir) {
  unsigned long now = millis();

  if (dir == 0) {
    // joystick neutral: no movement
    return;
  }

  if (now - lastServoUpdate >= SERVO_INTERVAL_MS) {
    lastServoUpdate = now;

    // move 1 degree per update in joystick direction
    servoAngle += dir;   // dir = -1 or +1
    if (servoAngle < SERVO_MIN_ANGLE) servoAngle = SERVO_MIN_ANGLE;
    if (servoAngle > SERVO_MAX_ANGLE) servoAngle = SERVO_MAX_ANGLE;

    gripServo.write(servoAngle);

    // print when angle changes
    if (servoAngle != lastPrintedServoAngle) {
      lastPrintedServoAngle = servoAngle;
      Serial.print("Servo angle: ");
      Serial.println(servoAngle);
    }
  }
}

// =====================
// Serial command parsing
// =====================

// Expected input: "<interval> <pulse>"
// Example:  "5000 5"
void handleSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();

    // End of command line
    if (c == '\n' || c == '\r') {
      serialBuffer[serialIndex] = '\0';

      if (serialIndex > 0) {
        unsigned long newInterval, newPulse;
        int count = sscanf(serialBuffer, "%lu %lu", &newInterval, &newPulse);

        if (count == 2) {
          STEP_INTERVAL_US = newInterval;
          STEP_PULSE_US    = newPulse;

          Serial.print("Updated speed: STEP_INTERVAL_US=");
          Serial.print(STEP_INTERVAL_US);
          Serial.print("  STEP_PULSE_US=");
          Serial.println(STEP_PULSE_US);
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
  Serial.println("Multi-stepper + servo controller (ESP32Servo) ready.");
  Serial.println("Enter: <STEP_INTERVAL_US> <STEP_PULSE_US>");
  Serial.println("Example: 5000 5");

  // ESP32Servo setup (same style as your example)
  gripServo.setPeriodHertz(50);   // standard 50 Hz
  gripServo.attach(GRIP_PIN);     // simple attach; default pulse range is fine
  gripServo.write(servoAngle);    // center servo
}

void loop() {
  // Read input speeds if user typed something
  handleSerialInput();

  // Read joysticks for steppers
  motorA.dir = joystickToDir(analogRead(JOY_A_PIN));
  motorB.dir = joystickToDir(analogRead(JOY_B_PIN));
  motorC.dir = joystickToDir(analogRead(JOY_C_PIN));

  // Read joystick for servo
  int servoDir = joystickToDir(analogRead(JOY_D_PIN));

  // Non-blocking updates
  updateMotor(motorA);
  updateMotor(motorB);
  updateMotor(motorC);
  updateServo(servoDir);
}
