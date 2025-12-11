#include <Wire.h>
#include <math.h>
#include <AS5600.h>
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

// Joysticks
const int JOY_A_PIN = 34;
const int JOY_B_PIN = 39;
const int JOY_C_PIN = 36;
const int JOY_D_PIN = 25;

// =====================
// Timing parameters
// =====================
unsigned long STEP_PULSE_US = 5;  // only pulse is global

// =====================
// Motor state struct
// =====================
struct Motor {
  int stepPin;
  int dirPin;
  int dir;
  bool stepState;
  unsigned long lastToggle;
  unsigned long intervalUs;   // individual motor speed
};

// Create motors with initial intervals
Motor motorA = { STP_A, DIR_A, 0, false, 0, 50000 }; 
Motor motorB = { STP_B, DIR_B, 0, false, 0, 4000  }; 
Motor motorC = { STP_C, DIR_C, 0, false, 0, 20000 };

// =====================
// Servo state
// =====================
Servo gripServo;
int servoAngle = 180;
int lastPrintedServoAngle = servoAngle;

const int SERVO_MIN_ANGLE = 120;
const int SERVO_MAX_ANGLE = 180;
const unsigned long SERVO_INTERVAL_MS = 10;
unsigned long lastServoUpdate = 0;

// =====================
// P control parameters
// =====================
float Kp_A = 0.5;  // Base
float Kp_B = 1.0;  // Joint1
float Kp_C = 0.8;  // Joint2

// =====================
// IMU + Encoder
// =====================
#define MPU1_ADDR 0x68
#define MPU2_ADDR 0x69
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define PWR_MGMT_1   0x6B

AS5600 enc;
float alpha = 0.9;
float dt = 0.01;
float compPitch1 = 0, compPitch2 = 0;
bool init1 = false, init2 = false;

// =====================
// Helper functions
// =====================
float normalize360(float angle){
  while(angle >= 360) angle -= 360;
  while(angle < 0) angle += 360;
  return angle;
}

float angleDiff(float target, float source){
  float diff = target - source;
  if(diff > 180) diff -= 360;
  if(diff < -180) diff += 360;
  return diff;
}

void wakeMPU(int addr){
  Wire.beginTransmission(addr);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission();
  delay(20);
}

bool checkDevice(byte addr){
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);  
}

float readIMU(int addr, float& compPitch, bool& initialized){
  int16_t ay_raw, az_raw, gx_raw;

  // accel
  Wire.beginTransmission(addr);
  Wire.write(ACCEL_YOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, 4, true);
  ay_raw = Wire.read()<<8 | Wire.read();
  az_raw = Wire.read()<<8 | Wire.read();

  // gyro
  Wire.beginTransmission(addr);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, 2, true);
  gx_raw = Wire.read()<<8 | Wire.read();

  float ay = ay_raw / 16384.0;
  float az = az_raw / 16384.0;
  float gx = gx_raw / 131.0;

  float accelPitch = atan2(-az, ay) * 180 / M_PI;
  accelPitch = normalize360(accelPitch + 180);

  if(!initialized){
    compPitch = accelPitch;
    initialized = true;
  }

  float gyroPitch = normalize360(compPitch + gx * dt);
  float diff = angleDiff(accelPitch, gyroPitch);
  compPitch = normalize360(alpha * gyroPitch + (1-alpha)*(gyroPitch + diff));

  return compPitch;
}

int joystickToDir(int raw) {
  if (raw <= 1)          return -1;
  else if (raw >= 4095)  return +1;
  else                   return 0;
}

// Non-blocking per-motor stepper update
void updateMotor(Motor &m) {
  if (m.dir == 0) {
    m.stepState = false;
    digitalWrite(m.stepPin, LOW);
    return;
  }

  digitalWrite(m.dirPin, (m.dir > 0) ? HIGH : LOW);

  unsigned long now = micros();

  if (!m.stepState) {
    if (now - m.lastToggle >= (m.intervalUs - STEP_PULSE_US)) {
      m.stepState = true;
      m.lastToggle = now;
      digitalWrite(m.stepPin, HIGH);
    }
  } else {
    if (now - m.lastToggle >= STEP_PULSE_US) {
      m.stepState = false;
      m.lastToggle = now;
      digitalWrite(m.stepPin, LOW);
    }
  }
}

// P control for stepper motor
void updateMotorP(Motor &m, float currentAngle, float targetAngle, float Kp) {
  float error = angleDiff(targetAngle, currentAngle); // -180~+180

  if (fabs(error) < 1.0) {
    m.dir = 0;
    return;
  } else {
    m.dir = (error > 0) ? 1 : -1;
  }

  unsigned long minInterval = 20000;

  unsigned long interval = (unsigned long)(fabs(1.0 / (Kp * error)) * 1000000);
  if (interval < minInterval) interval = minInterval;


  m.intervalUs = interval;

  updateMotor(m);
}

// Non-blocking servo update
void updateServo(int dir) {
  unsigned long now = millis();
  if (dir == 0) return;

  if (now - lastServoUpdate >= SERVO_INTERVAL_MS) {
    lastServoUpdate = now;
    servoAngle += dir;
    if (servoAngle < SERVO_MIN_ANGLE) servoAngle = SERVO_MIN_ANGLE;
    if (servoAngle > SERVO_MAX_ANGLE) servoAngle = SERVO_MAX_ANGLE;

    gripServo.write(servoAngle);
  }
}

// =====================
// Setup
// =====================
void setup() {
  pinMode(STP_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(STP_B, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(STP_C, OUTPUT);
  pinMode(DIR_C, OUTPUT);

  Serial.begin(115200);
  Serial.println("Multi-stepper + Servo with P control ready.");

  gripServo.setPeriodHertz(50);
  gripServo.attach(GRIP_PIN);
  gripServo.write(servoAngle);

  Wire.begin();

  // ---- Self test ----
  bool imu1_ok = checkDevice(MPU1_ADDR);
  bool imu2_ok = checkDevice(MPU2_ADDR);
  bool enc_ok  = enc.isConnected();

  Serial.printf("IMU1: %s  IMU2: %s  ENC: %s\n",
                imu1_ok?"OK":"FAIL",
                imu2_ok?"OK":"FAIL",
                enc_ok?"OK":"FAIL");

  if(imu1_ok) wakeMPU(MPU1_ADDR);
  if(imu2_ok) wakeMPU(MPU2_ADDR);

  delay(300);
}

// =====================
// Main loop
// =====================
void loop() {
  // Read angles
  float pitch1 = readIMU(MPU1_ADDR, compPitch1, init1);
  pitch1 = normalize360(180 - pitch1);
  float pitch2 = readIMU(MPU2_ADDR, compPitch2, init2);
  float angleEnc = enc.rawAngle() * AS5600_RAW_TO_DEGREES;
  angleEnc = normalize360(angleEnc + 95);

  // Target angles (can later use serial / joystick to set)
  float targetA = 90;   // Base
  float targetB = 120;   // Joint1
  float targetC = 120;  // Joint2

  // Update motors with P control
  updateMotorP(motorA, angleEnc, targetA, Kp_A);
  updateMotorP(motorB, pitch2, targetB, Kp_B);
  updateMotorP(motorC, pitch1, targetC, Kp_C);

  // Update servo from joystick
  int servoDir = joystickToDir(analogRead(JOY_D_PIN));
  updateServo(servoDir);

  // Optional: print angles
  Serial.printf("IMU1: %.2f°  IMU2: %.2f°  ENC: %.2f°\n",
                pitch1, pitch2, angleEnc);

  Serial.printf("INTERVAL A: %lu  B: %lu  C: %lu\n",
                motorA.intervalUs,
                motorB.intervalUs,
                motorC.intervalUs);


  delay(1);
}
