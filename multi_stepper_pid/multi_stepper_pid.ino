#include <Wire.h>
#include <math.h>
#include <AS5600.h>
#include <ESP32Servo.h>

// =====================
// Pin configuration
// =====================
const int STP_A = 13; const int DIR_A = 12;
const int STP_B = 27; const int DIR_B = 33;
const int STP_C = 15; const int DIR_C = 32;
const int GRIP_PIN = 4;
const int JOY_D_PIN = 25;

// =====================
// Timing
// =====================
unsigned long STEP_PULSE_US = 5;
float dt = 0.01; // Loop time for integral calculation

// =====================
// Motor state struct
// =====================
struct Motor {
  int stepPin; int dirPin; int8_t dir;
  bool stepState; unsigned long lastToggle; unsigned long intervalUs;
  float integralError;  // Integral term
};

Motor motorA = { STP_A, DIR_A, 0, false, 0, 50000, 0 };
Motor motorB = { STP_B, DIR_B, 0, false, 0, 3000, 0 };
Motor motorC = { STP_C, DIR_C, 0, false, 0, 60000, 0 };

// =====================
// Servo
// =====================
Servo gripServo;
int servoAngle = 180;
const int SERVO_MIN_ANGLE = 120;
const int SERVO_MAX_ANGLE = 180;
const unsigned long SERVO_INTERVAL_MS = 10;
unsigned long lastServoUpdate = 0;

// =====================
// PID / P / PD gains
// =====================
float Kp_A = 1;  // Base P
float Kp_B = 8, Ki_B = 8, Kd_B = 8;
float Kp_C = 2, Ki_C = 0, Kd_C = 0.0;

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

float readIMU(int addr, float& compPitch, bool& initialized, float& gyroRate){
  int16_t ay_raw, az_raw, gx_raw;

  Wire.beginTransmission(addr);
  Wire.write(ACCEL_YOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, 4, true);
  ay_raw = Wire.read()<<8 | Wire.read();
  az_raw = Wire.read()<<8 | Wire.read();

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

  gyroRate = gx;
  return compPitch;
}

int joystickToDir(int raw){
  if(raw <= 1) return -1;
  else if(raw >= 4095) return 1;
  else return 0;
}

// =====================
// Motor update
// =====================
void updateMotor(Motor &m){
  if(m.dir == 0){ digitalWrite(m.stepPin, LOW); m.stepState=false; return; }
  digitalWrite(m.dirPin, (m.dir>0)?HIGH:LOW);

  unsigned long now = micros();
  if(!m.stepState){
    if(now - m.lastToggle >= m.intervalUs - STEP_PULSE_US){
      m.stepState = true;
      m.lastToggle = now;
      digitalWrite(m.stepPin,HIGH);
    }
  }else{
    if(now - m.lastToggle >= STEP_PULSE_US){
      m.stepState=false;
      m.lastToggle=now;
      digitalWrite(m.stepPin,LOW);
    }
  }
}

// =====================
// Motor control
// =====================
// Base: P only
void updateBaseP(Motor &m, float currentAngle, float targetAngle, float Kp){
  float error = angleDiff(targetAngle,currentAngle);
  if(error>4) m.dir=1; else if(error<-4) m.dir=-1; else m.dir=0;

  unsigned long minInterval=40000;
  unsigned long maxInterval=80000;
  unsigned long interval = (unsigned long)(fabs(1.0/(Kp*error+0.001)*1000000));
  if(interval<minInterval) interval=minInterval;
  if(interval>maxInterval) interval=maxInterval;
  m.intervalUs = interval;
  updateMotor(m);
}

// =====================
// D low-pass filter parameters
// =====================
float D_alpha_B = 0.3;  // MotorB D low-pass filter coefficient
float D_alpha_C = 0.3;  // MotorC D low-pass filter coefficient
float filteredGyroB = 0;
float filteredGyroC = 0;

// =====================
// Motor PID with D low-pass
// =====================
void updateMotorBPID(Motor &m, float currentAngle, float targetAngle, float Kp, float Ki, float Kd, float gyroRate){
    static float lastError = 0;
    float error = angleDiff(targetAngle,currentAngle);

    // Deadzone threshold (in degrees)
    const float DEADZONE = 2.0;  

    if(fabs(error) <= DEADZONE){
        m.dir = 0;             
        m.integralError = 0;   
        return;                
    }

    // Reset integral if sign changes
    if(error*lastError < 0) m.integralError = 0;
    m.integralError += error*dt;
    if(m.integralError>100) m.integralError=100;
    if(m.integralError<-100) m.integralError=-100;

    // D low-pass filter
    filteredGyroB = D_alpha_B*gyroRate + (1-D_alpha_B)*filteredGyroB;

    float u = Kp*error + Ki*m.integralError + Kd*(-filteredGyroB);
    if(u>1) m.dir=1; else if(u<-1) m.dir=-1; else m.dir=0;

    unsigned long minInterval=3000;
    unsigned long interval = (unsigned long)(fabs(1.0/(fabs(u)+0.001)*1000000));
    if(interval<minInterval) interval=minInterval;
    m.intervalUs = interval;

    updateMotor(m);

    lastError = error;
}


void updateMotorCPID(Motor &m, float currentAngle, float targetAngle, float Kp, float Ki, float Kd, float gyroRate){
    static float lastError = 0;
    float error = angleDiff(targetAngle,currentAngle);

    if(error*lastError < 0) m.integralError = 0;
    m.integralError += error*dt;
    if(m.integralError>100) m.integralError=100;
    if(m.integralError<-100) m.integralError=-100;

    // D low-pass filter
    filteredGyroC = D_alpha_C*gyroRate + (1-D_alpha_C)*filteredGyroC;

    float u = Kp*error + Ki*m.integralError + Kd*(-filteredGyroC);
    if(u>1) m.dir=-1; else if(u<-1) m.dir=1; else m.dir=0;

    unsigned long minInterval=60000;
    unsigned long interval = (unsigned long)(fabs(1.0/(fabs(u)+0.001)*1000000));
    if(interval<minInterval) interval=minInterval;
    m.intervalUs = interval;
    updateMotor(m);

    lastError = error;
}

// =====================
// Servo
// =====================
void updateServo(int dir){
  unsigned long now = millis();
  if(dir==0) return;
  if(now - lastServoUpdate >= SERVO_INTERVAL_MS){
    lastServoUpdate=now;
    servoAngle += dir;
    if(servoAngle<SERVO_MIN_ANGLE) servoAngle=SERVO_MIN_ANGLE;
    if(servoAngle>SERVO_MAX_ANGLE) servoAngle=SERVO_MAX_ANGLE;
    gripServo.write(servoAngle);
  }
}

// =====================
// Setup
// =====================
void setup(){
  pinMode(STP_A, OUTPUT); pinMode(DIR_A, OUTPUT);
  pinMode(STP_B, OUTPUT); pinMode(DIR_B, OUTPUT);
  pinMode(STP_C, OUTPUT); pinMode(DIR_C, OUTPUT);

  Serial.begin(115200);
  Serial.println("Multi-stepper + Servo with P/PD/PID control ready.");
  gripServo.setPeriodHertz(50); gripServo.attach(GRIP_PIN); gripServo.write(servoAngle);
  Wire.begin();

  bool imu1_ok = checkDevice(MPU1_ADDR);
  bool imu2_ok = checkDevice(MPU2_ADDR);
  bool enc_ok  = enc.isConnected();
  Serial.printf("IMU1: %s  IMU2: %s  ENC: %s\n", imu1_ok?"OK":"FAIL", imu2_ok?"OK":"FAIL", enc_ok?"OK":"FAIL");

  if(imu1_ok) wakeMPU(MPU1_ADDR);
  if(imu2_ok) wakeMPU(MPU2_ADDR);
  delay(300);
}

// =====================
// Main loop
// =====================
void loop(){
  float gyro1, gyro2;
  float pitch1 = readIMU(MPU1_ADDR, compPitch1, init1, gyro1);
  pitch1 = normalize360(180-pitch1);
  float pitch2 = readIMU(MPU2_ADDR, compPitch2, init2, gyro2);
  float angleEnc = normalize360(enc.rawAngle()*AS5600_RAW_TO_DEGREES + 95);

  float targetA = 180;
  float targetB = 230;
  float targetC = 90;

  updateBaseP(motorA, angleEnc, targetA, Kp_A);   
  updateMotorBPID(motorB, pitch2, targetB, Kp_B, Ki_B, Kd_B, gyro2); 
  updateMotorCPID(motorC, pitch1, targetC, Kp_C, Ki_C, Kd_C, gyro1);

  int servoDir = joystickToDir(analogRead(JOY_D_PIN));
  updateServo(servoDir);

  Serial.printf("IMU1: %.2f  IMU2: %.2f  ENC: %.2f\n", pitch1, pitch2, angleEnc);
  Serial.printf("INTERVAL A: %lu  B: %lu  C: %lu\n", motorA.intervalUs, motorB.intervalUs, motorC.intervalUs);
  Serial.printf("DIR A: %d  B: %d  C: %d\n", motorA.dir, motorB.dir, motorC.dir);


  delay(1);
}
