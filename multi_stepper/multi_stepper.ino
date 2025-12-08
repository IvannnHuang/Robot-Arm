// ------------ Pin definitions ------------

// Joystick analog inputs (SET THESE TO YOUR PINS)
const int JOY_A_PIN = 36;  // e.g., joystick A
const int JOY_B_PIN = 39;  // e.g., joystick B
const int JOY_C_PIN = 34;  // e.g., joystick C

// Stepper driver pins
const int STP_A = 13;
const int DIR_A = 12;

const int STP_B = 27;
const int DIR_B = 33;

const int STP_C = 15;
const int DIR_C = 32;

// 200 steps for 360 degree (example, depends on your motor/driver)
const int STEPS_PER_TURN = 200;

// Adjust this delay to change step speed (smaller = faster)
const int STEP_DELAY_US = 5000;



// Map joystick ADC reading to -1, 0, +1 as you specified
int joystickToDir(int raw)
{
  // Use 12-bit ADC: 0..4095
  if (raw <= 1) {
    return -1;  // CCW
  } else if (raw >= 4095) {
    return +1;  // CW
  } else {
    return 0;   // Stop
  }
}

// Step a single motor one step in the given direction:
// dir = -1 => CCW, dir = +1 => CW, dir = 0 => no move
void stepMotor(int stepPin, int dirPin, int dir)
{
  if (dir == 0) {
    return; // no movement
  }

  // Set direction pin
  if (dir > 0) {
    digitalWrite(dirPin, HIGH);  // CW
  } else {
    digitalWrite(dirPin, LOW);   // CCW
  }

  // Generate one step pulse
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_DELAY_US);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY_US);
}


// ------------ Setup & loop ------------

void setup()
{
  Serial.begin(115200);

  // On ESP32, ADC is 12-bit by default, but we can be explicit
  // (If your core doesn't support this, you can comment it out.)
  analogReadResolution(12);

  // Joystick pins are analog inputs
  pinMode(JOY_A_PIN, INPUT);
  pinMode(JOY_B_PIN, INPUT);
  pinMode(JOY_C_PIN, INPUT);

  // Stepper driver pins
  pinMode(STP_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);

  pinMode(STP_B, OUTPUT);
  pinMode(DIR_B, OUTPUT);

  pinMode(STP_C, OUTPUT);
  pinMode(DIR_C, OUTPUT);
}

void loop()
{
  // Read joystick raw values
  int rawA = analogRead(JOY_A_PIN);
  int rawB = analogRead(JOY_B_PIN);
  int rawC = analogRead(JOY_C_PIN);

  // Convert to -1 / 0 / +1
  int dirA = joystickToDir(rawA);
  int dirB = joystickToDir(rawB);
  int dirC = joystickToDir(rawC);

  Serial.print("A: "); Serial.print(rawA);
  Serial.print(" -> "); Serial.print(dirA);
  Serial.print("   B: "); Serial.print(rawB);
  Serial.print(" -> "); Serial.print(dirB);
  Serial.print("   C: "); Serial.print(rawC);
  Serial.print(" -> "); Serial.println(dirC);
  

  // Step each motor once in its current direction
  // This loop repeats forever, so motors "keep moving"
  // as long as joystick direction stays -1 or +1.
  stepMotor(STP_A, DIR_A, dirA);
  stepMotor(STP_B, DIR_B, dirB);
  stepMotor(STP_C, DIR_C, dirC);

  delay(1);
}
