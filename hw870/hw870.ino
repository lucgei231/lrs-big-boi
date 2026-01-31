// Ultrasonic sensor (HC-SR04) - using analog read on echo pin
const int TRIG_PIN = 21;  // Trigger pin
const int ECHO_PIN = 22;  // Echo pin (will read analog)

// TCRT5000 IR Sensor 2
const int SENSOR2_A0 = 2;   // Analog output
const int SENSOR2_D0 = 16;  // Digital output
//m1: d32, d33
//m2: d25, d26

//m3 d19, d17
//m4 d18, d23

//m5 d27, d14
//m6 d12, d13


// Motor pin assignments (from user comments)
// m1: d32, d33
const int M1_PIN1 = 32;
const int M1_PIN2 = 33;

// m2: d25, d26
const int M2_PIN1 = 25;
const int M2_PIN2 = 26;

// m3: d19, d17
const int M3_PIN1 = 19;
const int M3_PIN2 = 23;

// m4: d18, d23
const int M4_PIN1 = 18;
const int M4_PIN2 = 17;

// m5: d27, d14
const int M5_PIN1 = 14;
const int M5_PIN2 = 27;

// m6: d12, d13
const int M6_PIN1 = 12;
const int M6_PIN2 = 13;

// M5 control variables
unsigned long m5LastSwitchTime = 0;
const unsigned long M5_INTERVAL = 3000; // 3 seconds
bool m5Forward = true; // Track direction for all motors

void setup(){
  Serial.begin(115200);
  
  // Configure ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);  // Read ECHO_PIN as analog

  // Configure Sensor 2 D0 pin as digital input
  pinMode(SENSOR2_D0, INPUT);

  Serial.println("Ultrasonic + TCRT5000 IR Sensor Test");
  Serial.println("====================================");
  Serial.print("Ultrasonic Trig -> "); Serial.print(TRIG_PIN); Serial.print(", Echo -> "); Serial.println(ECHO_PIN);
  Serial.print("Sensor2 A0 -> "); Serial.print(SENSOR2_A0); Serial.print(", D0 -> "); Serial.println(SENSOR2_D0);
  Serial.println();

  // Configure motor pins as outputs
  pinMode(M1_PIN1, OUTPUT); pinMode(M1_PIN2, OUTPUT);
  pinMode(M2_PIN1, OUTPUT); pinMode(M2_PIN2, OUTPUT);
  pinMode(M3_PIN1, OUTPUT); pinMode(M3_PIN2, OUTPUT);
  pinMode(M4_PIN1, OUTPUT); pinMode(M4_PIN2, OUTPUT);
  pinMode(M5_PIN1, OUTPUT); pinMode(M5_PIN2, OUTPUT);
  pinMode(M6_PIN1, OUTPUT); pinMode(M6_PIN2, OUTPUT);
  pinMode(0, INPUT_PULLUP); // user button

  // Turn all motors off
  digitalWrite(M1_PIN1, LOW); digitalWrite(M1_PIN2, LOW);
  digitalWrite(M2_PIN1, LOW); digitalWrite(M2_PIN2, LOW);
  digitalWrite(M3_PIN1, LOW); digitalWrite(M3_PIN2, LOW);
  digitalWrite(M4_PIN1, LOW); digitalWrite(M4_PIN2, LOW);
  digitalWrite(M5_PIN1, LOW); digitalWrite(M5_PIN2, LOW);
  digitalWrite(M6_PIN1, LOW); digitalWrite(M6_PIN2, LOW);
}

// Function to read ultrasonic sensor via analog input
int readUltrasonicAnalog() {
    digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout after 30ms (~5m max)
  
  // Calculate distance: speed of sound is ~343 m/s (0.0343 cm/µs)
  // distance = (duration / 2) * 0.0343 cm
  float distance = (duration / 2.0) * 0.0343;

  return distance;

}

// Function to drive all 6 motors with alternating direction every 3 seconds
void driveAllMotors(int speed) {
  unsigned long currentTime = millis();
  
  // Check if 3 seconds have passed
  if (currentTime - m5LastSwitchTime >= M5_INTERVAL) {
    m5LastSwitchTime = currentTime;
    m5Forward = !m5Forward; // Toggle direction for all motors
  }
  
  // Fixed speed for all motors
  int motorSpeed = 200;
  
  if (m5Forward) {
    // All motors forward: PIN1 HIGH, PIN2 LOW
    analogWrite(M1_PIN1, motorSpeed); digitalWrite(M1_PIN2, LOW);
    analogWrite(M2_PIN1, motorSpeed); digitalWrite(M2_PIN2, LOW);
    analogWrite(M3_PIN1, motorSpeed); digitalWrite(M3_PIN2, LOW);
    analogWrite(M4_PIN1, motorSpeed); digitalWrite(M4_PIN2, LOW);
    analogWrite(M5_PIN1, motorSpeed); digitalWrite(M5_PIN2, LOW);
    analogWrite(M6_PIN1, motorSpeed); digitalWrite(M6_PIN2, LOW);
  } else {
    // All motors backward: PIN1 LOW, PIN2 HIGH
    digitalWrite(M1_PIN1, LOW); analogWrite(M1_PIN2, motorSpeed);
    digitalWrite(M2_PIN1, LOW); analogWrite(M2_PIN2, motorSpeed);
    digitalWrite(M3_PIN1, LOW); analogWrite(M3_PIN2, motorSpeed);
    digitalWrite(M4_PIN1, LOW); analogWrite(M4_PIN2, motorSpeed);
    digitalWrite(M5_PIN1, LOW); analogWrite(M5_PIN2, motorSpeed);
    digitalWrite(M6_PIN1, LOW); analogWrite(M6_PIN2, motorSpeed);
  }
  delay(300);
}

// Function to alternate M5 direction every 3 seconds
void alternateM5() {
  // Direction alternation is now handled in driveAllMotors()
}

// Helper: run a single motor in the requested direction and print status
void runMotor(int pin1, int pin2, int motorNum, bool forward, int speed, unsigned long runMs) {
  if (forward) {
    Serial.print("Motor "); Serial.print(motorNum); Serial.println(" -> Forward");
    analogWrite(pin1, speed); digitalWrite(pin2, LOW);
  } else {
    Serial.print("Motor "); Serial.print(motorNum); Serial.println(" -> Backward");
    digitalWrite(pin1, LOW); analogWrite(pin2, speed);
  }
  delay(runMs);
  // Stop motor
  digitalWrite(pin1, LOW); digitalWrite(pin2, LOW);
  delay(200); // short pause after stopping
}

// Set motor direction without delay (for simultaneous control)
void setMotor(int pin1, int pin2, bool forward, int speed) {
  if (forward) {
    digitalWrite(pin1, HIGH); digitalWrite(pin2, LOW);
  } else {
    digitalWrite(pin1, LOW); digitalWrite(pin2, HIGH);
  }
}

void stopAllMotors() {
  digitalWrite(M1_PIN1, LOW); digitalWrite(M1_PIN2, LOW);
  digitalWrite(M2_PIN1, LOW); digitalWrite(M2_PIN2, LOW);
  digitalWrite(M3_PIN1, LOW); digitalWrite(M3_PIN2, LOW);
  digitalWrite(M4_PIN1, LOW); digitalWrite(M4_PIN2, LOW);
  digitalWrite(M5_PIN1, LOW); digitalWrite(M5_PIN2, LOW);
  digitalWrite(M6_PIN1, LOW); digitalWrite(M6_PIN2, LOW);
}

void moveForward(int speed, unsigned long runMs) {
  Serial.println("Action: moveForward (m4,m5,m6,m3) -> Forward");
  setMotor(M4_PIN1, M4_PIN2, true, speed);
  setMotor(M5_PIN1, M5_PIN2, true, speed);
  setMotor(M6_PIN1, M6_PIN2, true, speed);
  setMotor(M3_PIN1, M3_PIN2, true, speed);
  delay(runMs);
  stopAllMotors();
  delay(150);
}

void moveBackward(int speed, unsigned long runMs) {
  Serial.println("Action: moveBackward (m4,m5,m6,m3) -> Backward");
  setMotor(M4_PIN1, M4_PIN2, false, speed);
  setMotor(M5_PIN1, M5_PIN2, false, speed);
  setMotor(M6_PIN1, M6_PIN2, false, speed);
  setMotor(M3_PIN1, M3_PIN2, false, speed);
  delay(runMs);
  stopAllMotors();
  delay(150);
}

void turnLeft(int speed, unsigned long runMs) {
  Serial.println("Action: turnLeft -> M6 forward, M5 backward, M3 forward, M4 backward");
  setMotor(M6_PIN1, M6_PIN2, true, speed);
  setMotor(M5_PIN1, M5_PIN2, false, speed);
  setMotor(M3_PIN1, M3_PIN2, true, speed);
  setMotor(M4_PIN1, M4_PIN2, false, speed);
  delay(runMs);
  stopAllMotors();
  delay(150);
}

void turnRight(int speed, unsigned long runMs) {
  Serial.println("Action: turnRight -> opposite of turnLeft");
  setMotor(M6_PIN1, M6_PIN2, false, speed);
  setMotor(M5_PIN1, M5_PIN2, true, speed);
  setMotor(M3_PIN1, M3_PIN2, false, speed);
  setMotor(M4_PIN1, M4_PIN2, true, speed);
  delay(runMs);
  stopAllMotors();
  delay(150);
}
// // Motor 3 F
// Motor 4 E
// Motor 5 B
// // Motor 6 A
void loop() {
  const int motorSpeed = 200;
  const unsigned long runTime = 2000; // ms per action

  moveForward(motorSpeed, runTime);
  moveBackward(motorSpeed, runTime);
  turnLeft(motorSpeed, runTime);
  turnRight(motorSpeed, runTime);

  Serial.println("Cycle complete.");
  delay(1000);
}