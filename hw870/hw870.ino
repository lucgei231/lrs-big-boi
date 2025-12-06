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
const int M3_PIN2 = 17;

// m4: d18, d23
const int M4_PIN1 = 18;
const int M4_PIN2 = 23;

// m5: d27, d14
const int M5_PIN1 = 27;
const int M5_PIN2 = 14;

// m6: d12, d13
const int M6_PIN1 = 12;
const int M6_PIN2 = 13;

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

// Function to measure distance from ultrasonic sensor (HC-SR04)
float measureDistance() {
  // Send 10 microsecond pulse to trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout after 30ms (~5m max)
  
  // Calculate distance: speed of sound is ~343 m/s (0.0343 cm/µs)
  // distance = (duration / 2) * 0.0343 cm
  float distance = (duration / 2.0) * 0.0343;
  
  return distance;
}

void loop(){
  // Read ultrasonic distance
  float distance = measureDistance();
  
  // Read Sensor 2
  int s2_a = analogRead(SENSOR2_A0);
  int s2_d = digitalRead(SENSOR2_D0);

  // Print readings
  Serial.print("Ultrasonic: ");
  Serial.print(distance, 1); // 1 decimal place
  Serial.print(" cm   |   S2 A("); Serial.print(SENSOR2_A0); Serial.print("):"); Serial.print(s2_a);
  Serial.print(" D("); Serial.print(SENSOR2_D0); Serial.print("):"); Serial.println(s2_d?1:0);

  // Drive motors based on sensor values
  analogWrite(M1, s2_a / 4); // Scale 0-4095 to 0-255 for M1
  analogWrite(M2, 0);
  analogWrite(M3, (int)distance * 5); // Scale distance to motor speed (0-5cm * 5 = 0-25, capped at 255)
  analogWrite(M4, 0);

  delay(300);
}