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


// Keep motor/aux pins (unused for sensor test but preserved)
const int M1 = 27;
const int M2 = 14;
const int M3 = 12;
const int M4 = 13;
const int P17 = 17;
const int P19 = 19;
const int P18 = 18;
const int P23 = 23;
const int P32 = 32;
const int P33 = 33;
const int P25 = 25;
const int P26 = 26;

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

  // Configure motor pins as outputs (kept from original file)
  pinMode(M1, OUTPUT); pinMode(M2, OUTPUT); pinMode(M3, OUTPUT); pinMode(M4, OUTPUT);
  pinMode(P17, OUTPUT); pinMode(P19, OUTPUT); pinMode(P18, OUTPUT); pinMode(P23, OUTPUT);
  pinMode(P32, OUTPUT); pinMode(P33, OUTPUT); pinMode(P25, OUTPUT); pinMode(P26, OUTPUT);
  pinMode(0, INPUT_PULLUP); // user button

  // Turn motors off
  digitalWrite(M1, LOW); digitalWrite(M2, LOW); digitalWrite(M3, LOW); digitalWrite(M4, LOW);
}

// Function to read ultrasonic sensor via analog input
int readUltrasonicAnalog() {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read the analog voltage on echo pin (proportional to distance)
  int analogValue = analogRead(ECHO_PIN);
  return analogValue;
}

void loop(){
  // Read ultrasonic sensor analog value
  int ultrasonicValue = readUltrasonicAnalog();
  
  // Read Sensor 2
  int s2_a = analogRead(SENSOR2_A0);
  int s2_d = digitalRead(SENSOR2_D0);

  // Print readings
  Serial.print("Ultrasonic (Analog): ");
  Serial.print(ultrasonicValue);
  Serial.print("   |   S2 A("); Serial.print(SENSOR2_A0); Serial.print("):"); Serial.print(s2_a);
  Serial.print(" D("); Serial.print(SENSOR2_D0); Serial.print("):"); Serial.println(s2_d?1:0);

  // Drive motors based on sensor values
  analogWrite(M1, s2_a / 4); // Scale 0-4095 to 0-255 for M1
  analogWrite(M2, 0);
  analogWrite(M3, ultrasonicValue / 4); // Scale analog reading to 0-255
  analogWrite(M4, 0);

  delay(300);
}