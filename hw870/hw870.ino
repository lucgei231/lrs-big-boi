// The user can only use these pins: 2, 22, 21, 4, 16
// We'll test analogRead on the allowed pins (2 and 4) and digital reads on the D0 pins (16 and 21).
// If you must use pin 22 for analog, note: some ESP32 modules do not support analogRead on GPIO22.

// Assign pins within the user's allowed set.
// Sensor 1: A0 -> GPIO4, D0 -> GPIO16
// Sensor 2: A0 -> GPIO2, D0 -> GPIO21
const int SENSOR1_A0 = 4;   // Analog output (candidate)
const int SENSOR1_D0 = 16;  // Digital output

const int SENSOR2_A0 = 2;   // Analog output (candidate)
const int SENSOR2_D0 = 21;  // Digital output

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

  // Configure D0 pins as digital inputs
  pinMode(SENSOR1_D0, INPUT);
  pinMode(SENSOR2_D0, INPUT);

  Serial.println("TCRT5000 IR Sensor Module Test (allowed pins: 2,22,21,4,16)");
  Serial.println("---------------------------------------------------------");
  Serial.print("Sensor1 A0 -> "); Serial.print(SENSOR1_A0); Serial.print(", D0 -> "); Serial.println(SENSOR1_D0);
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

void loop(){
  // Read analog and digital values
  int s1_a = analogRead(SENSOR1_A0);
  int s1_d = digitalRead(SENSOR1_D0);
  int s2_a = analogRead(SENSOR2_A0);
  int s2_d = digitalRead(SENSOR2_D0);

  // Print readings
  Serial.print("S1 A(" ); Serial.print(SENSOR1_A0); Serial.print("):"); Serial.print(s1_a);
  Serial.print(" D("); Serial.print(SENSOR1_D0); Serial.print("):"); Serial.print(s1_d?1:0);
  Serial.print("   |   ");
  Serial.print("S2 A("); Serial.print(SENSOR2_A0); Serial.print("):"); Serial.print(s2_a);
  Serial.print(" D("); Serial.print(SENSOR2_D0); Serial.print("):"); Serial.println(s2_d?1:0);

  analogWrite(M1, s1_a / 4); // Scale 0-1023 to 0-255
  analogWrite(M2, 0); // Scale 0-102
  analogWrite(M3, s2_a / 4); // Scale 0-1023 to 0-255
  analogWrite(M4, 0); // Scale 0-102
}