// TCRT5000 IR Sensor 1
const int SENSOR1_A0 = 2;   // Analog output
const int SENSOR1_D0 = 16;  // Digital output

// TCRT5000 IR Sensor 2
const int SENSOR2_A0 = 22;  // Analog output
const int SENSOR2_D0 = 21;  // Digital output

void setup(){
  Serial.begin(115200);
  
  // Configure pins
  pinMode(SENSOR1_D0, INPUT);
  pinMode(SENSOR2_D0, INPUT);
  
  Serial.println("TCRT5000 IR Sensor Module Test");
  Serial.println("================================");
  Serial.println("Sensor 1 - A0: pin 2, D0: pin 16");
  Serial.println("Sensor 2 - A0: pin 22, D0: pin 21");
  Serial.println();
          pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);
  // additional direct-write motor pins (match code.ino)
  pinMode(P17, OUTPUT);
  pinMode(P19, OUTPUT);
  pinMode(P18, OUTPUT);
  pinMode(P23, OUTPUT);
  pinMode(P32, OUTPUT);
  pinMode(P33, OUTPUT);
  pinMode(P25, OUTPUT);
  pinMode(P26, OUTPUT);
  // user button on GPIO0 (pulled up)
  pinMode(0, INPUT_PULLUP);
  // ensure motors off
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  digitalWrite(M3, LOW);
  digitalWrite(M4, LOW);
  // ensure additional pins off
  digitalWrite(P17, LOW);
  digitalWrite(P19, LOW);
  digitalWrite(P18, LOW);
  digitalWrite(P23, LOW);
  digitalWrite(P32, LOW);
  digitalWrite(P33, LOW);
  digitalWrite(P25, LOW);
  digitalWrite(P26, LOW);
  const int M1 = 27;
const int M2 = 14;
const int M3 = 12;
const int M4 = 13;

// Additional GPIOs used by simple `code.ino` pattern
const int P17 = 17;
const int P19 = 19;
const int P18 = 18;
const int P23 = 23;
const int P32 = 32;
const int P33 = 33;
const int P25 = 25;
const int P26 = 26;//
}

void loop(){
  // Read Sensor 1
  int sensor1_analog = analogRead(SENSOR1_A0);
  int sensor1_digital = digitalRead(SENSOR1_D0);
  
  // Read Sensor 2
  int sensor2_analog = analogRead(SENSOR2_A0);
  int sensor2_digital = digitalRead(SENSOR2_D0);
  
  // Print values
  Serial.print("Sensor 1 - Analog: ");
  Serial.print(sensor1_analog);
  Serial.print(" | Digital: ");
  Serial.print(sensor1_digital ? "HIGH" : "LOW");
  
  Serial.print("  |  Sensor 2 - Analog: ");
  Serial.print(sensor2_analog);
  Serial.print(" | Digital: ");
  Serial.println(sensor2_digital ? "HIGH" : "LOW");
  
  delay(500);  // Update every 500ms
}