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