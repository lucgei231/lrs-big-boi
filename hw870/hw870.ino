// ESP32 LUCAS ROBOT - RADIO RECEIVER (NO WIRES!)
// Wireless communication with Micro:bit using NRF24L01 module
//
// ONLY wire the NRF24L01 module (separate wireless module):
// NRF24 VCC -> 3.3V
// NRF24 GND -> GND
// NRF24 CE -> GPIO 4
// NRF24 CSN -> GPIO 5
// NRF24 MOSI -> GPIO 23 (SPI standard)
// NRF24 MISO -> GPIO 19 (SPI standard)
// NRF24 SCK -> GPIO 18 (SPI standard)

#include <SPI.h>
#include <RF24.h>
#include <WiFi.h>
#include <WebServer.h>

// WiFi credentials
const char* WIFI_SSID = "potato";
const char* WIFI_PASS = "potato123";

WebServer server(80);

// NRF24L01 Radio - NO GPIO PIN ASSIGNMENTS ON ESP32!
// Uses standard SPI pins (always GPIO 18, 19, 23 on ESP32)
RF24 radio(4, 5);  // CE (GPIO 4), CSN (GPIO 5) - only 2 pins!
const byte radioAddress[6] = "00001";

// Motor pins
const int M1_PIN1 = 32, M1_PIN2 = 33;
const int M2_PIN1 = 25, M2_PIN2 = 26;
const int M3_PIN1 = 19, M3_PIN2 = 23;
const int M4_PIN1 = 18, M4_PIN2 = 17;
const int M5_PIN1 = 14, M5_PIN2 = 27;
const int M6_PIN1 = 12, M6_PIN2 = 13;

struct MotorState {
  bool active = false;
  unsigned long startTime = 0;
  unsigned long duration = 0;
  bool forward = false;
  int speed = 0;
};
MotorState currentMotor = {false, 0, 0, false, 0};

// Forward declarations
void setMotor(int pin1, int pin2, bool forward, int speed);
void stopAllMotors();
void scheduleMotorSequence(bool fwd, int spd, unsigned long ms);
void updateMotorTiming();
void handleRadioCommand(String cmd);

void setup(){
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== LUCAS ROBOT - WIRELESS STARTUP ===");
  Serial.println("Setting up motor pins...");
  
  // Setup motor pins
  for (int i = 0; i < 6; i++) {
    if (i == 0) { pinMode(M1_PIN1, OUTPUT); pinMode(M1_PIN2, OUTPUT); }
    if (i == 1) { pinMode(M2_PIN1, OUTPUT); pinMode(M2_PIN2, OUTPUT); }
    if (i == 2) { pinMode(M3_PIN1, OUTPUT); pinMode(M3_PIN2, OUTPUT); }
    if (i == 3) { pinMode(M4_PIN1, OUTPUT); pinMode(M4_PIN2, OUTPUT); }
    if (i == 4) { pinMode(M5_PIN1, OUTPUT); pinMode(M5_PIN2, OUTPUT); }
    if (i == 5) { pinMode(M6_PIN1, OUTPUT); pinMode(M6_PIN2, OUTPUT); }
  }
  
  stopAllMotors();
  Serial.println("All motor pins configured");
  
  // Initialize NRF24L01
  Serial.println("\nInitializing NRF24L01 wireless receiver...");
  Serial.println("Wiring check (MUST HAVE):");
  Serial.println("  NRF24 VCC -> 3.3V");
  Serial.println("  NRF24 GND -> GND");
  Serial.println("  NRF24 CE -> GPIO 4");
  Serial.println("  NRF24 CSN -> GPIO 5");
  Serial.println("  (SPI automatic: GPIO 18, 19, 23)");
  
  if (!radio.begin()) {
    Serial.println("\nERROR: NRF24L01 not found!");
    Serial.println("Check connections and restart.");
    while(1) { Serial.println("WAITING..."); delay(2000); }
  }
  
  radio.openReadingPipe(0, radioAddress);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();
  
  Serial.println("NRF24L01 initialized! Listening...\n");
  
  // WiFi (optional web interface)
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  server.on("/", [](){ server.send(200, "text/html", "<h1>Lucas Connected via Radio</h1>"); });
  server.begin();
}

void setMotor(int pin1, int pin2, bool forward, int speed) {
  if (forward) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  } else {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
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

void scheduleMotorSequence(bool fwd, int spd, unsigned long ms) {
  if (fwd) {
    analogWrite(M4_PIN1, spd); digitalWrite(M4_PIN2, LOW);
    analogWrite(M5_PIN1, spd); digitalWrite(M5_PIN2, LOW);
    analogWrite(M6_PIN1, spd); digitalWrite(M6_PIN2, LOW);
    analogWrite(M3_PIN1, spd); digitalWrite(M3_PIN2, LOW);
  } else {
    digitalWrite(M4_PIN1, LOW); analogWrite(M4_PIN2, spd);
    digitalWrite(M5_PIN1, LOW); analogWrite(M5_PIN2, spd);
    digitalWrite(M6_PIN1, LOW); analogWrite(M6_PIN2, spd);
    digitalWrite(M3_PIN1, LOW); analogWrite(M3_PIN2, spd);
  }
  currentMotor = {true, millis(), ms, fwd, spd};
}

void updateMotorTiming() {
  if (currentMotor.active && (millis() - currentMotor.startTime >= currentMotor.duration)) {
    stopAllMotors();
    currentMotor.active = false;
  }
}

void handleRadioCommand(String cmd) {
  Serial.print(">>> RADIO COMMAND: ");
  Serial.println(cmd);
  
  if (cmd == "FORWARD") {
    Serial.println(">>> MOTORS: FORWARD");
    scheduleMotorSequence(true, 200, 2000);
  }
  else if (cmd == "BACKWARD") {
    Serial.println(">>> MOTORS: BACKWARD");
    scheduleMotorSequence(false, 200, 2000);
  }
  else if (cmd == "STOP") {
    Serial.println(">>> MOTORS: STOP");
    stopAllMotors();
  }
}

void loop() {
  server.handleClient();
  updateMotorTiming();
  
  if (radio.available()) {
    char receivedData[32] = "";
    radio.read(&receivedData, sizeof(receivedData));
    handleRadioCommand(String(receivedData));
  }
  
  delay(10);
}
