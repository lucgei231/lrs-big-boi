// ESP32 + NRF24L01 Receiver (for Micro:bit Radio communication)
// Wire NRF24L01 to ESP32:
// NRF24 VCC -> ESP32 3.3V
// NRF24 GND -> ESP32 GND
// NRF24 CE -> GPIO 4
// NRF24 CSN -> GPIO 5
// NRF24 MOSI -> GPIO 23
// NRF24 MISO -> GPIO 19
// NRF24 SCK -> GPIO 18

#include <SPI.h>
#include <RF24.h>

RF24 radio(4, 5);  // CE, CSN pins
const byte address[6] = "00001";

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== LUCAS NRF24 RECEIVER ===");
  
  if (!radio.begin()) {
    Serial.println("NRF24L01 initialization FAILED!");
    while (1) {
      Serial.println("Check wiring!");
      delay(1000);
    }
  }
  
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("NRF24 initialized and listening...");
}

void loop() {
  if (radio.available()) {
    char receivedData[32] = "";
    radio.read(&receivedData, sizeof(receivedData));
    
    char cmd = receivedData[0];
    Serial.print("\n>>> COMMAND RECEIVED: '");
    Serial.print(cmd);
    Serial.println("'");
    
    switch(cmd) {
      case 'F':
        Serial.println("    ACTION: All Motors Forward");
        // Add your motor code here
        break;
      case 'S':
        Serial.println("    ACTION: All Motors STOP");
        // Add your motor code here
        break;
      case 'B':
        Serial.println("    ACTION: All Motors Backward");
        // Add your motor code here
        break;
      default:
        break;
    }
  }
}
