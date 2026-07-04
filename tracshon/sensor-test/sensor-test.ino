// Sensor Test — HW-870 IR Obstacle Sensor on D21
// Reads analog pin 21 and reports darkness/brightness over Serial (115200 baud)
//
// HW-870 behavior:
//   Black / dark surface absorbs IR  → HIGH analog value (closer to 4095)
//   White / bright surface reflects IR → LOW analog value (closer to 0)
//
// Thresholds are rough — tune for your surface and ambient light.

#define SENSOR_PIN 14       // D21 — analog input
#define DARK_THRESHOLD 2000  // Above this = DARK (black line / close obstacle)
#define BRIGHT_THRESHOLD 800 // Below this = BRIGHT (white floor / open space)

static uint16_t sensorRaw = 0;
static uint32_t lastPrintMs = 0;
static unsigned long loopCount = 0;

void setup() {
  Serial.begin(115200);
  // Wait a moment for serial to settle
  for (int i = 0; i < 50; i++) { yield(); }

   //ESP32 ADC setup — without these, analogRead returns 0
  analogReadResolution(12);       // 12-bit → 0..4095
  analogSetAttenuation(ADC_11db); // Full voltage range 0..3.3 V

  pinMode(SENSOR_PIN, INPUT);

  // Quick sanity check
  uint16_t testRead = analogRead(SENSOR_PIN);
  Serial.println();
  Serial.println("=== SENSOR TEST — HW-870 IR on D21 ===");
  Serial.print("ADC res: 12-bit | Atten: 11dB | Pin: D");
  Serial.println(SENSOR_PIN);
  Serial.print("Dark threshold:  > "); Serial.println(DARK_THRESHOLD);
  Serial.print("Bright threshold: < "); Serial.println(BRIGHT_THRESHOLD);
  Serial.print("Initial reading: "); Serial.println(testRead);
  Serial.println("----------------------------------------");
}

void loop() {
  sensorRaw = analogRead(SENSOR_PIN);
  loopCount++;

  // Classify the reading
  const char* label;
  if (sensorRaw > DARK_THRESHOLD) {
    label = "DARK";
  } else if (sensorRaw < BRIGHT_THRESHOLD) {
    label = "BRIGHT";
  } else {
    label = "MID";
  }

  // Print every ~200 ms so it's readable (5 Hz)
  uint32_t now = millis();
  if (now - lastPrintMs >= 200) {
    lastPrintMs = now;

    Serial.print("[");
    Serial.print(loopCount);
    Serial.print("]  Raw: ");
    Serial.print(sensorRaw);
    Serial.print(" / 4095  ->  ");
    Serial.print(label);

    // Visual bar (one char per ~200 units)
    Serial.print("  |");
    int barLen = map(sensorRaw, 0, 4095, 0, 40);
    for (int i = 0; i < barLen; i++) {
      Serial.print("#");
    }
    Serial.println("|");
  }

  delay(10);  // ~100 Hz sampling, 5 Hz printing
}
