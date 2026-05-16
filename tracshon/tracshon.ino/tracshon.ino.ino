#include <NimBLEDevice.h>
#include <FastLED.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

bool doConnect = false;
bool isConnected = false;
uint32_t scanTimeMs = 5000;

// WiFi credentials for OTA
const char* ssid = "potato";
const char* password = "potato123";

// OTA mode control
static bool otaModeActive = false;
static uint32_t otaModeStart = 0;
static const uint32_t OTA_MODE_TIMEOUT = 120000;  // 2 minutes
static String serialBuffer = "";

// RGB LED Configuration
#define RGB_LED_PIN 14  // D14
#define NUM_LEDS 12
CRGB leds[NUM_LEDS];

enum CarCommand : uint8_t { STOP, FORWARD, BACKWARD, LEFT, RIGHT };
volatile CarCommand currentCommand = STOP;
volatile CarCommand pendingCommand = STOP;  // Command from BLE callback, applied in main loop

// Motor control pins
#define MOTOR1_FWD 16   // D16
#define MOTOR1_REV 17   // D17
#define MOTOR2_FWD 25   // D25
#define MOTOR2_REV 26   // D26
#define LED_PIN 2       // D2

static uint8_t motorPWM = 0;  // Current motor PWM level (0-255)
static uint32_t motorRampStart = 0;
static CarCommand motorRampTarget = STOP;

// Color definitions for RGB LED status
static const CRGB COLOR_OFF       = CRGB::Black;
static const CRGB COLOR_SCANNING  = CRGB::Purple;
static const CRGB COLOR_CONNECTING= CRGB::Blue;
static const CRGB COLOR_CONNECTED = CRGB::Green;
static const CRGB COLOR_FORWARD   = CRGB::Cyan;
static const CRGB COLOR_BACKWARD  = CRGB::Yellow;
static const CRGB COLOR_LEFT      = CRGB::Orange;
static const CRGB COLOR_RIGHT     = CRGB::Red;

static bool     clickActive = false;
static uint8_t  clickGroup  = 0;
static int16_t  firstX = 0, firstY = 0;
static int16_t  lastX  = 0, lastY  = 0;
static uint16_t activeCount = 0;

const char* targetAddrStr = "de:54:3b:0a:b7:95";

static NimBLEAddress ringAddr;
static bool haveRingAddr = false;

static void startScanNow();
static void scanCompleteCB(NimBLEScanResults results);

static uint32_t lastScanKickMs = 0;
static volatile uint32_t lastOnResultMs = 0;

static void resetBluetoothStack() {
  Serial.println("RESET BLE STACK...");

  NimBLEScan* s = NimBLEDevice::getScan();
  if (s) {
    s->stop();
    s->clearResults();
  }

  NimBLEDevice::deinit(true);

  NimBLEDevice::init("");
  NimBLEDevice::deleteAllBonds();

  lastScanKickMs = millis();
  lastOnResultMs = millis();

  startScanNow();
}

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pClient) override {
    isConnected = true;
  }
  void onDisconnect(NimBLEClient* pClient, int reason) override {
    isConnected = false;
    startScanNow();
  }
} clientCallbacks;

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* d) override {
    lastOnResultMs = millis();
     static uint32_t lastMarkMs = 0;
    uint32_t now = millis();
    if (now - lastMarkMs > 250) {
      Serial.println("***");
      lastMarkMs = now;
    }

    if (!d->isAdvertisingService(NimBLEUUID((uint16_t)0x1812))) return;

    std::string macStr = d->getAddress().toString();
    Serial.print("\nFound device MAC: ");
    Serial.println(macStr.c_str());

    if (strcasecmp(macStr.c_str(), targetAddrStr) != 0) return;

    Serial.println("Found MY device!");
    ringAddr = d->getAddress();
    haveRingAddr = true;

    NimBLEDevice::getScan()->stop();
    doConnect = true;
  }
} scanCallbacks;

static void startScanNow() {
  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->stop();
  pScan->clearResults();

  pScan->setScanCallbacks(&scanCallbacks, false);
  pScan->setInterval(100);
  pScan->setWindow(100);
  pScan->setActiveScan(false);
  Serial.println("Scan start");
  pScan->start(scanTimeMs, scanCompleteCB, false);
  lastScanKickMs = millis();
}

static void scanCompleteCB(NimBLEScanResults results) {
  if (!isConnected && !doConnect) {
    Serial.println("Scan ended -> restart");
    startScanNow();
  } else {
    Serial.println("Scan ended");
  }
}

static inline uint8_t breathBrightness(uint8_t frame, uint8_t offset = 0) {
  float phase = (frame + offset) * 0.18f;
  return 96 + (uint8_t)(96.0f * (0.5f + 0.5f * sin(phase)));
}

static void setAllLEDs(const CRGB &color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
  }
}

static void showScanning(uint8_t frame) {
  int head = frame % NUM_LEDS;
  for (int i = 0; i < NUM_LEDS; i++) {
    int delta = (i + NUM_LEDS - head) % NUM_LEDS;
    if (delta == 0) {
      leds[i] = COLOR_SCANNING;
    } else if (delta < 3) {
      leds[i] = COLOR_SCANNING;
      leds[i].nscale8_video(170);
    } else if (delta < 6) {
      leds[i] = COLOR_SCANNING;
      leds[i].nscale8_video(100);
    } else {
      leds[i] = COLOR_OFF;
    }
  }
}

static void showConnecting(uint8_t frame) {
  uint8_t breath = breathBrightness(frame, 4);
  uint8_t dim = breath / 3;
  uint8_t minBright = dim < 24 ? 24 : dim;
  for (int i = 0; i < NUM_LEDS; i++) {
    if ((i + frame / 2) % 4 == 0) {
      leds[i] = CHSV(160, 255, breath);
    } else {
      leds[i] = CHSV(160, 255, minBright);
    }
  }
}

static void showCommandStatus(CarCommand cmd, uint8_t frame) {
  CRGB base;
  const uint8_t *activeIndexes = nullptr;
  int activeCount = 0;

  static const uint8_t forwardIndexes[] = {11, 0, 1, 2, 3};
  static const uint8_t backwardIndexes[] = {5, 6, 7, 8, 9};
  static const uint8_t leftIndexes[] = {2, 3, 4, 5, 6};
  static const uint8_t rightIndexes[] = {8, 9, 10, 11, 0};

  uint8_t bright = breathBrightness(frame, 8);
  uint8_t altBright = bright > 40 ? bright - 40 : 64;

  switch (cmd) {
    case FORWARD:
      base = COLOR_FORWARD;
      activeIndexes = forwardIndexes;
      activeCount = 5;
      break;
    case BACKWARD:
      base = COLOR_BACKWARD;
      activeIndexes = backwardIndexes;
      activeCount = 5;
      break;
    case LEFT:
      base = COLOR_LEFT;
      activeIndexes = leftIndexes;
      activeCount = 5;
      break;
    case RIGHT:
      base = COLOR_RIGHT;
      activeIndexes = rightIndexes;
      activeCount = 5;
      break;
    case STOP:
    default:
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CHSV(96, 255, (i % 2 == 0) ? bright : altBright);
      }
      return;
  }

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = base;
    leds[i].nscale8_video(90);
  }
  for (int j = 0; j < activeCount; j++) {
    leds[activeIndexes[j]] = base;
  }
}

void updateRGBLED() {
  static uint32_t lastUpdate = 0;
  static uint8_t animFrame = 0;
  uint32_t now = millis();

  if (now - lastUpdate > 50) {
    lastUpdate = now;
    animFrame++;
  }

  if (!isConnected && !doConnect) {
    showScanning(animFrame);
  } else if (doConnect && !isConnected) {
    showConnecting(animFrame);
  } else if (isConnected) {
    showCommandStatus(currentCommand, animFrame);
  } else {
    setAllLEDs(COLOR_OFF);
  }

  FastLED.show();
}


static inline int16_t u16le_to_i16(uint8_t lo, uint8_t hi) {
  uint16_t u = (uint16_t)lo | ((uint16_t)hi << 8);
  return (int16_t)u;
}

void setMotorControl(CarCommand cmd) {
  if (cmd != motorRampTarget) {
    motorRampTarget = cmd;
    motorRampStart = millis();
    motorPWM = 0;
  }

  // Ramp up over 200ms
  uint32_t elapsed = millis() - motorRampStart;
  if (elapsed < 200) {
    motorPWM = (uint8_t)((elapsed * 255) / 200);
  } else {
    motorPWM = 255;  // Full power
  }

  switch(cmd) {
    case FORWARD:
      analogWrite(MOTOR1_FWD, motorPWM);
      analogWrite(MOTOR1_REV, 0);
      analogWrite(MOTOR2_FWD, motorPWM);
      analogWrite(MOTOR2_REV, 0);
      if (elapsed == 0) Serial.println("MOTOR: Forward");
      break;
    
    case BACKWARD:
      analogWrite(MOTOR1_FWD, 0);
      analogWrite(MOTOR1_REV, motorPWM);
      analogWrite(MOTOR2_FWD, 0);
      analogWrite(MOTOR2_REV, motorPWM);
      if (elapsed == 0) Serial.println("MOTOR: Backward");
      break;
    
    case LEFT:
      analogWrite(MOTOR1_FWD, motorPWM);
      analogWrite(MOTOR1_REV, 0);
      analogWrite(MOTOR2_FWD, 0);
      analogWrite(MOTOR2_REV, motorPWM);
      if (elapsed == 0) Serial.println("MOTOR: Left Turn");
      break;
    
    case RIGHT:
      analogWrite(MOTOR1_FWD, 0);
      analogWrite(MOTOR1_REV, motorPWM);
      analogWrite(MOTOR2_FWD, motorPWM);
      analogWrite(MOTOR2_REV, 0);
      if (elapsed == 0) Serial.println("MOTOR: Right Turn");
      break;
    
    case STOP:
      analogWrite(MOTOR1_FWD, 0);
      analogWrite(MOTOR1_REV, 0);
      analogWrite(MOTOR2_FWD, 0);
      analogWrite(MOTOR2_REV, 0);
      if (elapsed == 0) Serial.println("MOTOR: Stop");
      break;
  }
}

void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic,
              uint8_t* pData, size_t length, bool isNotify) {
  if (length < 8) return;

  bool active  = (pData[6] != 0);
  bool release = (pData[6] == 0 && pData[7] == 0);

  if (active) {
    int16_t x = u16le_to_i16(pData[2], pData[3]);
    int16_t y = u16le_to_i16(pData[4], pData[5]);

    if (!clickActive) {
      clickActive = true;
      clickGroup = pData[1];
      firstX = lastX = x;
      firstY = lastY = y;
      activeCount = 1;
    } else {
      lastX = x;
      lastY = y;
      activeCount++;
    }
    return;
  }

  if (release) {
    if (!clickActive) return;

    CarCommand cmd = STOP;
    
    if (clickGroup == 6 && activeCount == 1 && firstX == 1200 && firstY == 1012) {
      cmd = STOP;
    } else if (clickGroup == 7) {
      cmd = STOP;
    } else if (clickGroup == 4 || clickGroup == 5) {
      if (lastX < firstX) cmd = LEFT;
      else if (lastX > firstX) cmd = RIGHT;
      else cmd = STOP;
    } else if (clickGroup == 6) {
      if (lastY < firstY) cmd = FORWARD;
      else if (lastY > firstY) cmd = BACKWARD;
      else cmd = STOP;
    } else {
      cmd = STOP;
    }

    // Set pending command - don't call motor functions from callback!
    pendingCommand = cmd;

    Serial.print("CLICK: ");
    Serial.println((int)cmd);

    clickActive = false;
    clickGroup = 0;
    activeCount = 0;
    return;
  }
}

void connectToServer() {
  Serial.println("Start connectToServer");
  doConnect = false;

  if (!haveRingAddr) {
    Serial.println("No ringAddr -> rescan");
    startScanNow();
    return;
  }

  NimBLEClient* pClient = NimBLEDevice::createClient();
  pClient->setClientCallbacks(&clientCallbacks, false);

  // connect по адресу
  if (!pClient->connect(ringAddr)) {
    NimBLEDevice::deleteClient(pClient);
    Serial.println("Client is not ready");
    resetBluetoothStack();
    return;
  }

  Serial.println("Client is ready");

  NimBLERemoteService* pSvc = pClient->getService(NimBLEUUID((uint16_t)0x1812));
  if (!pSvc) {
    Serial.println("HID service not found");
    pClient->disconnect();
    resetBluetoothStack();
    return;
  }

  Serial.println("Get Characteristics");
  auto chars = pSvc->getCharacteristics(true);
  int subCount = 0;

  for (auto chr : chars) {
    if (chr->getUUID().equals(NimBLEUUID((uint16_t)0x2A4D)) && chr->canNotify()) {
      if (chr->subscribe(true, notifyCB)) {
        Serial.print("Found button with UUID: 0x");
        Serial.println(chr->getHandle(), HEX);
        ++subCount;
      } else {
        Serial.print("Failed to subscribe to Handle: 0x");
        Serial.println(chr->getHandle(), HEX);
      }
    }
  }

  if (subCount == 0) {
    Serial.println("No notifiable 0x2A4D characteristic found!");
    pClient->disconnect();
    resetBluetoothStack();
    return;
  }

  isConnected = true;
}

void handleSerialCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  if (cmd == "OTA") {
    Serial.println("\n=== OTA MODE ACTIVATED ===");
    Serial.print("WiFi Status: ");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("CONNECTED - IP: ");
      Serial.println(WiFi.localIP());
      Serial.println("Listening for OTA updates for 2 minutes...");
      Serial.println("Use Arduino IDE: Tools -> Port -> Network Ports");
      Serial.println("or run: python -m esptool --chip esp32 --port <IP_ADDRESS> write_flash ...");
      otaModeActive = true;
      otaModeStart = millis();
    } else {
      Serial.println("NOT CONNECTED - Cannot enter OTA mode");
    }
  }
  else if (cmd == "STATUS") {
    Serial.println("\n=== SYSTEM STATUS ===");
    Serial.print("WiFi: ");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("CONNECTED - IP: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("DISCONNECTED");
    }
    Serial.print("BLE: ");
    Serial.println(isConnected ? "CONNECTED" : "DISCONNECTED");
    Serial.print("OTA Mode: ");
    Serial.println(otaModeActive ? "ACTIVE" : "INACTIVE");
    Serial.print("Current Command: ");
    Serial.println((int)currentCommand);
    Serial.print("Free Heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");
  }
  else if (cmd == "HELP") {
    Serial.println("\n=== AVAILABLE COMMANDS ===");
    Serial.println("OTA     - Enter OTA update mode (2 min timeout)");
    Serial.println("STATUS  - Show system status");
    Serial.println("WIFI_OFF - Disable WiFi (saves battery)");
    Serial.println("WIFI_ON  - Enable WiFi");
    Serial.println("HELP    - Show this help message");
  }
  else if (cmd == "WIFI_OFF") {
    Serial.println("Disabling WiFi...");
    WiFi.mode(WIFI_OFF);
    Serial.println("WiFi OFF - saves power on battery");
  }
  else if (cmd == "WIFI_ON") {
    Serial.println("Enabling WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(WIFI_PS_MIN_MODEM);
    WiFi.begin(ssid, password);
    Serial.println("Connecting to WiFi...");
  }
  else if (cmd.length() > 0) {
    Serial.println("Unknown command: " + cmd);
    Serial.println("Type HELP for available commands");
  }
}

void processSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        handleSerialCommand(serialBuffer);
        serialBuffer = "";
      }
    } else if (c >= 32 && c < 127) {  // Printable characters
      serialBuffer += c;
    }
  }
}

void setup(){
  Serial.begin(115200);  // Faster baud rate for better debugging
  delay(1000);
  
  Serial.println("\n\n=== TRACSHON STARTUP ===");
  Serial.println("Type HELP for commands");

  // Disable BLE library debug output
  esp_log_level_set("*", ESP_LOG_ERROR);
  
  // Initialize WiFi for OTA
  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(WIFI_PS_MIN_MODEM);  // Reduce power consumption
  WiFi.begin(ssid, password);
  
  uint32_t wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000) {
    delay(0);
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
    
    // Setup ArduinoOTA
    ArduinoOTA.setHostname("tracshon");
    ArduinoOTA.setPassword("123456");
    
    ArduinoOTA.onStart([=]() {
      Serial.println("\n[OTA] Update starting...");
    });
    ArduinoOTA.onEnd([=]() {
      Serial.println("\n[OTA] Update finished!");
    });
    ArduinoOTA.onProgress([=](unsigned int progress, unsigned int total) {
      Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([=](ota_error_t error) {
      Serial.printf("[OTA] Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
      else Serial.println("Unknown");
    });
    ArduinoOTA.begin();
  } else {
    Serial.println("WiFi connection failed - battery mode");
    WiFi.mode(WIFI_OFF);  // Disable WiFi on battery
  }
  
  // Initialize RGB LED
  FastLED.addLeds<WS2812, RGB_LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(100);
  leds[0] = COLOR_OFF;
  FastLED.show();
  
  // Initialize motor control pins
  pinMode(MOTOR1_FWD, OUTPUT);
  pinMode(MOTOR1_REV, OUTPUT);
  pinMode(MOTOR2_FWD, OUTPUT);
  pinMode(MOTOR2_REV, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Start with all motors off
  setMotorControl(STOP);
  
  NimBLEDevice::init("");
  NimBLEDevice::deleteAllBonds();
  lastScanKickMs = millis();
  lastOnResultMs = millis();
  startScanNow();
}
void loop(){
  static CarCommand lastCommand = STOP;
  static uint32_t lastLedToggle = 0;
  static bool ledState = false;
  
  // Process serial input for commands
  processSerialInput();
  
  // Handle OTA updates
  ArduinoOTA.handle();
  
  // Check OTA mode timeout
  if (otaModeActive) {
    uint32_t elapsed = millis() - otaModeStart;
    if (elapsed > OTA_MODE_TIMEOUT) {
      Serial.println("\n[OTA] Timeout - exiting OTA mode");
      otaModeActive = false;
    }
  }
  
  delay(0);  // Allow task switching
  
  if (doConnect) {
    connectToServer();
    delay(0);  // Allow task switching after connection attempt
  }
  
  delay(0);  // Allow task switching
  
  // Update RGB LED status
  updateRGBLED();
  
  // Apply pending command from BLE callback (safe to call motor functions here)
  if (pendingCommand != currentCommand) {
    currentCommand = pendingCommand;
    setMotorControl(currentCommand);
  } else if (currentCommand != lastCommand) {
    setMotorControl(currentCommand);
    lastCommand = currentCommand;
  } else if (currentCommand != STOP) {
    // Continue ramping up if still accelerating
    setMotorControl(currentCommand);
  }
  
  // LED flashing when not connected
  if (!isConnected) {
    uint32_t now = millis();
    if (now - lastLedToggle > 250) {  // Flash every 250ms
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      lastLedToggle = now;
    }
  } else {
    digitalWrite(LED_PIN, LOW);  // Turn off LED when connected
  }
  
  uint32_t now = millis();
  if (!isConnected && !doConnect && (now - lastScanKickMs > 2000)) {
    lastScanKickMs = now;

    if (now - (uint32_t)lastOnResultMs > 6000) {
      Serial.println("WD: scan stuck -> reset BLE");
      resetBluetoothStack();
    } else {
      if (!NimBLEDevice::getScan()->isScanning()) {
        Serial.println("WD: scan not running -> restart");
        startScanNow();
      }
    }
  }
}