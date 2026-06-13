#include <NimBLEDevice.h>
#include <FastLED.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>

// Web command queue (set from handlers, consumed in loop)
static CarCommand webPendingCmd = STOP;
static bool webHasCmd = false;

// User-adjustable motor speed (0-255), set via web UI
static uint8_t userMotorSpeed = 255;

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
static volatile bool hasNewCommand = false;     // True when BLE callback sets a fresh command

// HW-870 IR Obstacle Sensor (analog output)
#define HW870_PIN 21     // D21 - analog input
static uint16_t hw870Raw = 0;
static bool obstacleDetectionEnabled = false;
static const uint16_t OBSTACLE_THRESHOLD = 2000;  // Adjust based on testing (0-4095)

// Line-following mode (toggled by double-press middle button)
static bool lineFollowMode = false;           // OFF by default
static uint16_t LINE_THRESHOLD = 1500;  // Sensor value above = on line (dark) - adjustable via THRESH command
static uint32_t lastMiddleReleaseMs = 0;      // For double-press detection
static bool middleDoublePending = false;       // Waiting for possible double-press

// Line-follow motor tuning constants (ESP32 PWM 0-255)
#define LINE_FORWARD_SPEED   120   // Both motors cruising speed
#define LINE_CORRECT_FAST    170   // Outer (faster) wheel during correction
#define LINE_CORRECT_SLOW     50   // Inner (slower) wheel during correction
#define LINE_BURST_MS         40   // Burst duration on direction change

static char lfLastDir = 'S';            // Track last correction direction for burst-then-cruise
static uint32_t lfBurstStart = 0;       // When the current burst began

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

// Connecting animation: each LED picks a random color from {Red, Yellow, Orange}
// Colors re-randomize every ~200ms to create a dynamic flicker effect
static const CRGB CONNECT_COLORS[] = {
  CRGB::Red,
  CRGB::Yellow,
  CRGB::Orange
};
static const uint8_t NUM_CONNECT_COLORS = 3;

static void showConnecting(uint8_t frame) {
  uint8_t breath = breathBrightness(frame, 4);

  // Re-randomize colors every 4 frames (~200ms at 50fps)
  static uint8_t ledColors[NUM_LEDS];
  static uint8_t lastColorFrame = 255;
  if (frame - lastColorFrame >= 4) {
    for (int i = 0; i < NUM_LEDS; i++) {
      ledColors[i] = random(NUM_CONNECT_COLORS);
    }
    lastColorFrame = frame;
  }

  for (int i = 0; i < NUM_LEDS; i++) {
    CRGB color = CONNECT_COLORS[ledColors[i]];
    leds[i] = color;
    leds[i].nscale8_video(breath);
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

// Line-follow mode LED pattern: white chasing ring when following line
static void showLineFollowMode(uint8_t frame) {
  bool onLine = (hw870Raw > LINE_THRESHOLD);
  uint8_t breath = breathBrightness(frame, 0);

  if (onLine) {
    // On line: bright white chasing pattern
    int head = frame % NUM_LEDS;
    for (int i = 0; i < NUM_LEDS; i++) {
      int delta = (i + NUM_LEDS - head) % NUM_LEDS;
      if (delta == 0) {
        leds[i] = CRGB::White;
      } else if (delta < 3) {
        leds[i] = CRGB::White;
        leds[i].nscale8_video(128);
      } else {
        leds[i] = CRGB::White;
        leds[i].nscale8_video(40);
      }
    }
  } else {
    // Searching for line: pulsing dim white
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CHSV(0, 0, breath / 3);  // Dim white pulsing
    }
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
  } else if (lineFollowMode) {
    showLineFollowMode(animFrame);
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
  // Obstacle detection auto-stop: override any movement command
  if (obstacleDetectionEnabled && cmd != STOP && isObstacleDetected()) {
    cmd = STOP;
    static uint32_t lastObstacleMsg = 0;
    if (millis() - lastObstacleMsg > 1000) {
      Serial.print("HW-870 OBSTACLE! Sensor: ");
      Serial.println(hw870Raw);
      lastObstacleMsg = millis();
    }
  }

  if (cmd != motorRampTarget) {
    motorRampTarget = cmd;
    motorRampStart = millis();
    motorPWM = 0;
  }

  // Ramp up over 200ms to user-set speed (not hardcoded 255)
  uint32_t elapsed = millis() - motorRampStart;
  uint8_t targetPWM = userMotorSpeed;
  if (elapsed < 200) {
    motorPWM = (uint8_t)((elapsed * targetPWM) / 200);
  } else {
    motorPWM = targetPWM;
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

// Raw motor control with independent left/right speeds.
// Used by line-follow mode for proportional steering (both motors forward,
// one faster than the other) instead of pivot-in-place turns.
void setMotorsRaw(int leftSpeed, bool leftRev, int rightSpeed, bool rightRev) {
  // Left motor (MOTOR1)
  if (leftRev) {
    digitalWrite(MOTOR1_FWD, LOW);
    analogWrite(MOTOR1_REV, leftSpeed);
  } else {
    analogWrite(MOTOR1_FWD, leftSpeed);
    digitalWrite(MOTOR1_REV, LOW);
  }
  // Right motor (MOTOR2)
  if (rightRev) {
    digitalWrite(MOTOR2_FWD, LOW);
    analogWrite(MOTOR2_REV, rightSpeed);
  } else {
    analogWrite(MOTOR2_FWD, rightSpeed);
    digitalWrite(MOTOR2_REV, LOW);
  }
}

void stopMotorsRaw() {
  digitalWrite(MOTOR1_FWD, LOW);
  digitalWrite(MOTOR1_REV, LOW);
  digitalWrite(MOTOR2_FWD, LOW);
  digitalWrite(MOTOR2_REV, LOW);
}

// ─── Web Server ──────────────────────────────────────────────────────────────

WebServer server(80);

static const char WEB_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1.0,user-scalable=no">
<title>TRACSHON</title>
<style>
:root{--bg:#0a0a1e;--card:rgba(255,255,255,.04);--border:rgba(255,255,255,.08);--text:#c8c8d0;--accent:#7c6ff7;--danger:#f7546e;--green:#4ade80;--warn:#facc15}
*{margin:0;padding:0;box-sizing:border-box}
body{background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,sans-serif;min-height:100vh;padding:12px;display:flex;flex-direction:column;align-items:center;gap:12px}
.header{text-align:center}
.header h1{font-size:2.2em;font-weight:800;background:linear-gradient(135deg,#7c6ff7,#c084fc,#f7546e);-webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text}
.header .ip{font-size:.7em;color:#666;margin-top:2px}
.card{background:var(--card);backdrop-filter:blur(24px);-webkit-backdrop-filter:blur(24px);border:1px solid var(--border);border-radius:18px;padding:16px;width:100%;max-width:400px}
.card-title{font-size:.7em;text-transform:uppercase;letter-spacing:3px;color:#666;margin-bottom:12px}
/* D-Pad */
.dpad{display:grid;grid-template:50px 50px 50px/50px 50px 50px;gap:6px;justify-content:center;margin:8px 0}
.dpad button{border:none;border-radius:14px;background:rgba(255,255,255,.06);color:#999;font-size:1.3em;cursor:pointer;transition:all .12s;touch-action:manipulation;display:flex;align-items:center;justify-content:center;outline:none;-webkit-tap-highlight-color:transparent}
.dpad button:active,.dpad button.pressed{background:var(--accent);color:#fff;transform:scale(.9);box-shadow:0 0 30px rgba(124,111,247,.5)}
.dpad .up{grid-area:1/2}
.dpad .left{grid-area:2/1}
.dpad .stop{grid-area:2/2;background:rgba(255,255,255,.12);font-size:1.8em;border-radius:50%}
.dpad .stop:active,.dpad button.stop.pressed{background:var(--danger);box-shadow:0 0 30px rgba(247,84,110,.5)}
.dpad .right{grid-area:2/3}
.dpad .down{grid-area:3/2}
/* Speed row */
.speed-row{display:flex;align-items:center;gap:12px}
.speed-row input[type=range]{flex:1;accent-color:var(--accent);height:6px}
.speed-row .val{font-weight:700;color:var(--accent);min-width:32px;text-align:center;font-size:.9em}
/* Toggle */
.toggle-row{display:flex;align-items:center;justify-content:space-between;padding:8px 0}
.toggle-row span{font-size:.9em}
.toggle{position:relative;width:52px;height:28px;flex-shrink:0}
.toggle input{opacity:0;width:0;height:0}
.toggle .slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:rgba(255,255,255,.1);border-radius:28px;transition:.3s}
.toggle .slider::before{content:'';position:absolute;height:22px;width:22px;left:3px;bottom:3px;background:#666;border-radius:50%;transition:.3s}
.toggle input:checked+.slider{background:var(--accent)}
.toggle input:checked+.slider::before{transform:translateX(24px);background:#fff}
/* Sensor gauge */
.gauge-wrap{text-align:center}
.gauge-val{font-size:3em;font-weight:800;background:linear-gradient(180deg,#fff,#7c6ff7);-webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text}
.gauge-bar{height:8px;border-radius:4px;background:rgba(255,255,255,.08);margin-top:6px;overflow:hidden}
.gauge-bar .fill{height:100%;border-radius:4px;transition:width .3s;background:linear-gradient(90deg,var(--green),var(--warn),var(--danger))}
.gauge-label{font-size:.65em;color:#555;margin-top:4px}
/* Status dots */
.status-row{display:flex;gap:16px;justify-content:center}
.status-dot{display:flex;align-items:center;gap:6px;font-size:.75em}
.status-dot .dot{width:9px;height:9px;border-radius:50%}
.dot.on{background:var(--green);box-shadow:0 0 8px var(--green)}
.dot.off{background:#444}
/* Buttons */
.btn-row{display:flex;gap:8px;flex-wrap:wrap}
.btn{padding:8px 16px;border:1px solid var(--border);border-radius:10px;background:rgba(255,255,255,.04);color:var(--text);font-size:.8em;cursor:pointer;transition:.15s;touch-action:manipulation}
.btn:active{background:var(--accent);border-color:var(--accent);color:#fff}
</style>
</head><body>
<div class="header">
<h1>TRACSHON</h1>
<div class="ip" id="ip"></div>
</div>

<div class="card">
<div class="card-title">Control</div>
<div class="dpad" id="dpad">
<button class="up" ontouchstart="cmd('forward')" onmousedown="cmd('forward')" ontouchend="clr()" onmouseup="clr()" ontouchcancel="clr()">▲</button>
<button class="left" ontouchstart="cmd('left')" onmousedown="cmd('left')" ontouchend="clr()" onmouseup="clr()" ontouchcancel="clr()">◀</button>
<button class="stop" ontouchstart="cmd('stop')" onmousedown="cmd('stop')" ontouchend="clr()" onmouseup="clr()">■</button>
<button class="right" ontouchstart="cmd('right')" onmousedown="cmd('right')" ontouchend="clr()" onmouseup="clr()" ontouchcancel="clr()">▶</button>
<button class="down" ontouchstart="cmd('backward')" onmousedown="cmd('backward')" ontouchend="clr()" onmouseup="clr()" ontouchcancel="clr()">▼</button>
</div>
<div class="speed-row" style="margin-top:10px">
<span style="font-size:.7em;color:#666">SPD</span>
<input type="range" id="speed" min="80" max="255" value="255" oninput="setSpeed(this.value)">
<span class="val" id="speedVal">255</span>
</div>
</div>

<div class="card">
<div class="card-title">Line Follow</div>
<div class="toggle-row">
<span>Line Follow Mode</span>
<label class="toggle"><input type="checkbox" id="lfToggle" onchange="toggleLF()"><span class="slider"></span></label>
</div>
<div class="speed-row" style="margin-top:8px">
<span style="font-size:.7em;color:#666">THR</span>
<input type="range" id="threshold" min="200" max="3800" value="1500" oninput="setThreshold(this.value)">
<span class="val" id="threshVal">1500</span>
</div>
</div>

<div class="card">
<div class="card-title">Obstacle Detection</div>
<div class="toggle-row">
<span>Auto-Stop on Obstacle</span>
<label class="toggle"><input type="checkbox" id="obsToggle" onchange="toggleObstacle()"><span class="slider"></span></label>
</div>
</div>

<div class="card">
<div class="card-title">Sensor</div>
<div class="gauge-wrap">
<div class="gauge-val" id="sensorVal">--</div>
<div class="gauge-bar"><div class="fill" id="sensorFill" style="width:0%"></div></div>
<div class="gauge-label">HW-870 IR · D21 · 0–4095</div>
</div>
<div class="status-row" style="margin-top:12px">
<div class="status-dot"><div class="dot off" id="bleDot"></div>BLE</div>
<div class="status-dot"><div class="dot off" id="wifiDot"></div>WiFi</div>
<div class="status-dot"><div class="dot off" id="lfDot"></div>Line</div>
</div>
</div>

<div class="card">
<div class="btn-row">
<button class="btn" onclick="refresh()">Refresh</button>
<button class="btn" onclick="cmd('ota')" style="border-color:var(--warn);color:var(--warn)">OTA Mode</button>
</div>
</div>

<script>
const API='';
function post(url){fetch(url,{method:'POST'}).catch(()=>{})}
function cmd(c){
  const btns=document.querySelectorAll('.dpad button');
  btns.forEach(b=>b.classList.remove('pressed'));
  if(c==='stop'){post('/api/stop');document.querySelector('.dpad .stop').classList.add('pressed')}
  else if(c==='forward'){post('/api/forward');document.querySelector('.dpad .up').classList.add('pressed')}
  else if(c==='backward'){post('/api/backward');document.querySelector('.dpad .down').classList.add('pressed')}
  else if(c==='left'){post('/api/left');document.querySelector('.dpad .left').classList.add('pressed')}
  else if(c==='right'){post('/api/right');document.querySelector('.dpad .right').classList.add('pressed')}
  else if(c==='ota'){post('/api/ota')}
}
function clr(){document.querySelectorAll('.dpad button').forEach(b=>b.classList.remove('pressed'))}
function setSpeed(v){document.getElementById('speedVal').textContent=v;post('/api/speed?value='+v)}
function setThreshold(v){document.getElementById('threshVal').textContent=v;post('/api/threshold?value='+v)}
function toggleLF(){post('/api/linefollow')}
function toggleObstacle(){post('/api/obstacle')}
async function refresh(){
  try{
    const r=await fetch('/api/status');const s=await r.json();
    document.getElementById('sensorVal').textContent=s.sensor;
    document.getElementById('sensorFill').style.width=(s.sensor/4095*100)+'%';
    document.getElementById('bleDot').className='dot '+(s.ble?'on':'off');
    document.getElementById('wifiDot').className='dot '+(s.wifi?'on':'off');
    document.getElementById('lfDot').className='dot '+(s.linefollow?'on':'off');
    document.getElementById('lfToggle').checked=s.linefollow;
    document.getElementById('obsToggle').checked=s.obstacle;
    if(!document.getElementById('threshold').matches(':active')){
      document.getElementById('threshold').value=s.threshold;
      document.getElementById('threshVal').textContent=s.threshold;
    }
    if(!document.getElementById('speed').matches(':active')){
      document.getElementById('speed').value=s.speed;
      document.getElementById('speedVal').textContent=s.speed;
    }
    document.getElementById('ip').textContent=s.ip;
  }catch(e){}
}
setInterval(refresh,500);
refresh();
</script>
</body></html>
)rawliteral";

// ─── Web API Handlers ────────────────────────────────────────────────────────

void handleRoot() { server.send(200, "text/html", WEB_PAGE); }

void handleStatus() {
  String json = "{";
  json += "\"sensor\":" + String(readHW870());
  json += ",\"threshold\":" + String(LINE_THRESHOLD);
  json += ",\"linefollow\":" + String(lineFollowMode ? "true" : "false");
  json += ",\"obstacle\":" + String(obstacleDetectionEnabled ? "true" : "false");
  json += ",\"ble\":" + String(isConnected ? "true" : "false");
  json += ",\"wifi\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false");
  json += ",\"cmd\":" + String((int)currentCommand);
  json += ",\"speed\":" + String(userMotorSpeed);
  json += ",\"ip\":\"" + WiFi.localIP().toString() + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleForward()  { webPendingCmd = FORWARD;  webHasCmd = true; server.send(200); }
void handleBackward() { webPendingCmd = BACKWARD; webHasCmd = true; server.send(200); }
void handleLeft()     { webPendingCmd = LEFT;     webHasCmd = true; server.send(200); }
void handleRight()    { webPendingCmd = RIGHT;    webHasCmd = true; server.send(200); }
void handleStop()     { webPendingCmd = STOP;     webHasCmd = true; server.send(200); }

void handleLineFollow() {
  lineFollowMode = !lineFollowMode;
  lfLastDir = 'S';
  Serial.print("Web: line follow "); Serial.println(lineFollowMode ? "ON" : "OFF");
  server.send(200);
}
void handleObstacle() {
  obstacleDetectionEnabled = !obstacleDetectionEnabled;
  Serial.print("Web: obstacle detection "); Serial.println(obstacleDetectionEnabled ? "ON" : "OFF");
  server.send(200);
}
void handleThreshold() {
  if (server.hasArg("value")) {
    int v = server.arg("value").toInt();
    if (v > 0 && v < 4096) { LINE_THRESHOLD = (uint16_t)v; }
  }
  server.send(200);
}
void handleSpeed() {
  if (server.hasArg("value")) {
    int v = server.arg("value").toInt();
    if (v >= 60 && v <= 255) { userMotorSpeed = (uint8_t)v; }
  }
  server.send(200);
}
void handleSensor() { server.send(200, "text/plain", String(readHW870())); }

void handleOtaWeb() {
  otaModeActive = true;
  otaModeStart = millis();
  Serial.println("Web: OTA mode activated");
  server.send(200, "text/plain", "OTA mode active for 2 minutes. IP: " + WiFi.localIP().toString());
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/api/status", handleStatus);
  server.on("/api/forward", HTTP_POST, handleForward);
  server.on("/api/backward", HTTP_POST, handleBackward);
  server.on("/api/left", HTTP_POST, handleLeft);
  server.on("/api/right", HTTP_POST, handleRight);
  server.on("/api/stop", HTTP_POST, handleStop);
  server.on("/api/linefollow", HTTP_POST, handleLineFollow);
  server.on("/api/obstacle", HTTP_POST, handleObstacle);
  server.on("/api/threshold", HTTP_POST, handleThreshold);
  server.on("/api/speed", HTTP_POST, handleSpeed);
  server.on("/api/sensor", handleSensor);
  server.on("/api/ota", HTTP_POST, handleOtaWeb);
  server.begin();
  Serial.print("Web UI: http://");
  Serial.println(WiFi.localIP());
}

// ─── BLE Notify Callback ─────────────────────────────────────────────────────

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
    bool noSwipe = (abs(lastX - firstX) < 300 && abs(lastY - firstY) < 300);

    // Standard command mapping from click group + swipe direction
    if (clickGroup == 7) {
      cmd = STOP;
    } else if (clickGroup == 4 || clickGroup == 5) {
      if (lastX < firstX) cmd = LEFT;
      else if (lastX > firstX) cmd = RIGHT;
      else cmd = STOP;
    } else if (clickGroup == 6) {
      if (lastY < firstY) cmd = FORWARD;
      else if (lastY > firstY) cmd = BACKWARD;
      else cmd = STOP;
    }

    // Double-press detection: any two non-swipe STOP presses within 2 seconds toggles line-follow
    if (cmd == STOP && noSwipe) {
      uint32_t now = millis();
      if (middleDoublePending && (now - lastMiddleReleaseMs < 2000)) {
        lineFollowMode = !lineFollowMode;
        middleDoublePending = false;
        Serial.print("Line follow mode: ");
        Serial.println(lineFollowMode ? "ENABLED" : "DISABLED");
        clickActive = false;
        clickGroup = 0;
        activeCount = 0;
        return;
      }
      middleDoublePending = true;
      lastMiddleReleaseMs = now;
    }

    pendingCommand = cmd;
    hasNewCommand = true;

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

// Read HW-870 IR obstacle sensor (analog value 0-4095)
// Higher value = closer obstacle (more IR reflected back)
uint16_t readHW870() {
  hw870Raw = analogRead(HW870_PIN);
  return hw870Raw;
}

// Check if obstacle is too close (returns true if blocked)
bool isObstacleDetected() {
  return (readHW870() > OBSTACLE_THRESHOLD);
}

// Line-following motor control — called directly from loop() when lineFollowMode is active.
// Uses HW-870 IR sensor on D21 with edge-tracking:
// HW-870: black absorbs IR → LOW reflection → HIGH analog value
//         white reflects IR → HIGH reflection → LOW analog value
// So: sensorVal > LINE_THRESHOLD means we're ON the black line.
//
// Single-sensor edge-following:
// The sensor straddles the line edge. When it sees dark (line), the car has
// drifted too far toward the line and corrects away. When it sees light (floor),
// the car has drifted away and corrects back toward the line.
// Both motors always run forward — steering is done by speed differential,
// not by stopping or reversing. Burst-then-cruise pattern from working Mega code.
//
// Swap the correction directions below if the sensor is mounted on the other side.
void lineFollowMotors() {
  uint16_t sensorVal = readHW870();
  bool onLine = (sensorVal > LINE_THRESHOLD);
  uint32_t now = millis();

  // Non-blocking direction-change state: 0=cruise, 1=brake
  static uint8_t lfPhase = 0;
  static uint32_t lfPhaseStart = 0;

  if (onLine) {
    // ON dark line → car is too far toward the line → correct AWAY (right motor faster = turn left)
    if (lfLastDir != 'L') {
      lfLastDir = 'L';
      lfPhase = 1;
      lfPhaseStart = now;
      stopMotorsRaw();
      setMotorsRaw(LINE_CORRECT_SLOW, false, LINE_FORWARD_SPEED, false);
      Serial.println("Line: edge L");
    } else {
      lfPhase = 0;
      if (now - lfBurstStart < LINE_BURST_MS) {
        setMotorsRaw(LINE_CORRECT_SLOW, false, LINE_FORWARD_SPEED, false);
      } else {
        setMotorsRaw(LINE_CORRECT_SLOW, false, LINE_CORRECT_FAST, false);
      }
    }
  } else {
    // OFF line → car drifted away → correct TOWARD line (left motor faster = turn right)
    if (lfLastDir != 'R') {
      lfLastDir = 'R';
      lfPhase = 1;
      lfPhaseStart = now;
      stopMotorsRaw();
      setMotorsRaw(LINE_FORWARD_SPEED, false, LINE_CORRECT_SLOW, false);
      Serial.println("Line: edge R");
    } else {
      lfPhase = 0;
      if (now - lfBurstStart < LINE_BURST_MS) {
        setMotorsRaw(LINE_FORWARD_SPEED, false, LINE_CORRECT_SLOW, false);
      } else {
        setMotorsRaw(LINE_CORRECT_FAST, false, LINE_CORRECT_SLOW, false);
      }
    }
  }
  // After brake phase, record burst start for next call
  if (lfPhase == 1) {
    lfBurstStart = now;
    lfPhase = 0;
  }
}

// Clear stale double-press pending state (call from loop)
void updateDoublePressTimeout() {
  if (middleDoublePending && (millis() - lastMiddleReleaseMs > 2000)) {
    middleDoublePending = false;
  }
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
    Serial.print("HW-870 Sensor (D21): ");
    Serial.print(readHW870());
    Serial.print(" / 4095  (Threshold: ");
    Serial.print(OBSTACLE_THRESHOLD);
    Serial.println(")");
    Serial.print("Line Follow Mode: ");
    Serial.print(lineFollowMode ? "ON" : "OFF");
    Serial.print("  (Line Threshold: ");
    Serial.print(LINE_THRESHOLD);
    Serial.println(")");
    Serial.print("Obstacle Detection: ");
    Serial.println(obstacleDetectionEnabled ? "ON" : "OFF");
    Serial.print("Obstacle: ");
    Serial.println(isObstacleDetected() ? "DETECTED!" : "Clear");
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
    Serial.println("OBSTACLE ON  - Enable obstacle auto-stop");
    Serial.println("OBSTACLE OFF - Disable obstacle auto-stop");
    Serial.println("LINEFOLLOW   - Toggle line-follow mode (or double-press middle ring button)");
    Serial.println("THRESH <val> - Set line threshold (current: see STATUS)");
    Serial.println("SENSOR  - Read HW-870 sensor once");
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
  else if (cmd == "OBSTACLE ON") {
    obstacleDetectionEnabled = true;
    Serial.print("Obstacle Detection ENABLED (Threshold: ");
    Serial.print(OBSTACLE_THRESHOLD);
    Serial.println(")");
  }
  else if (cmd == "OBSTACLE OFF") {
    obstacleDetectionEnabled = false;
    Serial.println("Obstacle Detection DISABLED");
  }
  else if (cmd == "LINEFOLLOW") {
    lineFollowMode = !lineFollowMode;
    Serial.print("Line follow mode: ");
    Serial.println(lineFollowMode ? "ENABLED" : "DISABLED");
  }
  else if (cmd.startsWith("THRESH ")) {
    int val = cmd.substring(7).toInt();
    if (val > 0 && val < 4096) {
      LINE_THRESHOLD = (uint16_t)val;
      Serial.print("Line threshold set to: ");
      Serial.println(LINE_THRESHOLD);
    } else {
      Serial.println("Invalid threshold. Use 1-4095 (e.g., THRESH 2000)");
    }
  }
  else if (cmd == "SENSOR") {
    uint16_t val = readHW870();
    Serial.print("HW-870 Sensor (D21): ");
    Serial.print(val);
    Serial.print(" / 4095  ->  ");
    if (val > OBSTACLE_THRESHOLD) {
      Serial.println("OBSTACLE DETECTED");
    } else if (val > OBSTACLE_THRESHOLD / 2) {
      Serial.println("Object nearby");
    } else {
      Serial.println("Clear");
    }
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
  Serial.begin(115200);
  // Give serial a moment without blocking — just a few yield()s
  for (int i = 0; i < 50; i++) { yield(); }
  
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
    yield();
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
    // Start web server (only if WiFi connected)
    setupWebServer();
  } else {
    Serial.println("WiFi connection failed - battery mode");
    WiFi.mode(WIFI_OFF);  // Disable WiFi on battery
  }

  // Initialize RGB LED
  FastLED.addLeds<WS2812, RGB_LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(100);
  leds[0] = COLOR_OFF;
  FastLED.show();
  
  // Initialize HW-870 IR sensor (analog input on D21)
  pinMode(HW870_PIN, INPUT);
  Serial.print("HW-870 sensor initialized on pin D");
  Serial.println(HW870_PIN);

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

  // Serve web clients (non-blocking — processes one request per call)
  server.handleClient();

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

  yield();  // Allow FreeRTOS task switching

  if (doConnect) {
    connectToServer();
    yield();
  }

  yield();

  // Clear stale double-press pending after 2s timeout
  updateDoublePressTimeout();

  // Update RGB LED status
  updateRGBLED();

  // Determine the drive command for this frame
  CarCommand driveCmd = currentCommand;

  // Check for fresh web command (same priority as BLE)
  bool webOverride = false;
  if (webHasCmd) {
    webHasCmd = false;
    driveCmd = webPendingCmd;
    webOverride = true;
    lfLastDir = 'S';  // Reset line-follow burst tracking
  }

  // Check for fresh BLE ring command (takes priority over everything)
  bool bleOverride = false;
  if (hasNewCommand) {
    hasNewCommand = false;
    driveCmd = pendingCommand;
    bleOverride = true;
    lfLastDir = 'S';  // Reset line-follow burst tracking so it restarts cleanly
  }

  // Apply motors: line-follow uses proportional steering directly;
  // everything else goes through setMotorControl()
  if (lineFollowMode && isConnected && !webOverride && !bleOverride) {
    // Sensor drives motors directly — both always forward, steering by speed diff
    lineFollowMotors();
    // Keep currentCommand synced for status display
    if (currentCommand != FORWARD) {
      currentCommand = FORWARD;
      lastCommand = FORWARD;
    }
  } else {
    // BLE command, web command, or normal remote-control mode
    if (driveCmd != currentCommand) {
      currentCommand = driveCmd;
      setMotorControl(currentCommand);
      lastCommand = driveCmd;
    } else if (driveCmd != lastCommand) {
      setMotorControl(currentCommand);
      lastCommand = driveCmd;
    } else if (driveCmd != STOP) {
      // Continue ramping up / maintaining sustained movement
      setMotorControl(currentCommand);
    }
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