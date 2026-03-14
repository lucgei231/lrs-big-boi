// Ultrasonic sensor (HC-SR04) - using analog read on echo pin
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

// WiFi credentials (user requested)
const char* WIFI_SSID = "potato";
const char* WIFI_PASS = "potato123";

WebServer server(80);

const int TRIG_PIN = 21;  // Trigger pin
const int ECHO_PIN = 22;  // Echo pin (will read analog)

// TCRT5000 IR Sensor 2
const int SENSOR2_A0 = 2;   // Analog output
const int SENSOR2_D0 = 16;  // Digital output

// Motor pin assignments (from user comments)
// m1: d32, d33
const int M1_PIN1 = 32;
const int M1_PIN2 = 33;

// m2: d25, d26
const int M2_PIN1 = 25;
const int M2_PIN2 = 26;

// m3: d19, d17
const int M3_PIN1 = 19;
const int M3_PIN2 = 23;

// m4: d18, d23
const int M4_PIN1 = 18;
const int M4_PIN2 = 17;

// m5: d27, d14
const int M5_PIN1 = 14;
const int M5_PIN2 = 27;

// m6: d12, d13
const int M6_PIN1 = 12;
const int M6_PIN2 = 13;

// forward declarations for motor control functions
void setMotor(int pin1, int pin2, bool forward, int speed);
void stopAllMotors();
void moveForward(int speed, unsigned long runMs);
void moveBackward(int speed, unsigned long runMs);
void turnLeft(int speed, unsigned long runMs);
void turnRight(int speed, unsigned long runMs);
void runMotor(int pin1, int pin2, int motorNum, bool forward, int speed, unsigned long runMs);

// helper sets all motors to same direction/speed
void setAllMotors(bool forward, int speed) {
  setMotor(M1_PIN1, M1_PIN2, forward, speed);
  setMotor(M2_PIN1, M2_PIN2, forward, speed);
  setMotor(M3_PIN1, M3_PIN2, forward, speed);
  setMotor(M4_PIN1, M4_PIN2, forward, speed);
  setMotor(M5_PIN1, M5_PIN2, forward, speed);
  setMotor(M6_PIN1, M6_PIN2, forward, speed);
}

// HTML for web interface
String htmlPage() {
  return R"rawliteral(
<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Lucas Control</title><style>
body { font-family: Arial, sans-serif; text-align: center; background-color: #f0f0f0; }
h3 { color: #333; }
button { width: 140px; height: 50px; margin: 6px; font-size: 16px; background-color: #4CAF50; color: white; border: none; border-radius: 5px; cursor: pointer; }
button:hover { background-color: #45a049; }
.control-section { margin: 20px; padding: 10px; border: 1px solid #ccc; border-radius: 10px; background-color: white; }
</style></head><body>
<h3>Lucas Motor Control</h3>
<div class="control-section">
<h4>All Motors</h4>
<div><button onclick=location.href='/all/forward'>All Forward</button><button onclick=location.href='/all/backward'>All Back</button><button onclick=location.href='/all/stop'>All Stop</button></div>
</div>
<div class="control-section">
<h4>Directional Control</h4>
<div><button onclick=location.href='/forward'>Forward</button><button onclick=location.href='/backward'>Backward</button><button onclick=location.href='/left'>Left</button><button onclick=location.href='/right'>Right</button><button onclick=location.href='/stop'>Stop</button></div>
</div>
<div class="control-section">
<h4>Individual Motors</h4>
<div> M1 <button onclick=location.href='/m1/forward'>F</button> <button onclick=location.href='/m1/backward'>B</button></div>
<div> M2 <button onclick=location.href='/m2/forward'>F</button> <button onclick=location.href='/m2/backward'>B</button></div>
<div> M3 <button onclick=location.href='/m3/forward'>F</button> <button onclick=location.href='/m3/backward'>B</button></div>
<div> M4 <button onclick=location.href='/m4/forward'>F</button> <button onclick=location.href='/m4/backward'>B</button></div>
<div> M5 <button onclick=location.href='/m5/forward'>F</button> <button onclick=location.href='/m5/backward'>B</button></div>
<div> M6 <button onclick=location.href='/m6/forward'>F</button> <button onclick=location.href='/m6/backward'>B</button></div>
</div>
<div class="control-section">
<h4>All Motors</h4>
<div><button onclick=location.href='/all/forward'>All Forward</button><button onclick=location.href='/all/backward'>All Back</button><button onclick=location.href='/all/stop'>All Stop</button></div>
</div>
<div class="control-section">
<h4>Directional Control</h4>
<div><button onclick=location.href='/forward'>Forward</button><button onclick=location.href='/backward'>Backward</button><button onclick=location.href='/left'>Left</button><button onclick=location.href='/right'>Right</button><button onclick=location.href='/stop'>Stop</button></div>
</div>
<div class="control-section">
<h4>Individual Motors</h4>
<div> M1 <button onclick=location.href='/m1/forward'>F</button> <button onclick=location.href='/m1/backward'>B</button></div>
<div> M2 <button onclick=location.href='/m2/forward'>F</button> <button onclick=location.href='/m2/backward'>B</button></div>
<div> M3 <button onclick=location.href='/m3/forward'>F</button> <button onclick=location.href='/m3/backward'>B</button></div>
<div> M4 <button onclick=location.href='/m4/forward'>F</button> <button onclick=location.href='/m4/backward'>B</button></div>
<div> M5 <button onclick=location.href='/m5/forward'>F</button> <button onclick=location.href='/m5/backward'>B</button></div>
<div> M6 <button onclick=location.href='/m6/forward'>F</button> <button onclick=location.href='/m6/backward'>B</button></div>
</div>
<p>mDNS: <b>lucas.local</b></p>
</body></html>
)rawliteral";
}

// M5 control variables
unsigned long m5LastSwitchTime = 0;
const unsigned long M5_INTERVAL = 3000; // 3 seconds
bool m5Forward = true; // Track direction for all motors

// Non-blocking motor timing variables
struct MotorState {
  bool active = false;
  unsigned long startTime = 0;
  unsigned long duration = 0;
  int pin1 = 0, pin2 = 0;
  bool forward = false;
  int speed = 0;
};
MotorState currentMotor = {false, 0, 0, 0, 0, false, 0};

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

  // Connect to WiFi
  Serial.print("Connecting to WiFi: "); Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
    if (millis() - start > 20000) break; // timeout 20s
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect failed or timed out");
  }

  // Start mDNS
  if (MDNS.begin("lucas")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS responder started as lucas.local");
  } else {
    Serial.println("Failed to start mDNS");
  }

  // Web server endpoints
  server.on("/", [](){ server.send(200, "text/html", htmlPage()); });
  server.on("/all/forward", [](){ setAllMotors(true, 200); server.sendHeader("Location","/"); server.send(302,"text/plain",""); });
  server.on("/all/backward", [](){ setAllMotors(false, 200); server.sendHeader("Location","/"); server.send(302,"text/plain",""); });
  server.on("/all/stop", [](){ stopAllMotors(); server.sendHeader("Location","/"); server.send(302,"text/plain",""); });

  server.on("/forward", [](){ scheduleMotorSequence(M4_PIN1, M4_PIN2, M5_PIN1, M5_PIN2, M6_PIN1, M6_PIN2, M3_PIN1, M3_PIN2, true, 200, 2000); server.sendHeader("Location","/"); server.send(302,"text/plain",""); });
  server.on("/backward", [](){ scheduleMotorSequence(M4_PIN1, M4_PIN2, M5_PIN1, M5_PIN2, M6_PIN1, M6_PIN2, M3_PIN1, M3_PIN2, false, 200, 2000); server.sendHeader("Location","/"); server.send(302,"text/plain",""); });
  server.on("/left", [](){ turnLeftNonBlocking(200, 2000); server.sendHeader("Location","/"); server.send(302,"text/plain",""); });
  server.on("/right", [](){ turnRightNonBlocking(200, 2000); server.sendHeader("Location","/"); server.send(302,"text/plain",""); });
  server.on("/stop", [](){ stopAllMotors(); server.sendHeader("Location","/"); server.send(302,"text/plain",""); });

  // Individual motor endpoints
  server.onNotFound([](){
    String uri = server.uri();
    // support /mX/forward and /mX/backward
    for (int i=1;i<=6;i++){
      String fwd = "/m" + String(i) + "/forward";
      String bwd = "/m" + String(i) + "/backward";
      if (uri == fwd) {
        // map motor i to pins
        if (i==1) scheduleMotor(M1_PIN1,M1_PIN2,i,true,200,2000);
        if (i==2) scheduleMotor(M2_PIN1,M2_PIN2,i,true,200,2000);
        if (i==3) scheduleMotor(M3_PIN1,M3_PIN2,i,true,200,2000);
        if (i==4) scheduleMotor(M4_PIN1,M4_PIN2,i,true,200,2000);
        if (i==5) scheduleMotor(M5_PIN1,M5_PIN2,i,true,200,2000);
        if (i==6) scheduleMotor(M6_PIN1,M6_PIN2,i,true,200,2000);
        server.sendHeader("Location","/"); server.send(302,"text/plain",""); return;
      }
      if (uri == bwd) {
        if (i==1) scheduleMotor(M1_PIN1,M1_PIN2,i,false,200,2000);
        if (i==2) scheduleMotor(M2_PIN1,M2_PIN2,i,false,200,2000);
        if (i==3) scheduleMotor(M3_PIN1,M3_PIN2,i,false,200,2000);
        if (i==4) scheduleMotor(M4_PIN1,M4_PIN2,i,false,200,2000);
        if (i==5) scheduleMotor(M5_PIN1,M5_PIN2,i,false,200,2000);
        if (i==6) scheduleMotor(M6_PIN1,M6_PIN2,i,false,200,2000);
        server.sendHeader("Location","/"); server.send(302,"text/plain",""); return;
      }
    }
    server.send(404, "text/plain", "Not found");
  });

  server.begin();
  Serial.println("HTTP server started");
}

// Function to read ultrasonic sensor via analog input
int readUltrasonicAnalog() {
    digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout after 30ms (~5m max)
  
  // Calculate distance: speed of sound is ~343 m/s (0.0343 cm/µs)
  // distance = (duration / 2) * 0.0343 cm
  float distance = (duration / 2.0) * 0.0343;

  return distance;

}

// Function to drive all 6 motors with alternating direction every 3 seconds
void driveAllMotors(int speed) {
  unsigned long currentTime = millis();
  
  // Check if 3 seconds have passed
  if (currentTime - m5LastSwitchTime >= M5_INTERVAL) {
    m5LastSwitchTime = currentTime;
    m5Forward = !m5Forward; // Toggle direction for all motors
  }
  
  // Fixed speed for all motors
  int motorSpeed = 200;
  
  if (m5Forward) {
    // All motors forward: PIN1 HIGH, PIN2 LOW
    analogWrite(M1_PIN1, motorSpeed); digitalWrite(M1_PIN2, LOW);
    analogWrite(M2_PIN1, motorSpeed); digitalWrite(M2_PIN2, LOW);
    analogWrite(M3_PIN1, motorSpeed); digitalWrite(M3_PIN2, LOW);
    analogWrite(M4_PIN1, motorSpeed); digitalWrite(M4_PIN2, LOW);
    analogWrite(M5_PIN1, motorSpeed); digitalWrite(M5_PIN2, LOW);
    analogWrite(M6_PIN1, motorSpeed); digitalWrite(M6_PIN2, LOW);
  } else {
    // All motors backward: PIN1 LOW, PIN2 HIGH
    digitalWrite(M1_PIN1, LOW); analogWrite(M1_PIN2, motorSpeed);
    digitalWrite(M2_PIN1, LOW); analogWrite(M2_PIN2, motorSpeed);
    digitalWrite(M3_PIN1, LOW); analogWrite(M3_PIN2, motorSpeed);
    digitalWrite(M4_PIN1, LOW); analogWrite(M4_PIN2, motorSpeed);
    digitalWrite(M5_PIN1, LOW); analogWrite(M5_PIN2, motorSpeed);
    digitalWrite(M6_PIN1, LOW); analogWrite(M6_PIN2, motorSpeed);
  }
  delay(300);
}

// Function to alternate M5 direction every 3 seconds
void alternateM5() {
  // Direction alternation is now handled in driveAllMotors()
}

// Set motor direction without delay (for simultaneous control)
void setMotor(int pin1, int pin2, bool forward, int speed) {
  if (forward) {
    digitalWrite(pin1, HIGH); digitalWrite(pin2, LOW);
  } else {
    digitalWrite(pin1, LOW); digitalWrite(pin2, HIGH);
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

// Non-blocking motor scheduling
void scheduleMotor(int pin1, int pin2, int motorNum, bool forward, int speed, unsigned long runMs) {
  Serial.print("Scheduled Motor "); Serial.print(motorNum); Serial.print(" -> ");
  Serial.println(forward ? "Forward" : "Backward");
  // Start motor immediately
  if (forward) {
    analogWrite(pin1, speed); digitalWrite(pin2, LOW);
  } else {
    digitalWrite(pin1, LOW); analogWrite(pin2, speed);
  }
  // Schedule stop via millis
  currentMotor = {true, millis(), runMs, pin1, pin2, forward, speed};
}

void scheduleMotorSequence(int p1a, int p2a, int p1b, int p2b, int p1c, int p2c, int p1d, int p2d, bool fwd, int spd, unsigned long ms) {
  Serial.println(fwd ? "Scheduled: Forward" : "Scheduled: Backward");
  analogWrite(p1a, spd); digitalWrite(p2a, LOW);
  analogWrite(p1b, spd); digitalWrite(p2b, LOW);
  analogWrite(p1c, spd); digitalWrite(p2c, LOW);
  analogWrite(p1d, spd); digitalWrite(p2d, LOW);
  currentMotor = {true, millis(), ms, 0, 0, fwd, spd};
}

void turnLeftNonBlocking(int speed, unsigned long runMs) {
  Serial.println("Scheduled: turnLeft");
  setMotor(M5_PIN1, M5_PIN2, false, speed);
  setMotor(M6_PIN1, M6_PIN2, true, speed);
  setMotor(M4_PIN1, M4_PIN2, false, speed);
  setMotor(M3_PIN1, M3_PIN2, true, speed);
  currentMotor = {true, millis(), runMs, 0, 0, false, speed};
}

void turnRightNonBlocking(int speed, unsigned long runMs) {
  Serial.println("Scheduled: turnRight");
  setMotor(M6_PIN1, M6_PIN2, true, speed);
  setMotor(M5_PIN1, M5_PIN2, false, speed);
  setMotor(M4_PIN1, M4_PIN2, true, speed);
  setMotor(M3_PIN1, M3_PIN2, false, speed);
  currentMotor = {true, millis(), runMs, 0, 0, true, speed};
}

// Keep old functions for compatibility
void moveForward(int speed, unsigned long runMs) {
  Serial.println("Action: moveForward (m4,m5,m6,m3) -> Forward");
  setMotor(M4_PIN1, M4_PIN2, true, speed);
  setMotor(M5_PIN1, M5_PIN2, true, speed);
  setMotor(M6_PIN1, M6_PIN2, true, speed);
  setMotor(M3_PIN1, M3_PIN2, true, speed);
}

void moveBackward(int speed, unsigned long runMs) {
  Serial.println("Action: moveBackward (m4,m5,m6,m3) -> Backward");
  setMotor(M4_PIN1, M4_PIN2, false, speed);
  setMotor(M5_PIN1, M5_PIN2, false, speed);
  setMotor(M6_PIN1, M6_PIN2, false, speed);
  setMotor(M3_PIN1, M3_PIN2, false, speed);
}

void turnLeft(int speed, unsigned long runMs) {
  Serial.println("Action: turnLeft -> M6 forward, M5 backward, M3 forward, M4 backward");
  setMotor(M5_PIN1, M5_PIN2, false, speed);
  setMotor(M6_PIN1, M6_PIN2, true, speed);
  setMotor(M4_PIN1, M4_PIN2, false, speed);
  setMotor(M3_PIN1, M3_PIN2, true, speed);
}

void turnRight(int speed, unsigned long runMs) {
  Serial.println("Action: turnRight -> opposite of turnLeft");
  setMotor(M6_PIN1, M6_PIN2, true, speed);
  setMotor(M5_PIN1, M5_PIN2, false, speed);
  setMotor(M4_PIN1, M4_PIN2, true, speed);
  setMotor(M3_PIN1, M3_PIN2, false, speed);
}
// // Motor 3 F
// Motor 4 E
// Motor 5 B
// // Motor 6 A
// Check if motor timing has elapsed and stop if needed
void updateMotorTiming() {
  if (currentMotor.active && (millis() - currentMotor.startTime >= currentMotor.duration)) {
    stopAllMotors();
    currentMotor.active = false;
    Serial.println("Motor runtime complete");
  }
}

void loop() {
  server.handleClient();
  updateMotorTiming();  // Non-blocking check for motor timeout
  delay(5);  // Reduced from 10ms for better responsiveness
}