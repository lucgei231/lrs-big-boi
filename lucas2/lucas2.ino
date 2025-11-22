/*
  ESP32: Connect as station (SSID "potato", password "potato123"),
  also run an access point "captive wifi" at 192.168.4.1 with a captive DNS,
  run mDNS as lucas.local, and serve a webpage to toggle LED on pin 2.
  Requires: WiFi.h, WebServer.h, DNSServer.h, ESPmDNS.h
*/
int leftorright;
int a;
int b = 1;
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WebSocketsServer.h>
// We'll use a small software PWM driver so the sketch can use analogWrite(pin,duty)
// and avoid using LEDC APIs directly.

const char* STA_SSID = "potato";
const char* STA_PASS = "potato123";
const char* AP_SSID = "SKETCHY WIFI POOPOO";
const char* AP_PASS = "mypotato123";
const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_GW(192, 168, 4, 1);
const IPAddress AP_SN(255, 255, 255, 0);

WebServer server(80);
DNSServer dnsServer;
// WebSocket server for controller page
WebSocketsServer webSocket = WebSocketsServer(81);

// Forward declaration for WebSocket event handler (defined later)
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

const byte DNS_PORT = 53;
const int LED_PIN = 2;
bool ledState = false;

// Motor pins (example)
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
const int P26 = 26;

/*
  NOTE - wiring/behavior difference (remembered):
  - The small test `code.ino` in the workspace toggles individual GPIOs
    in a pattern that makes the hardware appear to "turn left then right".
  - `lucas2.ino` is written to treat M1/M2 as the forward pair and
    M3/M4 as the backward/turn pair. I adjusted movement logic so:
      * Forward/back endpoints drive BOTH pins of the pair (M1+M2 or M3+M4)
      * Joystick (WebSocket) now sets pins directly (non-blocking)
      * turnLeft/turnRight pivot the device using the correct side pair
    This avoids the one-sided spin seen with the simplistic toggles in
    `code.ino`. If your H-bridge wiring inverts a motor, we can swap or
    invert the pin mapping later.
*/

// Motor runtime state
bool motorRunning = false;
unsigned long motorLastMs = 0;
unsigned long MOTOR_INTERVAL = 5000; // ms between pattern toggles (modifiable). Use long interval to mimic code.ino behavior
int motorPhase = 0; // 0 or 1

// PWM (LEDC) channels for ESP32
const int PWM_FREQ = 2000;
const int PWM_RESOLUTION = 8; // 8-bit (0-255)
const int CH_M1 = 0;
const int CH_M2 = 1;
const int CH_M3 = 2;
const int CH_M4 = 3;
int motorDuty = 255; // 0-255

// Jump (single-cycle) request state
bool jumpPending = false;
int jumpStep = 0; // 0=start,1=second
unsigned long jumpTimestamp = 0;

// Proxy debug info
String lastProxyUrl = "";
int lastProxyStatus = 0;

// Software PWM implementation so sketch uses analogWrite(pin,duty)
// without relying on LEDC APIs.
const int pwmPins[4] = { M1, M2, M3, M4 };
uint8_t pwmDuty[4] = {0,0,0,0}; // 0..255
bool pwmIsOn[4] = {false,false,false,false};
uint32_t pwmOffTime[4] = {0,0,0,0};
uint32_t pwmPeriodMicros = 0;
uint32_t pwmLastCycleStart = 0;

// analogWrite sets duty (0..255). Changes take effect on the next cycle
void analogWrite(int pin, int duty) {
  if (duty < 0) duty = 0; if (duty > 255) duty = 255;
  for (int i = 0; i < 4; ++i) {
    if (pwmPins[i] == pin) {
      pwmDuty[i] = (uint8_t)duty;
      // immediate shortcuts for 0 and 255 for responsiveness
      if (duty == 0) {
        digitalWrite(pin, LOW);
        pwmIsOn[i] = false;
        pwmOffTime[i] = 0;
      } else if (duty == 255) {
        digitalWrite(pin, HIGH);
        pwmIsOn[i] = true;
        pwmOffTime[i] = UINT32_MAX;
      }
      return;
    }
  }
}

// Initialize software PWM: store period and ensure pins are low
void pwmInit() {
  pwmPeriodMicros = 1000000UL / PWM_FREQ; // period in microseconds
  pwmLastCycleStart = micros();
  for (int i = 0; i < 4; ++i) {
    pwmDuty[i] = 0;
    pwmIsOn[i] = false;
    pwmOffTime[i] = 0;
    pinMode(pwmPins[i], OUTPUT);
    digitalWrite(pwmPins[i], LOW);
  }
}

// Helpers to apply the simple, direct-digital patterns from code.ino
void applyPatternA() {
  // pattern A roughly corresponds to: (12 LOW,13 HIGH,14 HIGH,27 LOW,17 HIGH,18 LOW,23 HIGH,19 LOW,25 HIGH,26 LOW,32 HIGH,33 LOW)
  // Map pins used in this sketch to the same high/low values
  digitalWrite(M3, LOW);   // 12
  digitalWrite(M4, HIGH);  // 13
  digitalWrite(M2, HIGH);  // 14
  digitalWrite(M1, LOW);   // 27
  digitalWrite(P17, HIGH);
  digitalWrite(P18, LOW);
  digitalWrite(P23, HIGH);
  digitalWrite(P19, LOW);
  digitalWrite(P25, HIGH);
  digitalWrite(P26, LOW);
  digitalWrite(P32, HIGH);
  digitalWrite(P33, LOW);
}

void applyPatternB() {
  // pattern B roughly corresponds to the inverse pattern used in code.ino second block
  digitalWrite(M3, HIGH);  // 12
  digitalWrite(M4, LOW);   // 13
  digitalWrite(M2, LOW);   // 14
  digitalWrite(M1, HIGH);  // 27
  digitalWrite(P17, LOW);
  digitalWrite(P18, HIGH);
  digitalWrite(P23, LOW);
  digitalWrite(P19, HIGH);
  digitalWrite(P25, LOW);
  digitalWrite(P26, HIGH);
  digitalWrite(P32, LOW);
  digitalWrite(P33, HIGH);
}

// Call frequently (from loop) to update PWM outputs. Non-blocking.
void pwmUpdate() {
  uint32_t now = micros();
  // Start a new cycle if we've passed the period
  if ((uint32_t)(now - pwmLastCycleStart) >= pwmPeriodMicros) {
    pwmLastCycleStart = now;
    for (int i = 0; i < 4; ++i) {
      uint8_t d = pwmDuty[i];
      int pin = pwmPins[i];
      if (d == 0) {
        digitalWrite(pin, LOW);
        pwmIsOn[i] = false;
        pwmOffTime[i] = 0;
      } else if (d >= 255) {
        digitalWrite(pin, HIGH);
        pwmIsOn[i] = true;
        pwmOffTime[i] = UINT32_MAX;
      } else {
        // Turn on, schedule off time within this period
        digitalWrite(pin, HIGH);
        pwmIsOn[i] = true;
        uint32_t onTime = ((uint32_t)d * pwmPeriodMicros) / 255UL;
        pwmOffTime[i] = pwmLastCycleStart + onTime;
      }
    }
  }

  // Turn off pins whose on-time has elapsed
  for (int i = 0; i < 4; ++i) {
    if (pwmIsOn[i] && pwmOffTime[i] != UINT32_MAX) {
      if ((uint32_t)(now - pwmLastCycleStart) >= (uint32_t)(pwmOffTime[i] - pwmLastCycleStart)) {
        digitalWrite(pwmPins[i], LOW);
        pwmIsOn[i] = false;
      }
    }
  }
}

// Wait for given milliseconds while continuing to service network, websockets and PWM
void waitWithService(unsigned long ms) {
  unsigned long end = millis() + ms;
  while ((long)(end - millis()) > 0) {
    // keep PWM running so analogWrite has visible output
    pwmUpdate();
    // keep webserver and DNS responsive
    server.handleClient();
    dnsServer.processNextRequest();
    // service WebSocket connections
    webSocket.loop();
    // small sleep to yield CPU
    delay(1);
  }
}


const char indexPage[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <title>ESP32 — Super Control Panel</title>
    <style>
      :root{--bg:#0f1720;--card:#111827;--muted:#9ca3af;--accent:#10b981;--danger:#ef4444}
      body{font-family:Inter,Arial,Helvetica,sans-serif;margin:8px;background:linear-gradient(180deg,#071021,#0b1220);color:#e6eef6}
      .wrap{max-width:1100px;margin:14px auto}
      header{display:flex;align-items:center;justify-content:space-between}
      h1{font-weight:700;margin:0 0 8px 0}
      .grid{display:grid;grid-template-columns:2fr 1fr;gap:12px}
      .card{background:var(--card);padding:12px;border-radius:10px;box-shadow:0 6px 18px rgba(2,6,23,0.6)}
      .controls{display:flex;flex-wrap:wrap;gap:8px}
      button{background:var(--accent);border:0;color:#042018;padding:10px 12px;border-radius:8px;cursor:pointer}
      button.ghost{background:transparent;border:1px solid #1f2937;color:var(--muted)}
      .muted{color:var(--muted);font-size:13px}
      label{display:block;font-size:13px;margin-bottom:6px;color:var(--muted)}
      input[type=range]{width:100%}
      .row{display:flex;gap:8px;align-items:center}
      .log{height:180px;overflow:auto;background:#061018;padding:8px;border-radius:6px;font-family:monospace;font-size:12px}
      .big{font-size:20px;font-weight:700}
      .small{font-size:12px;color:var(--muted)}
      footer{margin-top:12px;color:var(--muted);font-size:12px}
    </style>
  </head>
  <body>
    <div class="wrap">
      <header>
        <div>
          <h1>ESP32 — Super Control Panel</h1>
          <div class="small">AP: <span id="apip">...</span> • STA: <span id="staip">...</span> • mDNS: <code>lucas.local</code></div>
        </div>
  <div class="small">Quick actions: <button onclick="openFunctions()" class="ghost">Functions</button> <button onclick="openController()" class="ghost">Controller</button></div>
      </header>

      <div class="grid">
        <section class="card">
          <div style="display:flex;justify-content:space-between;align-items:center">
            <div>
              <div class="big" id="ledLabel">LED: --</div>
              <div class="small">Use the buttons below or API to control</div>
            </div>
            <div style="text-align:right">
              <div class="small">Motor: <span id="motorStateMain">stopped</span></div>
              <div class="small">Duty: <span id="motorDutyMain">0</span> • Interval: <span id="motorIntMain">0</span>ms</div>
            </div>
          </div>

          <hr style="margin:10px 0;border:none;border-top:1px solid #071827">

          <div class="controls">
            <button id="btnToggleMain">Toggle LED</button>
            <button id="btnMotorStartMain">Motor Start</button>
            <button id="btnMotorStopMain">Motor Stop</button>
            <button id="btnMotorJumpMain">Motor Jump</button>
            <button id="btnProxyDebugMain" class="ghost">Proxy Debug</button>
          </div>

          <div style="margin-top:12px;display:grid;grid-template-columns:1fr 1fr;gap:10px">
            <div>
              <label>Motor Speed</label>
              <input id="speedRange" type="range" min="0" max="255" value="200">
              <div class="row"><button id="btnSetSpeed">Set Speed</button><div class="small" id="speedVal">200</div></div>
            </div>
            <div>
              <label>Motor Interval (ms)</label>
              <input id="intervalInput" type="number" value="100" style="width:100%">
              <div class="row"><button id="btnSetInterval">Set Interval</button><div class="small" id="intervalVal">100</div></div>
            </div>
          </div>

          <hr style="margin:12px 0;border:none;border-top:1px solid #071827">

          <div>
            <label>Movement controls (Forward/Back/Left/Right/Stop)</label>
            <div style="display:flex;gap:8px;flex-wrap:wrap">
              Speed <input id="mv-speed" type="number" value="200" style="width:80px"> 
              Dur(s) <input id="mv-dur" type="number" value="2" style="width:80px">
              <button onclick="move('forward')">Forward</button>
              <button onclick="move('backward')">Backward</button>
              <button onclick="move('left')">Left</button>
              <button onclick="move('right')">Right</button>
              <button onclick="move('stop')" class="ghost">Stop</button>
            </div>
          </div>

          <hr style="margin:12px 0;border:none;border-top:1px solid #071827">

          <div>
            <label>Console / logs</label>
            <div id="log" class="log">Log output will appear here...</div>
          </div>
        </section>

        <aside class="card">
          <div style="display:flex;justify-content:space-between;align-items:center">
            <div class="small">Device Info</div>
            <div><button onclick="refreshAll()" class="ghost">Refresh</button></div>
          </div>
          <div style="margin-top:8px">
            <div><strong>STA IP:</strong> <span id="sta_ip">...</span></div>
            <div><strong>AP IP:</strong> <span id="ap_ip">192.168.4.1</span></div>
            <div><strong>Last Proxy:</strong> <pre id="lastProxy" style="white-space:pre-wrap;background:#061018;padding:6px;border-radius:6px;margin-top:6px;font-size:12px"></pre></div>
          </div>

          <hr style="margin:8px 0;border:none;border-top:1px solid #071827">

          <div>
            <label>Quick Presets</label>
            <div class="controls">
              <button onclick="preset('walk')">Walk</button>
              <button onclick="preset('dash')">Dash</button>
              <button onclick="preset('spin')">Spin</button>
            </div>
          </div>

          <hr style="margin:8px 0;border:none;border-top:1px solid #071827">

          <div>
            <label>Advanced</label>
            <div class="small">Proxy debug, last HTTP status and url</div>
            <div style="margin-top:8px"><button onclick="showProxyDebug()">Show Proxy Debug</button></div>
          </div>
        </aside>
      </div>

      <footer>Built-in control panel — no auth. Use carefully.</footer>
    </div>

    <script>
      const logEl = document.getElementById('log');
      function log(msg){ const t = '['+new Date().toLocaleTimeString()+'] '+msg+'\n'; logEl.textContent = t + logEl.textContent; }

      async function api(path, opts){ try{ const r = await fetch(path,opts); const txt = await r.text(); log(path+' -> '+r.status+' '+txt); return {ok:r.ok,status:r.status,text:txt}; }catch(e){ log('ERR '+path+' '+e); return {ok:false,error:e}; } }

      async function refreshAll(){
        const s = await api('/status');
        try{ const j = JSON.parse(s.text); document.getElementById('ledLabel').textContent = 'LED: '+(j.led? 'ON':'OFF'); }catch(e){}
        const m = await api('/motor/status'); try{ const mj = JSON.parse(m.text); document.getElementById('motorStateMain').textContent = mj.motorRunning? 'running':'stopped'; document.getElementById('motorDutyMain').textContent = mj.motorDuty; document.getElementById('motorIntMain').textContent = mj.motorInterval; }catch(e){}
        const pd = await api('/proxy-debug'); try{ const pj = JSON.parse(pd.text); document.getElementById('lastProxy').textContent = pj.lastUrl+' ('+pj.lastStatus+')'; }catch(e){}
        // update IPs
        document.getElementById('apip').textContent = '192.168.4.1';
        // try STA ip
        const sta = await fetch('/sta-ip').catch(()=>null);
        if (sta && sta.ok){ const t = await sta.text(); document.getElementById('staip').textContent = t; document.getElementById('sta_ip').textContent = t; } else { document.getElementById('staip').textContent = 'unknown'; }
      }

      document.getElementById('btnToggleMain').addEventListener('click', async ()=>{ await api('/toggle'); await refreshAll(); });
      document.getElementById('btnMotorStartMain').addEventListener('click', async ()=>{ await api('/motor/start'); await refreshAll(); });
      document.getElementById('btnMotorStopMain').addEventListener('click', async ()=>{ await api('/motor/stop'); await refreshAll(); });
      document.getElementById('btnMotorJumpMain').addEventListener('click', async ()=>{ await api('/motor/jump'); await refreshAll(); });
      document.getElementById('btnProxyDebugMain').addEventListener('click', showProxyDebug);

      const speedRange = document.getElementById('speedRange'); const speedVal = document.getElementById('speedVal');
      speedRange.addEventListener('input', ()=>{ speedVal.textContent = speedRange.value; });
      document.getElementById('btnSetSpeed').addEventListener('click', async ()=>{ await api('/motor/speed?d='+encodeURIComponent(speedRange.value)); await refreshAll(); });
      document.getElementById('btnSetInterval').addEventListener('click', async ()=>{ const v = document.getElementById('intervalInput').value; await api('/motor/interval?ms='+encodeURIComponent(v)); await refreshAll(); });

      async function move(which){
        if (which === 'stop'){ await api('/move/stop'); await refreshAll(); return; }
        const s = document.getElementById('mv-speed').value;
        const d = document.getElementById('mv-dur').value;
        if (which === 'forward') await api('/move/forward?speed='+encodeURIComponent(s)+'&dur='+encodeURIComponent(d));
        if (which === 'backward') await api('/move/backward?speed='+encodeURIComponent(s)+'&dur='+encodeURIComponent(d));
        if (which === 'left') await api('/move/left?dur='+encodeURIComponent(d));
        if (which === 'right') await api('/move/right?dur='+encodeURIComponent(d));
        await refreshAll();
      }

  function openFunctions(){ window.open('/functions','_blank'); }
  function openController(){ window.open('/controller','_blank'); }
      async function showProxyDebug(){ const r = await api('/proxy-debug'); try{ const j = JSON.parse(r.text); alert('Last: '+j.lastUrl+' status:'+j.lastStatus); }catch(e){ alert(r.text || 'no data'); } }

      function preset(which){ if (which==='walk'){ speedRange.value=120; speedVal.textContent=120; document.getElementById('mv-dur').value=2; } if (which==='dash'){ speedRange.value=255; speedVal.textContent=255; document.getElementById('mv-dur').value=1; } if (which==='spin'){ document.getElementById('mv-dur').value=3; move('left'); } }


      document.addEventListener('keydown', function(event) {

        if (event.code === 'Space' || event.key === ' ') {
          event.preventDefault();
          api('/move/stop');
          refreshAll();
        }
      });

      refreshAll(); setInterval(refreshAll,3000);
    </script>
  </body>
</html>
)rawliteral";

// Controller page (verbatim SVG + JS provided by user)
const char controllerPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8"><title>Robium Controller</title></head>
<body>
    <div class="svg-container">
        <svg
   width="136.90706mm"
   height="82.806686mm"
   viewBox="0 0 136.90706 82.806686"
   version="1.1"
   id="svg1"
   inkscape:version="1.4 (86a8ad7, 2024-10-11)"
   sodipodi:docname="Joystic.svg"
   xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape"
   xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"
   xmlns="http://www.w3.org/2000/svg"
   xmlns:svg="http://www.w3.org/2000/svg">
  <sodipodi:namedview
     id="namedview1"
     pagecolor="#ffffff"
     bordercolor="#000000"
     borderopacity="0.25"
     inkscape:showpageshadow="2"
     inkscape:pageopacity="0.0"
     inkscape:pagecheckerboard="0"
     inkscape:deskcolor="#d1d1d1"
     inkscape:document-units="mm"
     inkscape:zoom="1.4378367"
     inkscape:cx="387.73527"
     inkscape:cy="217.3404"
     inkscape:window-width="1920"
     inkscape:window-height="1009"
     inkscape:window-x="-8"
     inkscape:window-y="-8"
     inkscape:window-maximized="1"
     inkscape:current-layer="layer1"
     showguides="true" />
  <defs
     id="defs1">
    <meshgradient
       inkscape:collect="always"
       id="meshgradient14"
       gradientUnits="userSpaceOnUse"
       x="26.130112"
       y="73.973984">
      <meshrow
         id="meshrow14">
        <meshpatch
           id="meshpatch14">
          <stop
             path="c 45.6357,0  91.2714,0  136.907,0"
             style="stop-color:#d7d7d7;stop-opacity:1"
             id="stop15" />
          <stop
             path="c 0,27.6022  0,55.2045  0,82.8067"
             style="stop-color:#666666;stop-opacity:1"
             id="stop16" />
          <stop
             path="c -45.6357,0  -91.2714,0  -136.907,0"
             style="stop-color:#e7e7e7;stop-opacity:1"
             id="stop17" />
          <stop
             path="c 0,-27.6022  0,-55.2045  0,-82.8067"
             style="stop-color:#666666;stop-opacity:1"
             id="stop18" />
        </meshpatch>
      </meshrow>
    </meshgradient>
    <linearGradient id="rainbow-gradient" x1="0%" y1="0%" x2="100%" y2="0%">
        <stop offset="0%"   stop-color="#FF0000"/>
        <stop offset="16%"  stop-color="#FF7F00"/>
        <stop offset="33%"  stop-color="#FFFF00"/>
        <stop offset="50%"  stop-color="#00FF00"/>
        <stop offset="66%"  stop-color="#0000FF"/>
        <stop offset="83%"  stop-color="#4B0082"/>
        <stop offset="100%" stop-color="#8B00FF"/>
      </linearGradient>
  </defs>
  <g inkscape:label="Layer 1"
     inkscape:groupmode="layer"
     id="layer1"
     transform="translate(-26.130112,-73.973984)">
    <rect
       style="fill:url(#meshgradient14);fill-opacity:1;stroke-width:0.303295"
       id="rect1"
       width="136.90706"
       height="82.806686"
       x="26.130112"
       y="73.973984" />
    <rect style="fill:url(#rainbow-gradient);fill-opacity:1;stroke-width:0.272325;cursor:pointer;"
       id="rect36"
       width="125.11385"
       height="8.1891403"
       x="32.026714"
       y="82.04776" />
    <g id="g59" style="cursor: pointer;">
      <path
         style="fill:#e7e7e7;fill-opacity:1;stroke-width:0.3"
         d="m 60.38299,124.80323 -14.846639,14.84664 a 21.161711,21.161711 0 0 0 14.826485,6.13658 21.161711,21.161711 0 0 0 14.846122,-6.15725 z"
         id="path25" />
      <path
         style="fill:#e7e7e7;fill-opacity:1;stroke-width:0.3"
         d="m 60.362836,103.46294 a 21.161711,21.161711 0 0 0 -14.846639,6.15724 l 14.866793,14.8668 14.883329,-14.88333 a 21.161711,21.161711 0 0 0 -14.903483,-6.14071 z"
         id="path18" />
      <circle
         style="fill:#333333;fill-opacity:1;stroke-width:0.21238"
         id="circle18"
         cx="60.362579"
         cy="124.62481"
         r="14.981098" />
      <path
         id="circle19"
         style="fill:#333333;fill-opacity:1;stroke-width:0.295388"
         d="m 60.362574,103.78836 a 20.836416,20.836416 0 0 0 -20.83645,20.83645 20.836416,20.836416 0 0 0 20.83645,20.83645 20.836416,20.836416 0 0 0 20.83646,-20.83645 20.836416,20.836416 0 0 0 -20.83646,-20.83645 z m 0,0.22789 a 20.608709,20.608709 0 0 1 20.60908,20.60856 20.608709,20.608709 0 0 1 -20.60908,20.60856 20.608709,20.608709 0 0 1 -20.608557,-20.60856 20.608709,20.608709 0 0 1 20.608557,-20.60856 z" />
      <path
         id="path20"
         style="fill:#333333;fill-opacity:1;stroke-width:0.31691"
         d="m 60.362573,102.27024 a 22.354536,22.354536 0 0 0 -22.354572,22.35457 22.354536,22.354536 0 0 0 22.354572,22.35457 22.354536,22.354536 0 0 0 22.354583,-22.35457 22.354536,22.354536 0 0 0 -22.354583,-22.35457 z m 0,0.24449 A 22.110238,22.110238 0 0 1 82.47321,124.62481 22.110238,22.110238 0 0 1 60.362573,146.73489 22.110238,22.110238 0 0 1 38.252498,124.62481 22.110238,22.110238 0 0 1 60.362573,102.51473 Z" />
      <rect
         style="fill:#ffffff;fill-opacity:1;stroke-width:0.171085"
         id="rect20"
         width="0.15741065"
         height="29.960497"
         x="130.21555"
         y="31.906748"
         transform="rotate(44.365062)" />
      <rect
         style="fill:#ffffff;fill-opacity:1;stroke-width:0.171085"
         id="rect21"
         width="0.15741065"
         height="29.960497"
         x="43.704025"
         y="-146.35017"
         transform="rotate(135.72446)" />
      <circle
         style="fill:#ffffff;fill-opacity:1;stroke-width:0.3"
         id="path26"
         cx="60.362579"
         cy="124.62481"
         r="4.5083642" />
      <path
         id="path27"
         style="fill:#ffffff;fill-opacity:1;stroke-width:0.0739703"
         d="m 60.362577,119.40698 a 5.2178191,5.2178191 0 0 0 -5.217827,5.21783 5.2178191,5.2178191 0 0 0 5.217827,5.21782 5.2178191,5.2178191 0 0 0 5.21783,-5.21782 5.2178191,5.2178191 0 0 0 -5.21783,-5.21783 z m 0,0.057 a 5.160797,5.160797 0 0 1 5.16089,5.16077 5.160797,5.160797 0 0 1 -5.16089,5.16076 5.160797,5.160797 0 0 1 -5.160758,-5.16076 5.160797,5.160797 0 0 1 5.160758,-5.16077 z" />
    </g>
    <g id="g37"
       transform="translate(2.886857,3.5430299)">
      <text
         xml:space="preserve"
         style="font-size:3.52778px;line-height:1.3;font-family:Ubuntu;-inkscape-font-specification:Ubuntu;text-align:start;writing-mode:lr-tb;direction:ltr;text-anchor:start;fill:#f9f9f9;fill-opacity:1;stroke-width:0.3"
         x="48.027882"
         y="96.42379"
         id="text36"><tspan
           sodipodi:role="line"
           id="tspan36"
           style="font-size:3.52778px;stroke-width:0.3"
           x="48.027882"
           y="96.42379">Value:</tspan></text>
      <text
         xml:space="preserve"
         style="font-size:3.52778px;line-height:1.3;font-family:Ubuntu;-inkscape-font-specification:Ubuntu;text-align:start;writing-mode:lr-tb;direction:ltr;text-anchor:start;fill:#000000;fill-opacity:1;stroke-width:0.3"
         x="59.669559"
         y="96.42379"
         id="text37"><tspan
           sodipodi:role="line"
           id="tspan37"
           style="font-size:3.52778px;fill:#000000;stroke-width:0.3"
           x="59.669559"
           y="96.42379">100</tspan></text>
    </g>
    <g id="g60" style="cursor: pointer;">
      <path
         style="fill:#e7e7e7;fill-opacity:1;stroke-width:0.3"
         d="m 128.62629,124.6452 -14.84664,-14.84664 a 21.161711,21.161711 0 0 0 -6.13658,14.82648 21.161711,21.161711 0 0 0 6.15725,14.84612 z"
         id="path28" />
      <path
         style="fill:#e7e7e7;fill-opacity:1;stroke-width:0.3"
         d="m 149.96658,124.62504 a 21.161711,21.161711 0 0 0 -6.15724,-14.84664 l -14.8668,14.8668 14.88333,14.88332 a 21.161711,21.161711 0 0 0 6.14071,-14.90348 z"
         id="path29" />
      <circle
         style="fill:#333333;fill-opacity:1;stroke-width:0.21238"
         id="circle29"
         cx="124.62479"
         cy="-128.80472"
         r="14.981098"
         transform="rotate(90)" />
      <path
         id="path30"
         style="fill:#333333;fill-opacity:1;stroke-width:0.295388"
         d="m 149.64116,124.62478 a 20.836416,20.836416 0 0 0 -20.83645,-20.83645 20.836416,20.836416 0 0 0 -20.83645,20.83645 20.836416,20.836416 0 0 0 20.83645,20.83646 20.836416,20.836416 0 0 0 20.83645,-20.83646 z m -0.22789,0 a 20.608709,20.608709 0 0 1 -20.60856,20.60908 20.608709,20.608709 0 0 1 -20.60856,-20.60908 20.608709,20.608709 0 0 1 20.60856,-20.60856 20.608709,20.608709 0 0 1 20.60856,20.60856 z" />
      <path
         id="path31"
         style="fill:#333333;fill-opacity:1;stroke-width:0.31691"
         d="m 151.15928,124.62478 a 22.354536,22.354536 0 0 0 -22.35457,-22.35457 22.354536,22.354536 0 0 0 -22.35457,22.35457 22.354536,22.354536 0 0 0 22.35457,22.35458 22.354536,22.354536 0 0 0 22.35457,-22.35458 z m -0.24448,0 a 22.110238,22.110238 0 0 1 -22.11009,22.11064 22.110238,22.110238 0 0 1 -22.11008,-22.11064 22.110238,22.110238 0 0 1 22.11008,-22.11008 22.110238,22.110238 0 0 1 22.11009,22.11008 z" />
      <rect
         style="fill:#ffffff;fill-opacity:1;stroke-width:0.171085"
         id="rect31"
         width="0.15741065"
         height="29.960497"
         x="-1.0483481"
         y="-194.20372"
         transform="rotate(134.36506)" />
      <rect
         style="fill:#ffffff;fill-opacity:1;stroke-width:0.171085"
         id="rect32"
         width="0.15741065"
         height="29.960497"
         x="-179.22873"
         y="-9.7590256"
         transform="rotate(-134.27554)" />
      <circle
         style="fill:#ffffff;fill-opacity:1;stroke-width:0.3"
         id="circle32"
         cx="124.62479"
         cy="-128.80472"
         r="4.5083642"
         transform="rotate(90)" />
      <path
         id="path32"
         style="fill:#ffffff;fill-opacity:1;stroke-width:0.0739703"
         d="m 134.02254,124.62478 a 5.2178191,5.2178191 0 0 0 -5.21783,-5.21782 5.2178191,5.2178191 0 0 0 -5.21782,5.21782 5.2178191,5.2178191 0 0 0 5.21782,5.21783 5.2178191,5.2178191 0 0 0 5.21783,-5.21783 z m -0.057,0 a 5.160797,5.160797 0 0 1 -5.16077,5.16089 5.160797,5.160797 0 0 1 -5.16076,-5.16089 5.160797,5.160797 0 0 1 5.16076,-5.16076 5.160797,5.160797 0 0 1 5.16077,5.16076 z" />
    </g>
    <g id="g39"
       transform="translate(71.141387,3.5430299)">
      <text
         xml:space="preserve"
         style="font-size:3.52778px;line-height:1.3;font-family:Ubuntu;-inkscape-font-specification:Ubuntu;text-align:start;writing-mode:lr-tb;direction:ltr;text-anchor:start;fill:#f9f9f9;fill-opacity:1;stroke-width:0.3"
         x="48.027882"
         y="96.42379"
         id="text38"><tspan
           sodipodi:role="line"
           id="tspan38"
           style="font-size:3.52778px;stroke-width:0.3"
           x="48.027882"
           y="96.42379">Value:</tspan></text>
      <text
         xml:space="preserve"
         style="font-size:3.52778px;line-height:1.3;font-family:Ubuntu;-inkscape-font-specification:Ubuntu;text-align:start;writing-mode:lr-tb;direction:ltr;text-anchor:start;fill:#000000;fill-opacity:1;stroke-width:0.3"
         x="59.669559"
         y="96.42379"
         id="text39"><tspan
           sodipodi:role="line"
           id="tspan39"
           style="font-size:3.52778px;fill:#000000;stroke-width:0.3"
           x="59.669559"
           y="96.42379">100</tspan></text>
    </g>
    <g id="g41"
       transform="translate(40.323936,-17.02088)">
      <g id="g53">
        <text
           xml:space="preserve"
           style="font-size:3.52778px;line-height:1.3;font-family:Ubuntu;-inkscape-font-specification:Ubuntu;text-align:start;writing-mode:lr-tb;direction:ltr;text-anchor:start;fill:#f9f9f9;fill-opacity:1;stroke-width:0.3"
           x="42.207043"
           y="96.42379"
           id="text40"><tspan
             sodipodi:role="line"
             id="tspan40"
             style="font-size:3.52778px;stroke-width:0.3"
             x="42.207043"
             y="96.42379">Value Hue:</tspan></text>
        <text
           xml:space="preserve"
           style="font-size:3.52778px;line-height:1.3;font-family:Ubuntu;-inkscape-font-specification:Ubuntu;text-align:start;writing-mode:lr-tb;direction:ltr;text-anchor:start;fill:#000000;fill-opacity:1;stroke-width:0.3"
           x="59.669559"
           y="96.42379"
           id="text41"><tspan
             sodipodi:role="line"
             id="tspan41"
             style="font-size:3.52778px;fill:#000000;stroke-width:0.3"
             x="59.669559"
             y="96.42379"> 100</tspan></text>
      </g>
    </g>
    <path
       id="rect41"
       style="fill:#333333;fill-opacity:1;stroke-width:0.283737"
       d="M 36.894906,82.047615 V 90.9528 h -0.134359 v 1.989331 h 0.544669 V 90.9528 h -0.134358 v -8.905185 z" />
    <g id="g57"  style="cursor:pointer;">
      <circle
         style="fill:#333333;fill-opacity:1;stroke-width:0.21"
         id="path42"
         cx="86.958687"
         cy="146.61284"
         r="5.9203753" />
      <path
         id="path47"
         style="fill:#ffffff;fill-opacity:1;stroke-width:0.0739703"
         d="m 86.95869,141.39506 a 5.2178191,5.2178191 0 0 0 -5.217827,5.21783 5.2178191,5.2178191 0 0 0 5.217827,5.21782 5.2178191,5.2178191 0 0 0 5.21783,-5.21782 5.2178191,5.2178191 0 0 0 -5.21783,-5.21783 z m 0,0.057 a 5.160797,5.160797 0 0 1 5.16089,5.16077 5.160797,5.160797 0 0 1 -5.16089,5.16076 5.160797,5.160797 0 0 1 -5.160758,-5.16076 5.160797,5.160797 0 0 1 5.160758,-5.16077 z" />
      <g id="g48"
         transform="translate(29.785713,56.955833)">
        <text
           xml:space="preserve"
           style="font-size:3.52778px;line-height:1.3;font-family:Ubuntu;-inkscape-font-specification:Ubuntu;text-align:start;writing-mode:lr-tb;direction:ltr;text-anchor:start;fill:#f9f9f9;fill-opacity:1;stroke-width:0.3"
           x="53.502323"
           y="90.581314"
           id="text47"><tspan
             sodipodi:role="line"
             id="tspan47"
             style="font-size:3.52778px;stroke-width:0.3"
             x="53.502323"
             y="90.581314">Stop</tspan></text>
      </g>
    </g>
    <g id="g58" style="cursor:pointer;">
      <circle
         style="fill:#333333;fill-opacity:1;stroke-width:0.21"
         id="circle48"
         cx="102.2086"
         cy="146.61284"
         r="5.9203753" />
      <path
         id="path48"
         style="fill:#ffffff;fill-opacity:1;stroke-width:0.0739703"
         d="m 102.20859,141.39506 a 5.2178191,5.2178191 0 0 0 -5.217819,5.21783 5.2178191,5.2178191 0 0 0 5.217819,5.21782 5.2178191,5.2178191 0 0 0 5.21783,-5.21782 5.2178191,5.2178191 0 0 0 -5.21783,-5.21783 z m 0,0.057 a 5.160797,5.160797 0 0 1 5.16089,5.16077 5.160797,5.160797 0 0 1 -5.16089,5.16076 5.160797,5.160797 0 0 1 -5.16075,-5.16076 5.160797,5.160797 0 0 1 5.16075,-5.16077 z" />
      <g id="g49"
         transform="translate(44.72165,56.955833)">
        <text
           xml:space="preserve"
           style="font-size:3.52778px;line-height:1.3;font-family:Ubuntu;-inkscape-font-specification:Ubuntu;text-align:start;writing-mode:lr-tb;direction:ltr;text-anchor:start;fill:#f9f9f9;fill-opacity:1;stroke-width:0.3"
           x="53.502323"
           y="90.581314"
           id="text48"><tspan
             sodipodi:role="line"
             id="tspan48"
             style="font-size:3.52778px;stroke-width:0.3"
             x="53.502323"
             y="90.581314">Start</tspan></text>
      </g>
    </g>
    <path
       id="rect52"
       style="fill:#333333;fill-opacity:1;stroke-width:0.289062"
       d="m 31.53555,81.565092 v 9.15448 h 126.09618 v -9.15448 z m 0.275953,0.29869 H 157.35577 v 8.5571 H 31.811503 Z" />
    <path
       id="path56"
       style="fill:#333333;fill-opacity:1;stroke-width:0.289062"
       d="M 26.553015,74.598907 V 156.15575 H 162.61427 V 74.598907 Z m 0.275953,0.29869 H 162.33831 V 155.85706 H 26.828968 Z"
       sodipodi:nodetypes="cccccccccc" />
       <text
       xml:space="preserve"
       style="font-style:italic;font-size:2.11667px;line-height:0.6;font-family:Ubuntu;-inkscape-font-specification:'Ubuntu Italic';text-align:start;writing-mode:lr-tb;direction:ltr;text-anchor:start;fill:#000000;fill-opacity:1;stroke-width:0.3"
       x="134.54373"
       y="79.408203"
       id="text1"><tspan
         sodipodi:role="line"
         id="tspan1"
         style="font-style:normal;font-variant:normal;font-weight:normal;font-stretch:normal;font-size:2.11667px;font-family:Ubuntu;-inkscape-font-specification:Ubuntu;fill:#000000;fill-opacity:1;stroke-width:0.3"
         x="134.54373"
         y="79.408203">Status: </tspan></text>
    <text
       xml:space="preserve"
       style="font-style:italic;font-size:2.11667px;line-height:0.6;font-family:Ubuntu;-inkscape-font-specification:'Ubuntu Italic';text-align:start;writing-mode:lr-tb;direction:ltr;text-anchor:start;fill:#000000;fill-opacity:1;stroke-width:0.3"
       x="141.95197"
       y="79.408203"
       id="text2"><tspan
         sodipodi:role="line"
         id="tspan2"
         style="font-style:normal;font-variant:normal;font-weight:normal;font-stretch:normal;font-size:2.11667px;font-family:Ubuntu;-inkscape-font-specification:Ubuntu;fill:#000000;fill-opacity:1;stroke-width:0.3"
         x="141.95197"
         y="79.408203">Connecting... </tspan></text>
  </g>
  <script
     id="mesh_polyfill"
     type="text/javascript">
!function(){const t=&quot;http://www.w3.org/2000/svg&quot;,e=&quot;http://www.w3.org/1999/xlink&quot;,s=&quot;http://www.w3.org/1999/xhtml&quot;,r=2;if(document.createElementNS(t,&quot;meshgradient&quot;).x)return;const n=(t,e,s,r)=&gt;{let n=new x(.5*(e.x+s.x),.5*(e.y+s.y)),o=new x(.5*(t.x+e.x),.5*(t.y+e.y)),i=new x(.5*(s.x+r.x),.5*(s.y+r.y)),a=new x(.5*(n.x+o.x),.5*(n.y+o.y)),h=new x(.5*(n.x+i.x),.5*(n.y+i.y)),l=new x(.5*(a.x+h.x),.5*(a.y+h.y));return[[t,o,a,l],[l,h,i,r]]},o=t=&gt;{let e=t[0].distSquared(t[1]),s=t[2].distSquared(t[3]),r=.25*t[0].distSquared(t[2]),n=.25*t[1].distSquared(t[3]),o=e&gt;s?e:s,i=r&gt;n?r:n;return 18*(o&gt;i?o:i)},i=(t,e)=&gt;Math.sqrt(t.distSquared(e)),a=(t,e)=&gt;t.scale(2/3).add(e.scale(1/3)),h=t=&gt;{let e,s,r,n,o,i,a,h=new g;return t.match(/(w+(s*[^)]+))+/g).forEach(t=&gt;{let l=t.match(/[w.-]+/g),d=l.shift();switch(d){case&quot;translate&quot;:2===l.length?e=new g(1,0,0,1,l[0],l[1]):(console.error(&quot;mesh.js: translate does not have 2 arguments!&quot;),e=new g(1,0,0,1,0,0)),h=h.append(e);break;case&quot;scale&quot;:1===l.length?s=new g(l[0],0,0,l[0],0,0):2===l.length?s=new g(l[0],0,0,l[1],0,0):(console.error(&quot;mesh.js: scale does not have 1 or 2 arguments!&quot;),s=new g(1,0,0,1,0,0)),h=h.append(s);break;case&quot;rotate&quot;:if(3===l.length&amp;&amp;(e=new g(1,0,0,1,l[1],l[2]),h=h.append(e)),l[0]){r=l[0]*Math.PI/180;let t=Math.cos(r),e=Math.sin(r);Math.abs(t)&lt;1e-16&amp;&amp;(t=0),Math.abs(e)&lt;1e-16&amp;&amp;(e=0),a=new g(t,e,-e,t,0,0),h=h.append(a)}else console.error(&quot;math.js: No argument to rotate transform!&quot;);3===l.length&amp;&amp;(e=new g(1,0,0,1,-l[1],-l[2]),h=h.append(e));break;case&quot;skewX&quot;:l[0]?(r=l[0]*Math.PI/180,n=Math.tan(r),o=new g(1,0,n,1,0,0),h=h.append(o)):console.error(&quot;math.js: No argument to skewX transform!&quot;);break;case&quot;skewY&quot;:l[0]?(r=l[0]*Math.PI/180,n=Math.tan(r),i=new g(1,n,0,1,0,0),h=h.append(i)):console.error(&quot;math.js: No argument to skewY transform!&quot;);break;case&quot;matrix&quot;:6===l.length?h=h.append(new g(...l)):console.error(&quot;math.js: Incorrect number of arguments for matrix!&quot;);break;default:console.error(&quot;mesh.js: Unhandled transform type: &quot;+d)}}),h},l=t=&gt;{let e=[],s=t.split(/[ ,]+/);for(let t=0,r=s.length-1;t&lt;r;t+=2)e.push(new x(parseFloat(s[t]),parseFloat(s[t+1])));return e},d=(t,e)=&gt;{for(let s in e)t.setAttribute(s,e[s])},c=(t,e,s,r,n)=&gt;{let o,i,a=[0,0,0,0];for(let h=0;h&lt;3;++h)e[h]&lt;t[h]&amp;&amp;e[h]&lt;s[h]||t[h]&lt;e[h]&amp;&amp;s[h]&lt;e[h]?a[h]=0:(a[h]=.5*((e[h]-t[h])/r+(s[h]-e[h])/n),o=Math.abs(3*(e[h]-t[h])/r),i=Math.abs(3*(s[h]-e[h])/n),a[h]&gt;o?a[h]=o:a[h]&gt;i&amp;&amp;(a[h]=i));return a},u=[[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0],[-3,3,0,0,-2,-1,0,0,0,0,0,0,0,0,0,0],[2,-2,0,0,1,1,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0],[0,0,0,0,0,0,0,0,-3,3,0,0,-2,-1,0,0],[0,0,0,0,0,0,0,0,2,-2,0,0,1,1,0,0],[-3,0,3,0,0,0,0,0,-2,0,-1,0,0,0,0,0],[0,0,0,0,-3,0,3,0,0,0,0,0,-2,0,-1,0],[9,-9,-9,9,6,3,-6,-3,6,-6,3,-3,4,2,2,1],[-6,6,6,-6,-3,-3,3,3,-4,4,-2,2,-2,-2,-1,-1],[2,0,-2,0,0,0,0,0,1,0,1,0,0,0,0,0],[0,0,0,0,2,0,-2,0,0,0,0,0,1,0,1,0],[-6,6,6,-6,-4,-2,4,2,-3,3,-3,3,-2,-1,-2,-1],[4,-4,-4,4,2,2,-2,-2,2,-2,2,-2,1,1,1,1]],f=t=&gt;{let e=[];for(let s=0;s&lt;16;++s){e[s]=0;for(let r=0;r&lt;16;++r)e[s]+=u[s][r]*t[r]}return e},p=(t,e,s)=&gt;{const r=e*e,n=s*s,o=e*e*e,i=s*s*s;return t[0]+t[1]*e+t[2]*r+t[3]*o+t[4]*s+t[5]*s*e+t[6]*s*r+t[7]*s*o+t[8]*n+t[9]*n*e+t[10]*n*r+t[11]*n*o+t[12]*i+t[13]*i*e+t[14]*i*r+t[15]*i*o},y=t=&gt;{let e=[],s=[],r=[];for(let s=0;s&lt;4;++s)e[s]=[],e[s][0]=n(t[0][s],t[1][s],t[2][s],t[3][s]),e[s][1]=[],e[s][1].push(...n(...e[s][0][0])),e[s][1].push(...n(...e[s][0][1])),e[s][2]=[],e[s][2].push(...n(...e[s][1][0])),e[s][2].push(...n(...e[s][1][1])),e[s][2].push(...n(...e[s][1][2])),e[s][2].push(...n(...e[s][1][3]));for(let t=0;t&lt;8;++t){s[t]=[];for(let r=0;r&lt;4;++r)s[t][r]=[],s[t][r][0]=n(e[0][2][t][r],e[1][2][t][r],e[2][2][t][r],e[3][2][t][r]),s[t][r][1]=[],s[t][r][1].push(...n(...s[t][r][0][0])),s[t][r][1].push(...n(...s[t][r][0][1])),s[t][r][2]=[],s[t][r][2].push(...n(...s[t][r][1][0])),s[t][r][2].push(...n(...s[t][r][1][1])),s[t][r][2].push(...n(...s[t][r][1][2])),s[t][r][2].push(...n(...s[t][r][1][3]))}for(let t=0;t&lt;8;++t){r[t]=[];for(let e=0;e&lt;8;++e)r[t][e]=[],r[t][e][0]=s[t][0][2][e],r[t][e][1]=s[t][1][2][e],r[t][e][2]=s[t][2][2][e],r[t][e][3]=s[t][3][2][e]}return r};class x{constructor(t,e){this.x=t||0,this.y=e||0}toString(){return`(x=undefined, y=undefined)`}clone(){return new x(this.x,this.y)}add(t){return new x(this.x+t.x,this.y+t.y)}scale(t){return void 0===t.x?new x(this.x*t,this.y*t):new x(this.x*t.x,this.y*t.y)}distSquared(t){let e=this.x-t.x,s=this.y-t.y;return e*e+s*s}transform(t){let e=this.x*t.a+this.y*t.c+t.e,s=this.x*t.b+this.y*t.d+t.f;return new x(e,s)}}class g{constructor(t,e,s,r,n,o){void 0===t?(this.a=1,this.b=0,this.c=0,this.d=1,this.e=0,this.f=0):(this.a=t,this.b=e,this.c=s,this.d=r,this.e=n,this.f=o)}toString(){return`affine: undefined undefined undefined
       undefined undefined undefined`}append(t){t instanceof g||console.error(&quot;mesh.js: argument to Affine.append is not affine!&quot;);let e=this.a*t.a+this.c*t.b,s=this.b*t.a+this.d*t.b,r=this.a*t.c+this.c*t.d,n=this.b*t.c+this.d*t.d,o=this.a*t.e+this.c*t.f+this.e,i=this.b*t.e+this.d*t.f+this.f;return new g(e,s,r,n,o,i)}}class w{constructor(t,e){this.nodes=t,this.colors=e}paintCurve(t,e){if(o(this.nodes)&gt;r){const s=n(...this.nodes);let r=[[],[]],o=[[],[]];for(let t=0;t&lt;4;++t)r[0][t]=this.colors[0][t],r[1][t]=(this.colors[0][t]+this.colors[1][t])/2,o[0][t]=r[1][t],o[1][t]=this.colors[1][t];let i=new w(s[0],r),a=new w(s[1],o);i.paintCurve(t,e),a.paintCurve(t,e)}else{let s=Math.round(this.nodes[0].x);if(s&gt;=0&amp;&amp;s&lt;e){let r=4*(~~this.nodes[0].y*e+s);t[r]=Math.round(this.colors[0][0]),t[r+1]=Math.round(this.colors[0][1]),t[r+2]=Math.round(this.colors[0][2]),t[r+3]=Math.round(this.colors[0][3])}}}}class m{constructor(t,e){this.nodes=t,this.colors=e}split(){let t=[[],[],[],[]],e=[[],[],[],[]],s=[[[],[]],[[],[]]],r=[[[],[]],[[],[]]];for(let s=0;s&lt;4;++s){const r=n(this.nodes[0][s],this.nodes[1][s],this.nodes[2][s],this.nodes[3][s]);t[0][s]=r[0][0],t[1][s]=r[0][1],t[2][s]=r[0][2],t[3][s]=r[0][3],e[0][s]=r[1][0],e[1][s]=r[1][1],e[2][s]=r[1][2],e[3][s]=r[1][3]}for(let t=0;t&lt;4;++t)s[0][0][t]=this.colors[0][0][t],s[0][1][t]=this.colors[0][1][t],s[1][0][t]=(this.colors[0][0][t]+this.colors[1][0][t])/2,s[1][1][t]=(this.colors[0][1][t]+this.colors[1][1][t])/2,r[0][0][t]=s[1][0][t],r[0][1][t]=s[1][1][t],r[1][0][t]=this.colors[1][0][t],r[1][1][t]=this.colors[1][1][t];return[new m(t,s),new m(e,r)]}paint(t,e){let s,n=!1;for(let t=0;t&lt;4;++t)if((s=o([this.nodes[0][t],this.nodes[1][t],this.nodes[2][t],this.nodes[3][t]]))&gt;r){n=!0;break}if(n){let s=this.split();s[0].paint(t,e),s[1].paint(t,e)}else{new w([...this.nodes[0]],[...this.colors[0]]).paintCurve(t,e)}}}class b{constructor(t){this.readMesh(t),this.type=t.getAttribute(&quot;type&quot;)||&quot;bilinear&quot;}readMesh(t){let e=[[]],s=[[]],r=Number(t.getAttribute(&quot;x&quot;)),n=Number(t.getAttribute(&quot;y&quot;));e[0][0]=new x(r,n);let o=t.children;for(let t=0,r=o.length;t&lt;r;++t){e[3*t+1]=[],e[3*t+2]=[],e[3*t+3]=[],s[t+1]=[];let r=o[t].children;for(let n=0,o=r.length;n&lt;o;++n){let o=r[n].children;for(let r=0,i=o.length;r&lt;i;++r){let i=r;0!==t&amp;&amp;++i;let h,d=o[r].getAttribute(&quot;path&quot;),c=&quot;l&quot;;null!=d&amp;&amp;(c=(h=d.match(/s*([lLcC])s*(.*)/))[1]);let u=l(h[2]);switch(c){case&quot;l&quot;:0===i?(e[3*t][3*n+3]=u[0].add(e[3*t][3*n]),e[3*t][3*n+1]=a(e[3*t][3*n],e[3*t][3*n+3]),e[3*t][3*n+2]=a(e[3*t][3*n+3],e[3*t][3*n])):1===i?(e[3*t+3][3*n+3]=u[0].add(e[3*t][3*n+3]),e[3*t+1][3*n+3]=a(e[3*t][3*n+3],e[3*t+3][3*n+3]),e[3*t+2][3*n+3]=a(e[3*t+3][3*n+3],e[3*t][3*n+3])):2===i?(0===n&amp;&amp;(e[3*t+3][3*n+0]=u[0].add(e[3*t+3][3*n+3])),e[3*t+3][3*n+1]=a(e[3*t+3][3*n],e[3*t+3][3*n+3]),e[3*t+3][3*n+2]=a(e[3*t+3][3*n+3],e[3*t+3][3*n])):(e[3*t+1][3*n]=a(e[3*t][3*n],e[3*t+3][3*n]),e[3*t+2][3*n]=a(e[3*t+3][3*n],e[3*t][3*n]));break;case&quot;L&quot;:0===i?(e[3*t][3*n+3]=u[0],e[3*t][3*n+1]=a(e[3*t][3*n],e[3*t][3*n+3]),e[3*t][3*n+2]=a(e[3*t][3*n+3],e[3*t][3*n])):1===i?(e[3*t+3][3*n+3]=u[0],e[3*t+1][3*n+3]=a(e[3*t][3*n+3],e[3*t+3][3*n+3]),e[3*t+2][3*n+3]=a(e[3*t+3][3*n+3],e[3*t][3*n+3])):2===i?(0===n&amp;&amp;(e[3*t+3][3*n+0]=u[0]),e[3*t+3][3*n+1]=a(e[3*t+3][3*n],e[3*t+3][3*n+3]),e[3*t+3][3*n+2]=a(e[3*t+3][3*n+3],e[3*t+3][3*n])):(e[3*t+1][3*n]=a(e[3*t][3*n],e[3*t+3][3*n]),e[3*t+2][3*n]=a(e[3*t+3][3*n],e[3*t][3*n]));break;case&quot;c&quot;:0===i?(e[3*t][3*n+1]=u[0].add(e[3*t][3*n]),e[3*t][3*n+2]=u[1].add(e[3*t][3*n]),e[3*t][3*n+3]=u[2].add(e[3*t][3*n])):1===i?(e[3*t+1][3*n+3]=u[0].add(e[3*t][3*n+3]),e[3*t+2][3*n+3]=u[1].add(e[3*t][3*n+3]),e[3*t+3][3*n+3]=u[2].add(e[3*t][3*n+3])):2===i?(e[3*t+3][3*n+2]=u[0].add(e[3*t+3][3*n+3]),e[3*t+3][3*n+1]=u[1].add(e[3*t+3][3*n+3]),0===n&amp;&amp;(e[3*t+3][3*n+0]=u[2].add(e[3*t+3][3*n+3]))):(e[3*t+2][3*n]=u[0].add(e[3*t+3][3*n]),e[3*t+1][3*n]=u[1].add(e[3*t+3][3*n]));break;case&quot;C&quot;:0===i?(e[3*t][3*n+1]=u[0],e[3*t][3*n+2]=u[1],e[3*t][3*n+3]=u[2]):1===i?(e[3*t+1][3*n+3]=u[0],e[3*t+2][3*n+3]=u[1],e[3*t+3][3*n+3]=u[2]):2===i?(e[3*t+3][3*n+2]=u[0],e[3*t+3][3*n+1]=u[1],0===n&amp;&amp;(e[3*t+3][3*n+0]=u[2])):(e[3*t+2][3*n]=u[0],e[3*t+1][3*n]=u[1]);break;default:console.error(&quot;mesh.js: &quot;+c+&quot; invalid path type.&quot;)}if(0===t&amp;&amp;0===n||r&gt;0){let e=window.getComputedStyle(o[r]).stopColor.match(/^rgbs*(s*(d+)s*,s*(d+)s*,s*(d+)s*)$/i),a=window.getComputedStyle(o[r]).stopOpacity,h=255;a&amp;&amp;(h=Math.floor(255*a)),e&amp;&amp;(0===i?(s[t][n]=[],s[t][n][0]=Math.floor(e[1]),s[t][n][1]=Math.floor(e[2]),s[t][n][2]=Math.floor(e[3]),s[t][n][3]=h):1===i?(s[t][n+1]=[],s[t][n+1][0]=Math.floor(e[1]),s[t][n+1][1]=Math.floor(e[2]),s[t][n+1][2]=Math.floor(e[3]),s[t][n+1][3]=h):2===i?(s[t+1][n+1]=[],s[t+1][n+1][0]=Math.floor(e[1]),s[t+1][n+1][1]=Math.floor(e[2]),s[t+1][n+1][2]=Math.floor(e[3]),s[t+1][n+1][3]=h):3===i&amp;&amp;(s[t+1][n]=[],s[t+1][n][0]=Math.floor(e[1]),s[t+1][n][1]=Math.floor(e[2]),s[t+1][n][2]=Math.floor(e[3]),s[t+1][n][3]=h))}}e[3*t+1][3*n+1]=new x,e[3*t+1][3*n+2]=new x,e[3*t+2][3*n+1]=new x,e[3*t+2][3*n+2]=new x,e[3*t+1][3*n+1].x=(-4*e[3*t][3*n].x+6*(e[3*t][3*n+1].x+e[3*t+1][3*n].x)+-2*(e[3*t][3*n+3].x+e[3*t+3][3*n].x)+3*(e[3*t+3][3*n+1].x+e[3*t+1][3*n+3].x)+-1*e[3*t+3][3*n+3].x)/9,e[3*t+1][3*n+2].x=(-4*e[3*t][3*n+3].x+6*(e[3*t][3*n+2].x+e[3*t+1][3*n+3].x)+-2*(e[3*t][3*n].x+e[3*t+3][3*n+3].x)+3*(e[3*t+3][3*n+2].x+e[3*t+1][3*n].x)+-1*e[3*t+3][3*n].x)/9,e[3*t+2][3*n+1].x=(-4*e[3*t+3][3*n].x+6*(e[3*t+3][3*n+1].x+e[3*t+2][3*n].x)+-2*(e[3*t+3][3*n+3].x+e[3*t][3*n].x)+3*(e[3*t][3*n+1].x+e[3*t+2][3*n+3].x)+-1*e[3*t][3*n+3].x)/9,e[3*t+2][3*n+2].x=(-4*e[3*t+3][3*n+3].x+6*(e[3*t+3][3*n+2].x+e[3*t+2][3*n+3].x)+-2*(e[3*t+3][3*n].x+e[3*t][3*n+3].x)+3*(e[3*t][3*n+2].x+e[3*t+2][3*n].x)+-1*e[3*t][3*n].x)/9,e[3*t+1][3*n+1].y=(-4*e[3*t][3*n].y+6*(e[3*t][3*n+1].y+e[3*t+1][3*n].y)+-2*(e[3*t][3*n+3].y+e[3*t+3][3*n].y)+3*(e[3*t+3][3*n+1].y+e[3*t+1][3*n+3].y)+-1*e[3*t+3][3*n+3].y)/9,e[3*t+1][3*n+2].y=(-4*e[3*t][3*n+3].y+6*(e[3*t][3*n+2].y+e[3*t+1][3*n+3].y)+-2*(e[3*t][3*n].y+e[3*t+3][3*n+3].y)+3*(e[3*t+3][3*n+2].y+e[3*t+1][3*n].y)+-1*e[3*t+3][3*n].y)/9,e[3*t+2][3*n+1].y=(-4*e[3*t+3][3*n].y+6*(e[3*t+3][3*n+1].y+e[3*t+2][3*n].y)+-2*(e[3*t+3][3*n+3].y+e[3*t][3*n].y)+3*(e[3*t][3*n+1].y+e[3*t+2][3*n+3].y)+-1*e[3*t][3*n+3].y)/9,e[3*t+2][3*n+2].y=(-4*e[3*t+3][3*n+3].y+6*(e[3*t+3][3*n+2].y+e[3*t+2][3*n+3].y)+-2*(e[3*t+3][3*n].y+e[3*t][3*n+3].y)+3*(e[3*t][3*n+2].y+e[3*t+2][3*n].y)+-1*e[3*t][3*n].y)/9}}this.nodes=e,this.colors=s}paintMesh(t,e){let s=(this.nodes.length-1)/3,r=(this.nodes[0].length-1)/3;if(&quot;bilinear&quot;===this.type||s&lt;2||r&lt;2){let n;for(let o=0;o&lt;s;++o)for(let s=0;s&lt;r;++s){let r=[];for(let t=3*o,e=3*o+4;t&lt;e;++t)r.push(this.nodes[t].slice(3*s,3*s+4));let i=[];i.push(this.colors[o].slice(s,s+2)),i.push(this.colors[o+1].slice(s,s+2)),(n=new m(r,i)).paint(t,e)}}else{let n,o,a,h,l,d,u;const x=s,g=r;s++,r++;let w=new Array(s);for(let t=0;t&lt;s;++t){w[t]=new Array(r);for(let e=0;e&lt;r;++e)w[t][e]=[],w[t][e][0]=this.nodes[3*t][3*e],w[t][e][1]=this.colors[t][e]}for(let t=0;t&lt;s;++t)for(let e=0;e&lt;r;++e)0!==t&amp;&amp;t!==x&amp;&amp;(n=i(w[t-1][e][0],w[t][e][0]),o=i(w[t+1][e][0],w[t][e][0]),w[t][e][2]=c(w[t-1][e][1],w[t][e][1],w[t+1][e][1],n,o)),0!==e&amp;&amp;e!==g&amp;&amp;(n=i(w[t][e-1][0],w[t][e][0]),o=i(w[t][e+1][0],w[t][e][0]),w[t][e][3]=c(w[t][e-1][1],w[t][e][1],w[t][e+1][1],n,o));for(let t=0;t&lt;r;++t){w[0][t][2]=[],w[x][t][2]=[];for(let e=0;e&lt;4;++e)n=i(w[1][t][0],w[0][t][0]),o=i(w[x][t][0],w[x-1][t][0]),w[0][t][2][e]=n&gt;0?2*(w[1][t][1][e]-w[0][t][1][e])/n-w[1][t][2][e]:0,w[x][t][2][e]=o&gt;0?2*(w[x][t][1][e]-w[x-1][t][1][e])/o-w[x-1][t][2][e]:0}for(let t=0;t&lt;s;++t){w[t][0][3]=[],w[t][g][3]=[];for(let e=0;e&lt;4;++e)n=i(w[t][1][0],w[t][0][0]),o=i(w[t][g][0],w[t][g-1][0]),w[t][0][3][e]=n&gt;0?2*(w[t][1][1][e]-w[t][0][1][e])/n-w[t][1][3][e]:0,w[t][g][3][e]=o&gt;0?2*(w[t][g][1][e]-w[t][g-1][1][e])/o-w[t][g-1][3][e]:0}for(let s=0;s&lt;x;++s)for(let r=0;r&lt;g;++r){let n=i(w[s][r][0],w[s+1][r][0]),o=i(w[s][r+1][0],w[s+1][r+1][0]),c=i(w[s][r][0],w[s][r+1][0]),x=i(w[s+1][r][0],w[s+1][r+1][0]),g=[[],[],[],[]];for(let t=0;t&lt;4;++t){(d=[])[0]=w[s][r][1][t],d[1]=w[s+1][r][1][t],d[2]=w[s][r+1][1][t],d[3]=w[s+1][r+1][1][t],d[4]=w[s][r][2][t]*n,d[5]=w[s+1][r][2][t]*n,d[6]=w[s][r+1][2][t]*o,d[7]=w[s+1][r+1][2][t]*o,d[8]=w[s][r][3][t]*c,d[9]=w[s+1][r][3][t]*x,d[10]=w[s][r+1][3][t]*c,d[11]=w[s+1][r+1][3][t]*x,d[12]=0,d[13]=0,d[14]=0,d[15]=0,u=f(d);for(let e=0;e&lt;9;++e){g[t][e]=[];for(let s=0;s&lt;9;++s)g[t][e][s]=p(u,e/8,s/8),g[t][e][s]&gt;255?g[t][e][s]=255:g[t][e][s]&lt;0&amp;&amp;(g[t][e][s]=0)}}h=[];for(let t=3*s,e=3*s+4;t&lt;e;++t)h.push(this.nodes[t].slice(3*r,3*r+4));l=y(h);for(let s=0;s&lt;8;++s)for(let r=0;r&lt;8;++r)(a=new m(l[s][r],[[[g[0][s][r],g[1][s][r],g[2][s][r],g[3][s][r]],[g[0][s][r+1],g[1][s][r+1],g[2][s][r+1],g[3][s][r+1]]],[[g[0][s+1][r],g[1][s+1][r],g[2][s+1][r],g[3][s+1][r]],[g[0][s+1][r+1],g[1][s+1][r+1],g[2][s+1][r+1],g[3][s+1][r+1]]]])).paint(t,e)}}}transform(t){if(t instanceof x)for(let e=0,s=this.nodes.length;e&lt;s;++e)for(let s=0,r=this.nodes[0].length;s&lt;r;++s)this.nodes[e][s]=this.nodes[e][s].add(t);else if(t instanceof g)for(let e=0,s=this.nodes.length;e&lt;s;++e)for(let s=0,r=this.nodes[0].length;s&lt;r;++s)this.nodes[e][s]=this.nodes[e][s].transform(t)}scale(t){for(let e=0,s=this.nodes.length;e&lt;s;++e)for(let s=0,r=this.nodes[0].length;s&lt;r;++s)this.nodes[e][s]=this.nodes[e][s].scale(t)}}document.querySelectorAll(&quot;rect,circle,ellipse,path,text&quot;).forEach((r,n)=&gt;{let o=r.getAttribute(&quot;id&quot;);o||(o=&quot;patchjs_shape&quot;+n,r.setAttribute(&quot;id&quot;,o));const i=r.style.fill.match(/^url(s*&quot;?s*#([^s&quot;]+)&quot;?s*)/),a=r.style.stroke.match(/^url(s*&quot;?s*#([^s&quot;]+)&quot;?s*)/);if(i&amp;&amp;i[1]){const a=document.getElementById(i[1]);if(a&amp;&amp;&quot;meshgradient&quot;===a.nodeName){const i=r.getBBox();let l=document.createElementNS(s,&quot;canvas&quot;);d(l,{width:i.width,height:i.height});const c=l.getContext(&quot;2d&quot;);let u=c.createImageData(i.width,i.height);const f=new b(a);&quot;objectBoundingBox&quot;===a.getAttribute(&quot;gradientUnits&quot;)&amp;&amp;f.scale(new x(i.width,i.height));const p=a.getAttribute(&quot;gradientTransform&quot;);null!=p&amp;&amp;f.transform(h(p)),&quot;userSpaceOnUse&quot;===a.getAttribute(&quot;gradientUnits&quot;)&amp;&amp;f.transform(new x(-i.x,-i.y)),f.paintMesh(u.data,l.width),c.putImageData(u,0,0);const y=document.createElementNS(t,&quot;image&quot;);d(y,{width:i.width,height:i.height,x:i.x,y:i.y});let g=l.toDataURL();y.setAttributeNS(e,&quot;xlink:href&quot;,g),r.parentNode.insertBefore(y,r),r.style.fill=&quot;none&quot;;const w=document.createElementNS(t,&quot;use&quot;);w.setAttributeNS(e,&quot;xlink:href&quot;,&quot;#&quot;+o);const m=&quot;patchjs_clip&quot;+n,M=document.createElementNS(t,&quot;clipPath&quot;);M.setAttribute(&quot;id&quot;,m),M.appendChild(w),r.parentElement.insertBefore(M,r),y.setAttribute(&quot;clip-path&quot;,&quot;url(#&quot;+m+&quot;)&quot;),u=null,l=null,g=null}}if(a&amp;&amp;a[1]){const o=document.getElementById(a[1]);if(o&amp;&amp;&quot;meshgradient&quot;===o.nodeName){const i=parseFloat(r.style.strokeWidth.slice(0,-2))*(parseFloat(r.style.strokeMiterlimit)||parseFloat(r.getAttribute(&quot;stroke-miterlimit&quot;))||1),a=r.getBBox(),l=Math.trunc(a.width+i),c=Math.trunc(a.height+i),u=Math.trunc(a.x-i/2),f=Math.trunc(a.y-i/2);let p=document.createElementNS(s,&quot;canvas&quot;);d(p,{width:l,height:c});const y=p.getContext(&quot;2d&quot;);let g=y.createImageData(l,c);const w=new b(o);&quot;objectBoundingBox&quot;===o.getAttribute(&quot;gradientUnits&quot;)&amp;&amp;w.scale(new x(l,c));const m=o.getAttribute(&quot;gradientTransform&quot;);null!=m&amp;&amp;w.transform(h(m)),&quot;userSpaceOnUse&quot;===o.getAttribute(&quot;gradientUnits&quot;)&amp;&amp;w.transform(new x(-u,-f)),w.paintMesh(g.data,p.width),y.putImageData(g,0,0);const M=document.createElementNS(t,&quot;image&quot;);d(M,{width:l,height:c,x:0,y:0});let S=p.toDataURL();M.setAttributeNS(e,&quot;xlink:href&quot;,S);const k=&quot;pattern_clip&quot;+n,A=document.createElementNS(t,&quot;pattern&quot;);d(A,{id:k,patternUnits:&quot;userSpaceOnUse&quot;,width:l,height:c,x:u,y:f}),A.appendChild(M),o.parentNode.appendChild(A),r.style.stroke=&quot;url(#&quot;+k+&quot;)&quot;,g=null,p=null,S=null}}})}();
</script>
</svg>
    </div>
    <script>
        let ws = new WebSocket('ws://lucas.local:81/');
    ws.onopen = () => {
      console.log('WebSocket opened');
      const tspan2 = document.getElementById('tspan2');
      tspan2.textContent = "Ready";
  }
    ws.onmessage = evt => console.log('From ESP32: ' + evt.data);

        const rect = document.getElementById('rect36');
        const marker = document.getElementById('rect41');
        const hueText = document.getElementById('tspan41');

    hueText.textContent = 0 + '°';
    marker.setAttribute('transform', 'translate(-5,0)');
    rect.addEventListener('click', function(evt) {
        const svg = rect.ownerSVGElement;
        const pt = svg.createSVGPoint();
        pt.x = evt.clientX;
        pt.y = evt.clientY;
        const cursorpt = pt.matrixTransform(svg.getScreenCTM().inverse());
    const minX = 6;
    const maxX = 131;
    const minT = -5;
    const maxT = 120;

    let cx = Math.max(minX, Math.min(maxX, cursorpt.x));
    let translateX = minT + ((cx - minX) / (maxX - minX)) * (maxT - minT);
    marker.setAttribute('transform', 'translate(' + translateX + ',0)');


    const hue = Math.round(((cx - minX) / (maxX - minX)) * 360);
    hueText.textContent = hue + '°';
    ws.send('hue,' + hue);
    });

    const stop = document.getElementById('g57');
    stop.addEventListener('click', function(evt) {
        console.log('stop clicked');
        ws.send('stop,');
    });

    const start = document.getElementById('g58');
    start.addEventListener('click', function(evt) {
        ws.send('start,');
    });

    const topY = 30;
    const bottomY = 72;
    const leftCircle = document.getElementById('g59');
    const tspan37 = document.getElementById('tspan37');
    leftCircle.addEventListener('click', function(evt) {
        const svg = rect.ownerSVGElement;
        const pt = svg.createSVGPoint();
        pt.x = evt.clientX;
        pt.y = evt.clientY;
        const cursorpt = pt.matrixTransform(svg.getScreenCTM().inverse());
        let v = 100 - ((cursorpt.y - topY) / (bottomY - topY)) * 200;
        v = Math.max(-100, Math.min(100, v));
        v = Math.round(v);
        tspan37.textContent = v;
        ws.send('fb,'+v);
    });

    const leftX = 81;
    const rightX = 123;
    const rightCircle = document.getElementById('g60');
    const tspan39 = document.getElementById('tspan39');
    rightCircle.addEventListener('click', function(evt) {
        const svg = rect.ownerSVGElement;
        const pt = svg.createSVGPoint();
        pt.x = evt.clientX;
        pt.y = evt.clientY;
        const cursorpt = pt.matrixTransform(svg.getScreenCTM().inverse());
        console.log(cursorpt.x);
        let v = 100 - ((cursorpt.x - leftX) / (rightX - leftX)) * 200;
        v = Math.max(-100, Math.min(100, v));
        v = Math.round(v)*(-1);
        tspan39.textContent = v;
        ws.send('lr,'+v);
    });

    document.addEventListener('keydown', function(event) {

        switch (event.key) {
          case 'ArrowUp':
          case 'w':
            tspan37.textContent = 100;
            ws.send('fb,100');
            break;
          case 'ArrowDown':
          case 's':
            tspan37.textContent = -100;
            ws.send('fb,-100');
            break;
          case 'ArrowLeft':
          case 'a':
            tspan39.textContent = -100;
            ws.send('lr,-100');
            break;
          case 'ArrowRight':
          case 'd':
            tspan39.textContent = 100;
            ws.send('lr,100');
            break;
          case 'space':
            tspan39.textContent = 100;
            ws.send('stop,');
            break;
          default:
            // other keys
            break;

        }
      });

      function isMobile() {
        return /Mobi|Android|iPhone|iPad|iPod|Opera Mini|IEMobile|BlackBerry/i.test(navigator.userAgent);
      }

      if (isMobile()) {
        console.log("This is a mobile device!");
      } else {
        console.log("This is not a mobile device.");
      }

      if (true) {
        // Make SVG fill viewport while keeping aspect ratio
        const svg = document.getElementById('svg1');
        svg.removeAttribute('width');
        svg.removeAttribute('height');
        svg.style.width = '100vw';
        svg.style.height = '100vh';
        svg.style.maxWidth = '100vw';
        svg.style.maxHeight = '100vh';
        svg.style.display = 'block';
      }

    </script>

    <style>
        body, html {
          margin: 0;
          padding: 0;
          width: 100vw;
          height: 100vh;
          overflow: hidden;
        }
        .svg-container {
          width: 100vw;
          height: 100vh;
          display: flex;
          align-items: center;
          justify-content: center;
        }
        svg {
          /* For desktop, keep original size and ratio */
          max-width: 100vw;
          max-height: 100vh;
          width: 136.90706mm;
          height: 82.806686mm;
          display: block;
        }
      </style>

</body>

</html>
)rawliteral";

void handleRoot() {
  // Redirect root to the provided remote image URL (as requested)
  const char* redirectUrl = "https://th.bing.com/th?id=ORMS.6ac2c113fac93d6ba4adcdb7c8362d92&pid=Wdp&w=612&h=304&qlt=90&c=1&rs=1&dpr=1.375&p=0";
  server.sendHeader("Location", String(redirectUrl), true);
  server.send(302, "text/plain", "");
}

void handleToggle() {
  // Toggle the actual pin state (read-modify-write) so web state stays in sync
  int cur = digitalRead(LED_PIN);
  int next = (cur == HIGH) ? LOW : HIGH;
  digitalWrite(LED_PIN, next);
  ledState = (next == HIGH);
  Serial.print("handleToggle: pin "); Serial.print(LED_PIN); Serial.print(" -> "); Serial.println(ledState ? "HIGH" : "LOW");
  server.send(200, "application/json", ledState ? "{\"led\":true}" : "{\"led\":false}");
}

void handleStatus() {
  // Ensure status reflects the real pin state
  ledState = (digitalRead(LED_PIN) == HIGH);
  server.send(200, "application/json", ledState ? "{\"led\":true}" : "{\"led\":false}");
}

// Proxy handler: build target from Host header + URI, fetch via STA and return content.
// Tries HTTP first; on failure attempts HTTPS (insecure TLS). Streams response to client when possible.
void handleProxy() {
  String host = server.hostHeader();
  String uri = server.uri();
  if (host.length() == 0) {
    server.send(400, "text/plain", "Bad Request: no Host header");
    return;
  }

  // Avoid proxying back to the device or AP to prevent loops
  String softAP = WiFi.softAPIP().toString();
  String staIP = WiFi.localIP().toString();
  if (host == softAP || host == staIP || host.indexOf("esp32") >= 0) {
    server.sendHeader("Location", String("http://") + AP_IP.toString(), true);
    server.send(302, "text/plain", "");
    return;
  }

  // Try plain HTTP first, then HTTPS if HTTP fails. Resolve host via STA DNS to avoid captive-AP DNS loop.
  int finalCode = -1;
  String finalType = "text/html; charset=utf-8";

  // Parse possible host:port in Host header
  String hostHeader = host;
  String hostName = hostHeader;
  int explicitPort = 0;
  int colonPos = hostHeader.indexOf(':');
  if (colonPos > 0) {
    hostName = hostHeader.substring(0, colonPos);
    explicitPort = hostHeader.substring(colonPos + 1).toInt();
  }

  for (int pass = 0; pass < 2; ++pass) {
    bool useHttps = (pass == 1);
    int port = explicitPort ? explicitPort : (useHttps ? 443 : 80);

    IPAddress remoteIP;
    if (!WiFi.hostByName(hostName.c_str(), remoteIP)) {
      Serial.print("DNS resolve failed for "); Serial.println(hostName);
      continue; // try next (https) or fail
    }

    // If resolved to our AP or local IP, abort to avoid looping back to ourselves
    if (remoteIP == AP_IP || remoteIP == WiFi.softAPIP() || remoteIP == WiFi.localIP()) {
      Serial.print("Resolved host resolves to local AP/IP (loop): "); Serial.println(remoteIP);
      break;
    }

    String targetIp = remoteIP.toString();
    lastProxyUrl = String(useHttps ? "https://" : "http://") + hostHeader + uri;
    Serial.print("Proxying to IP: "); Serial.print(targetIp); Serial.print(":"); Serial.print(port); Serial.print(" "); Serial.println(uri);

    HTTPClient http;
    WiFiClient *clientPtr = nullptr;
    WiFiClientSecure *secure = nullptr;
    if (useHttps) {
      secure = new WiFiClientSecure();
      secure->setInsecure();
      clientPtr = secure;
      http.begin(*clientPtr, targetIp, port, uri, true);
    } else {
      http.begin(targetIp, port, uri);
    }

    // Preserve original Host header when connecting by IP
    http.addHeader("Host", hostHeader);

    int httpCode = http.GET();
    finalCode = httpCode;
    lastProxyStatus = httpCode;

    if (httpCode > 0) {
      String cType = http.header("Content-Type");
      if (cType.length() > 0) finalType = cType;

      // Try to stream the response to the client to avoid buffering large strings
      Stream *upstream = http.getStreamPtr();
      if (upstream != nullptr) {
        // send headers first with the real HTTP status
        server.sendHeader("Connection", "close");
        server.send(httpCode, finalType.c_str(), "");
        WiFiClient client = server.client();
        const size_t BUF_SZ = 512;
        uint8_t buf[BUF_SZ];
        while (http.connected()) {
          int avail = upstream->available();
          if (avail > 0) {
            int toRead = avail > (int)BUF_SZ ? BUF_SZ : avail;
            int r = upstream->readBytes((char*)buf, toRead);
            if (r > 0) client.write(buf, r);
          } else {
            delay(5);
          }
        }
        client.flush();
      } else {
        // fallback to buffering small responses
        String payload = http.getString();
        server.send(httpCode, finalType.c_str(), payload);
      }

      http.end();
      if (secure) { delete secure; secure = nullptr; }
      return;
    }

    http.end();
    if (secure) { delete secure; secure = nullptr; }
    Serial.print("Attempt failed, code: "); Serial.println(httpCode);
  }

  Serial.print("All proxy attempts failed or aborted. last code: "); Serial.println(finalCode);
  server.send(502, "text/plain", "Bad Gateway (proxy failed)");
}

// Return last proxied URL and status for debugging
void handleProxyDebug() {
  String j = "{\"lastUrl\":\"" + lastProxyUrl + "\",\"lastStatus\":" + String(lastProxyStatus) + "}";
  server.send(200, "application/json", j);
}

// ----- Simple movement API (blocking) mapped to your motor pins -----
// Uses analogWrite(pin, speed) where available; durations are in seconds.
void stopMotor() {
  digitalWrite(M1, LOW);
  digitalWrite(M4, LOW);
  digitalWrite(M2, LOW);
  digitalWrite(M3, LOW);
}
void turnRight(float duration3) {
  // simple right turn: drive left-side motors forward, right-side motors off
  int turnDuty = 200;

  // In digital pattern mode, a 'right turn' will use pattern B briefly
  applyPatternB();
  waitWithService((unsigned long)(duration3 * 1000.0));
  stopMotor();
}

void turnLeft(float duration4) {
  // simple left turn: drive right-side motors forward, left-side motors off
  int turnDuty = 200;

  // In digital pattern mode, a 'left turn' will use pattern A briefly
  applyPatternA();
  waitWithService((unsigned long)(duration4 * 1000.0));
  stopMotor();
}

void moveForward(float speed2, float duration2) {
  // drive forward: use M1 and M2 as forward pair
  if (speed2 < 0) speed2 = 0; if (speed2 > 255) speed2 = 255;
  // In digital-mode, use pattern A as 'forward' behavior
  applyPatternA();
  waitWithService((unsigned long)(duration2 * 1000.0));
  stopMotor();
}

void moveBackward(float speed, float duration) {
  // drive backward: use M3 and M4 as backward pair
  if (speed < 0) speed = 0; if (speed > 255) speed = 255;
  // In digital-mode, use pattern B as 'backward' behavior
  applyPatternB();
  waitWithService((unsigned long)(duration * 1000.0));
  stopMotor();
}


void setupWiFi() {
  WiFi.mode(WIFI_AP_STA);

  // Start AP with fixed IP (captive portal)
  WiFi.softAPConfig(AP_IP, AP_GW, AP_SN);
  WiFi.softAP(AP_SSID);

  // Start DNS server to capture all queries when connected to AP
  dnsServer.start(DNS_PORT, "*", AP_IP);

  // Begin station connection (non-blocking)
  WiFi.begin(STA_SSID, STA_PASS);
  Serial.print("Connecting to STA network ");
  Serial.print(STA_SSID);
  Serial.println(" ...");
}

void setupMDNS() {
  if (MDNS.begin("lucas")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS responder started: lucas.local");
  } else {
    Serial.println("mDNS failed to start");
  }
}

const char functionsPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Motor Control Functions</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, sans-serif; margin: 20px; }
    .function { border: 1px solid #ccc; margin: 10px 0; padding: 10px; border-radius: 5px; }
    .endpoint { color: #0066cc; }
    .description { margin: 5px 0; }
  </style>
</head>
<body>
  <h1>Available Motor Control Functions</h1>
  
  <div class="function">
    <h3 class="endpoint">/motor/start</h3>
    <p class="description">Start continuous motor pattern with current speed and interval settings</p>
  </div>
  
  <div class="function">
    <h3 class="endpoint">/motor/stop</h3>
    <p class="description">Stop all motors immediately</p>
  </div>
  
  <div class="function">
    <h3 class="endpoint">/motor/jump</h3>
    <p class="description">Execute a single jump cycle</p>
  </div>
  
  <div class="function">
    <h3 class="endpoint">/motor/speed?value=0-255</h3>
    <p class="description">Set motor speed (0-255)</p>
  </div>
  
  <div class="function">
    <h3 class="endpoint">/motor/interval?ms=1000</h3>
    <p class="description">Set interval between motor pattern changes (milliseconds)</p>
  </div>
  
  <div class="function">
    <h3 class="endpoint">/move/forward?speed=0-100&duration=seconds</h3>
    <p class="description">Move forward at specified speed for duration</p>
  </div>
  
  <div class="function">
    <h3 class="endpoint">/move/backward?speed=0-100&duration=seconds</h3>
    <p class="description">Move backward at specified speed for duration</p>
  </div>
  
  <div class="function">
    <h3 class="endpoint">/move/left?duration=seconds</h3>
    <p class="description">Turn left for duration</p>
  </div>
  
  <div class="function">
    <h3 class="endpoint">/move/right?duration=seconds</h3>
    <p class="description">Turn right for duration</p>
  </div>
  
  <div class="function">
    <h3 class="endpoint">/controller</h3>
    <p class="description">Interactive motor control interface</p>
  </div>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  // motor pins
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
  // Setup LEDC PWM channels for ESP32
  // Initialize PWM channels via helper (sketch uses analogWrite only)
  pwmInit();
  // set initial duty 0
  // ensure motors and extra pins are off
  stopMotor();
  digitalWrite(P17, LOW); digitalWrite(P19, LOW); digitalWrite(P18, LOW); digitalWrite(P23, LOW);
  digitalWrite(P32, LOW); digitalWrite(P33, LOW); digitalWrite(P25, LOW); digitalWrite(P26, LOW);
  setupWiFi();

  // Web server routes
  server.on("/", handleRoot);
  // Redirect favicon requests to same remote image
  server.on("/favicon.ico", [](){
    const char* redirectUrl = "https://th.bing.com/th?id=ORMS.6ac2c113fac93d6ba4adcdb7c8362d92&pid=Wdp&w=612&h=304&qlt=90&c=1&rs=1&dpr=1.375&p=0";
    server.sendHeader("Location", String(redirectUrl), true);
    server.send(302, "text/plain", "");
  });
  server.on("/toggle", handleToggle);
  server.on("/status", handleStatus);
  server.on("/functions", [](){ server.send_P(200, "text/html; charset=utf-8", functionsPage); });
  // Serve the controller page (verbatim SVG + JS)
  server.on("/controller", [](){ server.send_P(200, "text/html; charset=utf-8", controllerPage); });
  server.on("/motor/start", [](){
    motorRunning = true;
    server.send(200, "application/json", "{\"motor\":\"started\"}");
  });
  server.on("/motor/stop", [](){
    motorRunning = false;
    // turn all motor outputs off via LEDC PWM channels
    stopMotor();
    // ensure extra pins off too
    digitalWrite(P17, LOW); digitalWrite(P19, LOW); digitalWrite(P18, LOW); digitalWrite(P23, LOW);
    digitalWrite(P32, LOW); digitalWrite(P33, LOW); digitalWrite(P25, LOW); digitalWrite(P26, LOW);
    server.send(200, "application/json", "{\"motor\":\"stopped\"}");
  });
  server.on("/motor/jump", [](){
    // schedule a single jump cycle (two phases)
    if (!jumpPending) {
      jumpPending = true;
      jumpStep = 0;
    }
    server.send(200, "application/json", "{\"motor\":\"jump_scheduled\"}");
  });
  server.on("/motor/status", [](){
    String j = "{";
    j += "\"motorRunning\":" + String(motorRunning ? "true" : "false") + ",";
    j += "\"jumpPending\":" + String(jumpPending ? "true" : "false") + ",";
    j += "\"motorPhase\":" + String(motorPhase) + ",";
    j += "\"motorDuty\":" + String(motorDuty) + ",";
    j += "\"motorInterval\":" + String(MOTOR_INTERVAL);
    j += "}";
    server.send(200, "application/json", j);
  });

  server.on("/motor/speed", [](){
    // set duty via query ?d=0-255
    if (server.hasArg("d")) {
      int d = server.arg("d").toInt();
      if (d < 0) d = 0; if (d > 255) d = 255;
      motorDuty = d;
      // In digital pattern mode, motorDuty is informational only.
      // If motors are running, toggle patterns immediately to reflect change
      if (motorRunning) {
        if (motorPhase == 0) applyPatternA(); else applyPatternB();
      }
      server.send(200, "application/json", String("{\"motorDuty\":") + String(motorDuty) + "}");
    } else {
      server.send(400, "text/plain", "missing d parameter");
    }
  });

  server.on("/motor/interval", [](){
    if (server.hasArg("ms")) {
      unsigned long v = (unsigned long) server.arg("ms").toInt();
      if (v < 10) v = 10;
      MOTOR_INTERVAL = v;
      server.send(200, "application/json", String("{\"motorInterval\":") + String(MOTOR_INTERVAL) + "}");
    } else {
      server.send(400, "text/plain", "missing ms parameter");
    }
  });
  // Movement endpoints (blocking)
  server.on("/move/stop", [](){
    stopMotor();
    server.send(200, "application/json", "{\"result\":\"stopped\"}");
  });

  server.on("/move/forward", [](){
    if (server.hasArg("speed") && server.hasArg("dur")) {
      int s = server.arg("speed").toInt();
      int d = server.arg("dur").toInt();
      moveForward(s, d);
      server.send(200, "application/json", "{\"result\":\"ok\"}");
    } else {
      server.send(400, "text/plain", "missing speed or dur parameter");
    }
  });

  server.on("/move/backward", [](){
    if (server.hasArg("speed") && server.hasArg("dur")) {
      int s = server.arg("speed").toInt();
      int d = server.arg("dur").toInt();
      moveBackward(s, d);
      server.send(200, "application/json", "{\"result\":\"ok\"}");
    } else {
      server.send(400, "text/plain", "missing speed or dur parameter");
    }
  });

  server.on("/move/left", [](){
    if (server.hasArg("dur")) {
      int d = server.arg("dur").toInt();
      turnLeft(d);
      server.send(200, "application/json", "{\"result\":\"ok\"}");
    } else {
      server.send(400, "text/plain", "missing dur parameter");
    }
  });

  server.on("/move/right", [](){
    if (server.hasArg("dur")) {
      int d = server.arg("dur").toInt();
      turnRight(d);
      server.send(200, "application/json", "{\"result\":\"ok\"}");
    } else {
      server.send(400, "text/plain", "missing dur parameter");
    }
  });
  server.on("/proxy-debug", handleProxyDebug);
  // Proxy other requests: forward the requested Host+URI over the STA connection
  server.onNotFound(handleProxy);
  server.begin();

  // Start WebSocket server for controller interactions
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // small delay to let WiFi attempt connect, then start mDNS
  delay(500);
  setupMDNS();

  Serial.println("Setup complete.");
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
}

void loop() {


  if (a == 10) {
    b = 1;

  } else if (a == 0) {
    b = 0;
  } else if (b == 1) {
    a = a - 1;
  } else if (b == 0) {
    a = a + 1;
  }
    Serial.println(b);
    Serial.println(a);
    // process webserver and DNS (DNS only useful when clients are on the AP)
    server.handleClient();
    dnsServer.processNextRequest();
  // service WebSocket connections
  webSocket.loop();

    // If user button (GPIO0) pressed, run the blocking move sequence provided
    if (digitalRead(0) == LOW) {
      moveForward(200, 2); // move forward at speed 200 for 2 seconds
      stopMotor();
      delay(500); // brief pause
      moveBackward(200, 2); // move backward at speed 200 for 2 seconds
      stopMotor();
      delay(500); // brief pause
      turnLeft(10);
      turnRight(10);
    } else {
      stopMotor();
    }

    // Optionally print STA status periodically
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 3000) {
      lastStatus = millis();
      if (WiFi.status() == WL_CONNECTED) {
        Serial.print("STA IP: ");
        Serial.println(WiFi.localIP());
      } else {
        Serial.print("STA connecting... (status ");
        Serial.print(WiFi.status());
        Serial.println(")");
      }
    }
    // Non-blocking motor state machine
    unsigned long now = millis();
    if (motorRunning) {
      if (now - motorLastMs >= MOTOR_INTERVAL) {
        motorLastMs = now;
        // toggle between the two direct-digital patterns (A/B) like code.ino
        if (motorPhase == 0) {
          applyPatternA();
          motorPhase = 1;
        } else {
          applyPatternB();
          motorPhase = 0;
        }
      }
    }

    // Handle a scheduled jump (single two-step cycle) without blocking
    if (jumpPending) {
      if (jumpStep == 0) {
        // first phase
          applyPatternA();
        jumpTimestamp = now;
        jumpStep = 1;
      } else if (jumpStep == 1) {
        if (now - jumpTimestamp >= MOTOR_INTERVAL) {
          // second phase
            applyPatternB();
          // complete jump after another interval
          jumpTimestamp = now;
          jumpStep = 2;
        }
      } else if (jumpStep == 2) {
        if (now - jumpTimestamp >= MOTOR_INTERVAL) {
          // finish: turn motors off
            stopMotor();
            // ensure additional pins are off
            digitalWrite(P17, LOW); digitalWrite(P19, LOW); digitalWrite(P18, LOW); digitalWrite(P23, LOW);
            digitalWrite(P32, LOW); digitalWrite(P33, LOW); digitalWrite(P25, LOW); digitalWrite(P26, LOW);
          jumpPending = false;
          jumpStep = 0;
        }
      }
    }
    // Update software PWM outputs
    pwmUpdate();
  }

// --- WebSocket event handler and action implementations ---
void setHue(int hue) {
  // Map 0..360 to 0..255 motorDuty for speed control
  int d = map(hue, 0, 360, 0, 255);
  if (d < 0) d = 0; if (d > 255) d = 255;
  motorDuty = d;
  Serial.printf("setHue(%d) -> motorDuty = %d (note: in digital-mode duty is not used)\n", hue, motorDuty);
}

void stop() {
  motorRunning = false;
  jumpPending = false; // Clear any pending jumps
  stopMotor();
  // also ensure additional pins off
  digitalWrite(P17, LOW); digitalWrite(P19, LOW); digitalWrite(P18, LOW); digitalWrite(P23, LOW);
  digitalWrite(P32, LOW); digitalWrite(P33, LOW); digitalWrite(P25, LOW); digitalWrite(P26, LOW);
  Serial.println("stop() -> all motors off (digital-mode)");
}

void start() {
  motorRunning = true;
  motorPhase = 0; // Reset phase
  motorLastMs = millis(); // Reset timing
  Serial.printf("start() -> continuous motion @ duty=%d, interval=%lu\n", motorDuty, MOTOR_INTERVAL);
}

void forwardBackwards(int FBValue) {
  // Manual immediate control in code.ino style: FBValue expected -100..100.
  // Positive -> apply pattern A (as a 'forward' block), Negative -> pattern B
  motorRunning = false; // joystick/manual overrides automatic pattern
  if (FBValue > 0) {
    applyPatternA();
    Serial.printf("forwardBackwards(%d) -> applied pattern A\n", FBValue);
  } else if (FBValue < 0) {
    applyPatternB();
    Serial.printf("forwardBackwards(%d) -> applied pattern B\n", FBValue);
  } else {
    stop();
  }
}
void leftRight(int LRValue) {
  // LRValue expected -100..100. Positive = left, Negative = right
  // Use single-pin control per user request:
  // left: M3 on, M4 off
  // right: M4 on, M3 off
  // forward/back motors (M1/M2) are turned off when doing LR control
  
  // Manual immediate turn control in code.ino style: use full patterns
  motorRunning = false; // joystick/manual overrides automatic pattern
  if (LRValue > 0) {
    // treat positive as pattern A
    applyPatternA();
    Serial.printf("leftRight(%d) -> applied pattern A (left)\n", LRValue);
    leftorright = 1;
  } else if (LRValue < 0) {
    // treat negative as pattern B
    applyPatternB();
    Serial.printf("leftRight(%d) -> applied pattern B (right)\n", LRValue);
    leftorright = 2;
  } else {
    stop();
  }
}


void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    String msg = String((char*)payload);
    Serial.print("WS RX: "); Serial.println(msg);

    int commaIndex = msg.indexOf(',');
    String cmd;
    String valStr;
    int value = 0;
    bool hasValue = false;
    if (commaIndex > 0) {
      cmd = msg.substring(0, commaIndex);
      valStr = msg.substring(commaIndex + 1);
      valStr.trim();
      if (valStr.length() > 0) { value = valStr.toInt(); hasValue = true; }
    } else {
      cmd = msg;
      cmd.trim();
    }

    // Handle WebSocket messages
    if (cmd == "hue" && hasValue) {
      setHue(value); // Control motor speed using hue slider (0-360 mapped to 0-255)
    } else if (cmd == "fb" && hasValue) {
      forwardBackwards(value); // Forward/backward control with joystick (-100 to 100)
    } else if (cmd == "lr" && hasValue) {
      leftRight(value); // Left/right control with joystick (-100 to 100)  
    } else if (cmd == "stop") {
      stop(); // Stop all motors
    } else if (cmd == "start") {
      start(); // Start continuous motion pattern
    } else {
      Serial.println("Unknown WS command: " + cmd);
    }

    // Send acknowledgment
    String resp = "{\"status\":\"ok\",\"cmd\":\"" + cmd + "\"";
    if (hasValue) resp += ",\"value\":" + String(value);
    resp += "}";
    webSocket.sendTXT(num, resp);
  } else if (type == WStype_CONNECTED) {
    Serial.printf("[WebSocket] Client #%u connected from %d.%d.%d.%d\n", num,
      webSocket.remoteIP(num)[0], webSocket.remoteIP(num)[1],
      webSocket.remoteIP(num)[2], webSocket.remoteIP(num)[3]);
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("[WebSocket] Client #%u disconnected\n", num);
    // Stop motors on disconnect for safety
    stop();
  }
}