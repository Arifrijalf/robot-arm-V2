/**
 * ============================================================================
 * PROYEK: Robot Arm 5-DOF Berbasis ESP32 (PlatformIO) - Kendali dari HP (WiFi)
 * FITUR:
 *   - Kendali murni dari handphone via WiFi (ESP32 Access Point + Web + WebSocket)
 *   - Halaman kontrol web ringan (virtual joystick sentuh, slider, gripper, home)
 *   - Gerakan halus (Butter-Smooth Motion) & Bebas Getaran (Zero Twitching)
 *   - Slew Rate Interpolator: Batas perubahan sudut bertahap per 15ms
 *   - Kontrol Gripper: dari HP, tombol GPIO 4, & Serial
 *   - Telemetri sudut real-time ke HP via WebSocket
 *
 * Cara pakai:
 *   1. Upload firmware ke ESP32.
 *   2. HP connect ke WiFi SSID "RobotArm-5DOF" (password "robotarm123").
 *   3. Buka browser: http://192.168.4.1
 *   4. (Opsional) Serial Monitor 115200 untuk debug.
 * ============================================================================
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// ============================================================================
// 1. KONFIGURASI WiFi ACCESS POINT
// ============================================================================
#define WIFI_AP_SSID     "RobotArm-5DOF"
#define WIFI_AP_PASSWORD "12345678"   // min 8 karakter; kosongkan untuk open AP
#define WIFI_AP_CHANNEL  1
#define WIFI_AP_HIDDEN   false
#define WIFI_AP_MAX_CONN 4

// --- WiFi Rumah (mode STA / client) ---
// ESP32 join WiFi ini dulu. Jika connect, HP cukup connect ke WiFi yang sama
// dan internet HP tetap hidup. Jika gagal, otomatis fallback ke Access Point.
#define WIFI_STA_SSID     "cip"
#define WIFI_STA_PASSWORD "syz4jumm"

// ============================================================================
// 2. PIN DEFINITIONS & MAPPING HARDWARE
// ============================================================================

// Pin Servo (5-DOF)
#define PIN_SERVO_GRIPPER   13
#define PIN_SERVO_WRIST     12
#define PIN_SERVO_ARM       14
#define PIN_SERVO_ELBOW     27
#define PIN_SERVO_BASE      26

// Pin Tombol Opsional untuk Buka/Tutup Gripper (Mode INPUT_PULLUP)
#define PIN_BUTTON_GRIPPER  4

// ============================================================================
// 3. PARAMETER TUNING SMOOTH MOTION
// ============================================================================

#define NUM_SERVOS          5

#define SERVO_LOOP_INTERVAL_MS 15
#define MAX_STEP_DEG           0.75f

#define ANGLE_MIN           0.0f
#define ANGLE_MAX           180.0f

// Kecepatan rate-control dari defleksi virtual joystick HP (-1..1 -> deg/loop)
#define JOY_SPEED_FACTOR    1.2f

#define GRIPPER_OPEN_ANGLE  100.0f
#define GRIPPER_CLOSE_ANGLE 20.0f

enum ServoID {
    INDEX_GRIPPER = 0,
    INDEX_WRIST   = 1,
    INDEX_ARM     = 2,
    INDEX_ELBOW   = 3,
    INDEX_BASE     = 4
};

// ============================================================================
// 4. VARIABEL GLOBAL
// ============================================================================

Servo servos[NUM_SERVOS];
const int servoPins[NUM_SERVOS] = {
    PIN_SERVO_GRIPPER,
    PIN_SERVO_WRIST,
    PIN_SERVO_ARM,
    PIN_SERVO_ELBOW,
    PIN_SERVO_BASE
};

float currentAngles[NUM_SERVOS] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f};
float targetAngles[NUM_SERVOS]  = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f};

unsigned long lastServoUpdate = 0;

bool gripperState = false; // false = Buka, true = Tutup
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY_MS = 50;

// Virtual joystick defleksi dari HP (-1..1)
float joyX1 = 0.0f, joyY1 = 0.0f;
float joyX2 = 0.0f, joyY2 = 0.0f;
bool  gripperCmdFromPhone = false;   // state gripper yang diminta HP
bool  gripperCmdChanged   = false;

unsigned long lastTelemetry = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 100;

WebServer server(80);
WebSocketsServer webSocket(81);

// ============================================================================
// 5. HALAMAN KONTROL (PROGMEM) - ringan, mobile-first, virtual joystick
// ============================================================================
static const char CONTROL_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="id">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<meta name="theme-color" content="#f4f4f4">
<title>Kontrol Robot Arm 5-DOF</title>
<style>
* { box-sizing:border-box; margin:0; padding:0; -webkit-tap-highlight-color:transparent; }
html { -webkit-text-size-adjust:100%; }
body { font-family:Arial,sans-serif; background:#f4f4f4; color:#1a1a1a; font-size:14px; padding:10px 10px 24px; }
.bar { display:flex; justify-content:space-between; align-items:center; background:#fff; border:1px solid #ccc; border-radius:6px; padding:10px 12px; }
.bar h1 { font-size:15px; }
.bar span { font-size:12px; color:#777; }
.dot { display:inline-block; width:9px; height:9px; border-radius:50%; background:#bbb; margin-right:5px; vertical-align:middle; }
.dot.on { background:#2e9e44; }
.panel { background:#fff; border:1px solid #ccc; border-radius:6px; padding:10px 12px; margin-top:10px; }
.panel h2 { font-size:11px; text-transform:uppercase; letter-spacing:.5px; color:#777; margin-bottom:8px; }
.joys { display:flex; justify-content:space-around; }
.joy { text-align:center; }
.pad { position:relative; width:130px; height:130px; background:#fafafa; border:2px solid #aaa; border-radius:50%; touch-action:none; user-select:none; -webkit-user-select:none; margin:0 auto; }
.pad .stick { position:absolute; left:50%; top:50%; width:40px; height:40px; margin:-20px 0 0 -20px; background:#3a7bd5; border-radius:50%; }
.joy p { font-size:11px; color:#666; margin-top:6px; }
button { -webkit-appearance:none; appearance:none; display:block; width:100%; margin-top:10px; padding:12px; font-size:14px; font-weight:bold; font-family:inherit; color:#1a1a1a; background:#fff; border:1px solid #999; border-radius:6px; }
button:active { background:#e6e6e6; }
.row { display:flex; gap:8px; }
.row button { margin-top:0; }
#gripBtn { flex:1; }
#openBtn { flex:0 0 34%; }
.btn-primary { background:#3a7bd5; border-color:#2f68b5; color:#fff; }
.btn-primary:active { background:#2f68b5; }
.btn-grip-on { background:#2e9e44; border-color:#27853a; color:#fff; }
.btn-grip-on:active { background:#27853a; }
.sl { display:flex; align-items:center; gap:10px; margin-top:8px; }
.sl label { flex:0 0 52px; font-size:12px; color:#555; }
.sl input[type=range] { flex:1; }
.sl .val { flex:0 0 40px; text-align:right; font-variant-numeric:tabular-nums; font-size:12px; color:#333; }
table { width:100%; border-collapse:collapse; font-size:12px; font-variant-numeric:tabular-nums; }
th, td { text-align:left; padding:4px 2px; border-bottom:1px solid #eee; }
th { color:#777; font-weight:normal; }
th.num, td.num { text-align:right; }
tr:last-child td { border-bottom:none; }
</style>
</head>
<body>
<div class="bar">
  <h1>Robot Arm 5-DOF</h1>
  <span><span class="dot" id="dot"></span><span id="st">Menyambung...</span></span>
</div>

<div class="panel">
  <h2>Joystick Virtual</h2>
  <div class="joys">
    <div class="joy">
      <div class="pad" id="j1pad"><div class="stick" id="j1stick"></div></div>
      <p>Joy 1: Base (kiri-kanan) / Arm (atas-bawah)</p>
    </div>
    <div class="joy">
      <div class="pad" id="j2pad"><div class="stick" id="j2stick"></div></div>
      <p>Joy 2: Elbow / Wrist</p>
    </div>
  </div>
</div>

<div class="panel">
  <h2>Gripper</h2>
  <div class="row">
    <button class="btn-primary" id="gripBtn">GRIPPER: BUKA</button>
    <button id="openBtn">BUKA</button>
  </div>
</div>

<div class="panel">
  <h2>Sudut Manual (derajat)</h2>
  <div class="sl"><label>Base</label><input type="range" id="sl-b" min="0" max="180" step="1" value="90"><span class="val" id="sl-bv">90&deg;</span></div>
  <div class="sl"><label>Arm</label><input type="range" id="sl-a" min="0" max="180" step="1" value="90"><span class="val" id="sl-av">90&deg;</span></div>
  <div class="sl"><label>Elbow</label><input type="range" id="sl-e" min="0" max="180" step="1" value="90"><span class="val" id="sl-ev">90&deg;</span></div>
  <div class="sl"><label>Wrist</label><input type="range" id="sl-w" min="0" max="180" step="1" value="90"><span class="val" id="sl-wv">90&deg;</span></div>
  <div class="sl"><label>Gripper</label><input type="range" id="sl-g" min="0" max="180" step="1" value="100"><span class="val" id="sl-gv">100&deg;</span></div>
</div>

<div class="panel">
  <h2>Target Sudut (telemetri)</h2>
  <table>
    <tr><th>Sendi</th><th class="num">Target</th></tr>
    <tr><td>Base</td><td class="num" id="tb">90.0</td></tr>
    <tr><td>Arm</td><td class="num" id="ta">90.0</td></tr>
    <tr><td>Elbow</td><td class="num" id="te">90.0</td></tr>
    <tr><td>Wrist</td><td class="num" id="tw">90.0</td></tr>
    <tr><td>Gripper</td><td class="num" id="tg">100.0</td></tr>
  </table>
</div>

<button id="homeBtn">HOME</button>

<script>
const dot=document.getElementById('dot'), st=document.getElementById('st');
const gripBtn=document.getElementById('gripBtn');
const j1={x:0,y:0}, j2={x:0,y:0};
let grip=false; // false=buka, true=tutup
let ws=null, timer=null, lastPayload='';
const pend={};

function setConn(ok){
  dot.classList.toggle('on', ok);
  st.textContent = ok ? 'Terhubung' : 'Terputus';
}
function renderGrip(){
  gripBtn.textContent = 'GRIPPER: ' + (grip ? 'TUTUP' : 'BUKA');
  gripBtn.classList.toggle('btn-grip-on', grip);
  gripBtn.classList.toggle('btn-primary', !grip);
}

function connect(){
  try {
    ws = new WebSocket('ws://' + location.hostname + ':81/');
    ws.onopen = () => { setConn(true); lastPayload=''; if(timer){ clearInterval(timer); timer=null; } };
    ws.onclose = () => { setConn(false); if(!timer) timer = setInterval(connect, 1500); };
    ws.onerror = () => { try { ws.close(); } catch(e) {} };
    ws.onmessage = (ev) => {
      const p = ev.data.split(',');
      if (p.length >= 6 && p[0] === 'S') {
        document.getElementById('tb').textContent = parseFloat(p[1]).toFixed(1);
        document.getElementById('ta').textContent = parseFloat(p[2]).toFixed(1);
        document.getElementById('te').textContent = parseFloat(p[3]).toFixed(1);
        document.getElementById('tw').textContent = parseFloat(p[4]).toFixed(1);
        document.getElementById('tg').textContent = parseFloat(p[5]).toFixed(1);
      }
    };
  } catch(e) { setConn(false); }
}
connect();

function send(){
  if (!ws || ws.readyState !== 1) return;
  for (const i in pend) { ws.send('A,' + i + ',' + pend[i]); delete pend[i]; }
  const s = j1.x.toFixed(3) + ',' + j1.y.toFixed(3) + ',' + j2.x.toFixed(3) + ',' + j2.y.toFixed(3) + ',' + (grip ? 1 : 0);
  if (s !== lastPayload) { lastPayload = s; ws.send(s); }
}
setInterval(send, 20);

// Joystick: Pointer Events + setPointerCapture (andal di Chrome Android & iOS)
function setupJoystick(pad, stick, obj){
  let active = -1;
  function update(ev){
    const rect = pad.getBoundingClientRect();
    const cx = rect.left + rect.width / 2, cy = rect.top + rect.height / 2;
    const rad = rect.width / 2 - 14;
    let dx = ev.clientX - cx, dy = ev.clientY - cy;
    const d = Math.hypot(dx, dy);
    if (d > rad) { dx = dx / d * rad; dy = dy / d * rad; }
    stick.style.transform = 'translate(' + dx + 'px,' + dy + 'px)';
    obj.x = Math.abs(dx / rad) < 0.15 ? 0 : dx / rad;
    obj.y = Math.abs(dy / rad) < 0.15 ? 0 : -(dy / rad);
  }
  function release(){
    if (active === -1) return;
    active = -1;
    stick.style.transform = 'translate(0px,0px)';
    obj.x = 0; obj.y = 0;
  }
  pad.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    try { pad.setPointerCapture(e.pointerId); } catch(err) {}
    active = e.pointerId;
    update(e);
  });
  pad.addEventListener('pointermove', (e) => { if (e.pointerId === active) update(e); });
  pad.addEventListener('pointerup', (e) => { if (e.pointerId === active) release(); });
  pad.addEventListener('pointercancel', (e) => { if (e.pointerId === active) release(); });
  pad.addEventListener('contextmenu', (e) => e.preventDefault());
}
setupJoystick(document.getElementById('j1pad'), document.getElementById('j1stick'), j1);
setupJoystick(document.getElementById('j2pad'), document.getElementById('j2stick'), j2);

// Slider manual: index servo 0=Gripper, 1=Wrist, 2=Arm, 3=Elbow, 4=Base
// Gripper: BUKA=100 deg, TUTUP=20 deg (tengah = 60 deg)
const sliders = [['sl-b','4'],['sl-a','2'],['sl-e','3'],['sl-w','1'],['sl-g','0']];
sliders.forEach(([id, idx]) => {
  const el = document.getElementById(id), val = document.getElementById(id + 'v');
  el.addEventListener('input', () => {
    val.textContent = el.value + '\u00B0';
    if (id === 'sl-g') { grip = (+el.value) < 60; renderGrip(); }
    pend[idx] = +el.value;
  });
});
function setSlider(id, v){
  document.getElementById(id).value = v;
  document.getElementById(id + 'v').textContent = v + '\u00B0';
}

gripBtn.addEventListener('click', () => {
  grip = !grip;
  setSlider('sl-g', grip ? 20 : 100);
  pend['0'] = grip ? 20 : 100;
  renderGrip();
});
document.getElementById('openBtn').addEventListener('click', () => {
  grip = false; renderGrip();
  setSlider('sl-g', 100); pend['0'] = 100;
  if (ws && ws.readyState === 1) ws.send('O');
});
document.getElementById('homeBtn').addEventListener('click', () => {
  ['sl-b','sl-a','sl-e','sl-w'].forEach((id) => setSlider(id, 90));
  setSlider('sl-g', 100);
  grip = false; renderGrip();
  if (ws && ws.readyState === 1) ws.send('H');
});
</script>
</body>
</html>
)HTML";

// ============================================================================
// 6. DEKLARASI FUNGSI
// ============================================================================
void initServos();
void updateServosSmooth();
void handleGripperButton();
void handleSerialCommands();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void broadcastTelemetry();

// ============================================================================
// 7. SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n==================================================");
    Serial.println("  ROBOT ARM 5-DOF - KENDALI DARI HP (WiFi AP)    ");
    Serial.println("==================================================");

    pinMode(PIN_BUTTON_GRIPPER, INPUT_PULLUP);

    initServos();

    bool staOk = false;
    // --- WiFi: coba mode STA (join WiFi rumah) dulu, fallback ke AP ---
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASSWORD);
    Serial.printf("[WiFi] Menghubungkan ke '%s'", WIFI_STA_SSID);
    unsigned long wifiT0 = millis();
    while (millis() - wifiT0 < 10000UL) {
        if (WiFi.status() == WL_CONNECTED) { staOk = true; break; }
        delay(250);
        Serial.print('.');
    }
    if (staOk) {
        Serial.printf(" OK (STA) -> IP: %s\n", WiFi.localIP().toString().c_str());
        if (MDNS.begin("robotarm")) MDNS.addService("http", "tcp", 80);
        Serial.println("[WiFi] Halaman kontrol juga bisa dibuka via: http://robotarm.local");
        WiFi.setSleep(false); // matikan modem sleep agar respons WebSocket tetap kencang
    } else {
        Serial.println(" GAGAL -> fallback ke mode Access Point");
        WiFi.disconnect();
        delay(100);
        WiFi.mode(WIFI_AP);
        bool apOk = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL, WIFI_AP_HIDDEN, WIFI_AP_MAX_CONN);
        delay(100);
        IPAddress ip = WiFi.softAPIP();
        Serial.printf("[WiFi] AP %s %s -> IP: %s\n", WIFI_AP_SSID, apOk ? "OK" : "GAGAL", ip.toString().c_str());
    }

    // --- Web Server ---
    server.on("/", HTTP_GET, []() {
        server.send_P(200, "text/html", CONTROL_PAGE);
    });
    server.onNotFound([]() {
        server.send(404, "text/plain", "Not found");
    });
    server.begin();
    Serial.println("[HTTP] Web server port 80 siap.");

    // --- WebSocket ---
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("[WS] WebSocket port 81 siap.");

    Serial.println("\n--- PANDUAN ---");
    if (staOk) {
        Serial.println("1. HP connect ke WiFi rumah '" WIFI_STA_SSID "' (jaringan yang SAMA dengan ESP32)");
        Serial.println("2. Buka browser: http://robotarm.local (atau lihat IP di atas)");
        Serial.println("3. Serial: 'o'=buka, 'g'=tutup, 'h'=home");
    } else {
        Serial.println("1. HP connect ke WiFi: " WIFI_AP_SSID " (pass: " WIFI_AP_PASSWORD ")");
        Serial.println("2. Buka browser: http://192.168.4.1");
        Serial.println("3. Serial: 'o'=buka, 'g'=tutup, 'h'=home");
    }
    Serial.println("----------------\n");
}

// ============================================================================
// 8. MAIN LOOP (NON-BLOCKING)
// ============================================================================
void loop() {
    server.handleClient();
    webSocket.loop();

    handleGripperButton();
    handleSerialCommands();

    unsigned long currentMillis = millis();
    if (currentMillis - lastServoUpdate >= SERVO_LOOP_INTERVAL_MS) {
        lastServoUpdate = currentMillis;

        // Rate control: integrasikan defleksi virtual joystick HP ke target sudut
        if (fabsf(joyX1) > 0.001f) {
            targetAngles[INDEX_BASE] = constrain(targetAngles[INDEX_BASE] + joyX1 * JOY_SPEED_FACTOR, ANGLE_MIN, ANGLE_MAX);
        }
        if (fabsf(joyY1) > 0.001f) {
            targetAngles[INDEX_ARM] = constrain(targetAngles[INDEX_ARM] + joyY1 * JOY_SPEED_FACTOR, ANGLE_MIN, ANGLE_MAX);
        }
        if (fabsf(joyX2) > 0.001f) {
            targetAngles[INDEX_ELBOW] = constrain(targetAngles[INDEX_ELBOW] + joyX2 * JOY_SPEED_FACTOR, ANGLE_MIN, ANGLE_MAX);
        }
        if (fabsf(joyY2) > 0.001f) {
            targetAngles[INDEX_WRIST] = constrain(targetAngles[INDEX_WRIST] + joyY2 * JOY_SPEED_FACTOR, ANGLE_MIN, ANGLE_MAX);
        }

        // Sinkronkan gripper sesuai perintah terakhir dari HP
        if (gripperCmdChanged) {
            gripperCmdChanged = false;
            gripperState = gripperCmdFromPhone;
            targetAngles[INDEX_GRIPPER] = gripperState ? GRIPPER_CLOSE_ANGLE : GRIPPER_OPEN_ANGLE;
        }

        updateServosSmooth();

        if (currentMillis - lastTelemetry >= TELEMETRY_INTERVAL_MS) {
            lastTelemetry = currentMillis;
            broadcastTelemetry();
        }
    }
}

// ============================================================================
// 9. INISIALISASI & SMOOTH INTERPOLATION SERVO
// ============================================================================
void initServos() {
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // Gripper mulai dari posisi BUKA (100 deg) agar konsisten dengan
    // assume state halaman kontrol di HP (grip=false -> BUKA).
    targetAngles[INDEX_GRIPPER] = GRIPPER_OPEN_ANGLE;
    gripperState = false;
    gripperCmdFromPhone = false;

    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].setPeriodHertz(50);
        servos[i].attach(servoPins[i], 500, 2400);
        servos[i].write((int)lroundf(currentAngles[i]));
    }
    Serial.println("[SERVO] Semua servo siap pada posisi awal 90 derajat.");
}

void updateServosSmooth() {
    for (int i = 0; i < NUM_SERVOS; i++) {
        float diff = targetAngles[i] - currentAngles[i];
        if (fabsf(diff) > 0.05f) {
            if (diff > 0.0f) {
                currentAngles[i] += min(diff, MAX_STEP_DEG);
            } else {
                currentAngles[i] -= min(-diff, MAX_STEP_DEG);
            }
            servos[i].write((int)lroundf(currentAngles[i]));
        }
    }
}

// ============================================================================
// 10. WEBSOCKET HANDLING
// ============================================================================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_DISCONNECTED) {
        Serial.printf("[WS] Client #%u disconnected\n", num);
        if (webSocket.connectedClients() == 0) {
            joyX1 = joyY1 = joyX2 = joyY2 = 0.0f;
            Serial.println("[WS] Tidak ada client, joystick di-reset.");
        }
    } else if (type == WStype_CONNECTED) {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[WS] Client #%u connected from %s\n", num, ip.toString().c_str());
    } else if (type == WStype_TEXT) {
        // Simpan payload ke buffer null-terminated
        char buf[128];
        size_t n = length < sizeof(buf) - 1 ? length : sizeof(buf) - 1;
        memcpy(buf, payload, n);
        buf[n] = '\0';

        // Perintah single-char
        if (n == 1) {
            char c = buf[0];
            if (c == 'H' || c == 'h') {
                for (int i = 0; i < NUM_SERVOS; i++) targetAngles[i] = 90.0f;
                targetAngles[INDEX_GRIPPER] = GRIPPER_OPEN_ANGLE;
                gripperState = false;
                gripperCmdFromPhone = false;
                Serial.println("[WS] Home");
                return;
            } else if (c == 'O' || c == 'o') {
                gripperCmdFromPhone = false;
                gripperCmdChanged = true;
                Serial.println("[WS] Buka Gripper");
                return;
            } else if (c == 'C' || c == 'c' || c == 'G' || c == 'g') {
                gripperCmdFromPhone = true;
                gripperCmdChanged = true;
                Serial.println("[WS] Tutup Gripper");
                return;
            }
        }

        // Perintah sudut manual (slider HP): "A,<index>,<sudut>"
        // index: 0=Gripper, 1=Wrist, 2=Arm, 3=Elbow, 4=Base (urutan ServoID)
        int idx; float ang;
        if (sscanf(buf, "A,%d,%f", &idx, &ang) == 2) {
            if (idx >= 0 && idx < NUM_SERVOS) {
                targetAngles[idx] = constrain(ang, ANGLE_MIN, ANGLE_MAX);
                if (idx == INDEX_GRIPPER) {
                    gripperState        = (targetAngles[idx] < 60.0f); // BUKA=100, TUTUP=20
                    gripperCmdFromPhone = gripperState;
                }
                Serial.printf("[WS] Manual: joint %d -> %.1f deg\n", idx, targetAngles[idx]);
            }
            return;
        }

        // Format rate-control: "x1,y1,x2,y2,g"
        float x1, y1, x2, y2;
        int g;
        if (sscanf(buf, "%f,%f,%f,%f,%d", &x1, &y1, &x2, &y2, &g) == 5) {
            joyX1 = constrain(x1, -1.0f, 1.0f);
            joyY1 = constrain(y1, -1.0f, 1.0f);
            joyX2 = constrain(x2, -1.0f, 1.0f);
            joyY2 = constrain(y2, -1.0f, 1.0f);

            bool newGrip = (g != 0);
            if (newGrip != gripperCmdFromPhone) {
                gripperCmdFromPhone = newGrip;
                gripperCmdChanged = true;
            }
        }
    }
}

void broadcastTelemetry() {
    if (webSocket.connectedClients() == 0) return; // hemat CPU: tidak ada HP terhubung -> tidak kirim apa-apa
    char msg[96];
    snprintf(msg, sizeof(msg), "S,%.1f,%.1f,%.1f,%.1f,%.1f",
             targetAngles[INDEX_BASE], targetAngles[INDEX_ARM],
             targetAngles[INDEX_ELBOW], targetAngles[INDEX_WRIST],
             targetAngles[INDEX_GRIPPER]);
    webSocket.broadcastTXT(msg);
}

// ============================================================================
// 11. KONTROL GRIPPER (TOMBOL GPIO 4 & SERIAL)
// ============================================================================
void handleGripperButton() {
    int reading = digitalRead(PIN_BUTTON_GRIPPER);

    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY_MS) {
        static int debouncedState = HIGH;
        if (reading != debouncedState) {
            debouncedState = reading;
            if (debouncedState == LOW) {
                gripperState = !gripperState;
                gripperCmdFromPhone = gripperState;
                targetAngles[INDEX_GRIPPER] = gripperState ? GRIPPER_CLOSE_ANGLE : GRIPPER_OPEN_ANGLE;
                Serial.println(gripperState ? "[GRIPPER] Menutup (Close)" : "[GRIPPER] Membuka (Open)");
            }
        }
    }
    lastButtonState = reading;
}

void handleSerialCommands() {
    if (!Serial.available()) return;

    char cmd = Serial.read();
    if (cmd == '\r' || cmd == '\n' || cmd == ' ') return;

    switch (cmd) {
        case 'o':
        case 'O':
            targetAngles[INDEX_GRIPPER] = GRIPPER_OPEN_ANGLE;
            gripperState = false;
            gripperCmdFromPhone = false;
            Serial.println("[SERIAL] Membuka Gripper");
            break;

        case 'g':
        case 'G':
            targetAngles[INDEX_GRIPPER] = GRIPPER_CLOSE_ANGLE;
            gripperState = true;
            gripperCmdFromPhone = true;
            Serial.println("[SERIAL] Menutup Gripper");
            break;

        case 'h':
        case 'H':
            for (int i = 0; i < NUM_SERVOS; i++) targetAngles[i] = 90.0f;
            targetAngles[INDEX_GRIPPER] = GRIPPER_OPEN_ANGLE;
            gripperState = false;
            gripperCmdFromPhone = false;
            Serial.println("[SERIAL] Mengembalikan semua servo ke posisi Home (90 deg)");
            break;

        default:
            Serial.printf("[INFO] Perintah '%c' tidak dikenali. Gunakan 'o', 'g', atau 'h'.\n", cmd);
            break;
    }
}