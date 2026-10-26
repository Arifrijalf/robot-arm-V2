/**
 * ============================================================================
 * PROYEK: Robot Arm 5-DOF Berbasis ESP32 (PlatformIO) - Kendali dari HP (WiFi)
 * FITUR:
 *   - 2 MODE KENDALI: MANUAL (joystick/slider dari HP) & OTOMATIS (pick&place)
 *   - Mode otomatis: 2 titik pose (A=pick, B=place) diajarkan (teach) dari HP,
 *     tersimpan permanen di NVS (tahan mati listrik)
 *   - Sensor inframerah FC-51 (GPIO 34) mendeteksi barang -> mulai siklus:
 *     Home -> A -> gripper tutup -> B -> gripper buka -> Home -> tunggu lagi
 *   - Kendali murni dari handphone via WiFi (AP + Web + WebSocket)
 *   - VISUAL 3D REAL-TIME (three.js dari flash ESP32, ikut bergerak bersama
 *     lengan asli), slider pasif mengikuti posisi robot
 *   - Gerakan halus (Butter-Smooth Motion) & Bebas Getaran (Zero Twitching)
 *   - Slew Rate Interpolator: Batas perubahan sudut bertahap per 15ms
 *   - Tombol GPIO 4: toggle gripper (manual) / E-STOP (saat otomatis jalan)
 *   - Telemetri sudut real-time ke HP via WebSocket (50 ms, currentAngles)
 *
 * Cara pakai:
 *   1. Upload firmware ke ESP32.
 *   2. HP connect ke WiFi (jaringan yang sama dengan ESP32 atau hotspot AP).
 *   3. Buka halaman kontrol -> pilih mode MANUAL, gerakkan lengan, tekan
 *      "Simpan Titik A" (posisi pick) & "Simpan Titik B" (posisi place).
 *   4. Ganti ke OTOMATIS -> MULAI. Lengan standby di Home; saat sensor IR
 *      mendeteksi barang, siklus pick & place berjalan sendiri.
 *   5. (Opsional) Serial Monitor 115200: m/a=mode, r=mulai, x=stop, p=info.
 * ============================================================================
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "three_js_gz.h"
#include "glb_stl_gz.h"

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
#define WIFI_STA_SSID     "naziefn"
#define WIFI_STA_PASSWORD "plmoknijb"

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
// Saat mode otomatis sedang berjalan, tombol ini berfungsi sebagai E-STOP.
#define PIN_BUTTON_GRIPPER  4

// Pin Sensor Inframerah FC-51 (output digital DO modul)
// GPIO 34 = input-only, aman dipakai bersama WiFi (tidak dipakai servo).
// Catatan: GPIO 34 tidak punya pull-up internal -> modul FC-51 umumnya sudah
// punya pull-up onboard; jika DO melayang, pasang resistor 10K ke 3.3V.
#define PIN_IR_SENSOR       34
#define IR_ACTIVE_LEVEL     LOW      // FC-51: LOW = barang terdeteksi
#define IR_DEBOUNCE_COUNT   3        // 3 x 15ms = 45ms konfirmasi stabil

// ============================================================================
// 3. PARAMETER TUNING SMOOTH MOTION & MODE OTOMATIS
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

// --- Mode otomatis (pick & place) ---
#define AUTO_MOVE_SETTLE_MS  300    // jeda ekstra setelah lengan sampai tujuan
#define AUTO_GRIP_SETTLE_MS  600    // jeda ekstra setelah gripper buka/tutup
#define MOTION_DONE_TOL      0.10f  // toleransi sudut dianggap "sampai" (deg)

// Telemetri ke HP (currentAngles) -> visual 3D & slider real-time
#define TELEMETRY_INTERVAL_MS 50

enum ServoID {
    INDEX_GRIPPER = 0,
    INDEX_WRIST   = 1,
    INDEX_ARM     = 2,
    INDEX_ELBOW   = 3,
    INDEX_BASE    = 4
};

// Mode kendali
enum CtrlMode {
    MODE_MANUAL = 0,
    MODE_AUTO   = 1
};

// State machine mode otomatis ( kode dikirim via telemetri ke HP )
enum AutoState {
    AS_IDLE   = 0,   // tidak menjalankan siklus
    AS_WAIT   = 1,   // standby di Home, menunggu IR mendeteksi barang
    AS_GO_A   = 2,   // menuju titik A (pick)
    AS_GRIP   = 3,   // gripper menutup
    AS_GO_B   = 4,   // menuju titik B (place)
    AS_RELEASE= 5,   // gripper membuka
    AS_HOME   = 6,   // kembali ke Home (standby / awal siklus)
    AS_HOME_TO_B = 7 // transit Home sambil membawa barang, lanjut Titik B
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

// --- Mode & otomatis ---
CtrlMode ctrlMode    = MODE_MANUAL;
AutoState autoState  = AS_IDLE;
unsigned long autoStateT0 = 0;

// Pose titik A (pick) & B (place) hasil teach dari HP, dipersist ke NVS
float poseA[NUM_SERVOS] = {90.0f, 90.0f, 90.0f, 90.0f, GRIPPER_OPEN_ANGLE};
float poseB[NUM_SERVOS] = {90.0f, 90.0f, 90.0f, 90.0f, GRIPPER_OPEN_ANGLE};
bool  hasPoseA = false;
bool  hasPoseB = false;
Preferences prefs;

// --- Sensor IR ---
int  irHoldCount = 0;
bool irPhysical  = false;   // true = FC-51 mendeteksi barang (sudah didebounce)
bool irSimulated = true;    // default simulasi: GPIO 34 boleh belum tersambung
bool irSimValue  = false;   // nilai sensor virtual dari tombol web

bool irDetected() {
    return irSimulated ? irSimValue : irPhysical;
}

unsigned long lastTelemetry = 0;

WebServer server(80);
WebSocketsServer webSocket(81);

// ============================================================================
// 5. HALAMAN KONTROL (PROGMEM) - mobile-first + visual 3D + mode otomatis
// ============================================================================
static const char CONTROL_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="id">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<meta name="theme-color" content="#14181c">
<title>Kontrol Robot Arm 5-DOF</title>
<style>
* { box-sizing:border-box; margin:0; padding:0; -webkit-tap-highlight-color:transparent; }
html { -webkit-text-size-adjust:100%; }
body { font-family:Arial,sans-serif; background:#14181c; color:#e8eaed; font-size:14px; padding:10px 10px 24px; }
.bar { display:flex; justify-content:space-between; align-items:center; background:#1e242b; border:1px solid #333a42; border-radius:6px; padding:10px 12px; }
.bar h1 { font-size:15px; color:#fff; }
.bar span { font-size:12px; color:#8a94a3; }
.dot { display:inline-block; width:9px; height:9px; border-radius:50%; background:#666; margin-right:5px; vertical-align:middle; }
.dot.on { background:#2e9e44; }
.panel { background:#1e242b; border:1px solid #333a42; border-radius:6px; padding:10px 12px; margin-top:10px; }
.panel h2 { font-size:11px; text-transform:uppercase; letter-spacing:.5px; color:#8a94a3; margin-bottom:8px; }
#c3dwrap { height:42vh; min-height:220px; border-radius:6px; background:#0e1114; border:1px solid #2a323c; touch-action:none; overflow:hidden; position:relative; }
#c3d { width:100%; height:100%; display:block; }
.noload { color:#8a94a3; font-size:12px; padding:20px 10px; text-align:center; }
.joys { display:flex; justify-content:space-around; }
.joy { text-align:center; }
.pad { position:relative; width:130px; height:130px; background:#171c21; border:2px solid #444c56; border-radius:50%; touch-action:none; user-select:none; -webkit-user-select:none; margin:0 auto; }
.pad .stick { position:absolute; left:50%; top:50%; width:40px; height:40px; margin:-20px 0 0 -20px; background:#3a7bd5; border-radius:50%; }
.joy p { font-size:11px; color:#8a94a3; margin-top:6px; }
button { -webkit-appearance:none; appearance:none; display:block; width:100%; margin-top:10px; padding:12px; font-size:14px; font-weight:bold; font-family:inherit; color:#e8eaed; background:#2a323c; border:1px solid #444c56; border-radius:6px; }
button:active { background:#232a32; }
button:disabled { opacity:.35; }
.row { display:flex; gap:8px; }
.row button { margin-top:0; }
#gripBtn { flex:1; }
#openBtn { flex:0 0 34%; }
.btn-primary { background:#3a7bd5; border-color:#2f68b5; color:#fff; }
.btn-primary:active { background:#2f68b5; }
.btn-grip-on { background:#2e9e44; border-color:#27853a; color:#fff; }
.btn-grip-on:active { background:#27853a; }
#stopBtn { border-color:#8f3a3a; color:#ff9d9d; }
.modebtn.active { background:#3a7bd5; border-color:#2f68b5; color:#fff; }
.locked { opacity:.4; pointer-events:none; }
#autoStatus { font-size:13px; color:#e8eaed; margin-bottom:2px; min-height:18px; }
#autoStatus.run { color:#9fe6b3; }
.pill { display:inline-block; padding:3px 9px; margin:3px 6px 0 0; border-radius:10px; font-size:11px; background:#232a32; border:1px solid #444c56; color:#aab3bd; }
.pill.ok { background:#12301b; border-color:#2e9e44; color:#9fe6b3; }
.pill.warn { background:#3a2412; border-color:#d58b3a; color:#ffd9a8; }
.sl { display:flex; align-items:center; gap:10px; margin-top:8px; }
.sl label { flex:0 0 52px; font-size:12px; color:#aab3bd; }
.sl input[type=range] { flex:1; }
.sl .val { flex:0 0 40px; text-align:right; font-variant-numeric:tabular-nums; font-size:12px; color:#e8eaed; }
table { width:100%; border-collapse:collapse; font-size:12px; font-variant-numeric:tabular-nums; }
th, td { text-align:left; padding:4px 2px; border-bottom:1px solid #2a323c; }
th { color:#8a94a3; font-weight:normal; }
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
  <h2>Visual 3D Real-Time</h2>
  <div id="c3dwrap"><canvas id="c3d"></canvas></div>
</div>

<div class="panel">
  <h2>Mode Kendali</h2>
  <div class="row">
    <button id="modeMan" class="modebtn active">MANUAL</button>
    <button id="modeAuto" class="modebtn">OTOMATIS</button>
  </div>
</div>

<div class="panel">
  <h2>Otomatis (Pick &amp; Place)</h2>
  <p id="autoStatus">Mode manual &mdash; atur lengan lalu simpan Titik A &amp; B.</p>
  <div class="row">
    <button id="saveA">Simpan Titik A</button>
    <button id="saveB">Simpan Titik B</button>
  </div>
  <div class="row" style="margin-top:8px">
    <button class="btn-primary" id="runBtn" disabled>MULAI</button>
    <button id="stopBtn" disabled>BERHENTI</button>
  </div>
  <div style="margin-top:10px">
    <span class="pill" id="pillA">Titik A: -</span>
    <span class="pill" id="pillB">Titik B: -</span>
    <span class="pill" id="pillIR">IR simulasi: bersih</span>
  </div>
  <div class="row" style="margin-top:8px">
    <button id="irDetectBtn">SIMULASI: TERDETEKSI</button>
    <button id="irClearBtn">SIMULASI: BERSIH</button>
  </div>
</div>

<div class="panel" id="panelJoy">
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

<div class="panel" id="panelGrip">
  <h2>Gripper</h2>
  <div class="row">
    <button class="btn-primary" id="gripBtn">GRIPPER: BUKA</button>
    <button id="openBtn">BUKA</button>
  </div>
</div>

<div class="panel" id="panelSl">
  <h2>Sudut Manual (derajat)</h2>
  <div class="sl"><label>Base</label><input type="range" id="sl-b" min="0" max="180" step="1" value="90"><span class="val" id="sl-bv">90&deg;</span></div>
  <div class="sl"><label>Arm</label><input type="range" id="sl-a" min="0" max="180" step="1" value="90"><span class="val" id="sl-av">90&deg;</span></div>
  <div class="sl"><label>Elbow</label><input type="range" id="sl-e" min="0" max="180" step="1" value="90"><span class="val" id="sl-ev">90&deg;</span></div>
  <div class="sl"><label>Wrist</label><input type="range" id="sl-w" min="0" max="180" step="1" value="90"><span class="val" id="sl-wv">90&deg;</span></div>
  <div class="sl"><label>Gripper</label><input type="range" id="sl-g" min="0" max="180" step="1" value="100"><span class="val" id="sl-gv">100&deg;</span></div>
</div>

<div class="panel">
  <h2>Posisi Lengan (real-time)</h2>
  <table>
    <tr><th>Sendi</th><th class="num">Sudut</th></tr>
    <tr><td>Base</td><td class="num" id="tb">90.0</td></tr>
    <tr><td>Arm</td><td class="num" id="ta">90.0</td></tr>
    <tr><td>Elbow</td><td class="num" id="te">90.0</td></tr>
    <tr><td>Wrist</td><td class="num" id="tw">90.0</td></tr>
    <tr><td>Gripper</td><td class="num" id="tg">100.0</td></tr>
  </table>
</div>

<button id="homeBtn">HOME</button>

<script src="/three.min.js"></script>
<script>
const dot=document.getElementById('dot'), st=document.getElementById('st');
const gripBtn=document.getElementById('gripBtn');
const j1={x:0,y:0}, j2={x:0,y:0};
let grip=false; // false=buka, true=tutup
let ws=null, timer=null, lastPayload='';
const pend={};

// --- Status dari telemetri firmware ---
const tgt=[100,90,90,90,90];   // sudut terkirim (currentAngles firmware)
const disp=[100,90,90,90,90];   // sudut tampilan (di-lerp tiap frame)
let modeVal=0, autoStateVal=0, irVal=0, irSimulated=true, hasA=false, hasB=false;
const runState={0:'Idle',1:'Menunggu barang...',2:'Menuju Titik A',3:'Menggenggam barang',4:'Menuju Titik B',5:'Melepaskan barang',6:'Kembali ke Home',7:'Transit Home membawa barang'};

function setConn(ok){
  dot.classList.toggle('on', ok);
  st.textContent = ok ? 'Terhubung' : 'Terputus';
}
function renderGrip(){
  gripBtn.textContent = 'GRIPPER: ' + (grip ? 'TUTUP' : 'BUKA');
  gripBtn.classList.toggle('btn-grip-on', grip);
  gripBtn.classList.toggle('btn-primary', !grip);
}

function autoStatusText(){
  const el=document.getElementById('autoStatus');
  if (modeVal===0) {
    el.classList.remove('run');
    el.textContent = 'Mode manual \u2014 atur lengan lalu simpan Titik A & B, lalu pindah ke OTOMATIS.';
    return;
  }
  if (autoStateVal===0) { el.classList.remove('run'); el.textContent='Siap. Tekan MULAI (standby ke Home).'; return; }
  el.classList.add('run');
  el.textContent = 'Berjalan: ' + (runState[autoStateVal] || '...') + (irVal ? ' \u2014 IR: terdeteksi' : '');
}
function updateModeUI(){
  document.getElementById('modeMan').classList.toggle('active', modeVal===0);
  document.getElementById('modeAuto').classList.toggle('active', modeVal===1);
  ['panelJoy','panelGrip','panelSl'].forEach(function(id){
    document.getElementById(id).classList.toggle('locked', modeVal===1);
  });
  const running = (autoStateVal!==0);
  document.getElementById('runBtn').disabled  = (modeVal!==1) || running;
  document.getElementById('stopBtn').disabled = !running;
  document.getElementById('saveA').disabled = (modeVal!==0);
  document.getElementById('saveB').disabled = (modeVal!==0);
  document.getElementById('gripBtn').disabled = (modeVal===1);
  document.getElementById('openBtn').disabled = (modeVal===1);
  const pA=document.getElementById('pillA'), pB=document.getElementById('pillB'), pIR=document.getElementById('pillIR');
  pA.textContent = hasA ? 'Titik A: tersimpan' : 'Titik A: -';
  pA.classList.toggle('ok', hasA);
  pB.textContent = hasB ? 'Titik B: tersimpan' : 'Titik B: -';
  pB.classList.toggle('ok', hasB);
  pIR.textContent = (irSimulated ? 'IR simulasi: ' : 'IR fisik: ') + (irVal ? 'terdeteksi' : 'bersih');
  pIR.classList.toggle('warn', !!irVal);
  document.getElementById('irDetectBtn').disabled = !irSimulated;
  document.getElementById('irClearBtn').disabled = !irSimulated;
  autoStatusText();
}
function updateTelemetryUI(){
  document.getElementById('tb').textContent = disp[4].toFixed(1);
  document.getElementById('ta').textContent = disp[2].toFixed(1);
  document.getElementById('te').textContent = disp[3].toFixed(1);
  document.getElementById('tw').textContent = disp[1].toFixed(1);
  document.getElementById('tg').textContent = disp[0].toFixed(1);
  // Slider pasif mengikuti robot, kecuali sedang digeser pengguna
  if (!holdSet[4]) setSlider('sl-b', Math.round(disp[4]));
  if (!holdSet[2]) setSlider('sl-a', Math.round(disp[2]));
  if (!holdSet[3]) setSlider('sl-e', Math.round(disp[3]));
  if (!holdSet[1]) setSlider('sl-w', Math.round(disp[1]));
  if (!holdSet[0]) { setSlider('sl-g', Math.round(disp[0])); grip = disp[0] < 35; renderGrip(); }
}
function applyTelemetry(p){
  // Firmware kirim: S,<base>,<arm>,<elbow>,<wrist>,<gripper>,<mode>,<state>,<ir>,<irSim>,<hasA>,<hasB>
  // Konversi ke indeks servo enum firmware: 0=Gripper, 1=Wrist, 2=Arm, 3=Elbow, 4=Base
  tgt[4] = parseFloat(p[1]);   // Base
  tgt[2] = parseFloat(p[2]);   // Arm
  tgt[3] = parseFloat(p[3]);   // Elbow
  tgt[1] = parseFloat(p[4]);   // Wrist
  tgt[0] = parseFloat(p[5]);   // Gripper
  modeVal=+p[6]; autoStateVal=+p[7]; irVal=+p[8]; irSimulated=(+p[9])!==0; hasA=(+p[10])!==0; hasB=(+p[11])!==0;
  updateModeUI();
}

function connect(){
  try {
    ws = new WebSocket('ws://' + location.hostname + ':81/');
    ws.onopen = () => { setConn(true); lastPayload=''; if(timer){ clearInterval(timer); timer=null; } };
    ws.onclose = () => { setConn(false); if(!timer) timer = setInterval(connect, 1500); };
    ws.onerror = () => { try { ws.close(); } catch(e) {} };
    ws.onmessage = (ev) => {
      const p = ev.data.split(',');
      if (p[0] === 'S' && p.length >= 12) { applyTelemetry(p); }
      else if (p[0] === 'E' && p[1] === 'np') {
        const el=document.getElementById('autoStatus');
        el.classList.remove('run');
        el.textContent = 'Titik A / Titik B belum disimpan! Atur lengan di mode manual lalu tekan "Simpan Titik A/B".';
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
// Gripper: BUKA=100 deg, HOME=50 deg, TUTUP=20 deg
const sliders = [['sl-b','4'],['sl-a','2'],['sl-e','3'],['sl-w','1'],['sl-g','0']];
const holdSet = {};  // slider yang sedang dipegang pengguna tidak dioverride robot
sliders.forEach(([id, idx]) => {
  const el = document.getElementById(id), val = document.getElementById(id + 'v');
  el.addEventListener('input', () => {
    val.textContent = el.value + '\u00B0';
    if (id === 'sl-g') { grip = (+el.value) < 35; renderGrip(); }
    pend[idx] = +el.value;
  });
  el.addEventListener('pointerdown', () => { holdSet[idx] = true; });
});
window.addEventListener('pointerup', () => { for (const k in holdSet) delete holdSet[k]; });
window.addEventListener('pointercancel', () => { for (const k in holdSet) delete holdSet[k]; });
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
document.getElementById('modeMan').addEventListener('click', () => { if (ws && ws.readyState===1) ws.send('M,0'); });
document.getElementById('modeAuto').addEventListener('click', () => { if (ws && ws.readyState===1) ws.send('M,1'); });
document.getElementById('saveA').addEventListener('click', () => {
  if (ws && ws.readyState===1) ws.send('S,a');
  document.getElementById('autoStatus').textContent = 'Titik A disimpan pada posisi lengan saat ini.';
});
document.getElementById('saveB').addEventListener('click', () => {
  if (ws && ws.readyState===1) ws.send('S,b');
  document.getElementById('autoStatus').textContent = 'Titik B disimpan pada posisi lengan saat ini.';
});
document.getElementById('runBtn').addEventListener('click', () => { if (ws && ws.readyState===1) ws.send('R'); });
document.getElementById('stopBtn').addEventListener('click', () => { if (ws && ws.readyState===1) ws.send('X'); });
document.getElementById('irDetectBtn').addEventListener('click', () => { if (ws && ws.readyState===1) ws.send('I,1'); });
document.getElementById('irClearBtn').addEventListener('click', () => { if (ws && ws.readyState===1) ws.send('I,0'); });

// ============================================================================
// VISUAL 3D REAL-TIME (three.js dari flash ESP32; mesh CAD asli di /mesh/*)
// ============================================================================
const rad = (d) => d * Math.PI / 180;
let renderer = null, scene3d = null, camera3d = null, rig = null, primStatic = null, camTarget = null;
let camAz = 0.9, camPol = 1.05, camR = 18;
let useParts = false; // true jika mesh CAD berhasil dimuat

// ====== KONFIGURASI PENEMPATAN PART CAD (dari GLB grippy-bot, satuan mm) ======
// Mesh di-bake dengan transform perakitan GLB -> semua part pas di posisi assembly.
// PIV = pivot tiap sendi = pusat mesh poros asli CAD (akurat, bukan tebakan).
const S = 0.05; // mm -> scene
const HOME = 90; // 90 = home 3D (robot home), konsisten dengan CAD & robot

// Kalibrasi visual dari pose fisik: servo 90/90/90 harus tampak seperti
// CAD pada Arm=165, Elbow=9, Wrist=28 (tegak lurus menurut pengujian manual).
// Hanya model 3D berubah; target servo tetap memakai sudut fisik asli.
const REST = {
  spin: 0,    // Base: 90 fisik = 90 CAD
  // Offset dihitung dari pose CAD terbukti: 165 / 9 / 28.
  // Rumus rotasi memakai (servo - 90 - REST), jadi REST = 90 - poseCAD.
  sh: -75,    // 90 fisik -> CAD 165
  el:  81,    // 90 fisik -> CAD 9
  wr:  62,    // 90 fisik -> CAD 28
  jaw:  0     // Gripper crank
};
const PIV = {
  spin : {x:  0.00, y:  35.00, z:   0.00},   // poros rotating platform (servo Base 26)
  sh   : {x: -6.00, y:  76.00, z:   0.00},   // poros shoulder joint    (servo Arm 14)
  el   : {x: 95.47, y: 109.32, z:   0.00},   // poros elbow joint       (servo Elbow 27)
  wr   : {x: 73.47, y: 199.70, z:   0.00},   // poros wrist joint       (servo Wrist 12)
  jawL : {x:137.30, y:229.49, z: 15.44},     // poros crank kiri        (servo Gripper 13)
  jawR : {x:137.35, y:229.49, z:-15.48}      // poros crank kanan
};
const AX  = { spin:'y', sh:'z', el:'z', wr:'z', jaw:'y' }; // poros rotasi tiap sendi (dari geometri CAD: crank Y-tipis=poros Y)
const SG  = { spin:1, sh:1, el:1, wr:1, jaw:1, jawR:-1 };  // tanda arah rotasi (+1/-1) ; jawR berlawanan (gear kiri-kanan)

function parseBinarySTL(buf){
  const dv = new DataView(buf);
  const tri = dv.getUint32(80, true);
  const pos = new Float32Array(tri*9), nor = new Float32Array(tri*9);
  for (let i=0, p=84; i<tri; i++, p+=50) {
    const nx=dv.getFloat32(p,true), ny=dv.getFloat32(p+4,true), nz=dv.getFloat32(p+8,true);
    for (let v=0; v<3; v++) {
      const o=p+12+v*12, j=i*9+v*3;
      pos[j]=dv.getFloat32(o,true); pos[j+1]=dv.getFloat32(o+4,true); pos[j+2]=dv.getFloat32(o+8,true);
      nor[j]=nx; nor[j+1]=ny; nor[j+2]=nz;
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos,3));
  g.setAttribute('normal',   new THREE.BufferAttribute(nor,3));
  return g;
}

function pivotGroup(parent, p){
  const g = new THREE.Group();
  g.position.set(p.x*S, p.y*S, p.z*S);
  parent.add(g);
  return g;
}
function chainGroup(parent, p, parentWorld){
  // p = pivot dunia (mm); parentWorld = pivot dunia parent (mm).
  // Offset relatif = p_world - parent_world -> saat ditumpuk hierarki, world child tetap di pivotnya.
  const g = new THREE.Group();
  g.position.set((p.x - parentWorld.x)*S, (p.y - parentWorld.y)*S, (p.z - parentWorld.z)*S);
  parent.add(g);
  return g;
}
function attachAbsolute(geoPtr, mat, parent, p){
  const m = new THREE.Mesh(geoPtr, mat);
  m.scale.setScalar(S);
  m.position.set(-p.x*S, -p.y*S, -p.z*S); // geometry absolut di ruang CAD -> offset -pivot
  parent.add(m);
  return m;
}
function clearRig(){
  // CAD mesh world-baked. Saat ganti rig, semua root lama wajib dibuang.
  // Jika hanya rig.base dibuang, STATIC dan primitive lama tertinggal di scene.
  for (const child of [...scene3d.children]) {
    if (child.userData.robotRig) scene3d.remove(child);
  }
}
function meshMat(hex){
  return new THREE.MeshStandardMaterial({color: hex, roughness: .55, metalness: .35, side: THREE.DoubleSide});
}

function buildPrimRig(){
  const matBase = new THREE.MeshStandardMaterial({color: 0x2e3440, roughness: .6, metalness: .35});
  const matArm  = new THREE.MeshStandardMaterial({color: 0x3a7bd5, roughness: .5, metalness: .4});
  const matArm2 = new THREE.MeshStandardMaterial({color: 0x5a8fd6, roughness: .5, metalness: .4});
  const matWri  = new THREE.MeshStandardMaterial({color: 0xd58b3a, roughness: .5, metalness: .4});
  const matGrip = new THREE.MeshStandardMaterial({color: 0xdddddd, roughness: .4, metalness: .5});

  const plate = new THREE.Mesh(new THREE.CylinderGeometry(3.2, 3.4, 0.4, 32), matBase);
  plate.position.y = 0.2;
  scene3d.add(plate);
  plate.userData.robotRig = true;

  rig = { base: new THREE.Group(), sh: new THREE.Group(), el: new THREE.Group(), wr: new THREE.Group() };

  const baseCol = new THREE.Mesh(new THREE.CylinderGeometry(1.6, 1.9, 1.1, 24), matBase);
  baseCol.position.y = 0.95;
  rig.base.add(baseCol);
  rig.base.position.y = 0.4;
  scene3d.add(rig.base);
  rig.base.userData.robotRig = true;

  rig.sh.position.y = 1.6;
  const upper = new THREE.Mesh(new THREE.BoxGeometry(0.8, 3.2, 0.8), matArm);
  upper.position.y = 1.6;
  rig.sh.add(upper);
  rig.base.add(rig.sh);

  rig.el.position.y = 3.2;
  const fore = new THREE.Mesh(new THREE.BoxGeometry(0.6, 2.7, 0.6), matArm2);
  fore.position.y = 1.35;
  rig.el.add(fore);
  rig.sh.add(rig.el);

  rig.wr.position.y = 2.7;
  const wristCol = new THREE.Mesh(new THREE.CylinderGeometry(0.35, 0.35, 0.7, 16), matWri);
  wristCol.position.y = 0.2;
  rig.wr.add(wristCol);
  rig.el.add(rig.wr);

  const palm = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.35, 0.7), matWri);
  palm.position.y = 0.65;
  rig.wr.add(palm);
  const jawGeo = new THREE.BoxGeometry(0.22, 1.1, 0.7);
  rig.jawL = new THREE.Mesh(jawGeo, matGrip);
  rig.jawR = new THREE.Mesh(jawGeo, matGrip);
  rig.jawL.position.set(-0.32, 1.25, 0);
  rig.jawR.position.set( 0.32, 1.25, 0);
  rig.jawL.geometry.translate(0, 0.45, 0);
  rig.wr.add(rig.jawL);
  rig.wr.add(rig.jawR);
}

function buildCadRig(geos){
  const mStat  = meshMat(0x556070);
  const mSpin  = meshMat(0x3a7bd5);
  const mArm   = meshMat(0x5a8fd6);
  const mFore  = meshMat(0x5a8fd6);
  const mWrist = meshMat(0xd58b3a);
  const mJaw   = meshMat(0xd9dde2);
  const O = {x:0, y:0, z:0};

  // Part statis: robot base + dudukan servo
  const stat = attachAbsolute(geos.stat, mStat, scene3d, O);
  stat.userData.robotRig = true;

  rig = {};
  // Rotating platform + vertical support (sendi Base, yaw pada porosnya)
  rig.base = pivotGroup(scene3d, PIV.spin);
  rig.base.userData.robotRig = true;
  attachAbsolute(geos.spin, mSpin, rig.base, PIV.spin);

  // Shoulder joint/cover + upper arm (sendi Arm)
  rig.sh = chainGroup(rig.base, PIV.sh, PIV.spin);
  attachAbsolute(geos.arm, mArm, rig.sh, PIV.sh);

  // Elbow joint/cover + forearm (sendi Elbow)
  rig.el = chainGroup(rig.sh, PIV.el, PIV.sh);
  attachAbsolute(geos.fore, mFore, rig.el, PIV.el);

  // Wrist joint/cover + wrist bracket + gripper body (sendi Wrist)
  rig.wr = chainGroup(rig.el, PIV.wr, PIV.el);
  attachAbsolute(geos.wrist, mWrist, rig.wr, PIV.wr);

  // Dua jari (sendi Gripper), masing-masing pada poros crank-nya sendiri
  rig.jawL = chainGroup(rig.wr, PIV.jawL, PIV.wr);
  attachAbsolute(geos.jawl, mJaw, rig.jawL, PIV.jawL);
  rig.jawR = chainGroup(rig.wr, PIV.jawR, PIV.wr);
  attachAbsolute(geos.jawr, mJaw, rig.jawR, PIV.jawR);

  useParts = true;
}

function init3D(){
  const canvas = document.getElementById('c3d');
  if (typeof THREE === 'undefined') {
    document.getElementById('c3dwrap').innerHTML =
      '<p class="noload">Visual 3D gagal dimuat (three.js tidak tersedia).<br>Kontrol tetap berfungsi normal.</p>';
    return;
  }
  renderer = new THREE.WebGLRenderer({canvas: canvas, antialias: true});
  // 1.25 cukup tajam di HP, kurangi beban GPU pada layar high-DPI.
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.25));
  scene3d = new THREE.Scene();
  scene3d.background = new THREE.Color(0x0e1114);
  scene3d.fog = new THREE.Fog(0x0e1114, 30, 80);

  scene3d.add(new THREE.AmbientLight(0xffffff, 0.55));
  const dl = new THREE.DirectionalLight(0xffffff, 0.9);
  dl.position.set(6, 12, 8);
  scene3d.add(dl);
  const grid = new THREE.GridHelper(24, 24, 0x3a4652, 0x232b33);
  scene3d.add(grid);

  // 0 derajat = lengan horizontal ke depan (konsisten dengan CAD & rotasi base).
  camTarget = new THREE.Vector3(4.0, 5.5, 0);
  camera3d = new THREE.PerspectiveCamera(45, 1, 0.1, 100);

  function onResize(){
    const wrap = document.getElementById('c3dwrap');
    if (!wrap || !renderer) return;
    const w = wrap.clientWidth, h = wrap.clientHeight;
    renderer.setSize(w, h, false);
    camera3d.aspect = w / h;
    camera3d.updateProjectionMatrix();
  }
  window.addEventListener('resize', onResize);
  onResize();

  // --- Orbit kamera custom (sentuh 1 jari = putar, 2 jari / scroll = zoom) ---
  const wrap = document.getElementById('c3dwrap');
  const pts = {};
  let pinchDist = 0;
  wrap.addEventListener('pointerdown', (e) => { pts[e.pointerId] = {x: e.clientX, y: e.clientY}; try { wrap.setPointerCapture(e.pointerId); } catch(err) {} });
  wrap.addEventListener('pointermove', (e) => {
    if (!pts[e.pointerId]) return;
    const prev = pts[e.pointerId];
    const dx = e.clientX - prev.x, dy = e.clientY - prev.y;
    pts[e.pointerId] = {x: e.clientX, y: e.clientY};
    const ids = Object.keys(pts);
    if (ids.length === 1) {
      camAz += dx * 0.006;
      camPol = Math.min(2.6, Math.max(0.2, camPol - dy * 0.006));
    } else if (ids.length === 2) {
      const a = pts[ids[0]], b = pts[ids[1]];
      const d = Math.hypot(a.x - b.x, a.y - b.y);
      if (pinchDist > 0) camR = Math.min(45, Math.max(6, camR * pinchDist / d));
      pinchDist = d;
    }
  });
  function lift(e){ delete pts[e.pointerId]; pinchDist = 0; }
  wrap.addEventListener('pointerup', lift);
  wrap.addEventListener('pointercancel', lift);
  wrap.addEventListener('wheel', (e) => { camR = Math.min(45, Math.max(6, camR * (1 + e.deltaY * 0.001))); }, {passive: true});

  frame();

  // Muat 7 grup mesh CAD; jika salah satu gagal -> fallback primitif
  const PATHS = {
    stat: '/mesh/stat', spin: '/mesh/spin', arm: '/mesh/arm',
    fore: '/mesh/fore', wrist: '/mesh/wrist', jawl: '/mesh/jawl', jawr: '/mesh/jawr'
  };
  // Tampil cepat: primitive aktif dulu. Mesh CAD diambil berurutan; ESP32 WebServer
  // hanya satu loop, jadi 7 fetch paralel malah berebut bandwidth dan heap.
  buildPrimRig();
  (async () => {
    const geos = {};
    for (const [k, u] of Object.entries(PATHS)) {
      const r = await fetch(u);
      if (!r.ok) throw new Error(u);
      geos[k] = parseBinarySTL(await r.arrayBuffer());
    }
    clearRig();
    buildCadRig(geos);
  })().catch(() => {});
}

function rot3(g, ax, a){
  const v = {x:0, y:0, z:0};
  v[ax] = a;
  g.rotation.set(v.x, v.y, v.z);
}

function update3D(){
  if (!rig) return;
  // Lerp tampilan mengikuti sudut terkirim -> mulus walau telemetri 20fps
  for (let i = 0; i < 5; i++) disp[i] += (tgt[i] - disp[i]) * 0.22;
  if (useParts) {
    // Kalibrasi dari uji manual: pose CAD lurus = Arm 165, Elbow 9, Wrist 28.
    // Rumus asli CAD memakai (sudut CAD - 90): +75, -81, -62 derajat.
    // Maka servo fisik 90 harus menghasilkan rotasi tersebut.
    rot3(rig.base, AX.spin, SG.spin * -rad(disp[4] - 90));       // Base
    rot3(rig.sh,   AX.sh,   SG.sh   *  rad(disp[2] - 15));       // 90 -> +75
    rot3(rig.el,   AX.el,   SG.el   *  rad(disp[3] - 171));      // 90 -> -81
    // Wrist CAD terbalik 180° terhadap horn fisik: 90 -> +118°, gripper mengarah ke atas.
    rot3(rig.wr,   AX.wr,   SG.wr   *  rad(disp[1] + 28));       // 90 -> +118
    const open = (disp[0] - 20) / 80;                                   // 0..1 (tutup..buka)
    const jawA = rad((open - 0.5) * 45) + REST.jaw;                     // ±22.5 deg sekitar pose CAD
    rot3(rig.jawL, AX.jaw, SG.jaw  * jawA);
    rot3(rig.jawR, AX.jaw, SG.jawR * jawA);                             // jawR berlawanan (gear)
  } else {
    // Primitif (fallback)
    rig.base.rotation.y = -rad(disp[4] - 90);
    rig.sh.rotation.z   =  rad(disp[2] - 15);
    rig.el.rotation.z   =  rad(disp[3] - 171);
    rig.wr.rotation.z   =  rad(disp[1] + 28);
    const open = (disp[0] - 20) / 80;
    const jawA = (0.10 + open * 0.35);
    rig.jawL.rotation.z = jawA;
    rig.jawR.rotation.z = -jawA;
  }
  updateTelemetryUI();
}

function frame(){
  requestAnimationFrame(frame);
  if (!renderer) return;
  camera3d.position.set(
    camTarget.x + camR * Math.sin(camPol) * Math.cos(camAz),
    camTarget.y + camR * Math.cos(camPol),
    camTarget.z + camR * Math.sin(camPol) * Math.sin(camAz)
  );
  camera3d.lookAt(camTarget);
  update3D();
  renderer.render(scene3d, camera3d);
}
init3D();
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
void loadPoses();
bool savePoseToNVS(const char which);
void setMode(CtrlMode m);
bool startAuto();
void stopAuto(bool freezeAngles);
void autoGoHomeTargets();
void runAutoTick();
bool motionDone();
void updateIR();

// ============================================================================
// 7. SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n==================================================");
    Serial.println("  ROBOT ARM 5-DOF - MANUAL + OTOMATIS + VISUAL 3D ");
    Serial.println("==================================================");

    pinMode(PIN_BUTTON_GRIPPER, INPUT_PULLUP);
    pinMode(PIN_IR_SENSOR, INPUT);

    initServos();
    loadPoses();

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
    server.on("/three.min.js", HTTP_GET, []() {
        // three.js r128 dalam bentuk gzip dari PROGMEM (149 KB), tidak perlu internet
        server.sendHeader("Content-Encoding", "gzip");
        server.sendHeader("Cache-Control", "max-age=86400");
        server.send_P(200, "application/javascript", (PGM_P)THREE_JS_GZ, THREE_JS_GZ_LEN);
    });
    // --- Mesh CAD asli robot (7 grup dari GLB perakitan grippy-bot, world-baked, gzip) ---
    struct MeshEndpoint { const char* url; PGM_P data; size_t len; };
    static const MeshEndpoint MESH_ENDPOINTS[] = {
        {"/mesh/stat",  (PGM_P)GLB_GZ_STATIC, GLB_GZ_STATIC_LEN},
        {"/mesh/spin",  (PGM_P)GLB_GZ_SPIN,   GLB_GZ_SPIN_LEN},
        {"/mesh/arm",   (PGM_P)GLB_GZ_ARM,    GLB_GZ_ARM_LEN},
        {"/mesh/fore",  (PGM_P)GLB_GZ_FORE,   GLB_GZ_FORE_LEN},
        {"/mesh/wrist", (PGM_P)GLB_GZ_WRIST,  GLB_GZ_WRIST_LEN},
        {"/mesh/jawl",  (PGM_P)GLB_GZ_JAWL,   GLB_GZ_JAWL_LEN},
        {"/mesh/jawr",  (PGM_P)GLB_GZ_JAWR,   GLB_GZ_JAWR_LEN},
    };
    for (const auto &me : MESH_ENDPOINTS) {
        server.on(me.url, HTTP_GET, [me]() {
            server.sendHeader("Content-Encoding", "gzip");
            server.sendHeader("Cache-Control", "max-age=604800");
            server.send_P(200, "application/octet-stream", me.data, me.len);
        });
    }
    server.onNotFound([]() {
        server.send(404, "text/plain", "Not found");
    });
    server.begin();
    Serial.println("[HTTP] Web server port 80 siap (halaman + /three.min.js).");

    // --- WebSocket ---
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("[WS] WebSocket port 81 siap.");

    Serial.println("\n--- PANDUAN ---");
    if (staOk) {
        Serial.println("1. HP connect ke WiFi rumah '" WIFI_STA_SSID "' (jaringan yang SAMA dengan ESP32)");
        Serial.println("2. Buka browser: http://robotarm.local (atau lihat IP di atas)");
        Serial.println("3. Mode manual: atur lengan -> Simpan Titik A (pick) & B (place)");
        Serial.println("4. Mode otomatis: MULAI -> IR mendeteksi barang -> pick & place berulang");
        Serial.println("5. Serial: 'm'=manual, 'a'=otomatis, 'r'=mulai, 'x'=stop, 'p'=info pose+IR");
    } else {
        Serial.println("1. HP connect ke WiFi: " WIFI_AP_SSID " (pass: " WIFI_AP_PASSWORD ")");
        Serial.println("2. Buka browser: http://192.168.4.1");
        Serial.println("3. Mode manual: atur lengan -> Simpan Titik A (pick) & B (place)");
        Serial.println("4. Mode otomatis: MULAI -> IR mendeteksi barang -> pick & place berulang");
        Serial.println("5. Serial: 'm'=manual, 'a'=otomatis, 'r'=mulai, 'x'=stop, 'p'=info pose+IR");
    }
    Serial.printf("[POSE] Titik A: %s | Titik B: %s (disimpan di NVS)\n",
                  hasPoseA ? "ada" : "belum", hasPoseB ? "ada" : "belum");
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

        updateIR();

        if (ctrlMode == MODE_AUTO) {
            if (autoState != AS_IDLE) runAutoTick();
        } else {
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

    // Boot dan siklus otomatis mulai dengan gripper BUKA.
    currentAngles[INDEX_GRIPPER] = GRIPPER_OPEN_ANGLE;
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
// 10. SENSOR INFRAMERAH (FC-51, GPIO 34)
// ============================================================================
void updateIR() {
    int reading = digitalRead(PIN_IR_SENSOR);
    if (reading == IR_ACTIVE_LEVEL) {
        if (irHoldCount < 10000) irHoldCount++;
    } else {
        irHoldCount = 0;
    }
    bool was = irPhysical;
    irPhysical = (irHoldCount >= IR_DEBOUNCE_COUNT);
    if (irPhysical != was) {
        Serial.printf("[IR] Fisik: barang %sdeteksi\n", irPhysical ? "" : "tidak ");
    }
}

// ============================================================================
// 11. MODE OTOMATIS (PICK & PLACE) + NVS POSE
// ============================================================================
void loadPoses() {
    if (!prefs.begin("robotarm", true)) {
        Serial.println("[NVS] Belum ada data pose (namespace kosong).");
        return;
    }
    hasPoseA = prefs.getBool("hasA", false);
    hasPoseB = prefs.getBool("hasB", false);
    if (hasPoseA && (prefs.getBytesLength("poseA") == sizeof(poseA))) {
        prefs.getBytes("poseA", poseA, sizeof(poseA));
    } else {
        hasPoseA = false;
    }
    if (hasPoseB && (prefs.getBytesLength("poseB") == sizeof(poseB))) {
        prefs.getBytes("poseB", poseB, sizeof(poseB));
    } else {
        hasPoseB = false;
    }
    prefs.end();
    if (hasPoseA || hasPoseB) {
        Serial.println("[NVS] Pose titik A/B dimuat dari flash (tahan mati listrik).");
    }
}

bool savePoseToNVS(const char which) {
    float *p    = (which == 'a') ? poseA : poseB;
    bool  *flag = (which == 'a') ? &hasPoseA : &hasPoseB;
    memcpy(p, targetAngles, sizeof(poseA));
    *flag = true;

    // Tulis pose saat ini ke buffer agar persis dengan yang di-flash
    if (!prefs.begin("robotarm", false)) return false;
    if (which == 'a') {
        prefs.putBool("hasA", true);
        prefs.putBytes("poseA", poseA, sizeof(poseA));
    } else {
        prefs.putBool("hasB", true);
        prefs.putBytes("poseB", poseB, sizeof(poseB));
    }
    prefs.end();

    Serial.printf("[POSE] Titik %c disimpan (NVS): ", toupper(which));
    for (int i = 0; i < NUM_SERVOS; i++) Serial.printf("%.1f ", p[i]);
    Serial.println();
    return true;
}

void autoGoHomeTargets() {
    for (int i = 0; i < NUM_SERVOS; i++) targetAngles[i] = 90.0f;
    // Siklus otomatis selalu pulang dengan gripper BUKA. HOME=50 hanya pose awal boot.
    targetAngles[INDEX_GRIPPER] = GRIPPER_OPEN_ANGLE;
    gripperState = false;
    gripperCmdFromPhone = false;
    gripperCmdChanged = false;
}

void autoSetState(AutoState st) {
    autoState = st;
    autoStateT0 = millis();
}

bool motionDone() {
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (fabsf(targetAngles[i] - currentAngles[i]) > MOTION_DONE_TOL) return false;
    }
    return true;
}

void stopAuto(bool freezeAngles) {
    autoState = AS_IDLE;
    autoStateT0 = 0;
    if (freezeAngles) {
        for (int i = 0; i < NUM_SERVOS; i++) {
            targetAngles[i] = currentAngles[i]; // bekukan lengan di posisi ini
        }
    }
}

void setMode(CtrlMode m) {
    if (m == ctrlMode) return;
    if (m == MODE_MANUAL) {
        stopAuto(true); // bekukan lengan, mode manual ambil alih
        Serial.println("[MODE] MANUAL (posisi lengan dibekukan)");
    } else {
        stopAuto(false);
        Serial.println("[MODE] OTOMATIS (siklus idle, tekan MULAI)");
    }
    ctrlMode = m;
}

bool startAuto() {
    if (ctrlMode != MODE_AUTO) {
        Serial.println("[AUTO] Tidak bisa MULAI: bukan mode otomatis.");
        return false;
    }
    if (!hasPoseA || !hasPoseB) {
        Serial.println("[AUTO] Ditolak: Titik A/B belum disimpan. Teach dulu di mode manual.");
        webSocket.broadcastTXT("E,np");
        return false;
    }
    Serial.println("[AUTO] Mulai -> standby ke Home, menunggu sensor IR.");
    autoGoHomeTargets();
    autoSetState(AS_HOME);
    return true;
}

void runAutoTick() {
    switch (autoState) {
        case AS_IDLE:
            break;

        case AS_HOME: // standby awal & akhir siklus
            if (motionDone() && (millis() - autoStateT0 >= AUTO_MOVE_SETTLE_MS)) {
                autoSetState(AS_WAIT);
                Serial.println("[AUTO] Standby di Home, menunggu barang (IR)...");
            }
            break;

        case AS_HOME_TO_B: // transit aman: Home dulu, gripper tetap TUTUP
            if (motionDone() && (millis() - autoStateT0 >= AUTO_MOVE_SETTLE_MS)) {
                targetAngles[INDEX_BASE]  = poseB[INDEX_BASE];
                targetAngles[INDEX_ARM]   = poseB[INDEX_ARM];
                targetAngles[INDEX_ELBOW] = poseB[INDEX_ELBOW];
                targetAngles[INDEX_WRIST] = poseB[INDEX_WRIST];
                // Jangan ubah gripper: barang tetap digenggam selama transit.
                autoSetState(AS_GO_B);
                Serial.println("[AUTO] Sampai Home -> menuju Titik B (gripper tetap TUTUP)");
            }
            break;

        case AS_WAIT: // menunggu IR mendeteksi barang selama 2 detik
            if (irDetected() && millis() - autoStateT0 >= 2000) {
                // Menuju titik A; gripper dipastikan BUKA untuk menyambar
                targetAngles[INDEX_BASE]  = poseA[INDEX_BASE];
                targetAngles[INDEX_ARM]   = poseA[INDEX_ARM];
                targetAngles[INDEX_ELBOW] = poseA[INDEX_ELBOW];
                targetAngles[INDEX_WRIST] = poseA[INDEX_WRIST];
                targetAngles[INDEX_GRIPPER] = GRIPPER_OPEN_ANGLE;
                gripperState = false;
                autoSetState(AS_GO_A);
                Serial.println("[AUTO] IR terdeteksi 2 detik -> menuju Titik A (pick)");
            } else if (!irDetected()) {
                // IR hilang sebelum 2 detik, reset dan menunggu deteksi baru
                autoSetState(AS_WAIT);
            }
            break;

        case AS_GO_A:
            if (motionDone() && (millis() - autoStateT0 >= AUTO_MOVE_SETTLE_MS)) {
                targetAngles[INDEX_GRIPPER] = GRIPPER_CLOSE_ANGLE;
                gripperState = true;
                gripperCmdFromPhone = true;
                autoSetState(AS_GRIP);
                Serial.println("[AUTO] Sampai di A -> gripper TUTUP");
            }
            break;

        case AS_GRIP:
            if (motionDone() && (millis() - autoStateT0 >= AUTO_GRIP_SETTLE_MS)) {
                // Setelah menggenggam, pulang Home dulu; gripper tetap TUTUP.
                autoGoHomeTargets();
                targetAngles[INDEX_GRIPPER] = GRIPPER_CLOSE_ANGLE;
                gripperState = true;
                gripperCmdFromPhone = true;
                autoSetState(AS_HOME_TO_B);
                Serial.println("[AUTO] Dari Titik A -> Home (gripper tetap TUTUP)");
            }
            break;


        case AS_GO_B:
            if (motionDone() && (millis() - autoStateT0 >= AUTO_MOVE_SETTLE_MS)) {
                targetAngles[INDEX_GRIPPER] = GRIPPER_OPEN_ANGLE;
                gripperState = false;
                gripperCmdFromPhone = false;
                autoSetState(AS_RELEASE);
                Serial.println("[AUTO] Sampai di B -> gripper BUKA (letakkan barang)");
            }
            break;

        case AS_RELEASE:
            if (motionDone() && (millis() - autoStateT0 >= AUTO_GRIP_SETTLE_MS)) {
                autoGoHomeTargets();
                autoSetState(AS_HOME);
                Serial.println("[AUTO] Barang diletakkan -> kembali ke Home");
            }
            break;
    }
}

// ============================================================================
// 12. WEBSOCKET HANDLING
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
        // Semua perintah UI < 64 byte; batasi stack dan cegah payload liar.
        char buf[64];
        size_t n = length < sizeof(buf) - 1 ? length : sizeof(buf) - 1;
        memcpy(buf, payload, n);
        buf[n] = '\0';

        // --- Perintah simulasi IR: "I,<0|1>" (0=bersih, 1=terdeteksi) ---
        int irValue;
        if (sscanf(buf, "I,%d", &irValue) == 1 && (irValue == 0 || irValue == 1)) {
            irSimulated = true;
            irSimValue = (irValue == 1);
            Serial.printf("[IR] Simulasi: %s\n", irSimValue ? "TERDETEKSI" : "BERSIH");
            return;
        }

        // --- Perintah ganti mode: "M,<0|1>" ---
        int mval;
        if (sscanf(buf, "M,%d", &mval) == 1) {
            if (mval == 0 || mval == 1) setMode((mval == 1) ? MODE_AUTO : MODE_MANUAL);
            return;
        }

        // --- Perintah teach pose: "S,a" / "S,b" (hanya mode manual) ---
        if ((n == 3) && (buf[0] == 'S') && (buf[1] == ',')) {
            char c = tolower(buf[2]);
            if (c == 'a' || c == 'b') {
                if (ctrlMode == MODE_MANUAL) {
                    savePoseToNVS(c);
                } else {
                    Serial.println("[POSE] Teach ditolak: harus di mode manual.");
                }
            }
            return;
        }

        // --- Perintah single-char ---
        if (n == 1) {
            char c = buf[0];
            if (c == 'H' || c == 'h') {
                stopAuto(true);
                autoGoHomeTargets();
                Serial.println("[WS] Home");
                return;
            } else if (c == 'R' || c == 'r') {
                startAuto();
                return;
            } else if (c == 'X' || c == 'x') {
                if (autoState != AS_IDLE) {
                    Serial.println("[WS] STOP siklus otomatis");
                }
                stopAuto(true);
                return;
            } else if (c == 'O' || c == 'o') {
                if (ctrlMode == MODE_MANUAL) {
                    gripperCmdFromPhone = false;
                    gripperCmdChanged = true;
                    Serial.println("[WS] Buka Gripper");
                }
                return;
            } else if (c == 'C' || c == 'c' || c == 'G' || c == 'g') {
                if (ctrlMode == MODE_MANUAL) {
                    gripperCmdFromPhone = true;
                    gripperCmdChanged = true;
                    Serial.println("[WS] Tutup Gripper");
                }
                return;
            }
        }

        if (ctrlMode != MODE_MANUAL) return; // joystick/slider diabaikan saat otomatis

        // Perintah sudut manual (slider HP): "A,<index>,<sudut>"
        // index: 0=Gripper, 1=Wrist, 2=Arm, 3=Elbow, 4=Base (urutan ServoID)
        int idx; float ang;
        if (sscanf(buf, "A,%d,%f", &idx, &ang) == 2) {
            if (idx >= 0 && idx < NUM_SERVOS) {
                targetAngles[idx] = constrain(ang, ANGLE_MIN, ANGLE_MAX);
                if (idx == INDEX_GRIPPER) {
                    gripperState        = (targetAngles[idx] < 35.0f); // BUKA=100, HOME=50, TUTUP=20
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
    // currentAngles = posisi asli lengan -> visual 3D & slider di HP ikut bergerak
    char msg[128];
    snprintf(msg, sizeof(msg), "S,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d",
             currentAngles[INDEX_BASE], currentAngles[INDEX_ARM],
             currentAngles[INDEX_ELBOW], currentAngles[INDEX_WRIST],
             currentAngles[INDEX_GRIPPER],
             (int)ctrlMode, (int)autoState, irDetected() ? 1 : 0,
             irSimulated ? 1 : 0, hasPoseA ? 1 : 0, hasPoseB ? 1 : 0);
    webSocket.broadcastTXT(msg);
}

// ============================================================================
// 13. KONTROL GRIPPER (TOMBOL GPIO 4 & SERIAL)
//     Saat siklus otomatis berjalan: tombol = E-STOP
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
                if (ctrlMode == MODE_AUTO && autoState != AS_IDLE) {
                    Serial.println("[E-STOP] Siklus otomatis dihentikan via tombol fisik!");
                    stopAuto(true);
                    webSocket.broadcastTXT("E,st");
                } else {
                    gripperState = !gripperState;
                    gripperCmdFromPhone = gripperState;
                    targetAngles[INDEX_GRIPPER] = gripperState ? GRIPPER_CLOSE_ANGLE : GRIPPER_OPEN_ANGLE;
                    Serial.println(gripperState ? "[GRIPPER] Menutup (Close)" : "[GRIPPER] Membuka (Open)");
                }
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
        case 'm':
        case 'M':
            setMode(MODE_MANUAL);
            break;

        case 'a':
        case 'A':
            setMode(MODE_AUTO);
            break;

        case 'r':
        case 'R':
            if (ctrlMode != MODE_AUTO) setMode(MODE_AUTO);
            startAuto();
            break;

        case 'x':
        case 'X':
            if (autoState != AS_IDLE) Serial.println("[SERIAL] Siklus otomatis dihentikan");
            stopAuto(true);
            break;

        case 'p':
        case 'P':
            Serial.printf("[INFO] Mode: %s | AutoState: %d | IR: %s\n",
                          ctrlMode == MODE_AUTO ? "OTOMATIS" : "MANUAL",
                          (int)autoState, irDetected() ? "BARANG" : "bersih");
            Serial.print("[INFO] Titik A: ");
            Serial.println(hasPoseA ? "" : "belum");
            if (hasPoseA) { for (int i = 0; i < NUM_SERVOS; i++) Serial.printf("%.1f ", poseA[i]); Serial.println(); }
            Serial.print("[INFO] Titik B: ");
            Serial.println(hasPoseB ? "" : "belum");
            if (hasPoseB) { for (int i = 0; i < NUM_SERVOS; i++) Serial.printf("%.1f ", poseB[i]); Serial.println(); }
            break;

        case 'o':
        case 'O':
            if (ctrlMode == MODE_MANUAL) {
                targetAngles[INDEX_GRIPPER] = GRIPPER_OPEN_ANGLE;
                gripperState = false;
                gripperCmdFromPhone = false;
                Serial.println("[SERIAL] Membuka Gripper");
            } else {
                Serial.println("[SERIAL] Diabaikan: mode otomatis");
            }
            break;

        case 'g':
        case 'G':
            if (ctrlMode == MODE_MANUAL) {
                targetAngles[INDEX_GRIPPER] = GRIPPER_CLOSE_ANGLE;
                gripperState = true;
                gripperCmdFromPhone = true;
                Serial.println("[SERIAL] Menutup Gripper");
            } else {
                Serial.println("[SERIAL] Diabaikan: mode otomatis");
            }
            break;

        case 'h':
        case 'H':
            stopAuto(true);
            autoGoHomeTargets();
            Serial.println("[SERIAL] Home (semua sendi 90 deg, gripper buka)");
            break;

        default:
            Serial.printf("[INFO] Perintah '%c' tidak dikenali. Gunakan m/a/r/x/p/o/g/h.\n", cmd);
            break;
    }
}
