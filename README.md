# Robot Arm 5-DOF ESP32

Robot arm 5-DOF berbasis ESP32. Dikendalikan dari browser HP/PC lewat WiFi, punya mode manual dan pick & place otomatis, penyimpanan pose ke NVS, simulasi sensor IR, serta visualisasi 3D real-time dari mesh CAD.

> Firmware aktif: [`src/main.cpp`](src/main.cpp). File `arduino/robot_arm/robot_arm.ino` hanya salinan lama.

## Fitur

- Kontrol WiFi tanpa Bluetooth.
- Mode **MANUAL**: virtual joystick, slider sudut, tombol gripper, tombol HOME.
- Mode **OTOMATIS**: teach Titik A dan Titik B, siklus pick & place.
- Urutan auto: `Home → A → tutup gripper → Home → B → buka gripper → Home`.
- Pose A/B disimpan permanen ke NVS namespace `robotarm`.
- Sensor barang FC-51 di GPIO 34, dengan konfirmasi deteksi 2 detik.
- Tombol simulasi IR dari web. Sensor fisik belum wajib dipasang.
- Tombol fisik GPIO 4: toggle gripper saat manual, E-STOP saat auto aktif.
- Gerakan servo halus: update 15 ms, maksimal 0.75° per update.
- Telemetri WebSocket 50 ms untuk slider dan model 3D.
- Three.js dan mesh CAD gzip tersimpan di flash ESP32. Tidak perlu internet.

## Hardware

| Komponen | Jumlah | Catatan |
|---|---:|---|
| ESP32 DOIT DevKit V1 | 1 | Board target PlatformIO |
| Servo motor | 5 | Gripper, wrist, arm, elbow, base |
| PSU servo | 1 | 5–6 V, arus cukup untuk servo. Jangan ambil daya servo dari ESP32. |
| Kapasitor | 1 | Disarankan 1000–2200 µF, ≥10 V, dekat rel servo |
| Tombol push button | 1 | GPIO 4 ke GND, opsional |
| FC-51 | 1 | Opsional; GPIO 34. UI punya simulasi IR. |

## Pinout

| Fungsi | GPIO ESP32 |
|---|---:|
| Servo gripper | 13 |
| Servo wrist | 12 |
| Servo arm | 14 |
| Servo elbow | 27 |
| Servo base | 26 |
| Tombol gripper / E-STOP | 4 |
| FC-51 digital output | 34 |

Diagram daya lengkap: [`WIRING.md`](WIRING.md).

## Penting: Daya Servo

```text
PSU servo 5–6 V (+) ── rel V+ servo
PSU servo GND       ── rel GND servo ── ESP32 GND
ESP32 GPIO          ── sinyal servo saja
```

- Jangan sambungkan kabel VCC servo ke pin `3V3`, `5V`, atau `VIN` ESP32.
- Semua servo dan ESP32 wajib berbagi **GND**.
- Gunakan kabel daya pendek dan cukup tebal.
- Dengung/panas saat diam biasanya berarti servo menahan beban atau menekan stopper mekanik. Kalibrasi limit mekanik sebelum operasi penuh.

## Instalasi

### Prasyarat

- [PlatformIO](https://platformio.org/)
- Python dan driver USB ESP32/CH340 bila dibutuhkan
- Kabel USB data

### Build

```powershell
& "C:\Users\arif\.platformio\penv\Scripts\pio.exe" run -e esp32doit-devkit-v1
```

### Upload

Cari port ESP32 terlebih dahulu:

```powershell
& "C:\Users\arif\.platformio\penv\Scripts\pio.exe" device list
```

Contoh upload pada `COM6`:

```powershell
& "C:\Users\arif\.platformio\penv\Scripts\pio.exe" run -t upload -e esp32doit-devkit-v1 --upload-port COM6
```

> Upload dengan partisi baru dapat menghapus pose A/B dari NVS. Teach ulang setelah flash bila diperlukan.

### Serial monitor

```powershell
& "C:\Users\arif\.platformio\penv\Scripts\pio.exe" device monitor --port COM6 --baud 115200
```

## Akses Web

Firmware mencoba bergabung ke WiFi rumah dulu. Jika gagal dalam 10 detik, ESP32 membuat hotspot.

| Mode | Akses |
|---|---|
| STA / WiFi rumah | `http://robotarm.local` atau IP yang tampil di Serial Monitor |
| Access Point fallback | Hubungkan HP ke `RobotArm-5DOF`, lalu buka `http://192.168.4.1` |
| WebSocket | Port `81`, dipakai otomatis oleh halaman web |

SSID/password WiFi berada di bagian konfigurasi `src/main.cpp`. Jangan commit kredensial pribadi bila repository dibagikan.

Jika UI atau model 3D tampak versi lama, lakukan hard refresh browser:

```text
Ctrl+F5
```

## Cara Pakai

### Manual

1. Buka halaman kontrol.
2. Pilih **MANUAL**.
3. Gunakan joystick atau slider.
4. Gunakan **GRIPPER** untuk toggle buka/tutup; tombol **BUKA** memaksa gripper terbuka.
5. Tekan **HOME** untuk kembali ke pose home.
6. Atur posisi pick lalu tekan **Simpan Titik A**.
7. Atur posisi place lalu tekan **Simpan Titik B**.

### Otomatis dengan simulasi IR

1. Pastikan Titik A dan Titik B sudah tersimpan.
2. Pilih **OTOMATIS**.
3. Tekan **MULAI**. Robot menuju Home lalu standby.
4. Tekan **SIMULASI: TERDETEKSI** selama minimal 2 detik.
5. Siklus berjalan: `Home → A → grip → Home → B → release → Home`.
6. Tekan **SIMULASI: BERSIH** sebelum siklus berikutnya.
7. Tekan **BERHENTI** atau tombol fisik GPIO 4 untuk E-STOP.

### Sensor FC-51 fisik

Hubungkan FC-51 sesuai [`WIRING.md`](WIRING.md). GPIO 34 tidak punya pull-up internal. Jika output DO melayang, pasang resistor 10 kΩ dari DO ke 3.3 V.

## Sudut Servo

| Posisi | Nilai |
|---|---:|
| Home Base / Arm / Elbow / Wrist | 90° |
| Gripper terbuka | 100° |
| Gripper tertutup | 20° |

Nilai ini harus dikalibrasi terhadap mekanik arm. Jangan memaksa servo ke stopper fisik.

## Perintah Serial

Serial baud rate: `115200`.

| Perintah | Fungsi |
|---|---|
| `m` | Mode manual |
| `a` | Mode otomatis |
| `r` | Mulai siklus otomatis |
| `x` | Stop otomatis, bekukan posisi saat ini |
| `p` | Cetak status dan pose A/B |
| `o` | Buka gripper, manual saja |
| `g` | Tutup gripper, manual saja |
| `h` | Home |

## Arsitektur Software

```text
HP / PC Browser
  ├─ HTTP port 80: halaman UI, three.js, mesh CAD gzip
  └─ WebSocket port 81: command dan telemetri

ESP32
  ├─ WebServer
  ├─ WebSocketsServer
  ├─ ESP32Servo
  ├─ Preferences / NVS: pose A dan B
  └─ State machine: manual dan otomatis
```

### Telemetri WebSocket

```text
S,<base>,<arm>,<elbow>,<wrist>,<gripper>,<mode>,<state>,<ir>,<irSim>,<hasA>,<hasB>
```

### Perintah WebSocket penting

```text
M,0      mode manual
M,1      mode otomatis
S,a      simpan Titik A
S,b      simpan Titik B
R        mulai otomatis
X        berhenti otomatis
H        home
O        buka gripper
I,1      simulasi IR terdeteksi
I,0      simulasi IR bersih
A,<index>,<angle>  atur slider servo
```

Index servo:

```text
0 = gripper
1 = wrist
2 = arm
3 = elbow
4 = base
```

## Memory

Build terakhir:

```text
RAM internal: sekitar 49 KB / 320 KB (15%)
Flash app:    sekitar 1.19 MB / 3 MB (38%)
```

Bluetooth dan BLE tidak dipakai. Karena tidak ada library Bluetooth/BLE pada project, Arduino-ESP32 membebaskan memori Bluetooth saat startup. Mesh dan three.js disimpan dalam `PROGMEM`/flash, bukan RAM internal.

## Struktur Project

```text
src/main.cpp        Firmware, web UI, WebSocket, state machine
src/glb_stl_gz.h    Mesh CAD gzip untuk model 3D
src/three_js_gz.h   Three.js gzip
platformio.ini      Konfigurasi PlatformIO
huge_app.csv        Partisi aplikasi 3 MB
WIRING.md           Wiring, daya, anti-jitter
```

## Troubleshooting

### ESP32 restart

- Lepas semua kabel eksternal; sisakan USB.
- Ganti kabel USB data dan port USB.
- Baca Serial Monitor untuk `Brownout detector was triggered` atau `Guru Meditation Error`.
- Pastikan ESP32 tidak diberi 5 V pada pin `3V3`.

### Servo bergetar, dengung, atau panas

- Pastikan PSU servo terpisah dari ESP32.
- Pastikan GND PSU dan GND ESP32 tersambung.
- Tambahkan kapasitor 1000–2200 µF pada rel servo.
- Uji servo satu per satu tanpa linkage.
- Pastikan servo tidak menekan stopper dan joint tidak seret.

### 3D gagal dimuat

- Pastikan firmware robot terbaru sudah di-upload, bukan firmware test/Blink.
- Buka `http://robotarm.local/three.min.js`; seharusnya file JavaScript termuat.
- Hard refresh `Ctrl+F5`.
- Jika mesh tidak cocok dengan pose fisik, kalibrasi mapping 3D di `src/main.cpp`; ini hanya mengubah visual, bukan sudut servo fisik.

### Upload gagal karena port busy

- Tutup PlatformIO Monitor, Arduino IDE, dan aplikasi serial lain.
- Cabut-pasang USB ESP32.
- Jalankan `pio device list` lalu gunakan port yang benar.

## Lisensi

Belum ditentukan.
