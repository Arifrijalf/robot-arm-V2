# Web Simulasi 3D Robot Arm 5-DOF (ESP32 Kinematics Simulator)

Simulasi 3D interaktif ini dibuat khusus agar Anda dapat menguji, memvisualisasikan, dan merasakan dinamika pergerakan robot arm 5-DOF secara real-time langsung di browser web (Laptop maupun HP/Smartphone) sementara proses 3D printing berlangsung.

Simulasi ini menerapkan **logika matematika, rate-control, dan slew-rate limiter yang 100% identik dengan kode C++ di firmware ESP32 (`src/main.cpp`)**.

---

## 🚀 Cara Membuka Simulasi

### Cara 1: Buka File Langsung di Browser (Paling Praktis)
1. Buka File Explorer di komputer Anda.
2. Masuk ke folder: `PlatformIO/Projects/robot arm/simulation/`.
3. Klik dua kali (*double-click*) file **`index.html`** untuk membukanya di Google Chrome, Microsoft Edge, atau Firefox.

### Cara 2: Buka di Smartphone (HP)
Jika ingin membukanya di HP dalam satu jaringan Wi-Fi yang sama:
- Gunakan ekstensi VS Code **Live Server** di laptop, lalu buka alamat IP lokal laptop (misal: `http://192.168.1.x:5500/simulation/`) di browser HP Anda.

---

## 🎥 Kontrol Kamera 3D (Laptop & HP)

Kamera 3D sekarang telah dioptimalkan penuh untuk sentuhan jari layar sentuh (*touchscreen*) maupun mouse:

| Aksi | Kontrol di Laptop / PC | Kontrol di HP / Tablet |
|---|---|---|
| **Putar Sudut Pandang 3D (Orbit)** | **Klik Kiri + Geser Mouse** di area luar tombol/joystick | **Geser dengan 1 Jari** di area layar |
| **Zoom In / Zoom Out** | **Scroll Roda Mouse** | **Cubit dengan 2 Jari (Pinch-to-zoom)** |
| **Geser Posisi Kamera (Pan)** | **Klik Kanan + Geser Mouse** | **Geser dengan 2 Jari** |

### Tombol Cepat Kamera di Header:
- **`🔄 Reset Cam`**: Mengembalikan posisi kamera ke sudut pandang diagonal awal.
- **`👁️ Depan`**: Sudut pandang lurus dari depan robot.
- **`📐 Atas`**: Sudut pandang *top-down* dari atas.
- **`🎛️ Panel`**: Menampilkan atau menyembunyikan panel kontrol (sangat praktis di HP agar tampilan 3D menjadi layar penuh).

---

## 🎮 Kontrol Lengan Robot

### 1. Dual Virtual Joystick (Pada Layar)
- **Joystick 1 (Kiri)**:
  - Sumbu Horizontal (X): Mengendalikan **Base (GPIO 26)** (Rotasi dasar kiri/kanan).
  - Sumbu Vertikal (Y): Mengendalikan **Arm / Shoulder (GPIO 14)** (Maju/Mundur).
- **Joystick 2 (Kanan)**:
  - Sumbu Horizontal (X): Mengendalikan **Elbow (GPIO 27)** (Siku naik/turun).
  - Sumbu Vertikal (Y): Mengendalikan **Wrist (GPIO 12)** (Pergelangan tangan tekuk atas/bawah).
- **Tombol Tengah (🤏 GPIO 4)**: Membuka dan menutup Gripper secara bergantian (*toggle*).

### 2. Shortcut Keyboard (Laptop)
- <kbd>A</kbd> / <kbd>D</kbd> : Putar Base Kiri / Kanan
- <kbd>W</kbd> / <kbd>S</kbd> : Lengan Arm Maju / Mundur
- <kbd>←</kbd> / <kbd>→</kbd> : Siku Elbow Naik / Turun
- <kbd>↑</kbd> / <kbd>↓</kbd> : Wrist Tekuk Atas / Bawah
- <kbd>Space</kbd> : Buka / Tutup Gripper (Toggle)
- <kbd>H</kbd> : Posisi Home (90°)

---

## ⚙️ Fitur Tambahan
1. **Slew Rate Limiter (0.85°/tick)**: Gerakan servo halus tanpa lonjakan mendadak, persis dengan karakteristik servo SG90 asli.
2. **Rate / Velocity Control**: Posisi servo terkunci saat joystick dilepas (tidak kembali ke tengah).
3. **Telemetri Real-Time**: Menampilkan sudut aktual, target, dan konversi sinyal PWM mikrodetik (`500µs – 2400µs`).
