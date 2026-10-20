# Robot Arm 5-DOF - Arduino IDE Sketch

Sketch ini adalah konversi dari proyek PlatformIO (`src/main.cpp`) ke format Arduino IDE (`.ino`).

## Cara pakai di Arduino IDE 2.3.10

1. Install **Board package "esp32" by Espressif** via Boards Manager.
2. Pilih board: **DOIT ESP32 DEVKIT V1** (atau **ESP32 Dev Module**).
3. Install library **ESP32Servo** by Kevin Harrington / John K. Bennett via Library Manager.
4. Buka folder `robot_arm` (pastikan nama folder sama dengan nama file `.ino`).
5. Upload ke board.
6. Buka Serial Monitor dengan baud rate **115200**.

## Catatan

- Pin mapping dan perilaku sama persis dengan versi PlatformIO.
- Pastikan folder sketch bernama `robot_arm` agar Arduino IDE mengenali file `robot_arm.ino`.
