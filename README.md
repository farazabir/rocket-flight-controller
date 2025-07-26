# FarazFlight: Pitch-Roll Tracker

A lightweight ESP32-based pitch and roll tracking system using the MPU9250 sensor. Built by Faraz Ahmed Abir under FarazInc, for model rocket attitude sensing.

## 🔧 Features
- MPU9250 (Accel + Gyro) over I2C
- Real-time pitch & roll estimation
- Gyroscope auto-calibration
- Complementary filter for accuracy
- Serial output for debugging/logging

## 📦 Hardware Setup

| Sensor Pin | ESP32 Pin |
|------------|-----------|
| VCC        | 3.3V      |
| GND        | GND       |
| SDA        | GPIO 21   |
| SCL        | GPIO 22   |

*Use 4.7kΩ pull-ups on SDA and SCL to 3.3V*

## 🛠️ Getting Started
1. Connect MPU9250 as shown.
2. Install `MPU9250_asukiaaa` library.
3. Upload `rocket_pitch_roll.ino` to your ESP32.
4. Open Serial Monitor at 115200 baud.
5. Tilt the board and observe pitch/roll angles.

## 🖼️ Example Output

Pitch: -1.35°, Roll: 0.42°

![Setup Photo](images/mpusetup.jpg)
