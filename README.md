# 🚗 Gesture Controlled Car using ESP32 Nano

This project demonstrates a wireless gesture-controlled robotic car built using two **Arduino Nano ESP32** boards. A **gyroscope (MPU6050)** on the transmitter detects hand gestures and sends directional commands to a receiver module, which drives a motor controller to move the car accordingly.

## 📸 Project Overview

- **Transmitter:** Captures hand motion using MPU6050 and sends movement commands over serial/ESP-NOW/Bluetooth.
- **Receiver:** Decodes the transmitted data and drives the DC motors via an L298N or L293D Motor Driver.
- **Car Chassis:** Two geared motors with wheels powered by a battery pack.

---

## 🧰 Hardware Requirements

| Component            | Quantity |
|----------------------|----------|
| Arduino Nano ESP32   | 2        |
| MPU6050 Gyroscope    | 1        |
| L298N Motor Driver   | 1        |
| 6V or 9V DC Motors   | 2        |
| Car Chassis + Wheels | 1 set    |
| 18650 Battery Pack   | 1        |
| Jumper Wires         | As needed|
| Breadboard (optional)| 1        |

---

## 📦 Folder Structure

### 📁 Folder Structure

```bash
gesture-control-car/
├── transmitter/
│   └── transmit_data.ino
├── receiver/
│   └── recieved_data.ino
├── images/
│   ├── receiver_circuit.png
│   └── full_setup.png
└── README.md






---

## 🔌 Circuit Diagram

- Receiver ESP32-Nano is connected to an **L298N motor driver** to drive two DC motors.
- Transmitter ESP32-Nano is connected to an **MPU6050** sensor to detect gesture directions.

📷 Circuit Diagram:  

# Reciever End 
![image](https://github.com/user-attachments/assets/7c019f82-4201-4a8c-8140-7161cd60a515)

# Transmitter End
![image](https://github.com/user-attachments/assets/3906adc6-6e79-42bc-83e2-742cb199c52f)

---

## 📜 How It Works

1. **Transmitter Setup:**
   - Reads pitch/roll from MPU6050.
   - Maps orientation to direction commands (FORWARD, BACKWARD, LEFT, RIGHT).
   - Sends command via serial or wireless.

2. **Receiver Setup:**
   - Receives command.
   - Controls motor direction and speed using the L298N driver.
   - Executes movement (e.g., FORWARD if pitch is forward).

---

## 🚀 Installation

### 1. Arduino IDE Setup
- Install the [ESP32 board package](https://github.com/espressif/arduino-esp32).
- Add libraries:
  - `Wire.h`
  - `Adafruit_MPU6050.h`
  - `Adafruit_Sensor.h`

### 2. Upload Code
- Open `transmit_data.ino` and upload it to the **transmitter ESP32**.
- Open `recieved_data.ino` and upload it to the **receiver ESP32**.

### 3. Wiring Guide

#### Transmitter:
- **MPU6050** → ESP32  
  - VCC → 3.3V  
  - GND → GND  
  - SDA → GPIO21  
  - SCL → GPIO22  

#### Receiver:
- **ESP32** GPIO pins (3, 4, 5, 6, 7, 8) → L298N  
- **L298N OUT1/OUT2** → Motor A  
- **L298N OUT3/OUT4** → Motor B  
- **Power**: 6V or 9V battery to `+12V` of L298N

---

## 🎮 Gesture Mapping

| Hand Tilt | Action     |
|-----------|------------|
| Forward   | Move Forward |
| Backward  | Move Backward |
| Left      | Turn Left |
| Right     | Turn Right |
| Stable    | Stop |

---

## 🧠 Future Improvements

- Use **ESP-NOW** for wireless communication instead of UART.
- Add obstacle detection with **Ultrasonic Sensor**.
- Add smartphone or voice control as fallback.
- Implement speed control via tilt angle (analog mapping).

---





## 👨‍💻 Author

**Tarush Nigam**  
Electronics Engineering  
VJTI, Mumbai 

**Krushna Tarde** 
Electronincs and Telecommunication Engineering 
VJTI, Mumbai

---

## 📄 License

This project is licensed under the MIT License.

