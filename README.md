# Plantina - ESP32-CAM Person Detection with Arduino Motor Control

This project integrates an ESP32-CAM module with an Arduino Uno and motor shield to create a smart robot that can detect people and respond accordingly.

## Components

1. **Arduino Uno with Motor Shield** - Controls 4 DC motors for movement
2. **ESP32-CAM Module** - Captures images and sends them to the Python server
3. **Python Server** - Processes images, detects people, and sends commands back to Arduino

## Wiring Instructions

### Arduino to Motor Shield
- The motor shield stacks directly on top of the Arduino Uno

### ESP32-CAM to Arduino
- ESP32-CAM TX (GPIO1) → Arduino RX
- ESP32-CAM RX (GPIO3) → Arduino TX via voltage divider (ESP32 is 3.3V, Arduino is 5V)
- ESP32-CAM GND → Arduino GND
- ESP32-CAM 5V → Arduino 5V

Note: For programming the ESP32-CAM, you'll need an FTDI programmer.

## Setup Instructions

### 1. Arduino Setup
1. Upload the `sketch` file to your Arduino Uno
2. Connect the motors to the motor shield (M1, M2, M3, M4)

### 2. ESP32-CAM Setup
1. Install the ESP32 board in Arduino IDE: 
   - Add `https://dl.espressif.com/dl/package_esp32_index.json` to Additional Board Manager URLs
   - Install ESP32 from Boards Manager
2. Upload `esp32_cam.ino` to the ESP32-CAM using an FTDI programmer
3. Update the WiFi credentials and server IP in the code before uploading

### 3. Python Server Setup
1. Install the required packages:
   ```
   pip install -r requirements.txt
   ```
2. Update the COM port in `server.py` to match your Arduino's port
3. Run the server:
   ```
   python server.py
   ```
4. Access the web interface at `http://localhost:5000` or `http://[your-computer-ip]:5000`

## How It Works

1. The ESP32-CAM captures images and sends them to the Python server
2. The server processes the images and detects people using OpenCV
3. If a person is detected, the server sends a command to the Arduino
4. The Arduino controls the motors based on the received commands
5. The web interface displays the live feed with detection results

## Customization

- Modify the Arduino code to change how the robot responds to person detection
- Update the ESP32-CAM code to change the image capture frequency or quality
- Enhance the Python server to add more advanced detection capabilities

## Troubleshooting

- If the ESP32-CAM isn't connecting to WiFi, double-check the credentials
- If the Arduino isn't responding to commands, verify the serial connection
- If person detection isn't working well, adjust the detection parameters in the Python code 