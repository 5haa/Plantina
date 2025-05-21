#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// WiFi credentials
const char* ssid = "Shaa";
const char* password = "shaa5844";

// Server details
String serverUploadEndpoint = "http://192.168.239.58:5000/upload";  // For sending images
String serverCommandEndpoint = "http://192.168.239.58:5000/command?cmd=H&manual=true";  // For command polling

// Arduino communication
#define RXD2 12 // ESP32 RX pin connected to Arduino TX
#define TXD2 13 // ESP32 TX pin connected to Arduino RX
#define SERIAL_TIMEOUT 50 // Timeout for serial operations

// Camera pins for ESP32-CAM AI-THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Set the image quality and frame size for performance optimization
// Using a better resolution for face detection
#define FRAME_SIZE FRAMESIZE_VGA  // VGA (640x480) for better detection
#define JPEG_QUALITY 12           // Higher quality (0-63, lower is better)

// Connection management
unsigned long lastConnectionAttempt = 0;
const unsigned long connectionRetryInterval = 5000; // 5 seconds between connection attempts
bool isConnected = false;
unsigned long lastImageSent = 0;
const unsigned long imageInterval = 100; // ~10 fps (1000ms / 10 = 100ms)

// Heartbeat management
unsigned long lastHeartbeatSent = 0;
const unsigned long heartbeatInterval = 2000; // Send heartbeat every 2 seconds

// Sensor data management
unsigned long lastSensorDataSent = 0;
const unsigned long sensorDataInterval = 1000; // Send sensor data every 1 second

// Arduino sensor data
int moistureValue = 0;
float distanceCm = 0.0;
String robotState = "UNKNOWN";

// Manual command checking
unsigned long lastManualCommandCheck = 0;
const unsigned long manualCommandInterval = 500; // Check for manual commands twice per second

// Status LED
#define BUILTIN_LED 33  // GPIO for the built-in LED on ESP32-CAM
unsigned long lastBlinkTime = 0;
bool blinkState = false;

// Initialization flag
bool arduinoInitialized = false;
unsigned long initializationStartTime = 0;
const unsigned long initializationTimeout = 5000; // 5 seconds timeout

// Camera initialization flag
bool cameraInitialized = false;
unsigned long lastInitSignalSent = 0;
const unsigned long initSignalInterval = 1000; // Send initialization signal every 1 second until confirmed

// Person detection variables
bool personDetected = false;
unsigned long lastPersonCommandSent = 0;
const unsigned long personCommandInterval = 500; // Send person detection command twice a second
unsigned long personDetectionTimeStart = 0;
bool personDetectionConfirmed = false;

// Person detection improvement
unsigned long lastPersonSignalSent = 0;
const unsigned long PERSON_SIGNAL_INTERVAL = 500; // Increased from 300ms to 500ms

// Person direction tracking
char personDirection = 'P'; // Default to center (P), can be 'L' for left, 'R' for right

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector
  
  Serial.begin(115200);
  
  // Add a delay before initializing serial communication with Arduino
  delay(500);
  
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // Serial connection to Arduino
  Serial2.setTimeout(SERIAL_TIMEOUT); // Set timeout for serial operations
  
  // Add delay after serial init for stability
  delay(300);
  
  // Initialize status LED
  pinMode(BUILTIN_LED, OUTPUT);
  // Blink LED rapidly at startup
  for (int i = 0; i < 3; i++) { // Reduced from 5 to 3 blinks
    digitalWrite(BUILTIN_LED, HIGH);
    delay(100);
    digitalWrite(BUILTIN_LED, LOW);
    delay(100);
  }
  
  // Initialize WiFi with a delay for stability
  WiFi.mode(WIFI_STA);
  delay(300); // Increase delay
  connectToWiFi();
  
  // Add additional delay before camera init
  delay(500);
  
  // Start Arduino initialization
  initializationStartTime = millis();
  
  // Camera configuration
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000; // 20MHz for better performance
  config.pixel_format = PIXFORMAT_JPEG;
  
  // Frame size and quality
  config.frame_size = FRAME_SIZE;
  config.jpeg_quality = JPEG_QUALITY;
  config.fb_count = 2; // Use 2 frame buffers for better performance
  
  // Camera initialization with retry mechanism
  Serial.println("Initializing camera...");
  esp_err_t err = esp_camera_init(&config);
  int retries = 0;
  while (err != ESP_OK && retries < 3) {
    Serial.printf("Camera init failed with error 0x%x, retrying...\n", err);
    delay(500);
    err = esp_camera_init(&config);
    retries++;
  }
  
  // Add a delay after camera initialization
  delay(300);
  
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x after %d retries\n", err, retries);
    delay(1000);
    ESP.restart(); // Restart ESP if camera initialization fails
    return;
  }
  
  // Add a short delay after successful camera init
  delay(200);
  
  Serial.println("Camera initialized successfully");
  cameraInitialized = true;
  
  // Send initialization signal to Arduino immediately
  for (int i = 0; i < 3; i++) { // Reduced from 5 to 3 signals
    Serial2.write('I'); // Send initialization command
    delay(50); // Longer pause between signals
  }
  Serial.println("Sent 'I' command to Arduino to signal camera initialization");
  
  // Set additional camera settings for better image quality and speed
  sensor_t * s = esp_camera_sensor_get();
  if (s) {
    // Balance quality and speed
    s->set_framesize(s, FRAME_SIZE);
    s->set_quality(s, JPEG_QUALITY);
    s->set_brightness(s, 1);     // -2 to 2, increasing brightness slightly
    s->set_contrast(s, 1);       // -2 to 2, increasing contrast slightly
    s->set_saturation(s, 1);     // -2 to 2, increasing saturation slightly
    s->set_special_effect(s, 0); // 0 - No Effect
    s->set_whitebal(s, 1);       // 1 = enable
    s->set_awb_gain(s, 1);       // 1 = enable
    s->set_wb_mode(s, 0);        // 0 - Auto
    s->set_exposure_ctrl(s, 1);  // 1 = enable
    s->set_aec2(s, 1);           // 1 = enable additional auto exposure
    s->set_gain_ctrl(s, 1);      // 1 = enable
    s->set_agc_gain(s, 0);       // 0 to 30 - gain
    s->set_gainceiling(s, (gainceiling_t)2);  // 0 to 6 - more gain
    s->set_bpc(s, 1);            // 1 = enable bad pixel correction
    s->set_wpc(s, 1);            // 1 = enable white pixel correction
    s->set_raw_gma(s, 1);        // 1 = enable gamma correction
    s->set_lenc(s, 1);           // 1 = enable lens correction
    s->set_hmirror(s, 0);        // 0 = disable
    s->set_vflip(s, 0);          // 0 = disable
    s->set_dcw(s, 1);            // 1 = enable downsize
  }
}

void connectToWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    isConnected = true;
    return;
  }
  
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  
  // Wait for connection with timeout
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) { // 10 second timeout
    delay(500);
    Serial.print(".");
    timeout++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    isConnected = true;
  } else {
    Serial.println("\nWiFi connection failed");
    isConnected = false;
  }
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Always check for Arduino data - but with timeout protection
  unsigned long serialStartTime = millis();
  checkArduinoData();
  if (millis() - serialStartTime > 100) {
    Serial.println("Warning: Serial operations took too long");
  }
  
  // Update status LED
  updateStatusLED(currentMillis);
  
  // Continuously send initialization signal to Arduino until confirmation is received
  if (cameraInitialized && !arduinoInitialized) {
    if (currentMillis - lastInitSignalSent >= initSignalInterval) {
      lastInitSignalSent = currentMillis;
      Serial2.write('I'); // Send 'I' for camera initialization
      Serial.println("Sending 'I' command to Arduino (camera initialized)");
    }
  }
  
  // Initialize Arduino if not already done
  if (!arduinoInitialized) {
    if (currentMillis - initializationStartTime < initializationTimeout) {
      // Still in initialization phase, send initial heartbeat
      if (currentMillis - lastHeartbeatSent >= 500) { // Faster heartbeat during initialization
        lastHeartbeatSent = currentMillis;
        Serial2.write('H'); // Send heartbeat signal to Arduino
        Serial.println("Sending initialization heartbeat to Arduino...");
      }
    } else {
      // Timeout reached, assume Arduino is ready
      arduinoInitialized = true;
      Serial.println("Arduino initialization complete or timed out");
    }
    
    // During initialization, don't proceed with other operations
    delay(10);
    return;
  }
  
  // Check WiFi connection and reconnect if needed
  if (WiFi.status() != WL_CONNECTED) {
    isConnected = false;
    // Only attempt reconnection every connectionRetryInterval
    if (currentMillis - lastConnectionAttempt > connectionRetryInterval) {
      lastConnectionAttempt = currentMillis;
      Serial.println("WiFi disconnected, attempting to reconnect...");
      connectToWiFi();
    }
    delay(10); // Short delay before checking again
    return;  // Skip the rest of the loop if not connected
  }
  
  // Send heartbeat to Arduino periodically
  if (currentMillis - lastHeartbeatSent >= heartbeatInterval) {
    lastHeartbeatSent = currentMillis;
    Serial2.write('H'); // Send heartbeat signal to Arduino
    Serial.println("Sent heartbeat to Arduino");
  }
  
  // Send sensor data periodically
  if (currentMillis - lastSensorDataSent >= sensorDataInterval) {
    lastSensorDataSent = currentMillis;
    sendSensorData();
  }
  
  // Only send images at a controlled rate
  if (currentMillis - lastImageSent < imageInterval) {
    delay(1); // Very small delay to prevent CPU overload
    return;
  }
  
  lastImageSent = currentMillis;
  
  camera_fb_t * fb = NULL;
  
  // Capture frame with minimal retry
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    esp_camera_fb_return(fb); // Still try to return in case of partial allocation
    delay(10);
    return;
  }
  
  // Send image to server
  HTTPClient http;
  http.begin(serverUploadEndpoint);
  http.addHeader("Content-Type", "image/jpeg");
  http.setTimeout(1000); // Shorter timeout for faster response
  
  // Use a faster POST method - don't wait for response data
  int httpResponseCode = http.sendRequest("POST", (uint8_t*)fb->buf, fb->len);
  
  // Minimal response processing for speed
  if (httpResponseCode > 0) {
    String response = http.getString();
    
    // Process response - added better debugging output
    Serial.print("Server response: ");
    Serial.println(response);
    
    // Check for directional person detection responses
    if (response.indexOf("person_left") >= 0) {
      personDetected = true;
      personDirection = 'L';
      personDetectionTimeStart = currentMillis;
      personDetectionConfirmed = false;
      Serial.println("PERSON DETECTED ON LEFT - Sending 'L' signal to Arduino");
      
      // Simplified command sending (reduced from 5 to 2)
      Serial2.write('L');
      delay(5);
      Serial2.write('L');
    }
    else if (response.indexOf("person_right") >= 0) {
      personDetected = true;
      personDirection = 'R';
      personDetectionTimeStart = currentMillis;
      personDetectionConfirmed = false;
      Serial.println("PERSON DETECTED ON RIGHT - Sending 'R' signal to Arduino");
      
      // Simplified command sending
      Serial2.write('R');
      delay(5);
      Serial2.write('R');
    }
    else if (response.indexOf("person_center") >= 0 || response.indexOf("person_detected") >= 0) {
      // Handle both specific "person_center" and generic "person_detected" for backward compatibility
      personDetected = true;
      personDirection = 'P';
      personDetectionTimeStart = currentMillis;
      personDetectionConfirmed = false;
      Serial.println("PERSON DETECTED IN CENTER - Sending 'P' signal to Arduino");
      
      // Simplified command sending
      Serial2.write('P');
      delay(5);
      Serial2.write('P');
    }
    else if (response.indexOf("no_person") >= 0) {
      // Send no-person signal for all no-person responses
      personDetected = false;
      Serial.println("NO PERSON DETECTED - Sending signal to Arduino");
      
      // Simplified command sending
      Serial2.write('N');
      delay(5);
      Serial2.write('N');
    }
    // Check for command_ prefix for other commands
    else if (response.startsWith("command_")) {
      char cmd = response.charAt(8); // Get the character after "command_"
      if (cmd) {
        Serial2.write(cmd);
        Serial.print("Forwarding command: ");
        Serial.println(cmd);
      }
    }
  }
  
  http.end();
  
  // Return the frame buffer back to be reused
  esp_camera_fb_return(fb);
  
  // Resend person detection commands if not confirmed - with less aggressive timing
  if (personDetected && !personDetectionConfirmed) {
    if (currentMillis - personDetectionTimeStart < 3000) { // Reduced from 5 to 3 seconds
      if (currentMillis - lastPersonCommandSent > personCommandInterval) {
        lastPersonCommandSent = currentMillis;
        Serial2.write(personDirection); // Send the appropriate direction command
        Serial.print("Resending '");
        Serial.print(personDirection);
        Serial.println("' command - person detection not confirmed");
      }
    } else {
      // Give up after 3 seconds
      personDetectionConfirmed = true;
    }
  }
  
  // Send periodic person detection signals with reduced frequency
  if (personDetected && (millis() - lastPersonSignalSent > PERSON_SIGNAL_INTERVAL)) {
    lastPersonSignalSent = millis();
    Serial2.write(personDirection);
    Serial.print("Resending '");
    Serial.print(personDirection);
    Serial.println("' signal for person detection");
  }
}

void updateStatusLED(unsigned long currentMillis) {
  // If Arduino not initialized, blink quickly
  if (!arduinoInitialized) {
    if (currentMillis - lastBlinkTime > 200) {
      lastBlinkTime = currentMillis;
      blinkState = !blinkState;
      digitalWrite(BUILTIN_LED, blinkState);
    }
  }
  // If WiFi not connected, blink slowly
  else if (!isConnected) {
    if (currentMillis - lastBlinkTime > 1000) {
      lastBlinkTime = currentMillis;
      blinkState = !blinkState;
      digitalWrite(BUILTIN_LED, blinkState);
    }
  }
  // If everything is connected, LED stays on
  else {
    digitalWrite(BUILTIN_LED, HIGH);
  }
}

void checkArduinoData() {
  // Limit the amount of time spent in this function
  unsigned long startTime = millis();
  int dataCount = 0;
  
  // Check if there is data available from Arduino, with timeout protection
  while (Serial2.available() > 0 && (millis() - startTime < 50) && dataCount < 5) {
    String data = Serial2.readStringUntil('\n');
    dataCount++;
    data.trim();
    
    // If we receive any data from Arduino, mark as initialized
    if (!arduinoInitialized) {
      arduinoInitialized = true;
      Serial.println("Arduino initialization confirmed by receiving data");
      
      // Send 'I' command again to ensure Arduino knows camera is initialized
      if (cameraInitialized) {
        Serial2.write('I');
        Serial.println("Re-sending 'I' command to confirm camera initialization");
      }
    }
    
    // Parse sensor data from Arduino (format: "SENSOR:moisture:distance:state")
    if (data.startsWith("SENSOR:")) {
      int firstColon = data.indexOf(':', 0);
      int secondColon = data.indexOf(':', firstColon + 1);
      int thirdColon = data.indexOf(':', secondColon + 1);
      
      if (firstColon > 0 && secondColon > 0 && thirdColon > 0) {
        String moistureStr = data.substring(firstColon + 1, secondColon);
        String distanceStr = data.substring(secondColon + 1, thirdColon);
        String stateStr = data.substring(thirdColon + 1);
        
        moistureValue = moistureStr.toInt();
        distanceCm = distanceStr.toFloat();
        robotState = stateStr;
        
        Serial.print("Received sensor data - Moisture: ");
        Serial.print(moistureValue);
        Serial.print(", Distance: ");
        Serial.print(distanceCm);
        Serial.print(", State: ");
        Serial.println(robotState);
      }
    }
    // Check for camera initialization acknowledgment
    else if (data.indexOf("CAMERA INITIALIZED") >= 0) {
      Serial.println("Arduino acknowledged camera initialization");
    }
    // Check for person detection acknowledgment
    else if (data.indexOf("PERSON DETECTED CONFIRMED") >= 0) {
      personDetectionConfirmed = true;
      Serial.println("Arduino acknowledged person detection");
    }
  }
  
  // If we spent too much time reading data, log a warning
  if (millis() - startTime > 50) {
    Serial.println("Warning: Spent too much time reading Arduino data");
  }
}

void sendSensorData() {
  if (!isConnected) return;
  
  // Create the sensor data string
  String sensorData = "SENSOR:" + String(moistureValue) + ":" + 
                      String(distanceCm) + ":" + robotState;
  
  // Send the data
  HTTPClient http;
  http.begin(serverUploadEndpoint);
  http.addHeader("Content-Type", "text/plain");
  
  int httpResponseCode = http.sendRequest("POST", sensorData.c_str());
  if (httpResponseCode > 0) {
    Serial.print("Sensor data sent, response: ");
    Serial.println(http.getString());
  } else {
    Serial.print("Error sending sensor data: ");
    Serial.println(httpResponseCode);
  }
  
  http.end();
}
