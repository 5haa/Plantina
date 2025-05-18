#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// WiFi credentials
const char* ssid = "Shaa";
const char* password = "shaa5844";

// Server details
String serverName = "http://192.168.239.58:5000/upload";

// Arduino communication
#define RXD2 12 // ESP32 RX pin connected to Arduino TX
#define TXD2 13 // ESP32 TX pin connected to Arduino RX

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

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector
  
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // Serial connection to Arduino
  
  // Initialize WiFi with a delay for stability
  WiFi.mode(WIFI_STA);
  delay(100);
  connectToWiFi();
  
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
  esp_err_t err = esp_camera_init(&config);
  int retries = 0;
  while (err != ESP_OK && retries < 3) {
    Serial.printf("Camera init failed with error 0x%x, retrying...\n", err);
    delay(500);
    err = esp_camera_init(&config);
    retries++;
  }
  
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x after %d retries\n", err, retries);
    delay(1000);
    ESP.restart(); // Restart ESP if camera initialization fails
    return;
  }
  
  Serial.println("Camera initialized successfully");
  
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
  // Check WiFi connection and reconnect if needed
  if (WiFi.status() != WL_CONNECTED) {
    isConnected = false;
    // Only attempt reconnection every connectionRetryInterval
    unsigned long currentMillis = millis();
    if (currentMillis - lastConnectionAttempt > connectionRetryInterval) {
      lastConnectionAttempt = currentMillis;
      Serial.println("WiFi disconnected, attempting to reconnect...");
      connectToWiFi();
    }
    delay(10); // Short delay before checking again
    return;  // Skip the rest of the loop if not connected
  }
  
  // Only send images at a controlled rate
  unsigned long currentMillis = millis();
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
  http.begin(serverName);
  http.addHeader("Content-Type", "image/jpeg");
  http.setTimeout(1000); // Shorter timeout for faster response
  
  // Use a faster POST method - don't wait for response data
  int httpResponseCode = http.sendRequest("POST", (uint8_t*)fb->buf, fb->len);
  
  // Minimal response processing for speed
  if (httpResponseCode > 0) {
    String response = http.getString();
    
    // Minimal command handling
    if (response.indexOf("person_detected") >= 0) {
      Serial2.write('P'); // P for Person detected
    } else {
      Serial2.write('N'); // N for No person
    }
  }
  
  http.end();
  
  // Return the frame buffer back to be reused
  esp_camera_fb_return(fb);
} 