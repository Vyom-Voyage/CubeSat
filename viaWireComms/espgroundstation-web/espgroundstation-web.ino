#include <ESP8266WiFi.h>
extern "C" {
  #include <espnow.h>
}

struct SensorData {
  float roll, pitch, yaw;
  float q0, q1, q2, q3;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float magX, magY, magZ;
  float temperature_aht;
  float humidity;
  float temperature_bmp;
  float pressure;
  float altitude_bmp;
  double latitude;
  double longitude;
  float altitude_gps;
  float speed;
  int satellites;
  bool gps_valid;
  unsigned long timestamp;
  bool imu_valid;
  bool environmental_valid;
  bool gps_data_valid;
};

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 100; // Print every 100ms for smooth visualization

void onReceive(uint8_t *mac, uint8_t *data, uint8_t len) {
  SensorData received;
  memcpy(&received, data, sizeof(SensorData));
  
  // Only print if enough time has passed (rate limiting)
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= printInterval) {
    
    Serial.println("=== Struct Received ===");
    
    // Orientation data
    Serial.print("Roll: "); Serial.println(received.roll, 2);
    Serial.print("Pitch: "); Serial.println(received.pitch, 2);
    Serial.print("Yaw: "); Serial.println(received.yaw, 2);
    Serial.print("Q0: "); Serial.println(received.q0, 4);
    Serial.print("Q1: "); Serial.println(received.q1, 4);
    Serial.print("Q2: "); Serial.println(received.q2, 4);
    Serial.print("Q3: "); Serial.println(received.q3, 4);
    
    // Accelerometer data
    Serial.print("AccelX: "); Serial.println(received.accelX, 2);
    Serial.print("AccelY: "); Serial.println(received.accelY, 2);
    Serial.print("AccelZ: "); Serial.println(received.accelZ, 2);
    
    // Gyroscope data
    Serial.print("GyroX: "); Serial.println(received.gyroX, 2);
    Serial.print("GyroY: "); Serial.println(received.gyroY, 2);
    Serial.print("GyroZ: "); Serial.println(received.gyroZ, 2);
    
    // Magnetometer data
    Serial.print("MagX: "); Serial.println(received.magX, 2);
    Serial.print("MagY: "); Serial.println(received.magY, 2);
    Serial.print("MagZ: "); Serial.println(received.magZ, 2);
    
    // Environmental data
    Serial.print("Temp AHT: "); Serial.println(received.temperature_aht, 1);
    Serial.print("Humidity: "); Serial.println(received.humidity, 1);
    Serial.print("Temp BMP: "); Serial.println(received.temperature_bmp, 1);
    Serial.print("Pressure: "); Serial.println(received.pressure, 1);
    Serial.print("Altitude BMP: "); Serial.println(received.altitude_bmp, 1);
    
    // GPS data
    Serial.print("Latitude: "); Serial.println(received.latitude, 6);
    Serial.print("Longitude: "); Serial.println(received.longitude, 6);
    Serial.print("GPS Alt: "); Serial.println(received.altitude_gps, 1);
    Serial.print("Speed: "); Serial.println(received.speed, 1);
    Serial.print("Satellites: "); Serial.println(received.satellites);
    Serial.print("GPS Valid: "); Serial.println(received.gps_valid ? "true" : "false");
    
    // Status flags
    Serial.print("IMU Valid: "); Serial.println(received.imu_valid ? "true" : "false");
    Serial.print("Env Valid: "); Serial.println(received.environmental_valid ? "true" : "false");
    Serial.print("GPS Data Valid: "); Serial.println(received.gps_data_valid ? "true" : "false");
    
    // Timestamp
    Serial.print("Timestamp: "); Serial.println(received.timestamp);
    
    Serial.println("=== End Data ===");
    Serial.println();
    
    lastPrintTime = currentTime;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("=== VYOM VOYAGE CUBESAT GROUND STATION ===");
  Serial.println("Initializing ESP-NOW receiver...");
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  if (esp_now_init() != 0) {
    Serial.println("ESP-NOW init failed!");
    return;
  }
  
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(onReceive);
  
  Serial.print("ESP8266 MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Ready to receive CubeSat data...");
  Serial.println("Connect this device to your computer via USB");
  Serial.println("Open Chrome browser and connect to serial port");
  Serial.println();
}

void loop() {
  // Keep the loop light - all work is done in the callback
  delay(10);
}