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

void onReceive(uint8_t *mac, uint8_t *data, uint8_t len) {
  SensorData received;
  memcpy(&received, data, sizeof(SensorData));

  Serial.println("=== Struct Received ===");
  Serial.print("Roll: "); Serial.println(received.roll);
  Serial.print("Temp AHT: "); Serial.println(received.temperature_aht);
  Serial.print("Satellites: "); Serial.println(received.satellites);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != 0) {
    Serial.println("ESP-NOW init failed!");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(onReceive);

  Serial.print("ESP2 MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void loop() {}
