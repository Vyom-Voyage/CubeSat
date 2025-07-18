#include <ESP8266WiFi.h>
extern "C" {
  #include <espnow.h>
}
#include <SoftwareSerial.h>

SoftwareSerial ss(4, 5); // RX = GPIO4, TX = GPIO5

// MAC Address of ESP2 (replace with your actual MAC)
uint8_t peerMac[] = {0xEC, 0x64, 0xC9, 0xCE, 0x06, 0x3D};
//ec:64:c9:ce:06:3d
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

SensorData incoming;
uint8_t buffer[sizeof(SensorData)];
size_t bufIndex = 0;

void onSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Delivery Status: ");
  Serial.println(sendStatus == 0 ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);
  ss.begin(9600);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != 0) {
    Serial.println("ESP-NOW init failed!");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(onSent);
  esp_now_add_peer(peerMac, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  Serial.println("ESP1 ready to send via ESP-NOW");
}

void loop() {
  while (ss.available()) {
    buffer[bufIndex++] = ss.read();
    if (bufIndex == sizeof(SensorData)) {
      memcpy(&incoming, buffer, sizeof(SensorData));
      esp_now_send(peerMac, (uint8_t*)&incoming, sizeof(SensorData));
      bufIndex = 0;
    }
    if (bufIndex > sizeof(SensorData)) bufIndex = 0;
  }
}

// void setup() {
//   pinMode(LED_BUILTIN, OUTPUT);  // Set LED pin as output
// }

// void loop() {
//   digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (LOW is ON for ESP)
//   delay(500);                       // Wait 500 ms
//   digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off (HIGH is OFF)
//   delay(500);                       // Wait 500 ms
// }
