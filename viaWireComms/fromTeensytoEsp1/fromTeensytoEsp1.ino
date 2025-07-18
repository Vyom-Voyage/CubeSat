#include <SoftwareSerial.h>

SoftwareSerial ss(4, 5); // RX = GPIO4 (D2), TX = GPIO5 (D1) → RX only used

struct SensorData {
  float roll, pitch, yaw;
  float temperature;
  int satellites;
};

SensorData incoming;
uint8_t buffer[sizeof(SensorData)];
size_t bufIndex = 0;

void setup() {
  Serial.begin(115200);    // Debug output
  ss.begin(9600);          // SoftwareSerial to Teensy
  Serial.println("ESP8266 ready to receive struct from Teensy Serial2...");
}

void loop() {
  while (ss.available()) {
    buffer[bufIndex++] = ss.read();

    if (bufIndex == sizeof(SensorData)) {
      memcpy(&incoming, buffer, sizeof(SensorData));

      Serial.println("=== Struct Received ===");
      Serial.print("Roll: "); Serial.println(incoming.roll);
      Serial.print("Pitch: "); Serial.println(incoming.pitch);
      Serial.print("Yaw: "); Serial.println(incoming.yaw);
      Serial.print("Temp: "); Serial.println(incoming.temperature);
      Serial.print("Satellites: "); Serial.println(incoming.satellites);
      Serial.println();

      bufIndex = 0; // Reset for next struct
    }
  }
}
