struct SensorData {
  float roll, pitch, yaw;
  float temperature;
  int satellites;
};

SensorData dummy = {1.23, 2.34, 3.45, 29.5123, 100};

void setup() {
  Serial.begin(115200);      // USB Serial for debug
  Serial2.begin(9600);       // Use TX2 (Pin 8), RX2 (Pin 7) to communicate with ESP
  while (!Serial);
  Serial.println("Teensy sending struct via Serial2...");
}

void loop() {
  Serial2.write((uint8_t*)&dummy, sizeof(SensorData));  // Send struct via Serial2
  Serial.println("Sent struct to ESP8266 via Serial2");
  delay(1000);
}
