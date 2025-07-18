#include <Wire.h>

void setup() {
  Wire.begin();             // Use Wire1 or Wire2 if needed
  Serial.begin(115200);
  while (!Serial);          // Wait for Serial Monitor to open
  Serial.println("I2C Scanner Starting...");
}

void loop() {
  byte error, address;
  int found = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      found++;
    }
  }

  if (found == 0)
    Serial.println("No I2C devices found.");
  else
    Serial.println("Scan complete.");

  delay(2000);  // Scan every 2 seconds
}
