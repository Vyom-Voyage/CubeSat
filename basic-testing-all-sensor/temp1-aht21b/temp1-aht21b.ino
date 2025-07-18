#include <Wire.h>
#include <Adafruit_AHTX0.h>

Adafruit_AHTX0 aht;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("AHT21B Sensor Test on Teensy 4.1");

  if (!aht.begin()) {
    Serial.println("Could not find AHT21B sensor! Check wiring.");
    while (1) delay(10);
  } 

  Serial.println("AHT21B found");
}

void loop() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);  // populate temp and humidity

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");

  Serial.print("Humidity: ");
  Serial.print(humidity.relative_humidity);
  Serial.println(" %");

  Serial.println();
  delay(1000);
}
