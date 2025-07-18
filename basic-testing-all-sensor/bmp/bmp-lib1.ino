#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// Use I2C interface
Adafruit_BMP280 bmp; // I2C by default

void setup() {
  Serial.begin(115200);
  delay(1000); // Give time for Serial Monitor

  Serial.println("BMP280 Test on Teensy 4.1");

  // Initialize the BMP280
  if (!bmp.begin(0x76)) {  // Change to 0x77 if needed
    Serial.println("Could not find BMP280 sensor! Check wiring or I2C address.");
    while (1); // Halt
  }

  // Optional: Configure settings
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,   // temperature
                  Adafruit_BMP280::SAMPLING_X16,  // pressure
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
}

void loop() {
  Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" °C");

  Serial.print("Pressure = ");
  Serial.print(bmp.readPressure() / 100.0F); // hPa
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(1013.25)); // Standard sea level pressure
  Serial.println(" m");

  Serial.println();
  delay(100);
}
