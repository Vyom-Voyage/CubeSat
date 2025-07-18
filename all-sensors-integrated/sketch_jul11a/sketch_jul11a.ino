#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>

// Use I2C interface

TinyGPSPlus gps;

Adafruit_AHTX0 aht;

Adafruit_BMP280 bmp; // I2C by default



void setup() {
  Serial.begin(115200);
  delay(1000);

  bmp_setup();
  aht_setup();
  gps_setup();

}

void aht_setup() {

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
  // gps_data();
  delay(100);
}




void bmp_setup() {
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




void gps_setup() {
  Serial1.begin(9600);      // Serial1 = UART RX1 on Pin 0 (connect GPS TX to this)
  
  Serial.println("NEO-6M GPS Test with Teensy 4.1");
}

void gps_data() {
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  if (gps.location.isUpdated()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);

    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);

    Serial.print("Altitude: ");
    Serial.print(gps.altitude.meters());
    Serial.println(" m");

    Serial.print("Speed: ");
    Serial.print(gps.speed.kmph());
    Serial.println(" km/h");

    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());

    Serial.print("Date: ");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.println(gps.date.year());

    Serial.print("Time (UTC): ");
    Serial.print(gps.time.hour());
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.println(gps.time.second());

    Serial.println("-------------------------");
  }
}

