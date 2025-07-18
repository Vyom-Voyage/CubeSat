/*
 * Unified Multi-Sensor Data Collection for Teensy 4.1
 * Combines ICM-20948 9DoF IMU, AHT21B, BMP280, and GPS NEO-6M
 * All sensor data unified into one struct for easy communication
 */

#include <Wire.h>
#include <ICM_20948.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>

// Sensor objects
ICM_20948_I2C myICM;
TinyGPSPlus gps;
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

#define LED_PIN 36
#define ESP_RST_PIN 21 // Teensy pin connected to ESP8266's RST pin


// Unified sensor data structure
struct SensorData {
  // IMU Orientation Data
  float roll;
  float pitch;
  float yaw;
  
  // Quaternion representation
  float q0, q1, q2, q3;  // w, x, y, z
  
  // Raw IMU data
  float accelX, accelY, accelZ;  // in g
  float gyroX, gyroY, gyroZ;     // in degrees/second
  float magX, magY, magZ;        // in uT
  
  // Environmental data
  float temperature_aht;         // from AHT21B in °C
  float humidity;                // from AHT21B in %
  float temperature_bmp;         // from BMP280 in °C
  float pressure;                // from BMP280 in hPa
  float altitude_bmp;            // from BMP280 in meters
  
  // GPS data
  double latitude;               // in degrees
  double longitude;              // in degrees
  float altitude_gps;            // in meters
  float speed;                   // in km/h
  int satellites;                // number of satellites
  bool gps_valid;                // GPS fix status
  
  // Timestamp
  unsigned long timestamp;       // in milliseconds
  
  // Data validity flags
  bool imu_valid;
  bool environmental_valid;
  bool gps_data_valid;
};

// Global sensor data instance
SensorData sensorData;

// IMU calculation variables
float compAngleX = 0, compAngleY = 0;
float gyroAngleX = 0, gyroAngleY = 0, gyroAngleZ = 0;
unsigned long timer;
float dt;

// IMU calibration offsets
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);      // TX2 = Pin 8 to ESP8266 RX (D2)
  pinMode(LED_PIN, OUTPUT);
  pinMode(ESP_RST_PIN, OUTPUT); // Configure ESP_RST_PIN as an output

  // while (!Serial) delay(10);
  
  Serial.println("=== Unified Multi-Sensor System ===");
  Serial.println("Initializing sensors...");
    // --- ESP8266 Reset Sequence ---
  Serial.println("Resetting ESP8266...");
  digitalWrite(ESP_RST_PIN, LOW);  // Pull RST pin low to reset ESP
  delay(100);                      // Hold low for a short period (e.g., 100ms)
  digitalWrite(ESP_RST_PIN, HIGH); // Release RST pin (ESP starts booting)
  delay(1000);                     // Give ESP time to boot up (adjust as needed)
  Serial.println("ESP8266 reset complete.");
  // --- End ESP8266 Reset Sequence ---
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C
  
  // Initialize all sensors
  imu_setup();
  environmental_setup();
  gps_setup();
  
  Serial.println("All sensors initialized successfully!");
  Serial.println("Starting data collection...");
  
  timer = micros();
}

void loop() {
  // Clear previous data validity flags
  sensorData.imu_valid = false;
  sensorData.environmental_valid = false;
  sensorData.gps_data_valid = false;
  
  // Set timestamp
  sensorData.timestamp = millis();
  
  // Read IMU data
  readIMUData();
  
  // Read environmental data
  readEnvironmentalData();
  
  // Read GPS data
  readGPSData();
  
  // Output unified data
  outputUnifiedData();

  if (sensorData.imu_valid   && sensorData.environmental_valid ){
    digitalWrite(LED_PIN, HIGH);  // Turn LED on
    Serial.println("Led is on");
  }
  else{
    digitalWrite(LED_PIN, LOW);   // Turn LED off
    Serial.println("Led is off");

  }

  // Send the struct to ESP8266
  Serial2.write((uint8_t*)&sensorData, sizeof(SensorData));

  
  delay(20); // 100Hz update rate
}

void imu_setup() {
  Serial.println("Initializing ICM-20948 IMU...");
  
  // Initialize the ICM-20948
  bool initialized = false;
  while (!initialized) {
    myICM.begin(Wire, 1);
    
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.println("Trying to connect to ICM-20948...");
      delay(500);
    } else {
      initialized = true;
      Serial.println("ICM-20948 connected successfully!");
    }
  }
  
  // Start up magnetometer
  Serial.println("Enabling magnetometer...");
  myICM.startupMagnetometer();
  if (myICM.status != ICM_20948_Stat_Ok) {
    Serial.print("Magnetometer startup issue: ");
    Serial.println(myICM.statusString());
  }
  
  // Calibrate gyroscope
  Serial.println("Calibrating gyroscope... Keep sensor still for 3 seconds!");
  calibrateGyro();
  Serial.println("IMU calibration complete!");
}

void environmental_setup() {
  Serial.println("Initializing environmental sensors...");
  
  // Initialize AHT21B
  if (!aht.begin()) {
    Serial.println("Could not find AHT21B sensor! Check wiring.");
    while (1) delay(10);
  }
  Serial.println("AHT21B found");
  
  // Initialize BMP280
  if (!bmp.begin(0x76)) { // Change to 0x77 if needed
    Serial.println("Could not find BMP280 sensor! Check wiring or I2C address.");
    while (1);
  }
  
  // Configure BMP280 settings
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,     // temperature
                  Adafruit_BMP280::SAMPLING_X16,    // pressure
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  
  Serial.println("Environmental sensors initialized!");
}

void gps_setup() {
  Serial1.begin(9600); // Serial1 = UART RX1 on Pin 0 (connect GPS TX to this)
  Serial.println("NEO-6M GPS initialized!");
}

void calibrateGyro() {
  int numReadings = 200;
  float sumX = 0, sumY = 0, sumZ = 0;
  
  for (int i = 0; i < numReadings; i++) {
    while (!myICM.dataReady()) {
      delay(1);
    }
    myICM.getAGMT();
    
    sumX += myICM.gyrX();
    sumY += myICM.gyrY();
    sumZ += myICM.gyrZ();
    
    delay(5);
  }
  
  gyroXoffset = sumX / numReadings;
  gyroYoffset = sumY / numReadings;
  gyroZoffset = sumZ / numReadings;
  
  Serial.print("Gyro offsets: X=");
  Serial.print(gyroXoffset);
  Serial.print(", Y=");
  Serial.print(gyroYoffset);
  Serial.print(", Z=");
  Serial.println(gyroZoffset);
}

void readIMUData() {
  if (myICM.dataReady()) {
    myICM.getAGMT(); // Get accelerometer, gyroscope, magnetometer, temperature
    
    // Calculate time step
    dt = (micros() - timer) / 1000000.0; // Convert to seconds
    timer = micros();
    
    // Read and process sensor data
    readRawIMUData();
    
    // Calculate orientation using complementary filter
    calculateOrientation();
    
    sensorData.imu_valid = true;
  }
}

void readRawIMUData() {
  // Read gyroscope (degrees per second) and apply calibration
  sensorData.gyroX = myICM.gyrX() - gyroXoffset;
  sensorData.gyroY = myICM.gyrY() - gyroYoffset;
  sensorData.gyroZ = myICM.gyrZ() - gyroZoffset;
  
  // Read accelerometer (milli-g, convert to g)
  sensorData.accelX = myICM.accX() / 1000.0;
  sensorData.accelY = myICM.accY() / 1000.0;
  sensorData.accelZ = myICM.accZ() / 1000.0;
  
  // Read magnetometer (micro-Tesla)
  sensorData.magX = myICM.magX();
  sensorData.magY = myICM.magY();
  sensorData.magZ = myICM.magZ();
}

void calculateOrientation() {
  // Calculate angles from accelerometer
  float accelAngleX = atan2(sensorData.accelY, sqrt(sensorData.accelX * sensorData.accelX + sensorData.accelZ * sensorData.accelZ)) * 180.0 / PI;
  float accelAngleY = atan2(-sensorData.accelX, sqrt(sensorData.accelY * sensorData.accelY + sensorData.accelZ * sensorData.accelZ)) * 180.0 / PI;
  
  // Integrate gyroscope data
  gyroAngleX += sensorData.gyroX * dt;
  gyroAngleY += sensorData.gyroY * dt;
  gyroAngleZ += sensorData.gyroZ * dt;
  
  // Complementary filter (98% gyroscope, 2% accelerometer)
  float alpha = 0.98;
  compAngleX = alpha * (compAngleX + sensorData.gyroX * dt) + (1 - alpha) * accelAngleX;
  compAngleY = alpha * (compAngleY + sensorData.gyroY * dt) + (1 - alpha) * accelAngleY;
  
  // Set final angles
  sensorData.roll = compAngleX;   // Roll around X-axis
  sensorData.pitch = compAngleY;  // Pitch around Y-axis
  sensorData.yaw = gyroAngleZ;    // Yaw around Z-axis (from gyro integration)
  
  // Keep yaw in range -180 to +180
  if (sensorData.yaw > 180) sensorData.yaw -= 360;
  if (sensorData.yaw < -180) sensorData.yaw += 360;
  
  // Convert Euler angles to quaternions
  calculateQuaternions();
}

void calculateQuaternions() {
  float rollRad = sensorData.roll * PI / 180.0;
  float pitchRad = sensorData.pitch * PI / 180.0;
  float yawRad = sensorData.yaw * PI / 180.0;
  
  // Convert Euler to quaternion
  float cy = cos(yawRad * 0.5);
  float sy = sin(yawRad * 0.5);
  float cp = cos(pitchRad * 0.5);
  float sp = sin(pitchRad * 0.5);
  float cr = cos(rollRad * 0.5);
  float sr = sin(rollRad * 0.5);
  
  sensorData.q0 = cr * cp * cy + sr * sp * sy;  // w
  sensorData.q1 = sr * cp * cy - cr * sp * sy;  // x
  sensorData.q2 = cr * sp * cy + sr * cp * sy;  // y
  sensorData.q3 = cr * cp * sy - sr * sp * cy;  // z
}

void readEnvironmentalData() {
  sensors_event_t humidity, temp;
  
  // Read AHT21B data
  aht.getEvent(&humidity, &temp);
  sensorData.temperature_aht = temp.temperature;
  sensorData.humidity = humidity.relative_humidity;
  
  // Read BMP280 data
  sensorData.temperature_bmp = bmp.readTemperature();
  sensorData.pressure = bmp.readPressure() / 100.0F; // Convert to hPa
  sensorData.altitude_bmp = bmp.readAltitude(1013.25); // Standard sea level pressure
  
  sensorData.environmental_valid = true;
}

void readGPSData() {
  // Process GPS data
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }
  
  if (gps.location.isUpdated()) {
    sensorData.latitude = gps.location.lat();
    sensorData.longitude = gps.location.lng();
    sensorData.altitude_gps = gps.altitude.meters();
    sensorData.speed = gps.speed.kmph();
    sensorData.satellites = gps.satellites.value();
    sensorData.gps_valid = true;
    sensorData.gps_data_valid = true;
  } else {
    sensorData.gps_valid = false;
    if (gps.satellites.isValid()) {
      sensorData.satellites = gps.satellites.value();
    }
  }
}

void outputUnifiedData() {
  // Output in structured format for easy parsing
  Serial.println("=== UNIFIED SENSOR DATA ===");
  Serial.print("Timestamp: "); Serial.println(sensorData.timestamp);
  
  // IMU Data
  if (sensorData.imu_valid) {
    Serial.println("--- IMU DATA ---");
    Serial.print("Orientation (Roll,Pitch,Yaw): ");
    Serial.print(sensorData.roll, 2); Serial.print(",");
    Serial.print(sensorData.pitch, 2); Serial.print(",");
    Serial.println(sensorData.yaw, 2);
    
    Serial.print("Quaternion (w,x,y,z): ");
    Serial.print(sensorData.q0, 4); Serial.print(",");
    Serial.print(sensorData.q1, 4); Serial.print(",");
    Serial.print(sensorData.q2, 4); Serial.print(",");
    Serial.println(sensorData.q3, 4);
    
    Serial.print("Accel (g): ");
    Serial.print(sensorData.accelX, 3); Serial.print(",");
    Serial.print(sensorData.accelY, 3); Serial.print(",");
    Serial.println(sensorData.accelZ, 3);
    
    Serial.print("Gyro (deg/s): ");
    Serial.print(sensorData.gyroX, 2); Serial.print(",");
    Serial.print(sensorData.gyroY, 2); Serial.print(",");
    Serial.println(sensorData.gyroZ, 2);
    
    Serial.print("Mag (uT): ");
    Serial.print(sensorData.magX, 2); Serial.print(",");
    Serial.print(sensorData.magY, 2); Serial.print(",");
    Serial.println(sensorData.magZ, 2);
  }
  
  // Environmental Data
  if (sensorData.environmental_valid) {
    Serial.println("--- ENVIRONMENTAL DATA ---");
    Serial.print("Temp AHT21B: "); Serial.print(sensorData.temperature_aht, 2); Serial.println(" °C");
    Serial.print("Humidity: "); Serial.print(sensorData.humidity, 2); Serial.println(" %");
    Serial.print("Temp BMP280: "); Serial.print(sensorData.temperature_bmp, 2); Serial.println(" °C");
    Serial.print("Pressure: "); Serial.print(sensorData.pressure, 2); Serial.println(" hPa");
    Serial.print("Altitude BMP: "); Serial.print(sensorData.altitude_bmp, 2); Serial.println(" m");
  }
  
  // GPS Data
  Serial.println("--- GPS DATA ---");
  if (sensorData.gps_valid) {
    Serial.print("Position: ");
    Serial.print(sensorData.latitude, 6); Serial.print(",");
    Serial.println(sensorData.longitude, 6);
    Serial.print("Altitude GPS: "); Serial.print(sensorData.altitude_gps, 2); Serial.println(" m");
    Serial.print("Speed: "); Serial.print(sensorData.speed, 2); Serial.println(" km/h");
  } else {
    Serial.println("GPS: No fix");
  }
  Serial.print("Satellites: "); Serial.println(sensorData.satellites);
  
  Serial.println("========================");
  Serial.println();
}

// Function to get sensor data struct (for external communication)
SensorData getSensorData() {
  return sensorData;
}

// Function to output data in CSV format for logging
void outputCSVData() {
  Serial.print(sensorData.timestamp); Serial.print(",");
  Serial.print(sensorData.roll, 2); Serial.print(",");
  Serial.print(sensorData.pitch, 2); Serial.print(",");
  Serial.print(sensorData.yaw, 2); Serial.print(",");
  Serial.print(sensorData.q0, 4); Serial.print(",");
  Serial.print(sensorData.q1, 4); Serial.print(",");
  Serial.print(sensorData.q2, 4); Serial.print(",");
  Serial.print(sensorData.q3, 4); Serial.print(",");
  Serial.print(sensorData.accelX, 3); Serial.print(",");
  Serial.print(sensorData.accelY, 3); Serial.print(",");
  Serial.print(sensorData.accelZ, 3); Serial.print(",");
  Serial.print(sensorData.gyroX, 2); Serial.print(",");
  Serial.print(sensorData.gyroY, 2); Serial.print(",");
  Serial.print(sensorData.gyroZ, 2); Serial.print(",");
  Serial.print(sensorData.magX, 2); Serial.print(",");
  Serial.print(sensorData.magY, 2); Serial.print(",");
  Serial.print(sensorData.magZ, 2); Serial.print(",");
  Serial.print(sensorData.temperature_aht, 2); Serial.print(",");
  Serial.print(sensorData.humidity, 2); Serial.print(",");
  Serial.print(sensorData.temperature_bmp, 2); Serial.print(",");
  Serial.print(sensorData.pressure, 2); Serial.print(",");
  Serial.print(sensorData.altitude_bmp, 2); Serial.print(",");
  Serial.print(sensorData.latitude, 6); Serial.print(",");
  Serial.print(sensorData.longitude, 6); Serial.print(",");
  Serial.print(sensorData.altitude_gps, 2); Serial.print(",");
  Serial.print(sensorData.speed, 2); Serial.print(",");
  Serial.print(sensorData.satellites); Serial.print(",");
  Serial.print(sensorData.gps_valid); Serial.print(",");
  Serial.print(sensorData.imu_valid); Serial.print(",");
  Serial.println(sensorData.environmental_valid);
}