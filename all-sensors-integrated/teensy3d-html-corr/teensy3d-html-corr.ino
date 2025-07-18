/*
 * ICM-20948 9DoF IMU 3D Orientation for Teensy 4.1
 * Simple and reliable code that works with SparkFun ICM-20948 library
 * Outputs: Roll, Pitch, Yaw angles via Serial
 */

#include <Wire.h>
#include <ICM_20948.h>

// Create ICM-20948 object
ICM_20948_I2C myICM;

// Variables for orientation calculation
float roll = 0, pitch = 0, yaw = 0;
float gyroX, gyroY, gyroZ;
float accelX, accelY, accelZ;

// Complementary filter variables
float compAngleX = 0, compAngleY = 0;
float gyroAngleX = 0, gyroAngleY = 0, gyroAngleZ = 0;

// Timing
unsigned long timer;
float dt;

// Calibration offsets
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C
  
  Serial.println("ICM-20948 3D Orientation Demo");
  Serial.println("Initializing sensor...");
  
  // Initialize the ICM-20948
  bool initialized = false;
  while (!initialized) {
    myICM.begin(Wire, 1);
    
    Serial.print("Initialization of the sensor returned: ");
    Serial.println(myICM.statusString());
    
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
    Serial.print("startupMagnetometer returned: ");
    Serial.println(myICM.statusString());
  }
  
  // Calibrate gyroscope
  Serial.println("Calibrating gyroscope... Keep sensor still for 3 seconds!");
  calibrateGyro();
  Serial.println("Calibration complete! Starting orientation tracking...");
  
  timer = micros();
}

void loop() {
  if (myICM.dataReady()) {
    myICM.getAGMT(); // Get accelerometer, gyroscope, magnetometer, temperature
    
    // Calculate time step
    dt = (micros() - timer) / 1000000.0; // Convert to seconds
    timer = micros();
    
    // Read and process sensor data
    readSensorData();
    
    // Calculate orientation using complementary filter
    calculateOrientation();
    
    // Output data
    outputData();
    
    delay(10); // Small delay for stability
  }
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

void readSensorData() {
  // Read gyroscope (degrees per second) and apply calibration
  gyroX = myICM.gyrX() - gyroXoffset;
  gyroY = myICM.gyrY() - gyroYoffset;
  gyroZ = myICM.gyrZ() - gyroZoffset;
  
  // Read accelerometer (milli-g, convert to g)
  accelX = myICM.accX() / 1000.0;
  accelY = myICM.accY() / 1000.0;
  accelZ = myICM.accZ() / 1000.0;
}

void calculateOrientation() {
  // Calculate angles from accelerometer
  float accelAngleX = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  float accelAngleY = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
  
  // Integrate gyroscope data
  gyroAngleX += gyroX * dt;
  gyroAngleY += gyroY * dt;
  gyroAngleZ += gyroZ * dt;
  
  // Complementary filter (98% gyroscope, 2% accelerometer)
  float alpha = 0.98;
  compAngleX = alpha * (compAngleX + gyroX * dt) + (1 - alpha) * accelAngleX;
  compAngleY = alpha * (compAngleY + gyroY * dt) + (1 - alpha) * accelAngleY;
  
  // Set final angles
  roll = compAngleX;   // Roll around X-axis
  pitch = compAngleY;  // Pitch around Y-axis
  yaw = gyroAngleZ;    // Yaw around Z-axis (from gyro integration)
  
  // Keep yaw in range -180 to +180
  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;
}

void outputData() {
  // Output format: ORIENTATION,roll,pitch,yaw,q0,q1,q2,q3
  // Convert Euler angles to quaternions for 3D visualization
  float rollRad = roll * PI / 180.0;
  float pitchRad = pitch * PI / 180.0;
  float yawRad = yaw * PI / 180.0;
  
  // Convert Euler to quaternion
  float cy = cos(yawRad * 0.5);
  float sy = sin(yawRad * 0.5);
  float cp = cos(pitchRad * 0.5);
  float sp = sin(pitchRad * 0.5);
  float cr = cos(rollRad * 0.5);
  float sr = sin(rollRad * 0.5);
  
  float q0 = cr * cp * cy + sr * sp * sy;  // w
  float q1 = sr * cp * cy - cr * sp * sy;  // x
  float q2 = cr * sp * cy + sr * cp * sy;  // y
  float q3 = cr * cp * sy - sr * sp * cy;  // z
  
  // Output data
  Serial.print("ORIENTATION,");
  Serial.print(roll, 2);
  Serial.print(",");
  Serial.print(pitch, 2);
  Serial.print(",");
  Serial.print(yaw, 2);
  Serial.print(",");
  Serial.print(q0, 4);
  Serial.print(",");
  Serial.print(q1, 4);
  Serial.print(",");
  Serial.print(q2, 4);
  Serial.print(",");
  Serial.println(q3, 4);
  
  // Optional: Print raw sensor data for debugging
  /*
  Serial.print("Raw - Accel: ");
  Serial.print(accelX, 3); Serial.print(",");
  Serial.print(accelY, 3); Serial.print(",");
  Serial.print(accelZ, 3);
  Serial.print(" | Gyro: ");
  Serial.print(gyroX, 2); Serial.print(",");
  Serial.print(gyroY, 2); Serial.print(",");
  Serial.println(gyroZ, 2);
  */
}