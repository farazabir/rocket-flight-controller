#include <Wire.h>
#include <MPU9250_asukiaaa.h>

#define MPU_SDA 21
#define MPU_SCL 22

MPU9250_asukiaaa mpu;

float pitch = 0, roll = 0;
float gyroOffsetX = 0, gyroOffsetY = 0;
float pitchZero = 0, rollZero = 0;

const float alpha = 0.98;  // Complementary filter constant
unsigned long lastTime = 0;

// Check if sensor is responding
bool checkSensorConnection() {
  Wire.beginTransmission(0x68); // MPU9250 default address
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("MPU9250 found at address 0x68");
    return true;
  } else {
    Serial.print("MPU9250 not found at 0x68, error: ");
    Serial.println(error);
    
    // Try alternative address
    Wire.beginTransmission(0x69);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.println("MPU9250 found at address 0x69");
      return true;
    } else {
      Serial.print("MPU9250 not found at 0x69, error: ");
      Serial.println(error);
      return false;
    }
  }
}

// Gyro calibration with error checking
void calibrateGyro(int samples = 500) {
  float sumX = 0, sumY = 0;
  int validSamples = 0;

  Serial.println("Calibrating gyro...");
  Serial.println("Keep sensor still during calibration...");
  
  for (int i = 0; i < samples; i++) {
    if (mpu.gyroUpdate() == 0) {  // Check for successful update
      sumX += mpu.gyroX();
      sumY += mpu.gyroY();
      validSamples++;
    } else {
      Serial.print("Gyro read failed at sample ");
      Serial.println(i);
    }
    delay(5);
  }

  if (validSamples > 0) {
    gyroOffsetX = sumX / validSamples;
    gyroOffsetY = sumY / validSamples;

    Serial.print("Gyro Offset X: "); Serial.println(gyroOffsetX, 4);
    Serial.print("Gyro Offset Y: "); Serial.println(gyroOffsetY, 4);
    Serial.print("Valid samples: "); Serial.print(validSamples);
    Serial.print("/"); Serial.println(samples);
  } else {
    Serial.println("Gyro calibration failed - no valid readings");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  
  Serial.println("Starting MPU9250 initialization (Pitch/Roll only)...");
  
  // Initialize I2C with custom pins
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(100000); // Start with slower clock speed
  
  delay(100);
  
  // Check sensor connection first
  if (!checkSensorConnection()) {
    Serial.println("ERROR: MPU9250 not found! Check wiring:");
    Serial.println("- VCC to 3.3V (NOT 5V)");
    Serial.println("- GND to GND");
    Serial.println("- SDA to pin 21");
    Serial.println("- SCL to pin 22");
    Serial.println("- Pull-up resistors (4.7kΩ) on SDA and SCL lines");
    while(1) {
      delay(1000);
    }
  }

  mpu.setWire(&Wire);

  // Initialize only accelerometer and gyroscope
  Serial.println("Initializing accelerometer...");
  mpu.beginAccel();
  delay(100);
  
  Serial.println("Initializing gyroscope...");
  mpu.beginGyro();
  delay(100);

  // Increase I2C speed after initialization
  Wire.setClock(400000);
  
  delay(2000); // Stabilize sensor

  // Test initial sensor readings
  Serial.println("Testing sensor readings...");
  int attempts = 0;
  bool sensorsOK = false;
  
  while (attempts < 10 && !sensorsOK) {
    bool accelOK = (mpu.accelUpdate() == 0);
    bool gyroOK = (mpu.gyroUpdate() == 0);
    
    if (accelOK && gyroOK) {
      sensorsOK = true;
      Serial.println("Accelerometer and gyroscope responding correctly");
    } else {
      Serial.print("Sensor test failed - Attempt ");
      Serial.print(attempts + 1);
      Serial.print("/10 - Accel: ");
      Serial.print(accelOK ? "OK" : "FAIL");
      Serial.print(", Gyro: ");
      Serial.println(gyroOK ? "OK" : "FAIL");
      
      attempts++;
      delay(500);
    }
  }
  
  if (!sensorsOK) {
    Serial.println("ERROR: Sensors not responding after multiple attempts");
    while(1) {
      delay(1000);
    }
  }

  // Calibrate gyroscope
  calibrateGyro();

  // Get initial orientation from accelerometer
  if (mpu.accelUpdate() == 0) {
    float ax = mpu.accelX();
    float ay = mpu.accelY();
    float az = mpu.accelZ();

    pitch = atan2(ay, az) * 180.0 / PI;
    roll = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    
    Serial.println("Initial orientation calculated from accelerometer");
  }

  // Set zero reference points
  pitchZero = pitch;
  rollZero = roll;

  lastTime = millis();

  Serial.println("Setup complete. Pitch/Roll orientation zeroed.");
  Serial.println("Format: Pitch, Roll (degrees relative to startup position)");
  Serial.println("Tip the sensor to see values change!");
}

void loop() {
  bool accelOK = (mpu.accelUpdate() == 0);
  bool gyroOK = (mpu.gyroUpdate() == 0);
  
  if (!accelOK || !gyroOK) {
    Serial.println("Sensor update failed - skipping loop");
    delay(100);
    return;
  }

  // Read sensor data
  float ax = mpu.accelX();
  float ay = mpu.accelY();
  float az = mpu.accelZ();
  float gx = mpu.gyroX() - gyroOffsetX;
  float gy = mpu.gyroY() - gyroOffsetY;

  // Calculate pitch and roll from accelerometer
  float pitchAcc = atan2(ay, az) * 180.0 / PI;
  float rollAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  // Calculate time delta for gyro integration
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // Integrate gyroscope data
  pitch += gx * dt;
  roll += gy * dt;

  // Apply complementary filter (combines gyro and accelerometer)
  pitch = alpha * pitch + (1 - alpha) * pitchAcc;
  roll = alpha * roll + (1 - alpha) * rollAcc;

  // Calculate relative angles from startup position
  float pitchRel = pitch - pitchZero;
  float rollRel = roll - rollZero;

  // Display results
  Serial.print("Pitch: "); 
  Serial.print(pitchRel, 2); 
  Serial.print("°, Roll: "); 
  Serial.print(rollRel, 2); 
  Serial.println("°");

  delay(50); // 20Hz update rate
}
