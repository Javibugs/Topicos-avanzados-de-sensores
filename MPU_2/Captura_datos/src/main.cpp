#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10); // Wait for serial monitor

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // ±2G range
  Serial.println("MPU6050 Ready!");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); // Get all sensor events

  // Calculate pitch and roll (in degrees) from accelerometer
  //float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
  //float roll = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;

  // Output angles
  // Output in requested format
  Serial.print(a.acceleration.x, 2); // x in m/s²
  Serial.print(",");
  Serial.println(a.acceleration.y, 2); // y in m/s²

  // Output angles
 // Serial.print(pitch, 1);
  //Serial.print(",");
  //Serial.println(roll, 1);
  delay(10); // Adjust as needed
}