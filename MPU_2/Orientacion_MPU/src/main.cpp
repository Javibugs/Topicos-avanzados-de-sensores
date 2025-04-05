#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32_BT_Classic");

  if (!mpu.begin()) {
    Serial.println("Error: no se encontró el MPU6050");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.println("MPU6050 listo!");
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
    float roll = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;

    if (abs(pitch)<30 && roll<-30){
      SerialBT.println("Adelante");
    }
    else if(abs(pitch)<30 && roll>30){
      SerialBT.println("Atras");
    }
    else if(pitch<-30 && abs(roll)<30 || pitch<-70 && abs(roll)>50){
      SerialBT.println("Izquierda");
    }
    else{
      SerialBT.println("Derecha");
    }
}
