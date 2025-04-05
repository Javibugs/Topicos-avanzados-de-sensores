#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
Adafruit_MPU6050 mpu;

const float MASA = 65;  // Masa en gramos (la fuerza se calcula en mN)
float aceleracionX, aceleracionY, aceleracion, fuerza;
float fuerzaMax = 0;  // Fuerza máxima registrada (mN)
int c = 0;

unsigned long previousMillis = 0;
const unsigned long INTERVALO_MUESTREO = 10;  // Intervalo de 10 ms
const int TOTAL_MUESTRAS = 500;               // 500 muestras en 5 segundos

void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32_BT_Classic");
  Serial.println("El dispositivo Bluetooth está listo para emparejarse.");

  if (!mpu.begin()) {
    Serial.println("Error: no se encontró el MPU6050");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.println("MPU6050 listo!");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= INTERVALO_MUESTREO) {
    previousMillis = currentMillis;
    
    // Lectura de aceleraciones
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    aceleracionX = a.acceleration.x;
    aceleracionY = a.acceleration.y;

    // Cálculo de la magnitud de la aceleración (sin usar sqrt para eficiencia)
    aceleracion = sqrt(sq(aceleracionX) + sq(aceleracionY));
    fuerza = MASA * aceleracion;

    // Actualización del máximo
    if (fuerza > fuerzaMax) {
      fuerzaMax = fuerza;
    }

    c++;

    // Cuando se completan 500 muestras (5 segundos), se transmite el resultado
    if (c >= TOTAL_MUESTRAS) {
      SerialBT.printf("Fuerza Maxima: %.2f mN\n", fuerzaMax);

      // Reiniciar variables para la próxima medición
      c = 0;
      fuerzaMax = 0;
    }
  }
}
