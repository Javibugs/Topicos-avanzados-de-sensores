#include <Arduino.h>
#include <wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
Adafruit_MPU6050 mpu;

int pitch=0, roll=0;
unsigned long past_time=0;
const float MASA = 66;  // Masa en gramos (la fuerza se calcula en mN)
float aceleracionX, aceleracionY, aceleracion, fuerza;

//Redes neuronales
const int node = 2, input = 2, outputs = 2;
float P[input][1];
float W1[node][input]={{-6.67705142789080,-2.77523619769209},{6.73939808943474,-1.74001477136550}};    //|2  3|
                                                        //|1  2|
float b1[node][1]={{0.727426041308733},{-0.386598988630392}};//|2|
                            //|1|
float a1[node][1];
float W2[outputs][node]={{4.01417681680687,0.152321527163279},{1.57161378023102,3.69809485324486}};    //|2  3|
                                                        //|1  2|                  
float a2[outputs][1];
float b2[node][1]={{-2.33269282323668},{-0.0469993323049857}};//|2|
                            //|1|
float aux = 0;                                  
int maxvalue = 1023, minvalue = 0;

float tansig(float n){
  float a = exp(n);
  float b = exp(-n);
  return (a-b)/(a+b);
}

float dataNormalized(int inputData, int minData, int maxData)
{
  float valueNorm;
  valueNorm = 2.0*(inputData-minData)/(maxData-minData)-1.0;
  return valueNorm;
}

void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32_BT_Classic");
  while (!Serial) delay(10); // Wait for serial monitor

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G); // ±2G range
}

void loop() {
                          // primera capa de entrada
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); // Get all sensor events
  // Calculate pitch and roll (in degrees) from accelerometer
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
  float roll = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  aceleracionX = a.acceleration.x;
  aceleracionY = a.acceleration.y;
  pitch=map(pitch,-180,180,0,1023);
  roll=map(roll,-180,180,0,1023);
  P[0][0]=dataNormalized((float) pitch,minvalue,maxvalue);
  P[1][0]=dataNormalized((float) roll,minvalue,maxvalue);

  //Segunda capa
  for(int i=0; i<node; i++){
    aux = 0;
    for(int j=0; j<input; j++){
      aux=aux+W1[i][j]*P[j][0];
    }
    a1[i][0]=tansig(aux+b1[i][0]);
  }

  //tercera capa
  for(int i = 0 ; i<outputs; i++){
    aux= 0.0;
    for(int j = 0 ; j<node; j++){
      aux = aux+W2[i][j]*a1[j][0];
    }
    a2[i][0]=tansig(aux+b2[i][0]);
  } 

  if (a2[0][0]<=0 && a2[1][0]<=0){
    SerialBT.println("inclinación hacia Atras");
  }
  else if (a2[0][0]<=0 && a2[1][0]>0){
    SerialBT.println("inclinación hacia la Derecha");
  }
    else if (a2[0][0]>0 && a2[1][0]<=0){
    SerialBT.println("inclinación hacia la Izquierda");
  }
    else {
    SerialBT.println("inclinación hacia Adelante");
  }
  // Cálculo de la magnitud de la aceleración (sin usar sqrt para eficiencia)
  aceleracion = sqrt(sq(aceleracionX) + sq(aceleracionY));
  fuerza = MASA * aceleracion;
  SerialBT.printf("Fuerza: %.2f mN\n\n", fuerza);

delay(200);
}
