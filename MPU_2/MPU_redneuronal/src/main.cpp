#include <Arduino.h>
#include <wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

int pitch=0, roll=0;
unsigned long past_time=0;

//Redes neuronales
const int node = 2, input = 2, outputs = 2;
float P[input][1];
float W1[node][input]={{1.77857983967752,2.35169321591261},{1.37062709976213,0.502695879457117}};    //|2  3|
                                                        //|1  2|
float b1[node][1]={{-0.00630937725497233},{-0.265130504804829}};//|2|
                            //|1|
float a1[node][1];
float W2[outputs][node]={{1.22957802808026,-2.02696434785660},{-0.262541319542139,2.25493190021567}};    //|2  3|
                                                        //|1  2|                  
float a2[outputs][1];
float b2[node][1]={{-0.361695922227333},{0.382649549836938}};//|2|
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
  while (!Serial) delay(10); // Wait for serial monitor

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G); // ±2G range
  Serial.println("MPU6050 Ready!");
}

void loop() {
                          // primera capa de entrada
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); // Get all sensor events
  // Calculate pitch and roll (in degrees) from accelerometer
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
  float roll = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
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
    Serial.println("Adelante");
  }
  else if (a2[0][0]<=0 && a2[1][0]>0){
    Serial.println("Derecha");
  }
    else if (a2[0][0]>0 && a2[1][0]<=0){
    Serial.println("Izquierda");
  }
    else {
    Serial.println("Atras");
  }
delay(50);
}
