#include <Arduino.h>              
#include <InterCom.h>            
#include <BluetoothSerial.h>      
#include <DHT.h>                  

#define DHTTYPE DHT22             // Definimos el tipo de sensor de humedad

// Timer y bandera para indicar que se ha cumplido el intervalo
hw_timer_t *timer = NULL;
volatile bool has_expired = false;

// Pines asignados a sensores y variables globales
const int DHTPin = 15, TMP36 = 4, LM35 = 2;
float h = 0, t = 0, tempLM35 = 0, tempTMP36 = 0, tc = 0, Ts = 10;

DHT dht(DHTPin, DHTTYPE);        // Inicialización del sensor DHT
SimpleCommand cmd;               // Comando de texto para cambiar parámetros por Bluetooth
BluetoothSerial bt_port;         // Canal de comunicación Bluetooth

// Función que convierte la lectura analógica en temperatura para TMP36 y LM35
float sens_temp(int sensor) {
  int lectura = analogRead(sensor);
  float voltaje = (lectura * 3.3) / 4095.0;  // Conversión ADC a voltaje (12 bits, 3.3V ref)
  float temperatura = 0;

  // Conversión específica según el sensor
  if (sensor == TMP36) {
    temperatura = (voltaje - 0.5) * 100.0;  // TMP36: 0.5V offset
  } else if (sensor == LM35) {
    temperatura = voltaje * 100.0;         // LM35: 10 mV por °C
  }
  return temperatura;
}

// Recolecta y calcula los datos de todos los sensores
void datos(){
  h = dht.readHumidity();                  // Humedad relativa
  t = dht.readTemperature();               // Temperatura ambiente (DHT)
  tempLM35 = sens_temp(LM35);              // Temperatura LM35
  tempTMP36 = sens_temp(TMP36);            // Temperatura TMP36
  tc = (tempLM35 + tempTMP36 + t) / 3;     // Promedio simple como referencia interna
}

// Función de interrupción por temporizador
void IRAM_ATTR timerInterrupt() {
  has_expired = true;
}

// Conversión de segundos a microsegundos para el temporizador
int muestreo(int segundos) {
  return segundos * 1000000;
}

void setup() {
  Serial.begin(9600);                      // Comunicación serial por depuración
  analogReadResolution(12);               // Resolución de 12 bits para lecturas analógicas

  pinMode(TMP36, INPUT);
  pinMode(LM35, INPUT);
  dht.begin();                            // Inicializa sensor DHT22

  // Configuración del temporizador de hardware (80 MHz base)
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &timerInterrupt, true);
  timerAlarmWrite(timer, muestreo(Ts), true);
  timerAlarmEnable(timer);

  // Inicialización Bluetooth
  bt_port.begin("Caja");

  // Inicialización del sistema de comandos
  cmd.begin(&bt_port);
  cmd.enable_echo(true);
  cmd.addCommand("Ts", &Ts);              // Permite cambiar el intervalo de muestreo desde el celular
}

void loop() {
  cmd.listen();                           // Revisa si hay comandos Bluetooth
  timerAlarmWrite(timer, muestreo(Ts), true);  // Refresca el valor de muestreo por si cambió

  // Si se cumplió el tiempo, se toman y envían los datos
  if (has_expired) {
    datos();                              // Lectura de sensores
    bt_port.print(h);                     // Envío de humedad
    bt_port.print(",");                   // Separador CSV
    bt_port.println(tc);                  // Envío de temperatura combinada
    has_expired = false;                  // Reinicia la bandera
  }
}
