  #include <QMC5883LCompass.h>
  #include <TinyGPSPlus.h>
  #include <WMM_Tinier.h>
  #include <ESP32Servo.h>
  #include "esp_sleep.h"
  #include <math.h>

  #define GPSRX        16
  #define GPSTX        17 
  #define captura      34

  // Motor azimuth
  #define STBY_PIN     15
  #define AIN1_PIN     4
  #define AIN2_PIN     19

  // Encoder
  #define C1_PIN       33
  #define C2_PIN       32   // no usar 19, ya en AIN2

  // Sleep timing
  #define uS_TO_S_FACTOR       1000000ULL
  #define TIEMPO_ENTRE_MUESTREOS 2  // segundos

  // PWM motor
  #define PWM_MOTOR_PIN    2       // Pin para el motor
  #define PWM_MOTOR_CH     0       // Canal LEDC 0
  #define PWM_FREQ_MOTOR   5000    // Frecuencia para el motor
  #define PWM_RESOLUTION   8       // Resolución en bits (0-255)

  //PWM Servo
  #define SERVO_PIN        23        // Pin del servo
  #define SERVO_CH         1         // Canal LEDC 1
  #define SERVO_FREQ       50        // Frecuencia típica del servo
  #define SERVO_RESOLUTION 16        // Alta resolución para control preciso

  // Sensores y GPS
  QMC5883LCompass compass;
  TinyGPSPlus     gps;
  WMM_Tinier      declinacionWMM;
  HardwareSerial  GPS(1);
  // Servo de elevación
  Servo servoelev;        // Crear objeto Servo

  // Encoder variables
  volatile int  lastEncoded     = 0;
  volatile long encoderValue    = 0;
  const float   pulsesPerRev    = 28.0 * 408.0;
  const float   degreesPerPulse = 360.0 / pulsesPerRev;

  // Estado global
  float currentAngle = 0.0;  // azimuth real actual

  // Variables sensado
  float lat=22.75778, lon=-102.59806, declin=0;
  uint8_t year = 0, month = 0, day = 0, UTC = -6;
  int hour = 0, minute = 0, second = 0;
  float azimutBruj=0, elevSol=0, azimutSol=0;

  // —————— INTERRUPT ENCODER ——————
  void IRAM_ATTR encoder() {
    int MSB = digitalRead(C1_PIN);
    int LSB = digitalRead(C2_PIN);
    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded << 2) | encoded;
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue--;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue++;
    lastEncoded = encoded;
  }

  float calcJulianDate(int year, int month, int day, int hour, int minute, int second) {
    if (month <= 2) {
      year -= 1;
      month += 12;
    }
    int A = year / 100;
    int B = 2 - A + A / 4;
    float JD = int(365.25 * (year + 4716)) + int(30.6001 * (month + 1)) +
              day + B - 1524.5 + (hour + minute / 60.0 + second / 3600.0) / 24.0;
    return JD;
  }
  float calcSolarDeclination(float d) {
    return 23.45 * sin(radians(360.0 / 365.0 * (284 + d)));  // en grados
  }
  float calcHourAngle(float timeUTC, float lon) {
    float solarTime = timeUTC * 15.0 + lon;
    return solarTime - 180.0;  // hora angular en grados
  }
  void calcSolarPosition(float lat, float lon, int year, int month, int day, int hour, int minute, int second, float &elev, float &azimutSol) {
    float JD = calcJulianDate(year, month, day, hour, minute, second);
    float d = JD - 2451545.0;
    float decl = calcSolarDeclination(fmod(d, 365.25));
    float HRA = calcHourAngle(hour + minute / 60.0 + second / 3600.0, lon);

    float latRad = radians(lat);
    float declRad = radians(decl);
    float HRARad = radians(HRA);

    elev = degrees(asin(sin(latRad) * sin(declRad) + cos(latRad) * cos(declRad) * cos(HRARad)));

    float sinAz = -sin(HRARad) * cos(declRad);
    float cosAz = (sin(declRad) - sin(latRad) * sin(radians(elev))) / (cos(latRad) * cos(radians(elev)));
    azimutSol = degrees(atan2(sinAz, cosAz));
    if (azimutSol < 0) azimutSol += 360.0;
  }

  float calcularAzimut(float dec = 0.0) {
    compass.read();
    int a = compass.getAzimuth();
    if (a < 0) a += 360;
    a += dec;
    if (a >= 360) a -= 360;
    return a;
  }

  void procesarGPS() {
    unsigned long start = millis();
    while ((millis() - start) < 1500) {
      while (GPS.available() > 0) {
        gps.encode(GPS.read());
      }
    }

    if (gps.location.isUpdated() && digitalRead(captura)==LOW){
      lat = gps.location.lat();
      lon = gps.location.lng();
    }
    
    else if (digitalRead(captura)==HIGH){
        if (Serial.available()){
        String line = Serial.readStringUntil('\n');
        int comma = line.indexOf(',');
        if(comma > 0){
          lat = line.substring(0, comma).toFloat();
          lon = line.substring(comma+1).toFloat();
          Serial.printf("Modo MANUAL: lat=%.6f, lon=%.6f\n", lat, lon);
        }
      }
    }

      year = gps.date.year();
      month = gps.date.month();
      day = gps.date.day();

      hour = gps.time.hour();
      minute = gps.time.minute();
      second = gps.time.second();

      declin = declinacionWMM.magneticDeclination(lat, lon, year-2000, month, day);
  }

  void setupPWM_Motor() {
    ledcSetup(PWM_MOTOR_CH, PWM_FREQ_MOTOR, PWM_RESOLUTION);
    ledcAttachPin(PWM_MOTOR_PIN, PWM_MOTOR_CH);
    ledcWrite(PWM_MOTOR_CH, 0);  // Apagar al inicio
  }

  void setupPWM_Servo() {
    ledcSetup(SERVO_CH, SERVO_FREQ, SERVO_RESOLUTION);
    ledcAttachPin(SERVO_PIN, SERVO_CH);
  }

  // Función auxiliar: convierte microsegundos a valor de duty cycle
  uint32_t usToDuty(uint16_t us) {
    // Para 50 Hz y 16 bits => periodo = 20,000 us, resolución = 65536 niveles
    return map(us, 500, 2400, 1638, 7864);  // 500us → duty bajo, 2400us → duty alto
  }

  void moverServo(int angulo) {
    // Limita ángulo de 0 a 180
    angulo = constrain(angulo, 0, 180);
    // Mapear ángulo a pulso entre 500us y 2400us
    uint16_t pulso = map(angulo, 0, 180, 500, 2400);
    ledcWrite(SERVO_CH, usToDuty(pulso));
  }

  void moveToAngle(float targetAngle) {
    float deltaAngle = targetAngle - currentAngle;
    long deltaPulses = lround(deltaAngle / degreesPerPulse);

    if (deltaPulses == 0) {
      Serial.println("Ya en posición.");
      return;
    }
    setupPWM_Motor();
    digitalWrite(STBY_PIN, HIGH);

    bool forward = (deltaPulses > 0);
    digitalWrite(AIN1_PIN, forward ? HIGH : LOW);
    digitalWrite(AIN2_PIN, forward ? LOW : HIGH);

    long startCount = encoderValue;
    long goalCount = startCount + deltaPulses;

    const uint8_t speed = 120; // duty 0–255
    ledcWrite(PWM_MOTOR_CH, speed);

    if (forward) {
      while (encoderValue < goalCount) yield();
    } else {
      while (encoderValue > goalCount) yield();
    }
    digitalWrite(STBY_PIN, LOW);
    ledcWrite(PWM_MOTOR_CH, 0);
    currentAngle = targetAngle;
  }

  void setup() {
    pinMode(PWM_MOTOR_PIN, OUTPUT);
    pinMode(SERVO_PIN, OUTPUT);
    pinMode(STBY_PIN, OUTPUT);
    pinMode(AIN1_PIN, OUTPUT);
    pinMode(AIN2_PIN, OUTPUT);
    pinMode(C1_PIN, INPUT_PULLUP);
    pinMode(C2_PIN, INPUT_PULLUP);
    pinMode(captura, INPUT_PULLUP);
    digitalWrite(PWM_MOTOR_PIN,LOW);

    attachInterrupt(digitalPinToInterrupt(C1_PIN), encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(C2_PIN), encoder, CHANGE);

    Serial.begin(9600);
    GPS.begin(9600, SERIAL_8N1, GPSRX, GPSTX);
    compass.init();
    compass.setCalibrationOffsets(-87.00, -810.00, -616.00);
    compass.setCalibrationScales(0.79, 0.94, 1.50);
    declinacionWMM.begin();

    delay(100);
    noInterrupts();
      encoderValue = 0;
    interrupts();
    currentAngle = 0.0;

      // Alinea al norte inicial
    procesarGPS();
    float north = calcularAzimut();
    moveToAngle(north);
    currentAngle = north;
    Serial.printf("Norte inicial: %.2f°\n", north);
    currentAngle = 0.0;
    esp_sleep_enable_timer_wakeup(TIEMPO_ENTRE_MUESTREOS * uS_TO_S_FACTOR);
  }

  void loop() {
    unsigned long tInicio = millis();  // Marca de inicio del ciclo
    // ---- PROCESAMIENTO ----
    procesarGPS();
    azimutBruj=calcularAzimut(declin);
    calcSolarPosition(lat, lon, year, month, day, hour, minute, second, elevSol, azimutSol);

    setupPWM_Servo();    
    moverServo(180-lat+10);   // Mueve a 180 grados
    delay(500);
    //moveToAngle(-180);
    moveToAngle(currentAngle+(azimutBruj-azimutSol));
    delay(500);
    Serial.printf("Altura solar: %.2f°, Azimut solar: %.2f°, brujula: %.2f°, angulo: %.2f°\n\n", elevSol, azimutSol, azimutBruj, -currentAngle);
    

    // ---- CONTROL DE TIEMPO Y SUEÑO ----
    unsigned long tProceso = millis() - tInicio;
    long tRestante = TIEMPO_ENTRE_MUESTREOS * 1000 - tProceso;
    if (tRestante > 0) {
      // Asegúrate de que toda la transmisión haya terminado antes de dormir
      Serial.flush();
      esp_sleep_enable_timer_wakeup(tRestante * 1000ULL);
      esp_light_sleep_start();
    }
    
  }