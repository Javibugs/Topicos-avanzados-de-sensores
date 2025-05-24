// MQ2 conectado al pin A0
const int mq2Pin = A0;

// Variables para la lectura
int rawValue = 0;
float voltage = 0.0;
float ppm_estimate = 0.0;

// Variables para conversión simulada a ppm (NO es precisa)
float Ro = 10.0; // Valor de resistencia base del sensor calibrado en aire limpio (kΩ)

void setup() {
  Serial.begin(9600);
  delay(2000); // Esperar a que el sensor se caliente un poco
  Serial.println("Iniciando sensor MQ2...");
}

void loop() {
  rawValue = analogRead(mq2Pin);              // Leer valor analógico
  voltage = (rawValue * 5.0) / 1023.0;         // Convertir a voltaje (si Vref = 5V)

  // Convertir a resistencia del sensor (Rs)
  float Rs = ((5.0 - voltage) / voltage) * 10.0; // 10kΩ es la resistencia de carga RL
  float ratio = Rs / Ro;                         // Rs/Ro

  // Estimar ppm con una curva genérica tipo MQ2 (log-log)
  // Esto es solo ilustrativo y depende del gas
  // Fórmula: ppm = a * (Rs/Ro)^b  → Aquí usamos valores simulados para CO
  float a = 100;     // Constante simulada
  float b = -1.5;    // Exponente negativo típico en gases

  ppm_estimate = a * pow(ratio, b);

  // Imprimir resultados
  Serial.print("Lectura A0: ");
  Serial.print(rawValue);
  Serial.print(" | Voltaje: ");
  Serial.print(voltage, 2);
  Serial.print(" V | Estimado ppm: ");
  Serial.println(ppm_estimate, 2);

  delay(2000); // Esperar 2 segundos entre lecturas
}