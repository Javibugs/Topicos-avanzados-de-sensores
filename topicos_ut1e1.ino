#include <SoftwareSerial.h>  // Incluye la librería SoftwareSerial para usar puertos serie en pines digitales

#define RX_PIN 10  // Pin de recepción para leer los datos del GPS
#define TX_PIN 11  // Pin de transmisión, aunque no se usa en este código


SoftwareSerial gpsSerial(RX_PIN, TX_PIN);  // Crea un objeto SoftwareSerial para manejar la comunicación con el GPS
char nmeaBuffer[100];  // Buffer para almacenar los datos de la línea NMEA recibida


void setup() {
    Serial.begin(9600);  // Inicia la comunicación con el monitor serie (PC) a 9600 baudios
    gpsSerial.begin(9600);  // Inicia la comunicación con el módulo GPS a 9600 baudios
}

void loop() {
    static byte index = 0;  // Variable para controlar el índice en el buffer

    // Lee los datos disponibles del GPS
    while (gpsSerial.available()) {
        char c = gpsSerial.read();  // Lee un carácter del GPS

        // Si el carácter leído es un salto de línea ('\n'), significa que terminó una línea de datos
        if (c == '\n') {
            nmeaBuffer[index] = '\0';  // Termina la cadena de caracteres con un carácter nulo
            index = 0;  // Reinicia el índice del buffer

            // Si la línea contiene "$GPRMC", procesamos la información de latitud y longitud
            if (strncmp(nmeaBuffer, "$GPRMC", 6) == 0) {
                procesarGPRMC(nmeaBuffer);  // Llama a la función para procesar los datos
            }
        } else if (index < sizeof(nmeaBuffer) - 1) {
            nmeaBuffer[index++] = c;  // Si el buffer no está lleno, sigue almacenando caracteres
        }
    }
}

/**
 * Procesa la línea NMEA tipo $GPRMC, que contiene la latitud y longitud
 */
void procesarGPRMC(char *nmea) {
    char *tok = strtok(nmea, ",");  // Separa la línea NMEA en fragmentos usando la coma como delimitador
    int i = 0;
    char latitudStr[10], longitudStr[11], ns, ew;

    // Recorre los fragmentos de la línea NMEA
    while (tok) {
        i++;
        if (i == 4) strncpy(latitudStr, tok, sizeof(latitudStr) - 1);  // Extrae la latitud
        if (i == 5) ns = tok[0];  // Guarda el hemisferio de la latitud (Norte o Sur)
        if (i == 6) strncpy(longitudStr, tok, sizeof(longitudStr) - 1);  // Extrae la longitud
        if (i == 7) ew = tok[0];  // Guarda el hemisferio de la longitud (Este u Oeste)
        tok = strtok(NULL, ",");  // Continúa separando la siguiente parte de la cadena
    }

    // Si se obtuvieron valores válidos de latitud y longitud
    if (latitudStr[0] && longitudStr[0]) {
        // Convierte las coordenadas y las imprime en formato decimal
        Serial.print("Latitud: ");
        Serial.print(convertirCoordenada(latitudStr));  // Convierte latitud a formato decimal
        Serial.print((ns == 'S') ? " S" : " N");  // Imprime el hemisferio de la latitud
        Serial.print(" | Longitud: ");
        Serial.print(convertirCoordenada(longitudStr));  // Convierte longitud a formato decimal
        Serial.println((ew == 'W') ? " W" : " E");  // Imprime el hemisferio de la longitud
    } else {
        // Si no se obtuvo información válida de latitud o longitud
        Serial.println("No se obtuvo ubicación.");
    }
}

/**
 * Convierte las coordenadas en formato NMEA (DDMM.MMMM) a formato decimal (DD.DDDDD)
 */
float convertirCoordenada(char *coordenada) {
    float valor = atof(coordenada);  // Convierte la cadena a un número flotante
    int grados = (int)(valor / 100);  // Extrae los grados (DD)
    float minutos = valor - (grados * 100);  // Extrae los minutos (MM.MMMM)
    return grados + (minutos / 60);  // Convierte la coordenada a formato decimal (DD.DDDDD)
}