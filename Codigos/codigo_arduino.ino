#include <NewPing.h> 
#include <VarSpeedServo.h> 
 
// Configuracion de pines para los sensores ultrasonicos 
#define TRIGGER_PIN_1 3 // Sensor ultrasonico para carton 
#define ECHO_PIN_1 4 
#define MAX_DISTANCE 40 
 
#define TRIGGER_PIN_2 12 // Sensor ultrasonico para plastico 
#define ECHO_PIN_2 13 
 
// Configuracion de servos 
VarSpeedServo servo_plastico; 
VarSpeedServo servo_carton; 
 
const int pin_plastico = 9; 
const int pin_carton = 10; 
 
const int angulo_cerrado = 180; 
const int angulo_abierto = 0; 
 
const int velocidad_servo = 80; 
const unsigned long intervaloTapa = 7000; 
 
unsigned long tiempoUltimaAccion = 0; 
unsigned long inicioAperturaTapa = 0; 
bool tapaAbierta = false; 
 
// Etiqueta recibida 
String etiquetaRecibida = ""; 
 
// Sensores ultrasonicos 
NewPing sensor_carton(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE); 
NewPing sensor_plastico(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE); 
 
void setup() { 
    Serial.begin(115200); 
 
    // Configurar servos 
    servo_plastico.attach(pin_plastico); 
    servo_carton.attach(pin_carton); 
    servo_plastico.write(angulo_cerrado, velocidad_servo, true); 
    servo_carton.write(angulo_cerrado, velocidad_servo, true); 
 
    Serial.println("Sistema listo."); 
} 
 
void loop() { 
    // Verificar si hay datos recibidos 
    if (Serial.available()) { 
        etiquetaRecibida = Serial.readStringUntil('\n'); 
        etiquetaRecibida.trim(); 
        if (etiquetaRecibida == "plastico") { 
            abrirTapa(servo_plastico); 
        } else if (etiquetaRecibida == "carton") { 
            abrirTapa(servo_carton); 
        } 
    } 
 
    // Controlar el tiempo de apertura de las tapas 
    if (tapaAbierta && millis() - inicioAperturaTapa >= intervaloTapa) { 
        cerrarTapa(); 
    } 
 
    // Leer niveles de llenado 
    int nivelCarton = calcularNivelLlenado(sensor_carton); 
    int nivelPlastico = calcularNivelLlenado(sensor_plastico); 
 
    // Enviar niveles a traves del puerto serie 
    enviarNiveles(nivelPlastico, nivelCarton); 
 
    // Espera entre mediciones para evitar saturar el puerto serie 
    delay(500); 
} 
 
void abrirTapa(VarSpeedServo &servo) { 
    Serial.println("Abriendo tapa..."); 
    servo.write(angulo_abierto, velocidad_servo, true); 
    inicioAperturaTapa = millis(); 
    tapaAbierta = true; 
} 
 
void cerrarTapa() { 
    Serial.println("Cerrando tapa..."); 
    servo_plastico.write(angulo_cerrado, velocidad_servo, true); 
    servo_carton.write(angulo_cerrado, velocidad_servo, true); 
    tapaAbierta = false; 
} 
 
int calcularNivelLlenado(NewPing &sensor) { 
    // Leer la distancia del sensor ultrasonico 
    int distancia = sensor.ping_cm(); 
 
    // Validar la distancia (si es 0, el sensor no detecto nada) 
    if (distancia == 0) { 
        distancia = MAX_DISTANCE; // Asumir que esta vacio 
    } 
 
    // Calcular el nivel de llenado como un porcentaje (entre 0 y 100) 
    int nivel = map(distancia, MAX_DISTANCE, 0, 0, 100); 
    return constrain(nivel, 0, 100); 
} 
 
void enviarNiveles(int nivelPlastico, int nivelCarton) { 
    // Enviar los niveles de llenado al puerto serie 
    Serial.print("P:"); 
    Serial.print(nivelPlastico); 
    Serial.print(",C:"); 
    Serial.println(nivelCarton); 
}