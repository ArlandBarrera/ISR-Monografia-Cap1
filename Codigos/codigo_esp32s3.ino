#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <esp_camera.h>
#include <epsilon3.14-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"


// Configuracion de WiFi
const char* ssid = "ESP32 DE ELBIN";
const char* password = "12345678";

// Definir el modelo de la camara (ajusta esto segun el modelo de camara)
#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM

// Configuracion de pines para la comunicacion Serial1 (RX, TX)
#define RX_PIN D7  // Pin GPIO 44 para RX
#define TX_PIN D6  // Pin GPIO 43 para TX
#define BAUD 115200 // Velocidad de comunicacion
#define LED_AZUL D3   // LED azul para plastico
#define LED_VERDE D4  // LED verde para carton
#define LED_ROJO D5   // LED rojo para fondo


// Definir pines de la camara
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

#define LED_GPIO_NUM      21

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

#define INTERVAL 7000 // Intervalo de 7s entre lecturas

// Variables globales
WebServer server(80);
HardwareSerial mySerial(1);

int nivelPlastico = 0;
int nivelCarton = 0;
bool is_initialised = false;
uint8_t *snapshot_buf;


static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

// Prototipos de funciones
bool ei_camera_init();
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
void manejarEtiqueta(String etiqueta);
void encenderLed(int pin);
void apagarTodosLosLeds();
void realizarInferencia();

// Control de tiempos
unsigned long inicioAccion = 0;
const unsigned long intervaloTapa = 7000; // Mantener la tapa abierta 7 segundos

bool accionEnProgreso = false;
String etiquetaActual = "";

void setup() {
    Serial.begin(115200);
    Serial1.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

    WiFi.softAP(ssid, password);
    Serial.print("IP del AP: ");
    Serial.println(WiFi.softAPIP());
    
    pinMode(LED_AZUL, OUTPUT);
    pinMode(LED_VERDE, OUTPUT);
    pinMode(LED_ROJO, OUTPUT);
    apagarTodosLosLeds();

    // Configurar servidor web
    server.on("/", HTTP_GET, []() {
        String html = "<html><body>";
        html += "<h1>Niveles del Contenedor</h1>";
        html += "<p>Nivel Plastico: " + String(nivelPlastico) + "%</p>";
        html += "<p>Nivel Carton: " + String(nivelCarton) + "%</p>";
        html += "<script>setTimeout(function(){location.reload()},1000);</script>";
        html += "</body></html>";
        server.send(200, "text/html", html);
    });

    server.on("/datos", HTTP_GET, []() {
        StaticJsonDocument<200> doc;
        doc["nivelPlastico"] = nivelPlastico;
        doc["nivelCarton"] = nivelCarton;
        String response;
        serializeJson(doc, response);
        server.send(200, "application/json", response);
    });

    server.begin();

    while (!Serial);  // Espera a que se abra el puerto serie
    Serial.println("Iniciando el sistema de inferencia de Edge Impulse");

    if (!ei_camera_init()) {
        Serial.println("Error al inicializar la camara!");
        return;
    }
    Serial.println("Camara inicializada");
}

void loop() {
    server.handleClient();

    // Procesar datos de clasificacion
    realizarInferencia();

    // Leer datos desde otro dispositivo
    if (mySerial.available()) {
        String datos = mySerial.readStringUntil('\n');
        datos.trim();

        // Parsear datos en formato P:<nivelPlastico>,C:<nivelCarton>
        int pIndex = datos.indexOf("P:");
        int cIndex = datos.indexOf(",C:");

                if (pIndex != -1 && cIndex != -1) {
            String plasticoStr = datos.substring(pIndex + 2, cIndex); // Extraer nivel de plastico
            String cartonStr = datos.substring(cIndex + 3);          // Extraer nivel de carton

            // Convertir a numeros enteros y validar
            int plasticoTemp = plasticoStr.toInt();
            int cartonTemp = cartonStr.toInt();

            if (plasticoTemp >= 0 && plasticoTemp <= 100) {
                nivelPlastico = plasticoTemp; // Actualizar nivel de plastico
            }
            if (cartonTemp >= 0 && cartonTemp <= 100) {
                nivelCarton = cartonTemp; // Actualizar nivel de carton
            }

            // Depuracion: imprimir niveles recibidos
            Serial.print("Niveles recibidos -> Plastico: ");
            Serial.print(nivelPlastico);
            Serial.print("%, Carton: ");
            Serial.print(nivelCarton);
            Serial.println("%");
        }
    }

    // Manejar acciones en progreso
    if (accionEnProgreso && millis() - inicioAccion > intervaloTapa) {
        apagarTodosLosLeds();
        etiquetaActual = "";
        accionEnProgreso = false;
        Serial.println("Accion completada: Tapa cerrada y LEDs apagados.");
    }

    // Si no hay accion en progreso, realizar una nueva clasificacion
    if (!accionEnProgreso) {
        realizarInferencia();
    }
}

void realizarInferencia() {
    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    if (snapshot_buf == nullptr) {
        Serial.println("Error: No se pudo asignar el buffer para la imagen.");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
        Serial.println("Error al capturar la imagen");
        free(snapshot_buf);
        return;
    }

    // Realizar la inferencia
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
    if (err != EI_IMPULSE_OK) {
        Serial.printf("Error al ejecutar la inferencia (%d)\n", err);
        free(snapshot_buf);
        return;
    }

    // Determinar la mejor etiqueta
    String etiqueta = "fondo";
    if (EI_CLASSIFIER_OBJECT_DETECTION == 1) {
        float max_value = 0;

        for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
            ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
            if (bb.value > max_value) {
                max_value = bb.value;
                etiqueta = bb.label;
            }
        }
    } else {
        float scorePlastico = result.classification[0].value;
        float scoreCarton = result.classification[1].value;
        float scoreFondo = result.classification[2].value;

        if (scorePlastico > scoreCarton && scorePlastico > scoreFondo) {
            etiqueta = "plastico";
        } else if (scoreCarton > scorePlastico && scoreCarton > scoreFondo) {
            etiqueta = "carton";
        }
    }

    manejarEtiqueta(etiqueta);

    free(snapshot_buf);
}

void manejarEtiqueta(String etiqueta) {
    etiquetaActual = etiqueta;
    accionEnProgreso = true;
    inicioAccion = millis();

    if (etiqueta == "plastico") {
        Serial1.println("plastico");
        encenderLed(LED_AZUL);
    } else if (etiqueta == "carton") {
        Serial1.println("carton");
        encenderLed(LED_VERDE);
    } else {
        Serial1.println("fondo");
        encenderLed(LED_ROJO);
        accionEnProgreso = false;  // No mantener LED rojo encendido mas tiempo
    }

    if (etiqueta != "fondo") {
        Serial.printf("Tapa abierta para: %s\n", etiqueta.c_str());
    } else {
        Serial.println("No se detecto plastico ni carton, LED rojo encendido.");
    }
}

void encenderLed(int pin) {
    apagarTodosLosLeds();
    digitalWrite(pin, HIGH);
    Serial.printf("LED encendido: %d\n", pin);
}

void apagarTodosLosLeds() {
    digitalWrite(LED_AZUL, LOW);
    digitalWrite(LED_VERDE, LOW);
    digitalWrite(LED_ROJO, LOW);
    Serial.println("Todos los LEDs apagados.");
}

bool ei_camera_init(void) {
    if (is_initialised) return true;

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Error al inicializar la camara: 0x%x\n", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);  // Voltear la imagen
    s->set_hmirror(s, 1); // Espejar la imagen
    is_initialised = true;
    return true;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    if (!is_initialised) {
        Serial.println("Error: La camara no esta inicializada");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Error al capturar la imagen");
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);

    if (!converted) {
        Serial.println("Error en la conversion de formato de imagen");
        return false;
    }

    if (img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS || img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS) {
        ei::image::processing::crop_and_interpolate_rgb888(out_buf, EI_CAMERA_RAW_FRAME_BUFFER_COLS, EI_CAMERA_RAW_FRAME_BUFFER_ROWS, out_buf, img_width, img_height);
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];
        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }
    return 0;
}