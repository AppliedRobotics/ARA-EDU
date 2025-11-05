#include "esp_camera.h"
#include "WiFi.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// Конфигурация пинов
#define LED_PIN 32

#define SD_D0 2
#define SD_D1 4
#define SD_D2 12
#define SD_D3 13
#define SD_CMD 15
#define SD_CLK 14

#define CAM_SDA 26
#define CAM_SCL 27
#define CAM_CSI_VSYNC 25
#define CAM_CSI_HREF  23
#define CAM_CSI_7 35
#define CAM_CSI_XCLK 0
#define CAM_CSI_6 34
#define CAM_CSI_5 39
#define CAM_CSI_PCLK 22
#define CAM_CSI_4 36
#define CAM_CSI_0 5
#define CAM_CSI_3 21
#define CAM_CSI_1 18
#define CAM_CSI_2 19

#define ADC_PIN 33

// Настройки WiFi
const char* ssid = "WIFI_SSID";
const char* password = "WIFI_PASSWORD";
WiFiServer server(80);

// Конфигурация камеры
camera_config_t config;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  
  // Инициализация SD карты (опционально)
  initSDCard();
  
  // Инициализация камеры
  if(!initCamera()){
    Serial.println("Camera Init Failed");
    return;
  }
  
  // Настройка ориентации камеры
  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1);  // Перевернуть по вертикали (1 - включено)
  s->set_hmirror(s, 1); // Зеркально по горизонтали (1 - включено)
  
  // Подключение к WiFi
  connectToWiFi();
  
  // Запуск сервера
  server.begin();
}

void loop() {
  // Мигаем светодиодом для индикации работы
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  delay(100);
  
  // Обработка клиентских подключений
  handleClient();
}

bool initSDCard() {
  SPI.begin(SD_CLK, SD_D0, SD_CMD, SD_D1);
  if(!SD.begin(SD_CMD, SPI, 4000000, "/sd")) {
    Serial.println("SD Card Mount Failed");
    return false;
  }
  Serial.println("SD Card initialized");
  return true;
}

bool initCamera() {
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_CSI_0;
  config.pin_d1 = CAM_CSI_1;
  config.pin_d2 = CAM_CSI_2;
  config.pin_d3 = CAM_CSI_3;
  config.pin_d4 = CAM_CSI_4;
  config.pin_d5 = CAM_CSI_5;
  config.pin_d6 = CAM_CSI_6;
  config.pin_d7 = CAM_CSI_7;
  config.pin_xclk = CAM_CSI_XCLK;
  config.pin_pclk = CAM_CSI_PCLK;
  config.pin_vsync = CAM_CSI_VSYNC;
  config.pin_href = CAM_CSI_HREF;
  config.pin_sscb_sda = CAM_SDA;
  config.pin_sscb_scl = CAM_SCL;
  config.pin_reset = -1;
  config.pin_pwdn = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // Уменьшаем разрешение для повышения FPS
  config.frame_size = FRAMESIZE_VGA;  // 640x480 (было SVGA 800x600)
  config.jpeg_quality = 10;           // Качество (1-63, меньше - лучше качество, но медленнее)
  config.fb_count = 2;                // Количество буферов

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return false;
  }
  
  return true;
}

void connectToWiFi() {
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  
  // Выводим IP-адрес для подключения
  Serial.print("Camera Stream Ready! Go to: http://");
  Serial.println(WiFi.localIP());
  Serial.println("To view the stream, open the above URL in your browser");
}

void handleClient() {
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  // Ждем данные от клиента
  while(!client.available()){
    delay(1);
  }

  String req = client.readStringUntil('\r');
  client.flush();

  if (req.indexOf("/stream") != -1 || req.indexOf("/") != -1) {
    // Отправляем MJPEG поток
    sendVideoStream(client);
  } else {
    // Неизвестный запрос
    client.println("HTTP/1.1 404 Not Found");
    client.println();
  }
}

void sendVideoStream(WiFiClient &client) {
  Serial.println("New video client");

  // Заголовки HTTP ответа для MJPEG потока
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: close");
  client.println();

  // Увеличиваем приоритет задачи для более плавного стрима
  // (только для ESP32, для других платформ это не сработает)
  #ifdef ESP32
    vTaskPrioritySet(NULL, 5);
  #endif

  uint32_t last_frame = 0;
  while (client.connected()) {
    uint32_t now = millis();
    if (now - last_frame < 33) { // ~30 FPS
      delay(1);
      continue;
    }
    last_frame = now;

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      continue;
    }

    // Отправляем кадр
    client.println("--frame");
    client.println("Content-Type: image/jpeg");
    client.println("Content-Length: " + String(fb->len));
    client.println();
    client.write(fb->buf, fb->len);
    client.println();

    esp_camera_fb_return(fb);

    // Проверяем новые данные от клиента (может быть команда остановки)
    if(client.available()) {
      char c = client.read();
      if (c == '\n') break;
    }
  }

  #ifdef ESP32
    vTaskPrioritySet(NULL, 1); // Возвращаем стандартный приоритет
  #endif

  Serial.println("Client disconnected");
}