// Подключение необходимых библиотек
#include "esp_camera.h"  // Работа с камерой ESP32
#include "WiFi.h"        // Wi-Fi подключение
#include "FS.h"          // Файловая система
#include "SD.h"          // Работа с SD картой
#include "SPI.h"         // SPI интерфейс

// Конфигурация пинов
#define LED_PIN 32       // Пин светодиода для индикации работы

// Пины для SD карты в режиме SPI
#define SD_D0 2
#define SD_D1 4
#define SD_D2 12
#define SD_D3 13
#define SD_CMD 15
#define SD_CLK 14

// Пины для подключения камеры (CSI интерфейс)
#define CAM_SDA 26       // I2C данные для конфигурации камеры
#define CAM_SCL 27       // I2C тактовый сигнал
#define CAM_CSI_VSYNC 25 // Вертикальная синхронизация
#define CAM_CSI_HREF  23 // Горизонтальный синхроимпульс
#define CAM_CSI_7 35     // Линия данных 7
#define CAM_CSI_XCLK 0   // Тактовый сигнал камеры
#define CAM_CSI_6 34     // Линия данных 6
#define CAM_CSI_5 39     // Линия данных 5
#define CAM_CSI_PCLK 22  // Тактовый сигнал пикселей
#define CAM_CSI_4 36     // Линия данных 4
#define CAM_CSI_0 5      // Линия данных 0
#define CAM_CSI_3 21     // Линия данных 3
#define CAM_CSI_1 18     // Линия данных 1
#define CAM_CSI_2 19     // Линия данных 2

#define ADC_PIN 33       // Аналоговый вход (зарезервирован для будущего использования)

// Настройки WiFi сети
const char* ssid = "AR_Avia_301";        // SSID WiFi сети для подключения
const char* password = "listentome"; // Пароль WiFi сети
WiFiServer server(80);                 // HTTP сервер на порту 80

// Конфигурация камеры
camera_config_t config;  // Структура для хранения настроек камеры

void setup() {
  Serial.begin(115200);               // Инициализация последовательного порта
  pinMode(LED_PIN, OUTPUT);           // Настройка пина светодиода как выход
  
  // Инициализация SD карты (опционально)
  initSDCard();
  
  // Инициализация камеры
  if(!initCamera()){
    Serial.println("Camera Init Failed"); // Сообщение об ошибке инициализации камеры
    return;                             // Выход при неудачной инициализации
  }
  
  // Настройка ориентации изображения с камеры
  sensor_t *s = esp_camera_sensor_get(); // Получение указателя на сенсор камеры
  s->set_vflip(s, 1);                  // Включить вертикальное отражение (1 - включено)
  s->set_hmirror(s, 1);                // Включить горизонтальное зеркалирование (1 - включено)
  
  // Подключение к WiFi сети
  connectToWiFi();
  
  // Запуск HTTP сервера
  server.begin();
}

void loop() {
  // Мигаем светодиодом для индикации работы системы
  digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Инвертируем состояние светодиода
  delay(100);                              // Задержка 100 мс между переключениями
  
  // Обработка клиентских подключений к HTTP серверу
  handleClient();
}

// Функция инициализации SD карты
bool initSDCard() {
  SPI.begin(SD_CLK, SD_D0, SD_CMD, SD_D1); // Инициализация SPI с указанием пинов
  if(!SD.begin(SD_CMD, SPI, 4000000, "/sd")) { // Попытка монтирования SD карты
    Serial.println("SD Card Mount Failed"); // Сообщение об ошибке
    return false;                         // Возврат false при ошибке
  }
  Serial.println("SD Card initialized");  // Сообщение об успешной инициализации
  return true;                            // Возврат true при успехе
}

// Функция инициализации камеры
bool initCamera() {
  // Настройка канала LEDC для тактирования камеры
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  
  // Назначение пинов для данных камеры (CSI интерфейс)
  config.pin_d0 = CAM_CSI_0;
  config.pin_d1 = CAM_CSI_1;
  config.pin_d2 = CAM_CSI_2;
  config.pin_d3 = CAM_CSI_3;
  config.pin_d4 = CAM_CSI_4;
  config.pin_d5 = CAM_CSI_5;
  config.pin_d6 = CAM_CSI_6;
  config.pin_d7 = CAM_CSI_7;
  
  // Назначение управляющих пинов камеры
  config.pin_xclk = CAM_CSI_XCLK;
  config.pin_pclk = CAM_CSI_PCLK;
  config.pin_vsync = CAM_CSI_VSYNC;
  config.pin_href = CAM_CSI_HREF;
  config.pin_sscb_sda = CAM_SDA;        // I2C данные для конфигурации
  config.pin_sscb_scl = CAM_SCL;        // I2C тактовый для конфигурации
  config.pin_reset = -1;                // Сброс не используется
  config.pin_pwdn = -1;                 // Режим power down не используется
  
  // Настройки тактовой частоты и формата изображения
  config.xclk_freq_hz = 20000000;       // Тактовая частота 20 MHz
  config.pixel_format = PIXFORMAT_JPEG; // Формат пикселей - JPEG
  
  // Настройки разрешения и качества для оптимизации производительности
  config.frame_size = FRAMESIZE_VGA;    // Разрешение 640x480 (оптимальное для потока)
  config.jpeg_quality = 10;             // Качество JPEG (1-63, меньше - лучше качество)
  config.fb_count = 2;                  // Количество frame buffer'ов

  // Инициализация камеры с проверкой ошибок
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {                  // Проверка на ошибки инициализации
    Serial.printf("Camera init failed with error 0x%x", err); // Вывод кода ошибки
    return false;                       // Возврат false при ошибке
  }
  
  return true;                          // Возврат true при успешной инициализации
}

// Функция подключения к WiFi сети
void connectToWiFi() {
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);           // Начало подключения к WiFi
  
  // Ожидание установления подключения
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);                         // Задержка между попытками
    Serial.print(".");                  // Индикация процесса подключения
  }
  Serial.println("\nWiFi connected");   // Сообщение об успешном подключении
  
  // Вывод IP-адреса для подключения к видеопотоку
  Serial.print("Camera Stream Ready! Go to: http://");
  Serial.println(WiFi.localIP());
  Serial.println("To view the stream, open the above URL in your browser");
}

// Функция обработки клиентских подключений к HTTP серверу
void handleClient() {
  WiFiClient client = server.available(); // Проверка наличия подключенного клиента
  if (!client) {
    return;                             // Выход если клиентов нет
  }

  // Ожидание данных от клиента
  while(!client.available()){
    delay(1);
  }

  // Чтение HTTP запроса от клиента
  String req = client.readStringUntil('\r'); // Чтение запроса до символа возврата каретки
  client.flush();                        // Очистка буфера клиента

  // Анализ запроса и маршрутизация
  if (req.indexOf("/stream") != -1 || req.indexOf("/") != -1) {
    // Запрос видеопотока - отправляем MJPEG поток
    sendVideoStream(client);
  } else {
    // Неизвестный запрос - отправляем ошибку 404
    client.println("HTTP/1.1 404 Not Found");
    client.println();
  }
}

// Функция отправки MJPEG видеопотока клиенту
void sendVideoStream(WiFiClient &client) {
  Serial.println("New video client");   // Логирование нового подключения

  // Заголовки HTTP ответа для MJPEG потока
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame"); // MJPEG формат
  client.println("Access-Control-Allow-Origin: *"); // Разрешить CORS запросы
  client.println("Connection: close");  // Закрыть соединение после передачи
  client.println();

  // Повышение приоритета задачи для более плавного стрима (только для ESP32)
  #ifdef ESP32
    vTaskPrioritySet(NULL, 5);          // Установка высокого приоритета задачи
  #endif

  uint32_t last_frame = 0;              // Время последнего отправленного кадра
  while (client.connected()) {          // Основной цикл потоковой передачи
    uint32_t now = millis();
    // Ограничение частоты кадров (~30 FPS)
    if (now - last_frame < 33) {        // 33 мс = ~30 кадров в секунду
      delay(1);
      continue;
    }
    last_frame = now;                   // Обновление времени последнего кадра

    // Захват кадра с камеры
    camera_fb_t *fb = esp_camera_fb_get(); // Получение frame buffer'а с изображением
    if (!fb) {
      Serial.println("Camera capture failed"); // Ошибка захвата кадра
      continue;                         // Продолжение цикла
    }

    // Отправка JPEG кадра клиенту в формате multipart
    client.println("--frame");          // Разделитель для multipart
    client.println("Content-Type: image/jpeg"); // Тип содержимого - JPEG
    client.println("Content-Length: " + String(fb->len)); // Размер изображения
    client.println();                   // Пустая строка - конец заголовков
    client.write(fb->buf, fb->len);     // Отправка бинарных JPEG данных
    client.println();                   // Завершение части

    // Возврат frame buffer'а камере для повторного использования
    esp_camera_fb_return(fb);

    // Проверка новых данных от клиента (возможность разрыва соединения)
    if(client.available()) {
      char c = client.read();
      if (c == '\n') break;             // Выход из цикла при определенных условиях
    }
  }

  // Возврат стандартного приоритета задачи (только для ESP32)
  #ifdef ESP32
    vTaskPrioritySet(NULL, 1);          // Восстановление стандартного приоритета
  #endif

  Serial.println("Client disconnected"); // Логирование отключения клиента
}