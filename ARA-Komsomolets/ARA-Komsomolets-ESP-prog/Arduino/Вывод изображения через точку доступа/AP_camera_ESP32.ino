// Подключение необходимых библиотек
#include "esp_camera.h"  // Для работы с камерой
#include "WiFi.h"        // Для Wi-Fi функционала
#include "FS.h"          // Файловая система
#include "SD.h"          // Работа с SD картой
#include "SPI.h"         // SPI интерфейс

// Конфигурация пинов
#define LED_PIN 32       // Пин для светодиода индикации

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

#define ADC_PIN 33       // Аналоговый вход (не используется в основном коде)

// Настройки Wi-Fi точки доступа
const char* ap_ssid = "ESP32-Camera-AP";      // Имя точки доступа
const char* ap_password = "12345678";         // Пароль (минимум 8 символов)
WiFiServer server(80);                        // HTTP сервер на порту 80

// Конфигурация камеры
camera_config_t config;                       // Структура для хранения настроек камеры

void setup() {
  Serial.begin(115200);                       // Инициализация последовательного порта
  pinMode(LED_PIN, OUTPUT);                   // Настройка пина светодиода как выход
  
  // Инициализация SD карты (опционально)
  initSDCard();
  
  // Инициализация камеры
  if(!initCamera()){
    Serial.println("Camera Init Failed");     // Сообщение об ошибке инициализации камеры
    return;                                   // Выход при неудачной инициализации
  }
  
  // Настройка ориентации камеры
  sensor_t *s = esp_camera_sensor_get();      // Получение указателя на сенсор камеры
  s->set_vflip(s, 1);                         // Включить вертикальное отражение (1 - включено)
  s->set_hmirror(s, 1);                       // Включить горизонтальное зеркалирование (1 - включено)
  
  // Создание Wi-Fi точки доступа
  createWiFiAP();
  
  // Запуск сервера
  server.begin();                             // Запуск HTTP сервера
}

void loop() {
  // Мигаем светодиодом для индикации работы
  digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Инвертируем состояние светодиода
  delay(100);                                 // Задержка 100 мс
  
  // Обработка клиентских подключений
  handleClient();                             // Проверка и обработка HTTP запросов
}

// Функция инициализации SD карты
bool initSDCard() {
  SPI.begin(SD_CLK, SD_D0, SD_CMD, SD_D1);   // Инициализация SPI с указанием пинов
  if(!SD.begin(SD_CMD, SPI, 4000000, "/sd")) { // Попытка монтирования SD карты
    Serial.println("SD Card Mount Failed");   // Сообщение об ошибке
    return false;                             // Возврат false при ошибке
  }
  Serial.println("SD Card initialized");      // Сообщение об успешной инициализации
  return true;                                // Возврат true при успехе
}

// Функция инициализации камеры
bool initCamera() {
  // Настройка канала LEDC для тактирования камеры
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  
  // Назначение пинов для данных камеры
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
  config.pin_sscb_sda = CAM_SDA;              // I2C данные
  config.pin_sscb_scl = CAM_SCL;              // I2C тактовый
  config.pin_reset = -1;                      // Сброс не используется
  config.pin_pwdn = -1;                       // Режим power down не используется
  
  // Настройки тактовой частоты и формата
  config.xclk_freq_hz = 20000000;             // Тактовая частота 20 MHz
  config.pixel_format = PIXFORMAT_JPEG;       // Формат пикселей - JPEG
  
  // Уменьшаем разрешение для повышения FPS
  config.frame_size = FRAMESIZE_VGA;          // Разрешение 640x480
  config.jpeg_quality = 10;                   // Качество JPEG (1-63, меньше - лучше)
  config.fb_count = 2;                        // Количество frame buffer'ов

  // Инициализация камеры
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {                        // Проверка на ошибки
    Serial.printf("Camera init failed with error 0x%x", err); // Вывод ошибки
    return false;                             // Возврат false при ошибке
  }
  
  return true;                                // Возврат true при успехе
}

// Функция создания Wi-Fi точки доступа
void createWiFiAP() {
  Serial.println("Creating Access Point...");
  
  // Создаем точку доступа
  WiFi.softAP(ap_ssid, ap_password);          // Запуск точки доступа
  
  // Настраиваем IP-адрес точки доступа (опционально)
  // WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  
  // Вывод информации о созданной точке доступа
  Serial.print("Access Point Created: ");
  Serial.println(ap_ssid);
  Serial.print("Password: ");
  Serial.println(ap_password);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("Connect to this WiFi network and go to the IP address above");
  Serial.println("to view the camera stream");
}

// Функция обработки клиентских подключений
void handleClient() {
  WiFiClient client = server.available();     // Проверка наличия подключенного клиента
  if (!client) {
    return;                                   // Выход если клиентов нет
  }

  // Ждем данные от клиента
  while(!client.available()){                 // Ожидание данных от клиента
    delay(1);
  }

  // Чтение HTTP запроса
  String req = client.readStringUntil('\r');  // Чтение запроса до символа возврата каретки
  client.flush();                             // Очистка буфера клиента

  // Обработка различных запросов
  if (req.indexOf("/stream") != -1 || req.indexOf("/") != -1) {
    // Отправляем MJPEG поток для запросов /stream или /
    sendVideoStream(client);
  } else {
    // Неизвестный запрос - отправляем 404 ошибку
    client.println("HTTP/1.1 404 Not Found");
    client.println();
  }
}

// Функция отправки видео потока
void sendVideoStream(WiFiClient &client) {
  Serial.println("New video client");

  // Заголовки HTTP ответа для MJPEG потока
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame"); // MJPEG формат
  client.println("Access-Control-Allow-Origin: *"); // Разрешить CORS запросы
  client.println("Connection: close");
  client.println();

  // Увеличиваем приоритет задачи для более плавного стрима
  #ifdef ESP32
    vTaskPrioritySet(NULL, 5);                // Повышение приоритета задачи
  #endif

  uint32_t last_frame = 0;                    // Время последнего кадра
  while (client.connected()) {                // Цикл пока клиент подключен
    uint32_t now = millis();
    if (now - last_frame < 33) {              // Ограничение FPS (~30 кадров в секунду)
      delay(1);
      continue;
    }
    last_frame = now;                         // Обновление времени последнего кадра

    // Получение кадра от камеры
    camera_fb_t *fb = esp_camera_fb_get();    // Захват frame buffer'а
    if (!fb) {
      Serial.println("Camera capture failed"); // Ошибка захвата кадра
      continue;                               // Продолжение цикла
    }

    // Отправка кадра клиенту
    client.println("--frame");                // Разделитель для multipart
    client.println("Content-Type: image/jpeg"); // Тип содержимого
    client.println("Content-Length: " + String(fb->len)); // Размер JPEG изображения
    client.println();                         // Пустая строка - конец заголовков
    client.write(fb->buf, fb->len);           // Отправка JPEG данных
    client.println();                         // Завершение части

    // Возврат frame buffer'а камере
    esp_camera_fb_return(fb);

    // Проверка новых данных от клиента (возможность разрыва соединения)
    if(client.available()) {
      char c = client.read();
      if (c == '\n') break;                   // Выход из цикла при определенных условиях
    }
  }

  // Возврат стандартного приоритета задачи
  #ifdef ESP32
    vTaskPrioritySet(NULL, 1);                // Стандартный приоритет
  #endif

  Serial.println("Client disconnected");      // Сообщение об отключении клиента
}