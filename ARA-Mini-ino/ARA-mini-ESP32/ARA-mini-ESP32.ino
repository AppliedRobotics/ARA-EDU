#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <stdio.h>
#include "esp_camera.h"
#include "ARA_ESP.h"
#include "ws_html.h"
#include "wp_html.h"

#define CAMERA_MODEL_AI_THINKER  /* PSRAM */
#include "camera_pins.h"

/*
 * Раскомментируйте строку #define CONF_USE_CUSTOM_SSID для использования 
 * пользовательского имени точки доступа коптера. 
 * Раскомментируйте строку #define CONF_USE_DOMAIN_NAME для возможности 
 * использования доменного имени для доступа к веб-интерфейсам управления
 * коптером.
 */
// #define CONF_USE_CUSTOM_SSID
#define CONF_USE_DOMAIN_NAME

const char *password = "12345678";            /* Пароль от точки доступа */
const char *ssid_custom = "ARA-Mini-User";  /* Пользовательское имя точки доступа */ 
const char *domain_name = "ara.mini";         /* Доменное имя веб-интерфейсов коптеров */

AsyncWebServer server(8775);  /* Порт подключения */
AsyncWebSocket ws("/ws");     /* Вебсокет */

const uint8_t DNS_PORT = 53;  /* Порт DNS-сервера */
DNSServer dnsServer;          /* DNS-сервер */

IPAddress local_ip(192, 168, 2, 113);  /* Локальный IP-адрес */
IPAddress gateway(192, 168, 2, 113);   /* IP-адрес шлюза */
IPAddress subnet(255, 255, 255, 0);    /* Маска подсети */

char ssid[16] = {0};

/* 
 * Флаги для запуска Web-пульта 
 */
bool msp_start = false;
bool msp_work = false;

const char* currentPage = html0;

/* 
 * Глобальные переменные для хранения значений крена, тангажа, 
 * рысканья и дросселя, которые будут получены через веб-сокет.
 */
int Arm = 0;
int Flight_state = 0;
int Nav_state = 0;
float RollValue = 0;
float PitchValue = 0;
float YawValue = 0;
float ThrottleValue = 0;
int Programming = 0;

void startCameraServer(void);
void setupLedFlash(int pin);

/*
 * Данная функция разделяет входную строку на массив значений с плавающей запятой
 * с использованием указанного разделителя. Разделенные значения преобразуются в 
 * числа с плавающей запятой и сохраняются в предоставленном массиве.
 */
void splitAndConvertToFloat(const String &inputString, float array[], int arraySize, char delimiter)
{
  char temp[inputString.length() + 1];
  inputString.toCharArray(temp, inputString.length() + 1);

  char *token = strtok(temp, ";");
  int count = 0;

  while (token != NULL && count < arraySize)
  {
    array[count] = atof(token);
    token = strtok(NULL, ";");
    count++;
  }
}

/* 
 * Данная функция обрабатывает входящие данные веб-сокета. Она извлекает данные в виде
 * строки и разделяет их на массив значений с плавающей запятой с использованием точки
 * с запятой в качестве разделителя. Затем разделенные значения присваиваются 
 * соответствующим глобальным переменным YawValue, ThrottleValue, RollValue и PitchValue.
 */
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (type == WS_EVT_DATA)
  {
    data[len] = 0;  /* Явное обозначение конца строки */

    // Serial.printf("Data received: %s\n", (char*)data);
    String inputString = String((char *)data);
    
    if (inputString.startsWith("START;")){
			msp_start = true;
		}

    float values[8];

    splitAndConvertToFloat(inputString, values, 8, ';');

    Arm = values[0];
    Flight_state = values[6];
    YawValue = values[2];
    ThrottleValue = values[3];
    RollValue = values[4];
    PitchValue = values[5];
    Nav_state = values[1];
    Programming = values[7];
  }
}

/*
 * Данная функция формирует обычное имя WiFi сети коптера с постфиксом
 * на основе ID микроконтроллера. Данное имя сети используется если не определено
 * CONF_USE_CUSTOM_SSID
 */
void getUniqName(uint8_t len)
{
  uint64_t chip_id = ESP.getEfuseMac();   /* Чтение ID микроконтроллера */
  uint32_t mask = (1 << (len * 4)) - 1;
  chip_id >>= 24;   /* Формировние постфикса для имени сети из ID микроконтроллера */
  snprintf(ssid, sizeof(ssid), "ARA-Mini-%0*X", len,  (uint32_t)(chip_id & mask));
}

void setup(void)
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  
  camera_config_t config;         /* Настройка конфигурации камеры */
  pinMode(XCLK_GPIO_NUM, OUTPUT); /* Установка пина XCLK (тактовый сигнал) в режим вывода */
  digitalWrite(XCLK_GPIO_NUM, 0); /* Установка пина XCLK в низкий уровень (0) */

  /* Настройка конфигурации камеры */
  config.ledc_channel = LEDC_CHANNEL_0;   /* Канал ШИМ */
  config.ledc_timer = LEDC_TIMER_0;       /* Таймер ШИМ */
  config.pin_d0 = Y2_GPIO_NUM;            /* Пины данных (D0-D7) */
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;        /* Пины управления камерой */
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;        /* Пины питания и сброса */
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;       /* Формат пикселей (JPEG для потоковой передачи) */
  //config.pixel_format = PIXFORMAT_RGB565;   /* Формат для распознавания/обнаружения лиц */
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;  /* Режим захвата */
  config.fb_location = CAMERA_FB_IN_PSRAM;    /* Расположение буфера кадра */
  config.jpeg_quality = 12;                   /* Качество JPEG */
  config.fb_count = 1;                        /* Количество буферов кадров */

  /* Настройка параметров камеры в зависимости от наличия PSRAM */
  if (config.pixel_format == PIXFORMAT_JPEG)
  {
    if (psramFound())
    {
      /* Если есть PSRAM, используем более высокое качество JPEG и больше буферов кадров */
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    }
    else 
    {
      /* Если PSRAM нет, ограничиваем разрешение и используем DRAM */
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } 
  else 
  {
    /* Наилучшие параметры для распознавания/обнаружения лиц */
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  /* Инициализация камеры с заданными параметрами */
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    /* Вывод ошибки в последовательный порт, если инициализация камеры не удалась. */
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  /* Получение объекта сенсора камеры */
  sensor_t *s = esp_camera_sensor_get();
  /* Настройка сенсора (отражение по вертикали, яркость, насыщенность) */ 
  if (s->id.PID == OV3660_PID)
  {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  /* Изменение размера кадра для повышения частоты кадров */
  if (config.pixel_format == PIXFORMAT_JPEG)
  {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

/* Дополнительная настройка для конкретных моделей камер */
#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

/* Настройка выводов для светодиодов, если данные выводы определены в camera_pins.h */
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif
  
#if defined(CONF_USE_CUSTOM_SSID)
  memcpy(ssid, ssid_custom, strlen(ssid_custom));
#else
  getUniqName(5);
#endif
  
  WiFi.softAP(ssid, password);        /* Создать точку доступа с именем ssid и паролем password */
  Serial.print("WiFi Ready! Use ");
  Serial.print(ssid);
  Serial.print(" with password: ");
  Serial.println(password);
  WiFi.softAPConfig(local_ip, gateway, subnet);   /* Настроить конфигурацию точки доступа (локальный IP, шлюз, маска подсети) */

  Serial.print("AP IP Address: ");    /* Вывести IP-адрес точки доступа в последовательный порт */
  Serial.println(WiFi.softAPIP());

  IPAddress ip_address = WiFi.softAPIP();

#if defined(CONF_USE_DOMAIN_NAME)
  Serial.print("Copter web interfaces domain name: ");
  Serial.println(domain_name);
  Serial.println("Ports: 80 - camera control, 8775 - joystics");
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(DNS_PORT, domain_name, ip_address);
#endif

  startCameraServer();

  Serial.println("Camera Ready!");
  Serial.print(" Use http://");
  Serial.print(ip_address);
#if defined(CONF_USE_DOMAIN_NAME)
  Serial.print(" or http://");
  Serial.print(domain_name);
#endif
  Serial.print("to connect");
  Serial.println("End!");

  ws.onEvent(onWsEvent);   /* Настроить веб-сокет */
  server.addHandler(&ws);  /* Добавить обработчик веб-сокета на сервер */

  /* Обработчик запросов GET по адресу "/" */
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", currentPage);
  });

  server.begin();     /* Запустить сервер */
  esp.begin(Serial);  /* Запустить сервис по отправки данных в полетный контроллер */
}


void loop(void)
{
  esp.arm(Arm);       /* Установить арм */
  delay(20);
  // delay(50);

  // Проверка на смену страницы
  if (msp_start && !msp_work) {
      // Обновляем страницу на другую
      currentPage = html1;

      // Обработчик для новой страницы
      server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", currentPage);
      });

    msp_work = true;
  }

  if (!Programming)
  {
    esp.nav_mode(Nav_state);        /* Установить навигационный режим */
    delay(20);
    // delay(50);

    // esp.flight_mode(Flight_state);
    // delay(20);

    esp.roll(RollValue);            /* Установить крен */
    delay(20);
    // delay(50);

    esp.pitch(PitchValue);          /* Установить тангаж */
    delay(20);

    esp.yaw(YawValue);              /* Установить рысканье */
    delay(20);

    esp.throttle(ThrottleValue);    /* Установить дроссель */
    delay(20);
  }

  if (Programming)
  {
    esp.nav_mode(0);    /* Установить навигационный режим */
    delay(20);
  }
}
