#include "esp_camera.h"
#include "WiFi.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "MSP.h"
#include <ArduinoJson.h>

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
#define CAM_CSI_HREF 23
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

extern MSP msp;

// Настройки Wi-Fi точки доступа
const char* ap_ssid = "ESP32-Camera-AP";
const char* ap_password = "12345678";

WiFiServer server(80);

// Конфигурация камеры
camera_config_t config;

// Структура для данных IMU
struct IMUData {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
  float sonar;
  uint32_t timestamp;
};

// Глобальные переменные
IMUData currentIMU;
unsigned long lastIMURead = 0;
unsigned long lastRCWrite = 0;
uint8_t blink_enabled = 0;
uint8_t motor_enabled = 0;
const unsigned long IMU_READ_INTERVAL = 100; // 100ms
const unsigned long RC_WRITE_INTERVAL = 50;  // 50ms
uint8_t action_counter = 0;
const uint8_t ACTION_DURATION = 20;         // ~1s (ACTION_DURATION * RC_WRITE_INTERVAL)
// Для потоковой передачи видео
WiFiClient streamClient;
unsigned long lastFrame = 0;
const unsigned long FRAME_INTERVAL = 1000 / 40; // 15 FPS
bool streamingActive = false;

void setup() {
  Serial.begin(115200);
  msp.begin(Serial);
  pinMode(LED_PIN, OUTPUT);
 
  // Инициализация SD карты
  initSDCard();
  
  // Инициализация камеры
  if(!initCamera()){
    Serial.println("Camera Init Failed");
    // Не останавливаем выполнение, продолжаем без камеры
  } else {
    Serial.println("Camera Init Success");
  }
 
  // Настройка ориентации камеры
  // sensor_t *s = esp_camera_sensor_get();
  // if(s) {
    // s->set_vflip(s, 1);
    // s->set_hmirror(s, 1);
  // }
 
  // Создание Wi-Fi точки доступа
  createWiFiAP();
 
  // Запуск сервера
  server.begin();
 
  // Инициализация данных IMU
  memset(&currentIMU, 0, sizeof(currentIMU));
}

void loop() {
  // Мигаем светодиодом для индикации работы
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 5000) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    lastBlink = millis();
  }
 
  // Опрос MSP (IMU данные)
  msp_task();
 
  // Обработка клиентских подключений
  handleClient();
 
  // Обработка потоковой передачи, если активна
  handleStreaming();

  delay(10); // Небольшая задержка для стабильности
}

msp_rc_t rc_motors = {0};
void msp_task() {
  // Чтение MSP данных с заданным интервалом
  if ((millis() - lastRCWrite) >= RC_WRITE_INTERVAL)
  {
    if (blink_enabled)
    {
      rc_motors.channelValue[4] = 2000; // ARM on
      if (action_counter >= ACTION_DURATION)
      {
        blink_enabled = 0;
        action_counter = 0;
        rc_motors.channelValue[2] = 1000; // Throttle off
        rc_motors.channelValue[4] = 1000; // ARM off
      }
      action_counter++;
    }
    else if (motor_enabled)
    {
      rc_motors.channelValue[4] = 2000; // ARM on
      rc_motors.channelValue[2] = 1050; // Throttle on
      if (action_counter >= ACTION_DURATION)
      {
        motor_enabled = 0;
        action_counter = 0;
        rc_motors.channelValue[2] = 1000; // Throttle off
        rc_motors.channelValue[4] = 1000; // ARM off
      }
      action_counter++;
    }
    rc_motors.channelValue[0] = 1500; // Roll neutral
    rc_motors.channelValue[1] = 1500; // Pitch neutral
    // rc_motors.channelValue[2] = 1000; // Throttle off
    rc_motors.channelValue[3] = 1500; // Yaw neutral
    msp.command(MSP_SET_RAW_RC, &rc_motors, sizeof(rc_motors));
    lastRCWrite = millis();
  }
  if (millis() - lastIMURead >= IMU_READ_INTERVAL) {
    // msp_rc_t rc;
    // if (msp.request(MSP_RC, &rc, sizeof(rc))) {
    //   uint16_t roll = rc.channelValue[0];
    //   uint16_t pitch = rc.channelValue[1];
    //   uint16_t yaw = rc.channelValue[2];
    //   uint16_t throttle = rc.channelValue[3];
    //   uint16_t arm = rc.channelValue[4];
     
    //   // Обновляем структуру IMU данных
    //   currentIMU.accel_x = map(roll, 1000, 2000, -1000, 1000) / 1000.0;
    //   currentIMU.accel_y = map(pitch, 1000, 2000, -1000, 1000) / 1000.0;
    //   currentIMU.accel_z = map(throttle, 1000, 2000, 0, 2000) / 1000.0;
    //   currentIMU.gyro_x = map(yaw, 1000, 2000, -1000, 1000) / 1000.0;
    //   currentIMU.timestamp = millis();
     
    //   // Управление светодиодом по каналу arm
    //   digitalWrite(LED_PIN, arm > 1500);
     
    //   // Вывод в Serial для отладки
    //   static unsigned long lastDebug = 0;
    //   if (millis() - lastDebug > 1000) {
    //     Serial.printf("RC - Roll: %d, Pitch: %d, Yaw: %d, Throttle: %d, Arm: %d\n",
    //                  roll, pitch, yaw, throttle, arm);
    //     lastDebug = millis();
    //   }
    // }
   
    // Попробуем получить сырые данные акселерометра если доступно
    msp_raw_imu_t raw_imu;
    if (msp.request(MSP_RAW_IMU, &raw_imu, sizeof(raw_imu))) {
      currentIMU.accel_x = raw_imu.acc[0] / 512.0; // Пример масштабирования
      currentIMU.accel_y = raw_imu.acc[1] / 512.0;
      currentIMU.accel_z = raw_imu.acc[2] / 512.0;
      currentIMU.gyro_x = raw_imu.gyro[0] / 4.1; // Пример масштабирования
      currentIMU.gyro_y = raw_imu.gyro[1] / 4.1;
      currentIMU.gyro_z = raw_imu.gyro[2] / 4.1;
      // Serial.printf("ACC: %.2f, %.2f, %.2f\n", currentIMU.accel_x, currentIMU.accel_y, currentIMU.accel_z);
      // Serial.printf("GYR: %.2f, %.2f, %.2f\n", currentIMU.gyro_x, currentIMU.gyro_y, currentIMU.gyro_z);
    }
    msp_sonar_altitude_t raw_sonar;
    if (msp.request(MSP_SONAR_ALTITUDE, &raw_sonar, sizeof(raw_sonar))) {
      currentIMU.sonar = raw_sonar.altitude; // Пример масштабирования
    }
    lastIMURead = millis();
  }
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
 
  // Настройки для лучшего баланса качества/производительности
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 12;
  config.fb_count = 2;
  config.grab_mode = CAMERA_GRAB_LATEST;
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return false;
  }
  return true;
}

void createWiFiAP() {
  Serial.println("Creating Access Point...");
 
  WiFi.softAP(ap_ssid, ap_password);
 
  Serial.print("Access Point: ");
  Serial.println(ap_ssid);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
}

void handleClient() {
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
  // Ждем данные от клиента
  unsigned long timeout = millis() + 5000;
  while(!client.available() && millis() < timeout){
    delay(1);
  }
  if (!client.available()) {
    client.stop();
    return;
  }
  String req = client.readStringUntil('\r');
  client.flush();
  
  if (req.indexOf("/stream") != -1) {
    if (!streamingActive) {
      startVideoStream(client);
    } else {
      client.println("HTTP/1.1 503 Service Unavailable");
      client.println("Content-Type: text/plain");
      client.println();
      client.println("Stream already active");
      client.stop();
    }
  } else if (req.indexOf("/imu") != -1) {
    sendIMUData(client);
    client.stop();
  } else if (req.indexOf("/command") != -1) {
    handleCommand(client, req);
    client.stop();
  } else if (req.indexOf("/") != -1) {
    sendHTML(client);
    client.stop();
  } else {
    client.println("HTTP/1.1 404 Not Found");
    client.println("Content-Type: text/plain");
    client.println();
    client.println("404 Not Found");
    client.stop();
  }
}

void startVideoStream(WiFiClient &client) {
  Serial.println("Starting video stream");
 
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: keep-alive");
  client.println();
 
  streamClient = client;
  streamingActive = true;
  lastFrame = millis();
}

void handleStreaming() {
  if (!streamingActive || !streamClient.connected()) {
    if (streamingActive) {
      Serial.println("Video stream ended");
      streamingActive = false;
      streamClient.stop();
    }
    return;
  }
 
  unsigned long now = millis();
  if (now - lastFrame < FRAME_INTERVAL) {
    return;
  }
  lastFrame = now;
 
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
 
  streamClient.println("--frame");
  streamClient.println("Content-Type: image/jpeg");
  streamClient.println("Content-Length: " + String(fb->len));
  streamClient.println();
 
  size_t written = streamClient.write(fb->buf, fb->len);
  streamClient.println();
 
  esp_camera_fb_return(fb);
 
  if (written != fb->len) {
    Serial.println("Stream write failed");
    // streamingActive = false;
    // streamClient.stop();
  }
}

void sendIMUData(WiFiClient &client) {
  Serial.println("Sending IMU data");
 
  DynamicJsonDocument doc(512);
  doc["accel_x"] = currentIMU.accel_x;
  doc["accel_y"] = currentIMU.accel_y;
  doc["accel_z"] = currentIMU.accel_z;
  // doc["gyro_x"] = currentIMU.gyro_x;
  // doc["gyro_y"] = currentIMU.gyro_y;
  // doc["gyro_z"] = currentIMU.gyro_z;
  doc["sonar"] = currentIMU.sonar;
  doc["timestamp"] = currentIMU.timestamp;
  String response;
  serializeJson(doc, response);
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: close");
  client.println();
  client.println(response);
}

void handleCommand(WiFiClient &client, String req) {
  String type = "";
  int typeIndex = req.indexOf("type=");
  if (typeIndex != -1) {
    type = req.substring(typeIndex + 5, req.indexOf(" ", typeIndex));
  }

  bool success = false;
  if (type == "blink") {
    // Отправка команды на мигание LED на полетнике
    blink_enabled = 1;
    // Serial.println("Blink command sent");
    success = true;
  } else if (type == "motors") {
    // Включение моторов на минимальную мощность
    // Arm on, throttle minimal (например, 1100)
    motor_enabled = 1;
    success = true;
  }

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.println(success ? "Command executed" : "Command failed");
}

void sendHTML(WiFiClient &client) {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Camera Test</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 20px; }
        .container { max-width: 1200px; margin: 0 auto; }
        .video-container { text-align: center; margin: 20px 0; }
        #video-stream {
            max-width: 100%;
            border: 1px solid #ccc;
            background: #f0f0f0;
        }
        .data-panel { background: #f5f5f5; padding: 15px; margin: 10px 0; border-radius: 5px; }
        .status { color: green; font-weight: bold; }
        .error { color: red; }
        .controls { margin: 10px 0; }
        .static-image { max-width: 400px; width: 100%; height: auto; border: 1px solid #ddd; }
        button { padding: 8px 16px; margin: 5px; }
    </style>
</head>
<body>
    <div class="container">
        <div class="logo">
				<img class="static-image" src="data:image/png;base64, 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uWCzgZmoLbjIlWDADcB1YA+6cZV5es3CP2ydGs2PlRtKZ7apF4qIyJihgD025AZ/5uP+IMZvJNjfhvrcSvDzYDqEKTIKDHTurO8td0+16M/HEJbh1oKTYIaamIlccTmgCeeruD9CYvtzxutWzG8EOmtdnIiIfDkUsMcWAxpwrnX4mgfuffKltneDr99YIde56ZWFZ1DHI5ERadGL6x/rKfc8RuRxjNtxnwmWYG6a1iJfIsOASYbf4+7NFnh84YttHcHY1p/vP6jVbBGR0U0Be+z5+Wr2PYbdgPtdbnZNEtOOxS+u252vbzq2qvC4Hv4iI8BjhZUT6yu5a5KYu8XNW3DmYTwE1gDkFKxFaqah+mPXELnBQpyJM72hXL+z5dtt79PAifbCoku1LlJERLKngD22TcSYFYl3WwhPpthP+kv9rwP7al2YiPxy+WJ+hgWejRb/J7BrcCYMrp6JyDBhZlNxb3HnEfA9lvjraaWyCvio1rWJiEj21ElWzLAATHLCLHNvrnVBIvL5mNsM3G7DbTrOOIVrkeHIDQgYjWDXRfdHkjQ3udZViYjIlaEVbPm5esenpYnX17oQEfl8DJrBpoCPRx3CRYa7HMYEos3wxBprXYyIiFwZWsEWERERERERyYACtoiIiIiIiEgGFLBFREREREREMqCALSIiIiIiIpIBBWwRERERERGRDChgi4iIiIiIiGRAAVtEREREREQkAwrYIiIiIiIiIhlQwBYRERERERHJgAK2iIiIiIiISAYUsEVEREREREQyoIAtIiIiIiIikgEFbBEREREREZEMKGCLiIiIiIiIZEABW0RERERERCQDCtgiIiIiIiIiGVDAFhEREREREcmAAraIiIiIiIhIBhSwRURERERERDKggC0iIiIiIiKSAQVsERERERERkQwoYIuIiIiIiIhkQAFbREREREREJAMK2CIiIiIiIiIZUMAWERERERERyYACtoiIiIiIiEgGFLBFREREREREMqCALSIiIiIiIpIBBWwRERERERGRDChgi4iIiIiIiGRAAVtEREREREQkAwrYIiIiIiIiIhlQwBYRERERERHJgAK2iIiIiIiISAYUsEVEREREREQyoIAtIiIiIiIikgEFbBEREREREZEMKGCLiIiIiIiIZEABW0RERERERCQDCtgiIiIiIiIiGVDAFhEREREREcmAAraIiIiIiIhIBhSwRURERERERDKggC0iIiIiIiKSAQVsERERERERkQwoYIuIiIiIiIhkQAFbREREREREJAMK2CIiIiIiIiIZUMAWERERERERyYACtoiIiIiIiEgGFLBFREREREREMqCALSIiIiIiIpIBBWwRERERERGRDChgi4iIiIiIiGRAAVtEREREREQkAwrYIiIiIiIiIhlQwBYRERERERHJgAK2iIiIiIiISAYUsEVEREREREQyoIAtIiIiIiIikgEFbBEREREREZEMKGCLiIiIiIiIZEABW0RERERERCQDCtgiIiIiIiIiGVDAFhEREREREcmAAraIiIiIiIhIBhSwRURERERERDKggC0iIiIiIiKSAQVsERERERERkQwoYIuIiIiIiIhkQAFbREREREREJAMK2CIiIiIiIiIZUMAWERERERERyYACtoiIiIiIiEgGFLBFREREREREMqCALSIiIiIiIpIBBWwRERERERGRDChgi4iIiIiIiGRAAVtEREREREQkAwrYIiIiIiIiIhlQwBYRERERERHJgAK2iIiIiIiISAYUsEVEREREREQyoIAtIiIiIiIikgEFbBEREREREZEMKGCLiIiIiIiIZEABW0RERERERCQDCtgiIiIiIiIiGVDAFhEREREREcmAAraIiIiIiIhIBhSwRURERERERDKggC0iIiIiIiKSAQVsERERERERkQwoYIuIiIiIiIhkQAFbREREREREJAMK2CIiIiIiIiIZUMAWERERERERyYACtoiIiIiIiEgGFLBFREREREREMqCALSIiIiIiIpIBBWwRERERERGRDChgi4iIiIiIiGRAAVtEREREREQkAwrYIiIiIiIiIhlQwBYRERERERHJgAK2iIiIiIiISAYUsEVEREREREQyoIAtIiIiIiIikgEFbBEREREREZEMKGCLiIiIiIiIZEABW0RERERERCQDCtgiIiIiIiIiGVDAFhEREREREcmAAraIiIiIiIhIBhSwRURERERERDKggC0iIiIiIiKSAQVsERERERERkQwoYIuIiIiIiIhkQAFbREREREREJAMK2CIiIiIiIiIZUMAWERERERERyYACtoiIiIiIiEgGFLBFREREREREMqCALSIiIiIiIpIBBWwRERERERGRDChgi4iIiIiIiGRAAVtEREREREQkAwrYIiIiIiIiIhlQ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J/iVrqyecqeLdufhC/z1T+dKtxkOejh/+hh+fPF7Ra2ywwA7XTOoC1Coeaf6IplZtsH2Rspm/6HNAHxT4ev1kUa2J7QCKjj2xXPcnj5mRIe5jTDnp0sgZRsAzgdvVQs9jSjUcdcBt54GI8kM5w6vU+exBzreSSjJXvvWZ24K/T5ufPFR6XZXz510nlOtM6KS+ey+C8aZ3sf639yDd0mni2/2vSl0R4SpQ2H101ONDbDfC7SfDGo4U5Wv2WQZqjsebZqrFds1hBke/ibAPtrsE4SdO8dzXC7UOV4H6ecq9PlVlaa8v2/RnnUKYigAfIzw2WAed5kZgAGqycZbu/AfT3/Ae2+yuttBRNV59l8r1rEa52Cn73OwHI/EGtINPFJyG+GDA57jMmPwHCeza+Dnqcw1xAnuYu23TWZQHUw8TkSUNAbY6dMXJN4wLbaGaoeGd9UYcern8hw3W4O6gScy4edpsRgdhBqDx3I8L4moNRhgt4O+MIWY99omM6jsJ4Pr+K4hboDHc/xwM6jjznOcbPDz9K7biNNBOG8CNfw5j/w6qSnA4JqIWoQBdnvoBbx4kVLGUDcynS71UYEJ1HzFcQWvNX+O5xW8XtONUd2xp/bh50kpoI7BrYpeT3dudP24a7ehvsd430JErcEAu31yqIvVJ+hecDmDysgMwPlusRVQx7iOG6Mc+zeoXTzHx9g/x4s6G0OtkEN9lgZQ359FnY2p0HdQn6G6Mso51HHXnWRdul4VUIH1AKpjo0vvnYg6gGW62ivfe/SgVi69ALUSc7+uBkWSQwV434PZgNgmUMf4AZqRbcjRjXNcH/dn6F6HAlWngMpof4P9UlAXEKaudBMUePtz1JSgboL9KTbZ3kN/l7VJjv1j34TrBxFRNKEC7BkY3Mxr0vHQGa/x3r91fcsMwPvYv3mqs+alRL7332dQN0oF6jvGdb1u1Z7t/TeHfQ3bKi06x4cAPgDP8VTkwu2aeh6a5MLtmvD+Jng7CBpCdVwNAZyBWx3squjvqmfYr2Pf5O+veTnePk9SOu7A/vGeAfhl7+cCDKiJqGO6UJOa7GQLfgb2A/KQnh34d4H9IYrzPxOFks39fDA7x3OcyM78Zyg78LszCJf9Pvg50sHcwZ+7Yv64H/we+wBhR/IcPPbA2x0BqXRgeGMdbKLucamDzSHidFC+4GeitsgX/ExE9uYD27yuRnQQjzsRUUNxkTMiIiIiIiKiABhgExEREREREQXAAJuIiIiIiIgoAAbYRERERERERAEwwCYiIiIiIiIKgAE2ERERERERUQAMsImIiIiIiIgCYIBNREREREREFAADbCIiIiIiIqIAGGATERERERERBcAAm4iIiIiIiCgABthEREREREREARypuwFERERERE33Bm8m7715r+5mEBEREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREVF9/h9h7JYxQI8qQwAAAABJRU5ErkJggg==
        " alt="Logo">
        <h1>ESP32 Camera Test Panel</h1>
       
        <div class="controls">
            <button onclick="startStream()">Start Video Stream</button>
            <button onclick="stopStream()">Stop Video Stream</button>
            <button onclick="location.reload()">Refresh Page</button>
        </div>
       
        <div class="video-container">
            <h3>Video Stream</h3>
            <img id="video-stream" src="" alt="Video Stream">
            <div id="stream-status">Click 'Start Video Stream' to begin</div>
        </div>
       
        <div class="data-panel">
            <h3>IMU Data</h3>
            <div id="imu-data">Loading...</div>
        </div>
       
        <div class="data-panel">
            <h3>Flight Controller Commands</h3>
            <button onclick="sendCommand('blink')">Test ARM</button>
            <button onclick="sendCommand('motors')">Test Motors (Min Power)</button>
            <div id="command-status">Ready</div>
        </div>
    </div>
    <script>
        let streamInterval = null;
        let streamActive = false;
       
        // Функция для запуска видеопотока
        function startStream() {
            if (streamActive) return;
           
            const videoElement = document.getElementById('video-stream');
            const statusElement = document.getElementById('stream-status');
           
            videoElement.src = '/stream?t=' + new Date().getTime();
            streamActive = true;
            statusElement.innerHTML = '<span class="status">Stream started...</span>';
           
            // Периодическое обновление для поддержания соединения
            streamInterval = setInterval(() => {
                if (streamActive) {
                    videoElement.src = videoElement.src.split('?')[0] + '?t=' + new Date().getTime();
                }
            }, 30000); // Обновлять каждые 30 секунд
        }
       
        // Функция для остановки видеопотока
        function stopStream() {
            const videoElement = document.getElementById('video-stream');
            const statusElement = document.getElementById('stream-status');
           
            videoElement.src = '';
            streamActive = false;
            statusElement.innerHTML = 'Stream stopped';
           
            if (streamInterval) {
                clearInterval(streamInterval);
                streamInterval = null;
            }
        }
       
        // Обновление IMU данных
        function updateIMU() {
            fetch('/imu')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('imu-data').innerHTML =
                        `Accel: X=${data.accel_x.toFixed(2)}, Y=${data.accel_y.toFixed(2)}, Z=${data.accel_z.toFixed(2)}<br>
                         Sonar: X=${data.sonar.toFixed(2)}`;
                })
                .catch(err => {
                    console.error('IMU error:', err);
                    document.getElementById('imu-data').innerHTML =
                        '<span class="error">IMU data unavailable</span>';
                });
        }
       
        // Функция отправки команд
        function sendCommand(cmd) {
            const statusElement = document.getElementById('command-status');
            statusElement.innerHTML = '<span class="status">Sending...</span>';
            
            fetch('/command?type=' + cmd)
                .then(response => response.text())
                .then(text => {
                    statusElement.innerHTML = '<span class="status">' + text + '</span>';
                })
                .catch(err => {
                    console.error('Command error:', err);
                    statusElement.innerHTML = '<span class="error">Command failed</span>';
                });
        }
       
        // Обработка ошибок видео потока
        document.getElementById('video-stream').onerror = function() {
            if (streamActive) {
                document.getElementById('stream-status').innerHTML =
                    '<span class="error">Stream error - trying to reconnect...</span>';
                // Автоматическая перезагрузка потока при ошибке
                setTimeout(startStream, 2000);
            }
        };
       
        document.getElementById('video-stream').onload = function() {
            if (streamActive) {
                document.getElementById('stream-status').innerHTML =
                    '<span class="status">Stream active</span>';
            }
        };
       
        // Автоматическое обновление данных
        setInterval(updateIMU, 100);
       
        // Инициализация
        updateIMU();
       
        // Автозапуск потока при загрузке страницы (опционально)
        // setTimeout(startStream, 1000);
    </script>
</body>
</html>
)rawliteral";
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();
  client.println(html);
}