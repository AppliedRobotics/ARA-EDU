#include <ARA_ESP.h>
#include <ESP32Servo.h>

const int nasosPin = 32; // Пин подключения насоса и сервопривода
const int channel = 7;     // Канал для управления насосом

Servo servo;
ARA_ESP esp_a;

void setup() {
  delay(2000);
  Serial.begin(115200);
  
  servo.attach(nasosPin);
  
  Serial2.begin(115200, SERIAL_8N1, 2, 4);
  esp_a.begin(Serial2);
}

void loop() {

  uint16_t aux = esp_a.get_channel(channel);

  if (esp_a.flag_data) {
    esp_a.main_f();
    esp_a.flag_data = false;
  }
  

  if (aux < 1500) {
    servo.write(50);  
    Serial.println("Насос выключен");
  }
  if (aux >= 1500) {
    servo.write(130); 
    Serial.println("Насос включен");
  }
  
  delay(50); 
}