#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ARA_ESP.h>

int pin = 32;
int channel = 7;
void setup(){
  delay(2000);
  Serial.begin(115200);
  pinMode(pin, OUTPUT);
  Serial2.begin(115200, SERIAL_8N1, 2, 4);
  esp.begin(Serial2);
}
void loop(){
  uint16_t aux = esp.get_channel(channel);
  if (esp.flag_data){
    esp.main_f();
    esp.flag_data = false;
  }
  if (aux < 1500){
    digitalWrite(pin,LOW);
    Serial.println("отпуск");
  }
  if (aux >= 1500){
    digitalWrite(pin, HIGH);
    Serial.println("захват");
  }
}
