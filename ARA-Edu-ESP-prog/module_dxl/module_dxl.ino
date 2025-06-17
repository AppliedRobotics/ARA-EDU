/*Данный кейс выполняет проверку датчика освещенности, если показатель низкий (недостаточно освещенности),
то включается трехцветный светодиод и пьезоизлучатель. Когда освещенности становится достаточно, то 
трехцветный светодиод и пьезоизлучатель выключаются*/

#include "DxlMaster2.h"

//Регистры трехцветного светодиода
#define GREEN_LED_DATA (26)
#define RED_LED_DATA (27)
#define BLUE_LED_DATA (28)

//Регистры датчика освещенности
#define LIGHT_DATA (26)

// Регистры звукового пьезоизлучателя
#define MOS_FREQ (26)
#define MOS_POR (28)

// ID трехцветного светодиода
const uint8_t id_rgb = 21;

// ID датчика освещенности
const uint8_t id_light = 25;

// ID звукового пьезоизлучателя
const uint8_t id_buzzer = 24;


///Объявляем Dynamixel устройство 
DynamixelDevice device_rgb(id_rgb);
DynamixelDevice device_light(id_light);
DynamixelDevice device_buzzer(id_buzzer);

//Выставляем скорость обмена данными с Dynamixel устройствами
const unsigned long dynamixel_baudrate = 1000000;
//Выставляем скорость сериал порта
const unsigned long serial_baudrate = 115200;

//Параметры для трехцветного светодиода
uint8_t red_led = 255;
uint8_t green_led = 0;
uint8_t blue_led = 204;

//Параметры для датчика освещенности
uint16_t brightness = 0;

//Параметры для звукового пьезоизлучателя
uint16_t frequency = 500;
uint8_t filling_factor = 127;

void setup() {
 DxlMaster.begin(dynamixel_baudrate);
 Serial.begin(serial_baudrate);
 device_rgb.init();
 device_buzzer.init();
 device_light.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  device_light.read(LIGHT_DATA, brightness);
  delay(500);
  
  Serial.println(brightness);
  
  if(brightness < 200){//если освещенность меньше 200, то начинает мигать светодиод и звуковой пьезоизлучатель издает звук

    device_buzzer.write(MOS_POR, filling_factor);
   
    device_rgb.write(GREEN_LED_DATA, green_led);
    device_rgb.write(RED_LED_DATA, red_led);
    device_rgb.write(BLUE_LED_DATA, blue_led);
    delay(100);

    device_buzzer.write(MOS_FREQ, frequency);

    device_rgb.write(GREEN_LED_DATA, 0);
    device_rgb.write(RED_LED_DATA, 0);
    device_rgb.write(BLUE_LED_DATA, 0);
    delay(100);
  }
  else{//если больше, то всё отключаем
    device_buzzer.write(MOS_POR, 0);
    device_rgb.write(GREEN_LED_DATA, 0);
    device_rgb.write(RED_LED_DATA, 0);
    device_rgb.write(BLUE_LED_DATA, 0);

  }
}