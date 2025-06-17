#include "DxlMaster2.h"

//Регистры трехцветного светодиода
#define GREEN_LED_DATA (26)
#define RED_LED_DATA (27)
#define BLUE_LED_DATA (28)

//Регистры датчика дальномера
#define MEAS_CONTROL (25)
#define RESULT_0 (26)
#define RESULT_1 (27)


// Регистры звукового пьезоизлучателя
#define MOS_FREQ (26)
#define MOS_POR (28)

// ID трехцветного светодиода
const uint8_t id_rgb = 21;

// ID дальномера
const uint8_t id_rangefinder = 42;//34;

// ID звукового пьезоизлучателя
const uint8_t id_buzzer = 24;



///Объявляем Dynamixel устройство 
DynamixelDevice device_rgb(id_rgb);
DynamixelDevice device_rangefinder(id_rangefinder);
DynamixelDevice device_buzzer(id_buzzer);

//Выставляем скорость обмена данными с Dynamixel устройствами
const unsigned long dynamixel_baudrate = 1000000;
//Выставляем скорость сериал порта
const unsigned long serial_baudrate = 115200;

//Параметры для трехцветного светодиода
uint8_t red_led = 255;
uint8_t green_led = 255;
uint8_t blue_led = 0;

//Параметры для дальномера
uint8_t distance = 0;
uint8_t mas[2];

//Параметры для звукового пьезоизлучателя
uint16_t frequency = 500;
uint8_t filling_factor = 127;

void setup() {
 DxlMaster.begin(dynamixel_baudrate);
 Serial.begin(serial_baudrate);
 device_rgb.init();
 device_buzzer.init();
 device_rangefinder.init();
 delay(200);
}

void loop() {
  device_rangefinder.write(MEAS_CONTROL, 1);  
  delay(500);
  device_rangefinder.read(RESULT_0, mas[0]);
  device_rangefinder.read(RESULT_1, mas[1]);
  distance = mas[0] * 256 + mas[1];
  
  Serial.println(" ");
  Serial.print("Distance = ");
  Serial.println(distance);
  
  if(distance < 50){//если дистанция меньше 50, то начинает мигать светодиод и звуковой пьезоизлучатель издает звук

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