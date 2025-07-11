#include <DxlMaster2.h>

//Регистры трехцветного светодиода
#define GREEN_LED_DATA (26)
#define RED_LED_DATA (27)
#define BLUE_LED_DATA (28)

//Выставляем скорость обмена данными с Dynamixel устройствами
const unsigned long dynamixel_baudrate = 1000000;
//Выставляем скорость сериал порта
const unsigned long serial_baudrate = 115200;

// ID трехцветного светодиода
const uint8_t id_rgb = 21;

///Объявляем Dynamixel устройство 
DynamixelDevice device_rgb(id_rgb);

//Настройки общения с TrackingCam3
uint8_t cam_id = 1;
uint8_t protocol = 2;

//Регистры TrackingCam3
uint16_t id_reg = 16; 
uint16_t x_reg = 17; 
uint16_t y_reg = 19; 

//Параметры ArUco-маркера
uint8_t aruco_id = -1;
uint16_t aruco_x = 0;
uint16_t aruco_y = 0;

uint32_t idToColor(uint32_t id) {
    // Используем простую хеш-функцию для генерации RGB
    uint8_t r = (id * 13) % 256;  // Красный
    uint8_t g = (id * 17+85) % 256;  // Зеленый
    uint8_t b = (id * 23+178) % 256;  // Синий
    
    // Собираем цвет в формате 0xRRGGBB
    return (r << 16) | (g << 8) | b;
}

void setup() {
  // put your setup code here, to run once:
  DxlMaster.begin(dynamixel_baudrate);
  Serial.begin(serial_baudrate);
  device_rgb.init();
} 

void loop() {
  // put your main code here, to run repeatedly:
  DxlMaster.read(protocol, cam_id, id_reg, aruco_id);
  DxlMaster.read(protocol, cam_id, x_reg, aruco_x);
  DxlMaster.read(protocol, cam_id, y_reg, aruco_y);

  uint32_t color = idToColor(aruco_id);

  Serial.println("id: " + String(aruco_id));
  Serial.println("aruco_x: " + String(aruco_x));
  Serial.println("aruco_y: " + String(aruco_y));

 // Разделение на компоненты RGB
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

  device_rgb.write(GREEN_LED_DATA, g);
  device_rgb.write(RED_LED_DATA, r);
  device_rgb.write(BLUE_LED_DATA, b);
  //delay(100);
  delay(250);
}
