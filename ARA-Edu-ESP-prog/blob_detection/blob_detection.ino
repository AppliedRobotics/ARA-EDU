#include <DxlMaster2.h>

//Регистры трехцветного светодиода
#define GREEN_LED_DATA (26)
#define RED_LED_DATA (27)
#define BLUE_LED_DATA (28)

// ID трехцветного светодиода
const uint8_t id_rgb = 21;

///Объявляем Dynamixel устройство 
DynamixelDevice device_rgb(id_rgb);

// Настройки подключения
const unsigned long dynamixel_baudrate = 1000000;
const unsigned long serial_baudrate = 115200;

// ID камеры и протокол
const uint8_t cam_id = 1; // ID как в blob.ino
const uint8_t protocol = 2; // Protocol 2.0

const uint8_t n = 1;

// Структура для хранения данных Blob
struct Blob {
    uint8_t type;     // адрес 16 (1 byte)
    uint8_t padding;  // адрес 17 (1 byte)
    uint16_t cx;      // адрес 18 (2 bytes)
    uint16_t cy;      // адрес 20 (2 bytes)
    uint32_t area;    // адрес 22 (4 bytes)
    uint16_t left;    // адрес 26 (2 bytes)
    uint16_t right;   // адрес 28 (2 bytes)
    uint16_t top;     // адрес 30 (2 bytes)
    uint16_t bottom;  // адрес 32 (2 bytes)
};

Blob blobs[10];

void setup() {
  DxlMaster.begin(dynamixel_baudrate);
  Serial.begin(serial_baudrate);
  device_rgb.init();
}

void readBlobs(uint8_t count) {
    for(uint8_t i = 0; i < count; i++) {
        DxlMaster.read(protocol, cam_id, 16 + i*18, blobs[i].type);
        DxlMaster.read(protocol, cam_id, 17 + i*18, blobs[i].padding);
        DxlMaster.read(protocol, cam_id, 18 + i*18, blobs[i].cx);
        DxlMaster.read(protocol, cam_id, 20 + i*18, blobs[i].cy);
        DxlMaster.read(protocol, cam_id, 22 + i*18, blobs[i].area);
        DxlMaster.read(protocol, cam_id, 26 + i*18, blobs[i].left);
        DxlMaster.read(protocol, cam_id, 28 + i*18, blobs[i].right);
        DxlMaster.read(protocol, cam_id, 30 + i*18, blobs[i].top);
        DxlMaster.read(protocol, cam_id, 32 + i*18, blobs[i].bottom);
    }
}

void loop() {
  readBlobs(n);
  
  for(uint8_t i = 0; i < n; i++){
    Serial.print(i+1);
    Serial.print(" ");
    Serial.print("type: " + String(blobs[i].type));
    Serial.print(" ");
    Serial.print("cx: " + String(blobs[i].cx));
    Serial.print(" ");
    Serial.print("cy: " + String(blobs[i].cy));
    Serial.print(" ");
    Serial.print("area: " + String(blobs[i].area));
    Serial.print(" ");
    Serial.print("left: " + String(blobs[i].left));
    Serial.print(" ");
    Serial.print("right: " + String(blobs[i].right));
    Serial.print(" ");
    Serial.print("top: " + String(blobs[i].top));
    Serial.print(" ");
    Serial.print("bottom: " + String(blobs[i].bottom));
    Serial.println();

    if (blobs[i].area < 103000)
    {
      device_rgb.write(GREEN_LED_DATA, 255);
      device_rgb.write(RED_LED_DATA, 0);
      device_rgb.write(BLUE_LED_DATA, 0);
      delay(100);
    }
    else
    {
      device_rgb.write(GREEN_LED_DATA, 0);
      device_rgb.write(RED_LED_DATA, 255);
      device_rgb.write(BLUE_LED_DATA, 0);
      delay(100);
    }


  }

  
  
  delay(500); // Задержка между опросами
}