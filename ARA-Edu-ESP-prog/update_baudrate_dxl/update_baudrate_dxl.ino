#include "DxlMaster2.h"

DynamixelDevice device_rgb(21);
DynamixelDevice device_buzzer(24);
DynamixelDevice device_light(25);
DynamixelDevice device_rangefinder(42);

uint32_t bauds[4] = {500000, 115200, 57600, 9600};


void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.print("Startig DXL sensor's setup for baudrate 1'000'000...");

  for (uint8_t i = 0; i < 4; i++) {
    DxlMaster.begin(bauds[i]);
    delay(200);

    device_rgb.init();
    device_buzzer.init();
    device_light.init();
    device_rangefinder.init();
    delay(100);

    if (device_rgb.ping() == DYN_STATUS_OK) {
      Serial.println("");
      Serial.print("Sensor 'RGB' on baudrate: ");
      Serial.println(bauds[i]);
      delay(100);
      device_rgb.write(4, 1);
      delay(100);
      Serial.println("");
      Serial.println("Changed to 1'000'000");
    }
    delay(100);

    if (device_buzzer.ping() == DYN_STATUS_OK) {
      Serial.println("");
      Serial.print("Sensor 'Buzzer' on baudrate: ");
      Serial.println(bauds[i]);
      delay(100);
      device_buzzer.write(4, 1);
      delay(100);
      Serial.println("");
      Serial.println("Changed to 1'000'000");
    }
    delay(100);

    if (device_light.ping() == DYN_STATUS_OK) {
      Serial.println("");
      Serial.print("Sensor 'Light' on baudrate: ");
      Serial.println(bauds[i]);
      delay(100);
      device_light.write(4, 1);
      delay(100);
      Serial.println("");
      Serial.println("Changed to 1'000'000");
    }
    delay(100);

    if (device_rangefinder.ping() == DYN_STATUS_OK) {
      Serial.println("");
      Serial.print("Sensor 'Rangefinder' on baudrate: ");
      Serial.println(bauds[i]);
      delay(100);
      device_rangefinder.write(4, 1);
      delay(100);
      Serial.println("");
      Serial.println("Changed to 1'000'000");
    }
    delay(100);
  }

  Serial.print("Done. Reboot all sensor to apply setup!");

}

void loop() {
  
  

}
