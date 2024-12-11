#include "esp32-hal-timer.h"
#include <Arduino.h>
#include <Stream.h>
#include "MSP.h"
#include "ARA_ESP.h"
#include <driver/uart.h>
#include <math.h>
#include "GeodeticConvert.h"

bool arm_enable = false;

unsigned long previousMillis = 0;


uint8_t led_white[] = { 10, 0xFF, 0xFF, 0x15, 0x06, 0x03, 0x1A, 0x00, 0xFF, 0x00, 0xC8 };
uint8_t led_black[] = { 10, 0xFF, 0xFF, 0x15, 0x06, 0x03, 0x1A, 0x00, 0x00, 0xFF, 0xC8 };

uint16_t led_count = 0;

ARA_ESP esp = ARA_ESP();

void IRAM_ATTR onTimer() {
  // esp.main_f();
  esp.flag_data = true;
}

MSP msp;
void ARA_ESP::begin(Stream& serial) {
  hw_timer_t* My_timer = NULL;
  Serial.begin(115200);
  msp.begin(serial, 10);

  ROLL = 1500;
  PITCH = 1500;
  YAW = 1500;
  THROTTLE = 1000;
  AUX1 = 1000;
  AUX2 = 1000;
  AUX3 = 1000;
  AUX4 = 1000;

  GPS_FIX = MSP_GPS_FIX_3D;
  GPS_numSat = 6;
  GPS_coord_LAT = 557331530;
  GPS_coord_LON = 376971180;
  GPS_altitude = 1000;
  GPS_speed = 0;
  GPS_ground_course = 0;
  
  local_angle = 0;
  local_pos_lat = 0;
  local_pos_lon = 0;
  local_pos_alt = 0;
  
  // My_timer = timerBegin(0, 1000, true);
  // timerAttachInterrupt(My_timer, &onTimer, true);
  // timerAlarmWrite(My_timer, 10000, true);
  // timerAlarmEnable(My_timer);
  My_timer = timerBegin(1000000);
  timerAttachInterrupt(My_timer, &onTimer);
  timerAlarm(My_timer, 1000, true, 0);
}


// void ARA_ESP::main_f() {
//   uint16_t channel[] = { ROLL, PITCH, THROTTLE, YAW, AUX1, AUX2, AUX3, AUX4 };

//   if (led_count++ > 0x11) {
//     msp.command(142, led_white, 11, false);
//   } else {
//     msp.command(142, led_black, 11, false);
//   }

//   if (led_count == 0x22) led_count = 0;

//   msp_set_raw_rc_t set_raw;
//   msp.command(MSP_SET_RAW_RC, channel, sizeof(channel), false);
  
//   uint8_t req[1];
//   msp.request(MSP_GET_ESP_SLEEP, req, 1, NULL);

//   if (req[0] == SLEEP_STATE_SLEEP_SUMMON) {
//     //Serial.println("sleep");
//     req[0] = SLEEP_STATE_IN_SLEEP;
//     bool status = msp.command(MSP_SET_ESP_SLEEP, req, sizeof(req));
//     uart_set_wakeup_threshold(UART_NUM_0, 20);
//     esp_sleep_enable_uart_wakeup(UART_NUM_0);

//     delay(100);
//     digitalWrite(32, HIGH);
//     esp_light_sleep_start();
//     digitalWrite(32, LOW);

//     delay(100);

//     req[0] = SLEEP_STATE_NO_SLEEP_NEEDED;
//     msp.command(MSP_SET_ESP_SLEEP, req, sizeof(req));
//   }
// }

void ARA_ESP::main_f()
{
  uint16_t channel[] = {ROLL, PITCH, THROTTLE, YAW};
  
  msp_set_raw_rc_t set_raw;
  msp.command(MSP_SET_RAW_RC, channel, sizeof(channel), false);

}

void ARA_ESP::gps_f() {

  uint8_t channel[14] = {0};
  channel[0] = GPS_FIX;                       //uint8_t
  channel[1] = GPS_numSat;                    //uint8_t
  // uint32_t new_lat = GPS_coord_LAT + local_pos_lat;
  // uint32_t new_lon = GPS_coord_LON + local_pos_lon;
  // uint16_t new_alt = GPS_altitude + local_pos_alt;
  // Serial.printf("%lu %lu %lu\n", new_lat, new_lon, new_alt);
  // Serial.printf("%lu %lu %lu\n", local_pos_lat, local_pos_lon, local_pos_alt);
  memcpy(&channel[2], &local_pos_lat, 4);     //uint32_t
  memcpy(&channel[6], &local_pos_lon, 4);     //uint32_t
  memcpy(&channel[10], &local_pos_alt, 2);     //uint16_t
  memcpy(&channel[12], &GPS_speed, 2);   //uint16_t

  msp.command(MSP_SET_RAW_GPS, channel, sizeof(channel), false);
}

void ARA_ESP::gps_fix(uint8_t fix) {
  switch (fix) {
    case MSP_GPS_FIX_2D: GPS_FIX = MSP_GPS_FIX_2D; break;
    case MSP_GPS_FIX_3D: GPS_FIX = MSP_GPS_FIX_3D; break;
    case MSP_GPS_NO_FIX: GPS_FIX = MSP_GPS_NO_FIX; break;
  }
  gps_f();
}

void ARA_ESP::gps_num_sat(uint8_t numSat) {
  GPS_numSat = numSat;
  gps_f();
}

void ARA_ESP::gps_longitude(float lon) {
  GPS_coord_LON = lon*10000000;
  gps_f();
}

void ARA_ESP::gps_latitude(float lat) {
  GPS_coord_LAT = lat*10000000;
  gps_f();
}

void ARA_ESP::gps_altitude(uint16_t alt) {
  GPS_altitude = alt;
  gps_f();
}

void ARA_ESP::gps_set_lpos_orientation(float angle) {
  local_angle = angle / 360 * 2 * M_PI;
}

void ARA_ESP::gps_local_position(float pos_x, float pos_y, int16_t pos_z) {
  GeodeticConvert converter;
  double ref_lat = (double) GPS_coord_LAT/10000000;
  double ref_lon = (double) GPS_coord_LON/10000000;
  float ref_alt = GPS_altitude;
  double ref_x = 0.0;
  double ref_y = 0.0;
  double ref_z = 0.0;
  double latitude = 0.0;   // Широта
  double longitude = 0.0;  // Долгота
  double altitude = 0.0;   // Высота над уровнем моря

  converter.geodetic2Ecef(ref_lat, ref_lon, ref_alt, ref_x, ref_y, ref_z);
  float k_cos = cos(local_angle);
  float k_sin = sin(local_angle);

  ref_x += pos_x * k_cos - pos_y * k_sin;
  ref_y += pos_y * k_cos + pos_x * k_sin;
  ref_z += pos_z;

  converter.ecef2Geodetic(ref_x, ref_y, ref_z, latitude, longitude, altitude);

  local_pos_lat = (double) latitude *10000000;
  local_pos_lon = (double) longitude*10000000;
  local_pos_alt = (uint16_t) GPS_altitude + pos_z;

  gps_f();
}

uint16_t ARA_ESP::get_channel(int channel)
{  
  uint8_t rb[32] = {0};
  uint16_t data[16];
  
  msp.request(MSP_RC, rb, 32, NULL);

  for (int i = 0; i < 16; i++) {
    data[i] = (uint16_t)rb[i * 2] | ((uint16_t)rb[i * 2 + 1] << 8);
  }

// for (uint8_t i = 1; i < 32; i+=2)
//   {
//     Serial.printf("%d | ", ((uint16_t)rb[i] << 8) | ((uint16_t)rb[i-1]));
//   }
//   Serial.println();

  return data[channel-1];
}

void ARA_ESP::roll(float roll) {
  float old_max_roll = 100;
  float old_min_roll = -100;
  float new_max_roll = 2000;
  float new_min_roll = 1000;

  uint16_t value_roll = (uint16_t)(roll * 100);

  ROLL = (uint16_t)map(value_roll, old_min_roll, old_max_roll, new_min_roll, new_max_roll);
  main_f();
}

void ARA_ESP::pitch(float pitch) {
  float old_max_pitch = 100;
  float old_min_pitch = -100;
  float new_max_pitch = 2000;
  float new_min_pitch = 1000;

  uint16_t value_pitch = (uint16_t)(pitch * 100);

  PITCH = (uint16_t)map(value_pitch, old_min_pitch, old_max_pitch, new_min_pitch, new_max_pitch);
  main_f();
}

void ARA_ESP::yaw(float yaw) {
  float old_max_yaw = 100;
  float old_min_yaw = -100;
  float new_max_yaw = 2000;
  float new_min_yaw = 1000;

  uint16_t value_yaw = (uint16_t)(yaw * 100);

  YAW = (uint16_t)map(value_yaw, old_min_yaw, old_max_yaw, new_min_yaw, new_max_yaw);
  main_f();
}

void ARA_ESP::throttle(float throttle) {
  float old_max_throttle = 100;
  float old_min_throttle = 0;
  float new_max_throttle = 2000;
  float new_min_throttle = 1000;

  uint16_t value_motor = (uint16_t)(throttle * 100);

  THROTTLE = map(value_motor, old_min_throttle, old_max_throttle, new_min_throttle, new_max_throttle);
  main_f();
}

void ARA_ESP::arm(int arm) {
  switch (arm) {
    case 0: AUX1 = 1000; break;
    case 1: AUX1 = 2000; break;
  }

  main_f();
}

void ARA_ESP::flight_mode(int f_mode) {
  switch (f_mode) {
    case 0: AUX2 = 1000; break;
    case 1: AUX2 = 2000; break;
  }

  main_f();
}

void ARA_ESP::nav_mode(int n_mode) {
  switch (n_mode) {
    case 0: AUX3 = 1000; break;
    case 1: AUX3 = 1500; break;
    case 2: AUX3 = 2000; break;
  }

  main_f();
}
