#pragma once

#include <Arduino.h>
#include <Stream.h>

class ARA_ESP
{
  public:
  void begin(Stream& serial);
  void roll(float roll);
  void pitch(float pitch);
  void yaw(float yaw);
  void throttle(float throttle);
  void arm(int arm);
  void flight_mode(int f_mode);
  void nav_mode(int n_mode);
  uint16_t get_channel(int channel);
  
  void main_f();
  void gps_f();
  void gps_fix(uint8_t fix);
  void gps_num_sat(uint8_t numSat);
  void gps_longitude(float lon);
  void gps_latitude(float lat);
  void gps_altitude(uint16_t alt);
  void gps_local_position(float pos_x, float pos_y, int16_t pos_z);
  void gps_set_lpos_orientation(float angle);                         //в градусах
  
  bool flag_data;
  private:

  uint16_t ROLL;
  uint16_t PITCH;
  uint16_t YAW;
  uint16_t THROTTLE;
  uint16_t AUX1;
  uint16_t AUX2;
  uint16_t AUX3;
  uint16_t AUX4;
  hw_timer_t *My_timer;
  
  uint8_t GPS_FIX;
  uint8_t GPS_numSat;
  uint32_t GPS_coord_LAT;
  uint32_t GPS_coord_LON;
  uint16_t GPS_altitude;
  uint16_t GPS_speed;
  uint16_t GPS_ground_course;


  float local_angle;         //поворот в градусах, чтобы выставить ориентацию осей в пространстве
  int32_t local_pos_lat;
  int32_t local_pos_lon;
  int16_t local_pos_alt;
};

extern ARA_ESP esp;
