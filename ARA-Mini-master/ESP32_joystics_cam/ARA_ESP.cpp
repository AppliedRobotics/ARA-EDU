#include <Arduino.h>
#include <Stream.h>
#include "MSP.h"
#include "ARA_ESP.h"

bool arm_enable = false;

unsigned long previousMillis = 0;

ARA_ESP esp = ARA_ESP();

void IRAM_ATTR onTimer() {
  // esp.main_f();
  esp.flag_data = true;
}

MSP msp;
void ARA_ESP::begin(Stream& serial)
{
  hw_timer_t *My_timer = NULL;
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

  My_timer = timerBegin(0, 1000, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 10000, true);
  timerAlarmEnable(My_timer);
}


void ARA_ESP::main_f()
{
  uint16_t channel[] = {ROLL, PITCH, THROTTLE, YAW, AUX1, AUX2, AUX3, AUX4};
  
  msp_set_raw_rc_t set_raw;
  msp.command(MSP_SET_RAW_RC, channel, sizeof(channel), false);

}

uint16_t ARA_ESP::get_channel(int channel)
{  
  uint8_t rb[32] = {0};
  uint16_t data[16];
  
  msp.request(MSP_RC, rb, 32, NULL);

  for (int i = 0; i < 16; i++) {
  data[i] = (uint16_t)rb[i * 2] | ((uint16_t)rb[i * 2 + 1] << 8);
}
  return data[channel-1];
}

void ARA_ESP::roll(float roll)
{
  float old_max_roll = 100;
  float old_min_roll = -100;
  float new_max_roll = 1800;
  float new_min_roll = 1200;

  uint16_t value_roll = (uint16_t)(roll * 100);

  ROLL = (uint16_t)map(value_roll, old_min_roll, old_max_roll, new_min_roll, new_max_roll);
  main_f();
}

void ARA_ESP::pitch(float pitch)
{
  float old_max_pitch = 100;
  float old_min_pitch = -100;
  float new_max_pitch = 1800;
  float new_min_pitch = 1200;

  uint16_t value_pitch = (uint16_t)(pitch * 100);

  PITCH = (uint16_t)map(value_pitch, old_min_pitch, old_max_pitch, new_min_pitch, new_max_pitch);
  main_f();
}


void ARA_ESP::yaw(float yaw)
{
  float old_max_yaw = 100;
  float old_min_yaw = -100;
  float new_max_yaw = 1800;
  float new_min_yaw = 1200;

  uint16_t value_yaw = (uint16_t)(yaw * 100);

  YAW = (uint16_t)map(value_yaw, old_min_yaw, old_max_yaw, new_min_yaw, new_max_yaw);
  main_f();
}

void ARA_ESP::throttle(float throttle)
{
  float old_max_throttle = 100;
  float old_min_throttle = 0;
  float new_max_throttle = 1800;
  float new_min_throttle = 1000;

  uint16_t value_motor = (uint16_t)(throttle * 100);

  THROTTLE = map(value_motor, old_min_throttle, old_max_throttle, new_min_throttle, new_max_throttle);
  main_f();
}

void ARA_ESP::arm(int arm)
{
  switch (arm) {
    case 0: AUX1 = 1000; break;
    case 1: AUX1 = 2000; break;
  }

  main_f();
}

void ARA_ESP::flight_mode(int f_mode)
{
  switch (f_mode) {
    case 0: AUX2 = 1000; break;
    case 1: AUX2 = 2000; break;
  }
  
  main_f();
}

void ARA_ESP::nav_mode(int n_mode)
{
  switch (n_mode) {
    case 0: AUX3 = 1000; break;
    case 1: AUX3 = 2000; break;
  }
  
  main_f();
}
