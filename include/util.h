#pragma once
#include <ros/ros.h>
#include <chrono>
#include <thread>

#define FREE 0
#define SCANNING 1
#define TRACKING 2
#define TRANSIENT 3
#define PI 3.1415926535

struct ConfigSetting
{
  bool debug_output_scan;
  bool debug_output_encoder;
  float cycle_time_second;
  float reset_speed;
  float transient_time;
  float scan_init_time;
  float track_p;
  float init_time;
} config;

void loadConfig(ros::NodeHandle n)
{
  n.getParam("/basic/debug_output_scan", config.debug_output_scan);
  n.getParam("/basic/debug_output_encoder", config.debug_output_encoder);
  n.getParam("/basic/init_time", config.init_time);
  n.getParam("/basic/cycle_time_second", config.cycle_time_second);
  n.getParam("/basic/reset_speed", config.reset_speed);
  n.getParam("/basic/transient_time", config.transient_time);
  n.getParam("/scan/init_time", config.scan_init_time);
  n.getParam("/track/track_p", config.track_p);
}

void crc16_update(uint16_t length, uint8_t *data, uint8_t crc[2])
{
  uint16_t counter;
  uint16_t polynom = 0x8005;
  uint16_t crc_register = (uint16_t)crc[0] | ((uint16_t)crc[1] << 8);
  uint8_t shift_register;
  uint8_t data_bit, crc_bit;
  for (counter = 0; counter < length; counter++)
  {
    for (shift_register = 0x01; shift_register > 0x00; shift_register <<= 1)
    {
      data_bit = (data[counter] & shift_register) ? 1 : 0;
      crc_bit = crc_register >> 15;
      crc_register <<= 1;
      if (data_bit != crc_bit)
        crc_register ^= polynom;
    }
  }
  crc[0] = crc_register;
  crc[1] = (crc_register >> 8);
}

void crc16_calculate(uint16_t length, uint8_t *data, uint8_t crc[2])
{
  crc[0] = 0;
  crc[1] = 0;
  crc16_update(length, data, crc);
}

struct command_buffer_type
{
  uint8_t power_buffer[6];
  uint8_t PID_buffer[7];
  uint8_t angle_speed_buffer[19];
};

struct platform_info
{
  uint8_t status;
  uint8_t command;
  int transient_tick;
  float roll, pitch, yaw;
  float encoder_yaw_init;
  float encoder_roll, encoder_pitch, encoder_yaw;
  void init()
  {
    roll = 0;
    pitch = 0;
    yaw = 0;
    status = FREE;
  }
};

struct scan_info
{
  bool is_initialized;
  int working_tick;
  int init_tick;
  int scan_tick;
  float init_speed;
  float init_yaw_error;
  float range;
  float cycle_time;
  float scan_center_offset;
  float scan_start, scan_end;

  void init()
  {
    working_tick = 0;
    init_tick = config.scan_init_time / config.cycle_time_second;
    is_initialized = false;
  }
};

struct track_info
{
  float p;
  float error;
  void init()
  {
    p = config.track_p;
    error = 0;
  }
};

void delay_ms(const int msecs)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(msecs));
}
