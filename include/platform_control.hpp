#include <platform_driver/command.h>
#include <platform_driver/platform_control.h>
#include "util.h"
class platformController
{
public:
  platformController();
  platform_info platform_infos;
  platform_driver::command platform_commands;

  void power_on();
  void power_off();
  void move_angle(float speed_yaw, float angle_yaw);
  void scan(float range, float cycle_time);
  void track(float error, float p, float i, float d);
};

platformController::platformController()
{
  platform_infos.init();
}

void platformController::power_on()
{
  platform_commands.command = platform_commands.POWER_ON;
}

void platformController::power_off()
{
  platform_commands.command = platform_commands.POWER_OFF;
}

void platformController::move_angle(float yaw_speed, float yaw_angle)
{
  platform_commands.command = platform_commands.ANGLE_CONTROL;
  platform_commands.roll_speed = 0;
  platform_commands.pitch_speed = 0;
  platform_commands.yaw_speed = yaw_speed;
  platform_commands.roll_angle = 0;
  platform_commands.pitch_angle = 0;
  platform_commands.yaw_angle = yaw_angle;
}

void platformController::scan(float range, float cycle_time)
{
  platform_commands.command = platform_commands.ANGLE_CONTROL;
  platform_commands.roll_speed = 0;
  platform_commands.pitch_speed = 0;
  platform_commands.yaw_speed = yaw_speed;
  platform_commands.roll_angle = 0;
  platform_commands.pitch_angle = 0;
  platform_commands.yaw_angle = yaw_angle;
}

void platformController::track(float error, float p = 1, float i = 0.1, float d = 1)
{
  platform_commands.command = platform_commands.ANGLE_CONTROL;
  platform_commands.roll_speed = 0;
  platform_commands.pitch_speed = 0;
  platform_commands.yaw_speed = error * p + error * i + error * d;
  platform_commands.roll_angle = 0;
  platform_commands.pitch_angle = 0;
  platform_commands.yaw_angle = yaw_angle;
}
