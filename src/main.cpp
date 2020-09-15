#include <platform_driver/command.h>
#include <ros/ros.h>
#include <serial/serial.h>

#include <sensor_msgs/Imu.h>
#include <sys/sysinfo.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <time.h>
#include <ctime>

#include "util.h"

class toCtr
{
public:
  toCtr();
  int init_serial();
  void timerCb(const ros::TimerEvent &e);                  //定时器回调函数5hz
  void command_hub(platform_driver::command command_msg);  //车控数据发送

  //底层通讯
  void power_on();
  void power_off();
  void move_angle(float speed_roll, float angle_roll, float speed_pitch, float angle_pitch, float speed_yaw,
                  float angle_yaw, int mode);
  void set_pid(uint8_t id, uint8_t pid_value);

  //一级封装
  void move_angle_yaw(float speed_yaw, float angle_yaw, int mode);

  //二级封装
  void scan();
  void track();

  command_buffer_type command_buffer;

  scan_info scan_infos;
  platform_info platform_infos;
  track_info track_infos;

  serial::Serial ros_serial;
  ros::Subscriber pertocon_sub;
};

toCtr::toCtr()
{
  platform_infos.init();
  scan_infos.init();
  track_infos.init();
  ros::NodeHandle nh("~");
  pertocon_sub = nh.subscribe<platform_driver::command>("/write", 10, &toCtr::command_hub, this);
}

int toCtr::init_serial()
{
  std::string port = "/dev/ttyUSB0";
  int baud = 115200;
  try
  {
    //设置串口属性，并打开串口
    ros_serial.setPort(port);                //端口
    ros_serial.setBaudrate((uint32_t)baud);  //波特率
    serial::Timeout to = serial::Timeout::simpleTimeout(10);
    ros_serial.setTimeout(to);
    ros_serial.open();
  }
  catch (serial::IOException &e)
  {
    ROS_ERROR_STREAM("Unable to open port ");
    return -1;
  }
  //检测串口是否已经打开，并给出提示信息
  if (ros_serial.isOpen())
  {
    ROS_INFO_STREAM("Serial Port initialized");
    return 1;
  }
  else
  {
    return -1;
  }
}

void toCtr::power_on()
{
  command_buffer.power_buffer[0] = 0x24;
  command_buffer.power_buffer[1] = 0x4d;
  command_buffer.power_buffer[2] = 0x00;
  command_buffer.power_buffer[3] = 0x4d;
  command_buffer.power_buffer[4] = 0x44;
  command_buffer.power_buffer[5] = 0x0a;
}

void toCtr::power_off()
{
  command_buffer.power_buffer[0] = 0x24;
  command_buffer.power_buffer[1] = 0x6d;
  command_buffer.power_buffer[2] = 0x00;
  command_buffer.power_buffer[3] = 0x6d;
  command_buffer.power_buffer[4] = 0x0c;
  command_buffer.power_buffer[5] = 0x0a;
}

void toCtr::move_angle(float speed_roll, float angle_roll, float speed_pitch, float angle_pitch, float speed_yaw,
                       float angle_yaw, int mode = 3)
{
  command_buffer.angle_speed_buffer[0] = 0x24;  // header
  command_buffer.angle_speed_buffer[1] = 0x43;  //
  command_buffer.angle_speed_buffer[2] = 0x0d;
  command_buffer.angle_speed_buffer[3] = command_buffer.angle_speed_buffer[1] + command_buffer.angle_speed_buffer[2];
  if (mode == 3)
    command_buffer.angle_speed_buffer[4] = 0x03;
  else if (mode == 1)
    command_buffer.angle_speed_buffer[4] = 0x01;

  int16_t speed, angle;
  speed = int16_t(speed_roll / 0.1220740379);
  angle = int16_t(angle_roll / 0.02197265625);
  command_buffer.angle_speed_buffer[5] = speed & 0xff;
  command_buffer.angle_speed_buffer[6] = (speed >> 8) & 0xff;
  command_buffer.angle_speed_buffer[7] = angle & 0xff;
  command_buffer.angle_speed_buffer[8] = (angle >> 8) & 0xff;

  speed = int16_t(speed_pitch / 0.1220740379);
  angle = int16_t(angle_pitch / 0.02197265625);
  command_buffer.angle_speed_buffer[9] = speed & 0xff;
  command_buffer.angle_speed_buffer[10] = (speed >> 8) & 0xff;
  command_buffer.angle_speed_buffer[11] = angle & 0xff;
  command_buffer.angle_speed_buffer[12] = (angle >> 8) & 0xff;

  speed = int16_t(speed_yaw / 0.1220740379);
  angle = int16_t(angle_yaw / 0.02197265625);
  command_buffer.angle_speed_buffer[13] = speed & 0xff;
  command_buffer.angle_speed_buffer[14] = (speed >> 8) & 0xff;
  command_buffer.angle_speed_buffer[15] = angle & 0xff;
  command_buffer.angle_speed_buffer[16] = (angle >> 8) & 0xff;

  uint8_t crc[2];
  crc16_calculate(16, command_buffer.angle_speed_buffer + 1, crc);
  command_buffer.angle_speed_buffer[17] = crc[0];
  command_buffer.angle_speed_buffer[18] = crc[1];

  platform_infos.roll = angle_roll;
  platform_infos.pitch = angle_pitch;
  platform_infos.yaw = angle_yaw;
}

void toCtr::move_angle_yaw(float speed_yaw, float angle_yaw, int mode = 3)
{
  move_angle(0, 0, 0, 0, speed_yaw, angle_yaw, mode);
}

void toCtr::set_pid(uint8_t id, uint8_t pid_value)
{
  command_buffer.PID_buffer[0] = 0x24;

  command_buffer.PID_buffer[1] = id;
  command_buffer.PID_buffer[2] = 0x01;
  command_buffer.PID_buffer[3] = command_buffer.PID_buffer[1] + command_buffer.PID_buffer[2];

  // body
  command_buffer.PID_buffer[4] = pid_value;

  uint8_t crc[2];
  crc16_calculate(4, command_buffer.PID_buffer + 1, crc);
  command_buffer.PID_buffer[5] = crc[0];
  command_buffer.PID_buffer[6] = crc[1];
}

//主状态机
void toCtr::command_hub(platform_driver::command command_msg)
{
  platform_infos.command = command_msg.command;

  if (platform_infos.status == FREE || platform_infos.status == TRACKING)
  {
    if (command_msg.mode == SCANNING)
    {
      scan_infos.init();
      scan_infos.range = command_msg.scan_range;
      scan_infos.cycle_time = command_msg.scan_cycle_time;
    }
    platform_infos.status = command_msg.mode;
  }
  else if (platform_infos.status == SCANNING || platform_infos.status == TRACKING)
  {
    if (command_msg.mode == FREE)
    {
      move_angle(5.0, 0.0, 5.0, 0.0, 5.0, 0.0);  // MOVE TO ZERO POSITION
      platform_infos.status = TRANSIENT;
      platform_infos.transient_tick = 0;
    }
  }
  else
  {
    platform_infos.status = command_msg.mode;
  }

  if (platform_infos.status == FREE)
  {
    if (platform_infos.command == command_msg.POWER_ON)
    {
      power_on();
    }
    else if (platform_infos.command == command_msg.POWER_OFF)
    {
      power_off();
    }
    else if (platform_infos.command == command_msg.ANGLE_CONTROL)
    {
      move_angle(command_msg.roll_speed, command_msg.roll_angle, command_msg.pitch_speed, command_msg.pitch_angle,
                 command_msg.yaw_speed, command_msg.yaw_angle, 3);
    }
    else if (platform_infos.command == command_msg.SPEED_CONTROL)
    {
      move_angle(command_msg.roll_speed, command_msg.roll_angle, command_msg.pitch_speed, command_msg.pitch_angle,
                 command_msg.yaw_speed, command_msg.yaw_angle, 1);
    }
    else
    {
      uint8_t id;
      if (platform_infos.command == command_msg.SET_ROLL_P)
        id = 0x00;
      else if (platform_infos.command == command_msg.SET_PITCH_P)
        id = 0x01;
      else if (platform_infos.command == command_msg.SET_YAW_P)
        id = 0x02;
      else if (platform_infos.command == command_msg.SET_ROLL_I)
        id = 0x03;
      else if (platform_infos.command == command_msg.SET_PITCH_I)
        id = 0x04;
      else if (platform_infos.command == command_msg.SET_YAW_I)
        id = 0x05;
      else if (platform_infos.command == command_msg.SET_ROLL_D)
        id = 0x06;
      else if (platform_infos.command == command_msg.SET_PITCH_D)
        id = 0x07;
      else if (platform_infos.command == command_msg.SET_YAW_D)
        id = 0x08;
      set_pid(id, command_msg.set_pid_value);
    }
  }
  else if (platform_infos.status == TRACKING)
  {
    track_infos.error = command_msg.error;
    track();
  }
}

void toCtr::scan()
{
  if (scan_infos.is_initialized == false)
  {
    if (scan_infos.working_tick == 0)
    {
      scan_infos.init_yaw_error = scan_infos.range - platform_infos.yaw;
      scan_infos.init_speed = scan_infos.init_yaw_error / (0.01 * scan_infos.init_tick);
    }

    if (scan_infos.working_tick < scan_infos.init_tick)
    {
      move_angle_yaw(scan_infos.init_speed, scan_infos.init_yaw_error);
      scan_infos.working_tick++;
    }
    else
    {
      scan_infos.is_initialized = true;
      scan_infos.working_tick = 0;
      scan_infos.scan_tick = scan_infos.cycle_time / 0.01;
    }
  }

  if (scan_infos.is_initialized == true)
  {
    float angle = scan_infos.range * cos(2 * PI * scan_infos.working_tick / scan_infos.scan_tick);
    float speed = -scan_infos.range * 2 * PI / scan_infos.cycle_time *
                  sin(2 * PI * scan_infos.working_tick / scan_infos.scan_tick);
    move_angle_yaw(speed, angle);
    scan_infos.working_tick++;
  }
}

void toCtr::track()
{
  move_angle_yaw(track_infos.error * track_infos.p, 0, 1);
}

void toCtr::timerCb(const ros::TimerEvent &e)
{
  platform_driver::command command_msg;
  if (platform_infos.status == FREE)
  {
    if (platform_infos.command == command_msg.POWER_ON || platform_infos.command == command_msg.POWER_OFF)
      ros_serial.write(command_buffer.power_buffer, sizeof(command_buffer.power_buffer));
    else if (platform_infos.command == command_msg.ANGLE_CONTROL)
      ros_serial.write(command_buffer.angle_speed_buffer, sizeof(command_buffer.angle_speed_buffer));
    else
      ros_serial.write(command_buffer.PID_buffer, sizeof(command_buffer.PID_buffer));
  }
  else if (platform_infos.status == SCANNING)
  {
    scan();
    ros_serial.write(command_buffer.angle_speed_buffer, sizeof(command_buffer.angle_speed_buffer));
  }
  else if (platform_infos.status == TRACKING)
  {
    ros_serial.write(command_buffer.angle_speed_buffer, sizeof(command_buffer.angle_speed_buffer));
  }
  else if (platform_infos.status == TRANSIENT)
  {
    if (platform_infos.transient_tick < 1000)
    {
      platform_infos.transient_tick++;
    }
    else
    {
      platform_infos.transient_tick = 0;
      platform_infos.status = FREE;
    }
    ros_serial.write(command_buffer.angle_speed_buffer, sizeof(command_buffer.angle_speed_buffer));
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "toCtr");
  toCtr m_ctrobj;
  m_ctrobj.init_serial();
  ros::NodeHandle nh_p("~");
  ros::Timer timer = nh_p.createTimer(ros::Duration(0.01), &toCtr::timerCb, &m_ctrobj);
  ros::spin();
}
