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
  void timerCb(const ros::TimerEvent &e);           //定时器回调函数5hz
  void send(platform_driver::command command_msg);  //车控数据发送

  command_buffer_type command_buffer;
  serial::Serial ros_serial;
  ros::Subscriber pertocon_sub;
};

toCtr::toCtr()
{
  ros::NodeHandle nh("~");
  pertocon_sub = nh.subscribe<platform_driver::command>("/write", 10, &toCtr::send, this);
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

void toCtr::send(platform_driver::command command_msg)
{
  command_buffer.command = command_msg.command;
  if (command_buffer.command == command_msg.POWER_ON)
  {
    command_buffer.power_buffer[0] = 0x24;
    command_buffer.power_buffer[1] = 0x4d;
    command_buffer.power_buffer[2] = 0x00;
    command_buffer.power_buffer[3] = 0x4d;
    command_buffer.power_buffer[4] = 0x44;
    command_buffer.power_buffer[5] = 0x0a;
  }
  else if (command_buffer.command == command_msg.POWER_OFF)
  {
    command_buffer.power_buffer[0] = 0x24;
    command_buffer.power_buffer[1] = 0x6d;
    command_buffer.power_buffer[2] = 0x00;
    command_buffer.power_buffer[3] = 0x6d;
    command_buffer.power_buffer[4] = 0x0c;
    command_buffer.power_buffer[5] = 0x0a;
  }
  else if (command_buffer.command == command_msg.ANGLE_CONTROL || command_buffer.command == command_msg.SPEED_CONTROL)
  {
    command_buffer.angle_speed_buffer[0] = 0x24;  // header
    command_buffer.angle_speed_buffer[1] = 0x43;  //
    command_buffer.angle_speed_buffer[2] = 0x0d;
    command_buffer.angle_speed_buffer[3] = command_buffer.angle_speed_buffer[1] + command_buffer.angle_speed_buffer[2];
    if (command_buffer.command == command_msg.ANGLE_CONTROL)
      command_buffer.angle_speed_buffer[4] = 0x03;
    else if (command_buffer.command == command_msg.SPEED_CONTROL)
      command_buffer.angle_speed_buffer[4] = 0x01;

    int16_t speed, angle;
    speed = int16_t(command_msg.roll_speed / 0.1220740379);
    angle = int16_t(command_msg.roll_angle / 0.02197265625);
    command_buffer.angle_speed_buffer[5] = speed & 0xff;
    command_buffer.angle_speed_buffer[6] = (speed >> 8) & 0xff;
    command_buffer.angle_speed_buffer[7] = angle & 0xff;
    command_buffer.angle_speed_buffer[8] = (angle >> 8) & 0xff;

    speed = int16_t(command_msg.pitch_angle / 0.1220740379);
    angle = int16_t(command_msg.pitch_angle / 0.02197265625);
    command_buffer.angle_speed_buffer[9] = speed & 0xff;
    command_buffer.angle_speed_buffer[10] = (speed >> 8) & 0xff;
    command_buffer.angle_speed_buffer[11] = angle & 0xff;
    command_buffer.angle_speed_buffer[12] = (angle >> 8) & 0xff;

    speed = int16_t(command_msg.yaw_angle / 0.1220740379);
    angle = int16_t(command_msg.yaw_angle / 0.02197265625);
    command_buffer.angle_speed_buffer[13] = speed & 0xff;
    command_buffer.angle_speed_buffer[14] = (speed >> 8) & 0xff;
    command_buffer.angle_speed_buffer[15] = angle & 0xff;
    command_buffer.angle_speed_buffer[16] = (angle >> 8) & 0xff;

    uint8_t crc[2];
    crc16_calculate(16, command_buffer.angle_speed_buffer + 1, crc);
    command_buffer.angle_speed_buffer[17] = crc[0];
    command_buffer.angle_speed_buffer[18] = crc[1];
  }
  else
  {
    command_buffer.PID_buffer[0] = 0x24;
    uint8_t id;
    if (command_buffer.command == command_msg.SET_ROLL_P)
      id = 0x00;
    else if (command_buffer.command == command_msg.SET_PITCH_P)
      id = 0x01;
    else if (command_buffer.command == command_msg.SET_YAW_P)
      id = 0x02;
    else if (command_buffer.command == command_msg.SET_ROLL_I)
      id = 0x03;
    else if (command_buffer.command == command_msg.SET_PITCH_I)
      id = 0x04;
    else if (command_buffer.command == command_msg.SET_YAW_I)
      id = 0x05;
    else if (command_buffer.command == command_msg.SET_ROLL_D)
      id = 0x06;
    else if (command_buffer.command == command_msg.SET_PITCH_D)
      id = 0x07;
    else if (command_buffer.command == command_msg.SET_YAW_D)
      id = 0x08;

    command_buffer.PID_buffer[1] = id;
    command_buffer.PID_buffer[2] = 0x01;
    command_buffer.PID_buffer[3] = command_buffer.PID_buffer[1] + command_buffer.PID_buffer[2];

    // body
    command_buffer.PID_buffer[4] = command_msg.set_pid_value;

    uint8_t crc[2];
    crc16_calculate(4, command_buffer.PID_buffer + 1, crc);
    command_buffer.PID_buffer[5] = crc[0];
    command_buffer.PID_buffer[6] = crc[1];
  }
}

void toCtr::timerCb(const ros::TimerEvent &e)
{
  platform_driver::command command_msg;
  if (command_buffer.command == command_msg.POWER_ON || command_buffer.command == command_msg.POWER_OFF)
    ros_serial.write(command_buffer.power_buffer, sizeof(command_buffer.power_buffer));
  else if (command_buffer.command == command_msg.ANGLE_CONTROL)
    ros_serial.write(command_buffer.angle_speed_buffer, sizeof(command_buffer.angle_speed_buffer));
  else
    ros_serial.write(command_buffer.PID_buffer, sizeof(command_buffer.PID_buffer));
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
