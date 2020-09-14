#include <platform_driver/command.h>
#include <ros/ros.h>
#include <serial/serial.h>

#include <sensor_msgs/Imu.h>
#include <sys/sysinfo.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <time.h>
#include <ctime>

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
  uint8_t command;
  uint8_t power_buffer[6];
  uint8_t PID_buffer[7];
  uint8_t angle_speed_buffer[19];
};

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
  else if (command_buffer.command == command_msg.ANGLE_CONTROL)
  {
    command_buffer.angle_speed_buffer[0] = 0x24;  // header
    command_buffer.angle_speed_buffer[1] = 0x43;  //
    command_buffer.angle_speed_buffer[2] = 0x0d;
    command_buffer.angle_speed_buffer[3] = command_buffer.angle_speed_buffer[1] + command_buffer.angle_speed_buffer[2];
    command_buffer.angle_speed_buffer[4] = 0x03;

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
}

void toCtr::timerCb(const ros::TimerEvent &e)
{
  platform_driver::command command_msg;
  if (command_buffer.command == command_msg.POWER_ON || command_buffer.command == command_msg.POWER_OFF)
    ros_serial.write(command_buffer.power_buffer, sizeof(command_buffer.power_buffer));
  else if (command_buffer.command == command_msg.ANGLE_CONTROL)
    ros_serial.write(command_buffer.angle_speed_buffer, sizeof(command_buffer.angle_speed_buffer));
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
