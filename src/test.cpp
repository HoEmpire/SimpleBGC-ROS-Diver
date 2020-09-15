#include <ros/ros.h>
#include <serial/serial.h>  //ROS已经内置了的串口包
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <iostream>

serial::Serial ser;  //声明串口对象
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

//回调函数
void write_callback(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO_STREAM("Writing to serial port" << msg->data);
  uint8_t on_off[6];
  uint8_t set_pid[7];
  if (msg->data == "on")
  {
    on_off[0] = 0x24;
    on_off[1] = 0x4d;
    on_off[2] = 0x00;
    on_off[3] = 0x4d;
    on_off[4] = 0x44;
    on_off[5] = 0x0a;
    ser.write(on_off, sizeof(on_off));  //发送串口数据
  }
  else if (msg->data == "off")
  {
    on_off[0] = 0x24;
    on_off[1] = 0x6d;
    on_off[2] = 0x00;
    on_off[3] = 0x6d;
    on_off[4] = 0x0c;
    on_off[5] = 0x0a;
    ser.write(on_off, sizeof(on_off));  //发送串口数据
  }
  else if (msg->data == "pid1")
  {
    set_pid[0] = 0x24;
    set_pid[1] = 0x08;
    set_pid[2] = 0x01;
    set_pid[3] = 0x09;
    set_pid[4] = 0x64;
    uint8_t crc[2];
    crc16_calculate(4, set_pid + 1, crc);
    set_pid[5] = crc[0];
    set_pid[6] = crc[1];
    ser.write(set_pid, sizeof(set_pid));  //发送串口数据
  }
  else if (msg->data == "pid2")
  {
    set_pid[0] = 0x24;
    set_pid[1] = 0x08;
    set_pid[2] = 0x01;
    set_pid[3] = 0x09;
    set_pid[4] = 0x01;
    uint8_t crc[2];
    crc16_calculate(4, set_pid + 1, crc);
    set_pid[5] = crc[0];
    set_pid[6] = crc[1];
    ser.write(set_pid, sizeof(set_pid));  //发送串口数据
  }
}

int main(int argc, char **argv)
{
  //初始化节点
  ros::init(argc, argv, "serial_example_node");
  //声明节点句柄
  ros::NodeHandle nh;

  //订阅主题，并配置回调函数
  ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
  //发布主题
  ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

  try
  {
    //设置串口属性，并打开串口
    ser.setPort("/dev/ttyUSB0");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  }
  catch (serial::IOException &e)
  {
    ROS_ERROR_STREAM("Unable to open port ");
    return -1;
  }

  //检测串口是否已经打开，并给出提示信息
  if (ser.isOpen())
  {
    ROS_INFO_STREAM("Serial Port initialized");
  }
  else
  {
    return -1;
  }

  //指定循环的频率
  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    if (ser.available())
    {
      ROS_INFO_STREAM("Reading from serial port\n");
      std_msgs::String result;
      result.data = ser.read(ser.available());
      ROS_INFO_STREAM("Read: " << result.data);
      read_pub.publish(result);
    }

    //处理ROS的信息，比如订阅消息,并调用回调函数
    ros::spinOnce();
    loop_rate.sleep();
  }
}