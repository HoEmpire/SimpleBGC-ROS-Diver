#include <ros/ros.h>
#include <serial/serial.h>  //ROS已经内置了的串口包
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
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
  uint8_t info[27];
  uint8_t info2[16];
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
  else if (msg->data == "info1")  // 58 64 00 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  {
    on_off[0] = 0x24;
    on_off[1] = 0x3d;
    on_off[2] = 0x00;
    on_off[3] = 0x3d;
    uint8_t crc[2];
    crc16_calculate(3, on_off + 1, crc);
    on_off[4] = crc[0];
    on_off[5] = crc[1];
    ser.write(on_off, sizeof(on_off));  //发送串口数据
  }
  else if (msg->data == "info2")  // 58 64 00 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  {
    info[0] = 0x24;
    info[1] = 0x55;
    info[2] = 0x15;
    info[3] = 0x6a;

    info[4] = 0x58;
    info[5] = 0x09;
    info[6] = 0x00;

    info[7] = 0x08;
    info[8] = 0x00;
    info[9] = 0x00;
    info[10] = 0x00;

    info[11] = 0x00;
    info[12] = 0x00;
    info[13] = 0x00;
    info[14] = 0x00;

    info[15] = 0x00;
    info[16] = 0x00;
    info[17] = 0x00;
    info[18] = 0x00;
    info[19] = 0x00;
    info[20] = 0x00;
    info[21] = 0x00;
    info[22] = 0x00;
    info[23] = 0x00;
    info[24] = 0x00;

    uint8_t crc[2];
    crc16_calculate(24, info + 1, crc);
    info[25] = crc[0];
    info[26] = crc[1];
    ser.write(info, sizeof(info));  //发送串口数据
  }
  else if (msg->data == "info3")  // CMD_REALTIME_DATA_CUSTOM
  {
    info2[0] = 0x24;
    info2[1] = 0x58;
    info2[2] = 0x0a;
    info2[3] = 0x62;

    info2[4] = 0x08;
    info2[5] = 0x00;
    info2[6] = 0x00;
    info2[7] = 0x00;

    info2[8] = 0x00;
    info2[9] = 0x00;
    info2[10] = 0x00;
    info2[11] = 0x00;
    info2[12] = 0x00;
    info2[13] = 0x00;

    uint8_t crc[2];
    crc16_calculate(13, info2 + 1, crc);
    info2[14] = crc[0];
    info2[15] = crc[1];
    ser.write(info2, sizeof(info2));  //发送串口数据
  }
  else if (msg->data == "info4")  // 58 64 00 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  {
    info[0] = 0x24;
    info[1] = 0x55;
    info[2] = 0x15;
    info[3] = 0x6a;

    info[4] = 0x58;
    info[5] = 0x00;
    info[6] = 0x00;

    info[7] = 0x08;
    info[8] = 0x00;
    info[9] = 0x00;
    info[10] = 0x00;

    info[11] = 0x00;
    info[12] = 0x00;
    info[13] = 0x00;
    info[14] = 0x00;

    info[15] = 0x00;
    info[16] = 0x00;
    info[17] = 0x00;
    info[18] = 0x00;
    info[19] = 0x00;
    info[20] = 0x00;
    info[21] = 0x00;
    info[22] = 0x00;
    info[23] = 0x00;
    info[24] = 0x00;

    uint8_t crc[2];
    crc16_calculate(24, info + 1, crc);
    info[25] = crc[0];
    info[26] = crc[1];
    ser.write(info, sizeof(info));  //发送串口数据
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
  ros::Publisher read_pub = nh.advertise<std_msgs::UInt8MultiArray>("read", 1000);

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
      std_msgs::UInt8MultiArray result;

      ser.flush();
      ser.read(result.data, ser.available());
      read_pub.publish(result);

      float roll, pitch, yaw;
      roll = int16_t(result.data[6] + (result.data[7] << 8)) * 0.02197265625;
      pitch = int16_t(result.data[8] + (result.data[9] << 8)) * 0.02197265625;
      yaw = int16_t(result.data[10] + (result.data[11] << 8)) * 0.02197265625;
      ROS_INFO_STREAM("roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw
                               << ", size = " << sizeof(result.data));
    }

    //处理ROS的信息，比如订阅消息,并调用回调函数
    ros::spinOnce();
    loop_rate.sleep();
  }
}