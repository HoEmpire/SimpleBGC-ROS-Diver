#include <platform_driver/command.h>
#include <ros/ros.h>
#include <serial/serial.h>

#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sys/sysinfo.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <time.h>
#include <ctime>

#include "platform_driver/util.h"

class toCtr
{
public:
  toCtr();
  int init_serial();
  void init_encoder();
  void init_system();

  void timerCb(const ros::TimerEvent &e);                 //定时器回调函数5hz
  void command_hub(platform_driver::command command_msg); //主状态机

  //底层通讯
  void encoder_calibration();
  void power_on();
  void power_off();
  void move_angle(float speed_roll, float angle_roll, float speed_pitch, float angle_pitch, float speed_yaw,
                  float angle_yaw, int mode);  //角度运动控制基本函数
  void set_pid(uint8_t id, uint8_t pid_value); //设置PID
  void read_data_request(std::string mode);    //读取数据，mode="encoder"读取码盘（相对0位的值），“imu”读取imu
  void reset();

  //一级封装
  void move_angle_yaw(float speed_yaw, float angle_yaw, int mode);
  void move_relative_angle_yaw(float speed_yaw, float angle_yaw);
  void move_relative_yaw_in_cycle(float yaw);

  //读取encoder和imu的数据
  bool read_data_encoder();
  bool read_data_imu();
  bool read_data_all();

  //二级封装
  void scan();
  void track();

  command_buffer_type command_buffer;

  scan_info scan_infos;
  platform_info platform_infos;
  track_info track_infos;

  serial::Serial ros_serial;
  ros::Subscriber serial_command_sub;
};

toCtr::toCtr()
{
  ros::NodeHandle nh("~");
  loadConfig(nh);
  platform_infos.init();
  scan_infos.init();
  track_infos.init();
  serial_command_sub = nh.subscribe<platform_driver::command>("/write", 10, &toCtr::command_hub, this);
}

int toCtr::init_serial()
{
  std::string port = "/dev/ttyUSB0";
  int baud = 115200;
  try
  {
    //设置串口属性，并打开串口
    ros_serial.setPort(port);               //端口
    ros_serial.setBaudrate((uint32_t)baud); //波特率
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

void toCtr::init_encoder()
{
  platform_infos.encoder_yaw_init = 0.0; //use encoder  offset as initial position
  ROS_INFO_STREAM("Set initial encoder yaw to be: " << platform_infos.encoder_yaw_init << " degree.");
}

//云台初始化
//顺序：开启串口，reset平台，读取imu值，读取encoder值
void toCtr::init_system()
{
  ROS_INFO("Initialize serial port...");
  init_serial();
  delay_ms(100);
  ROS_INFO("Reset platform...");
  reset();
  delay_ms(config.init_time * 1000);
  // encoder_calibration();
  // delay_ms(100);
  ros_serial.flushInput();
  while (!read_data_all())
  {
  }
  ROS_INFO_STREAM("Initial IMU angles:");
  ROS_INFO_STREAM("roll: " << platform_infos.roll << ", pitch: " << platform_infos.pitch
                           << ", yaw: " << platform_infos.yaw);
  // delay_ms(100);
  // ros_serial.flushInput();
  // while (!read_data_encoder())
  // {
  // }
  ROS_INFO_STREAM("Initial Encoder angles:");
  ROS_INFO_STREAM("roll: " << platform_infos.encoder_roll << ", pitch: " << platform_infos.encoder_pitch
                           << ", yaw: " << platform_infos.encoder_yaw);
  init_encoder();
  platform_infos.status = FREE;
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

void toCtr::reset()
{
  uint8_t reset_buffer[6];
  reset_buffer[0] = 0x24;
  reset_buffer[1] = 0x72;
  reset_buffer[2] = 0x00;
  reset_buffer[3] = 0x72;

  uint8_t crc[2];
  crc16_calculate(3, reset_buffer + 1, crc);
  reset_buffer[4] = crc[0];
  reset_buffer[5] = crc[1];
  ros_serial.write(reset_buffer, sizeof(reset_buffer));
}

void toCtr::move_angle(float speed_roll, float angle_roll, float speed_pitch, float angle_pitch, float speed_yaw,
                       float angle_yaw, int mode = 2)
{
  command_buffer.angle_speed_buffer[0] = 0x24; // header
  command_buffer.angle_speed_buffer[1] = 0x43; //
  command_buffer.angle_speed_buffer[2] = 0x0d;
  command_buffer.angle_speed_buffer[3] = command_buffer.angle_speed_buffer[1] + command_buffer.angle_speed_buffer[2];
  if (mode == 3)
    command_buffer.angle_speed_buffer[4] = 0x03;
  else if (mode == 1)
    command_buffer.angle_speed_buffer[4] = 0x01;
  else if (mode == 2)
    command_buffer.angle_speed_buffer[4] = 0x02;

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

  // platform_infos.roll = angle_roll;
  // platform_infos.pitch = angle_pitch;
  // platform_infos.yaw = angle_yaw;
}

void toCtr::move_angle_yaw(float speed_yaw, float angle_yaw, int mode = 2)
{
  move_angle(0, 0, 0, 0, speed_yaw, angle_yaw, mode);
}

void toCtr::move_relative_yaw_in_cycle(float yaw)
{
  float current_yaw = platform_infos.encoder_yaw - platform_infos.encoder_yaw_init;
  // ROS_INFO_STREAM("current yaw: " << current_yaw << ", info yaw: " << platform_infos.yaw);
  move_relative_angle_yaw((yaw - current_yaw) / config.cycle_time_second, yaw);
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
  //状态机切换
  if (platform_infos.status == FREE || platform_infos.status == TRANSIENT)
  {
    if (command_msg.mode == SCANNING)
    {
      ROS_INFO("Status Free/Tracking to Status Scanning!");
      scan_infos.init();
      scan_infos.range = command_msg.scan_range;
      scan_infos.cycle_time = command_msg.scan_cycle_time;
    }
    platform_infos.status = command_msg.mode;
  }
  else if (platform_infos.status == SCANNING)
  {
    if (command_msg.mode == SCANNING)
    {
      ROS_INFO("Status Free/Tracking to Status Scanning!");
      scan_infos.init();
      scan_infos.range = command_msg.scan_range;
      scan_infos.cycle_time = command_msg.scan_cycle_time;
      platform_infos.status = command_msg.mode;
    }
    else if (command_msg.mode == FREE)
    {
      move_relative_angle_yaw(config.reset_speed, 0);
      platform_infos.status = TRANSIENT;
      platform_infos.transient_tick = 0;
    }
  }
  else if (platform_infos.status == TRACKING)
  {
    if (command_msg.mode == SCANNING)
    {
      ROS_INFO("Status Free/Tracking to Status Scanning!");
      scan_infos.init();
      scan_infos.range = command_msg.scan_range;
      scan_infos.cycle_time = command_msg.scan_cycle_time;
      platform_infos.status = command_msg.mode;
    }
    else if (command_msg.mode == FREE)
    {
      move_relative_angle_yaw(5.0, 0);
      platform_infos.status = TRANSIENT;
      platform_infos.transient_tick = 0;
    }
  }

  //=========================================================

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
                 command_msg.yaw_speed, command_msg.yaw_angle, 2);
    }
    else if (platform_infos.command == command_msg.SPEED_CONTROL)
    {
      move_angle(command_msg.roll_speed, command_msg.roll_angle, command_msg.pitch_speed, command_msg.pitch_angle,
                 command_msg.yaw_speed, command_msg.yaw_angle, 1);
    }
    else if (platform_infos.command == command_msg.RELATIVE_YAW_CONTROL)
    {
      ROS_INFO("FUCK!!!");
      move_relative_angle_yaw(command_msg.yaw_speed, command_msg.yaw_angle);
    }
    else if (platform_infos.command == command_msg.ENCODER_CALIBRATI0N)
    {
      encoder_calibration();
    }
    else
    {
      uint8_t id = 0x00;
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
    scan_infos.scan_tick = scan_infos.cycle_time / config.cycle_time_second; //initialize total tick
    float current_yaw = platform_infos.encoder_yaw - platform_infos.encoder_yaw_init;
    if (abs(current_yaw) > scan_infos.range)
    {
      if (current_yaw > 0)
      {
        move_relative_angle_yaw(scan_infos.range, 10); //TODO hardcode in here
      }
      else
      {
        move_relative_angle_yaw(-scan_infos.range, 10); //TODO hardcode in here
      }
      return;
    }
    else
    {
      if (current_yaw > 0)
      {
        scan_infos.working_tick = int(abs(current_yaw) / scan_infos.range * scan_infos.scan_tick / 4);
      }
      else
      {
        scan_infos.working_tick = int(abs(current_yaw) / scan_infos.range * scan_infos.scan_tick / 4) + scan_infos.scan_tick / 2;
      }
      ROS_INFO_STREAM("Initializing working ticks, set to: " << scan_infos.working_tick << ", total ticks: " << scan_infos.scan_tick);
      scan_infos.is_initialized = true;
    }
  }
  float angle = 0.0;
  if (scan_infos.is_initialized == true)
  {
    //Phase 1
    if (scan_infos.working_tick >= 0 && scan_infos.working_tick < scan_infos.scan_tick / 4)
    {
      angle = 1.0 * scan_infos.working_tick / (scan_infos.scan_tick / 4) * scan_infos.range;
    }
    //Phase 2
    else if (scan_infos.working_tick >= scan_infos.scan_tick / 4 && scan_infos.working_tick < scan_infos.scan_tick / 2)
    {
      angle = 1.0 * (scan_infos.scan_tick / 2 - scan_infos.working_tick) / (scan_infos.scan_tick / 4) * scan_infos.range;
    }
    //Phase 3
    else if (scan_infos.working_tick >= scan_infos.scan_tick / 2 && scan_infos.working_tick < scan_infos.scan_tick * 3 / 4)
    {
      angle = -1.0 * (scan_infos.working_tick - scan_infos.scan_tick / 2) / (scan_infos.scan_tick / 4) * scan_infos.range;
    }
    //Phase 4
    else if (scan_infos.working_tick >= scan_infos.scan_tick * 3 / 4 && scan_infos.working_tick < scan_infos.scan_tick)
    {
      angle = -1.0 * (scan_infos.scan_tick - scan_infos.working_tick) / (scan_infos.scan_tick / 4) * scan_infos.range;
    }
    move_relative_yaw_in_cycle(angle);
    scan_infos.working_tick++;
    scan_infos.working_tick = scan_infos.working_tick % scan_infos.scan_tick;
    if (config.debug_output_scan)
      ROS_INFO_STREAM("angle: " << angle);
  }
  /*
  if (scan_infos.is_initialized == false)
  {
    if (scan_infos.working_tick == 0)
    {
      scan_infos.scan_center_offset =
          platform_infos.yaw - (platform_infos.encoder_yaw - platform_infos.encoder_yaw_init);
      scan_infos.init_yaw_error = scan_infos.range + scan_infos.scan_center_offset;
      scan_infos.init_speed = scan_infos.init_yaw_error / (config.cycle_time_second * scan_infos.init_tick);
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
      scan_infos.scan_tick = scan_infos.cycle_time / config.cycle_time_second;
    }
  }

  if (scan_infos.is_initialized == true)
  {
    if (scan_infos.working_tick == 0)
    {
      scan_infos.scan_start = scan_infos.range + scan_infos.scan_center_offset;
      scan_infos.scan_end = -scan_infos.range + scan_infos.scan_center_offset;
    }
    else if (scan_infos.working_tick == scan_infos.scan_tick / 2)
    {
      scan_infos.scan_start = -scan_infos.range + scan_infos.scan_center_offset;
      scan_infos.scan_center_offset =
          platform_infos.yaw - (platform_infos.encoder_yaw - platform_infos.encoder_yaw_init);
      scan_infos.scan_end = scan_infos.range + scan_infos.scan_center_offset;
    }
    else if (scan_infos.working_tick == scan_infos.scan_tick)
    {
      scan_infos.scan_start = scan_infos.range + scan_infos.scan_center_offset;
      scan_infos.scan_center_offset =
          platform_infos.yaw - (platform_infos.encoder_yaw - platform_infos.encoder_yaw_init);
      scan_infos.scan_end = -scan_infos.range + scan_infos.scan_center_offset;
      scan_infos.working_tick = 0;
    }

    float speed = (scan_infos.scan_end - scan_infos.scan_start) / (scan_infos.scan_tick / 2 * config.cycle_time_second);
    float angle = scan_infos.scan_start +
                  speed * (scan_infos.working_tick % (scan_infos.scan_tick / 2)) * config.cycle_time_second;
    if (config.debug_output_scan)
      ROS_INFO_STREAM("angle: " << angle << ", speed: " << speed);
    move_angle_yaw(speed, angle);
    scan_infos.working_tick++;
  }*/
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
    else if (platform_infos.command == command_msg.ANGLE_CONTROL || platform_infos.command == command_msg.RELATIVE_YAW_CONTROL || platform_infos.command == command_msg.SPEED_CONTROL)
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
    if (platform_infos.transient_tick < (config.transient_time / config.cycle_time_second))
    {
      platform_infos.transient_tick++;
      if (platform_infos.transient_tick < 10) // STOP the current motion
        move_angle_yaw(0, platform_infos.yaw);
    }
    else
    {
      platform_infos.transient_tick = 0;
      platform_infos.status = FREE;
    }
    ros_serial.write(command_buffer.angle_speed_buffer, sizeof(command_buffer.angle_speed_buffer));
  }

  if (platform_infos.status != INIT)
  {
    // read_data_encoder();
    read_data_all();
  }
}

void toCtr::read_data_request(std::string mode)
{
  uint8_t read_request_buffer[16];
  read_request_buffer[0] = 0x24;
  read_request_buffer[1] = 0x58;
  read_request_buffer[2] = 0x0a;
  read_request_buffer[3] = 0x62;

  if (mode == "encoder")
    read_request_buffer[4] = 0x08;
  else if (mode == "imu")
    read_request_buffer[4] = 0x01;
  else if (mode == "all")
    read_request_buffer[4] = 0x09;
  else
    read_request_buffer[4] = 0x00;

  read_request_buffer[5] = 0x00;
  read_request_buffer[6] = 0x00;
  read_request_buffer[7] = 0x00;

  read_request_buffer[8] = 0x00;
  read_request_buffer[9] = 0x00;
  read_request_buffer[10] = 0x00;
  read_request_buffer[11] = 0x00;
  read_request_buffer[12] = 0x00;
  read_request_buffer[13] = 0x00;

  uint8_t crc[2];
  crc16_calculate(13, read_request_buffer + 1, crc);
  read_request_buffer[14] = crc[0];
  read_request_buffer[15] = crc[1];
  ros_serial.write(read_request_buffer, sizeof(read_request_buffer)); //发送串口数据
}

void toCtr::move_relative_angle_yaw(float speed_yaw, float angle_yaw)
{
  // ROS_INFO_STREAM("move to global:  " << angle_yaw << ", move to local:  " << angle_yaw - (platform_infos.encoder_yaw - platform_infos.encoder_yaw_init) + platform_infos.yaw);
  move_angle_yaw(speed_yaw, angle_yaw - (platform_infos.encoder_yaw - platform_infos.encoder_yaw_init) + platform_infos.yaw);
}

bool toCtr::read_data_encoder()
{
  std_msgs::UInt8MultiArray result;
  uint8_t buffer[11], crc[2];
  buffer[0] = 0x58;
  buffer[1] = 0x08;
  buffer[2] = 0x60;

  ros_serial.flush();
  read_data_request("encoder");
  delay_ms(1.0);

  // read
  while (ros_serial.available() != 0)
  {
    // header check
    if (ros_serial.read(result.data, 1) && result.data[0] == 0x24)
    {
      result.data.clear();
      // ROS_INFO("data1: %x", result.data[0]);
      if (ros_serial.read(result.data, 1) && result.data[0] == 0x58)

      {
        result.data.clear();
        // Check sum test
        if (ros_serial.read(result.data, 2) && (0x58 + result.data[0] == result.data[1]))
        {
          // ROS_INFO("data3: %x", result.data[0]);
          // ROS_INFO("data4: %x", result.data[1]);
          result.data.clear();
          if (ros_serial.read(result.data, 10) == 10)
          {
            for (int i = 3; i < 11; i++)
              buffer[i] = result.data[i - 3];
            //crc check
            crc16_calculate(11, buffer, crc);
            if (crc[0] == result.data[8] && crc[1] == result.data[9])
            {
              platform_infos.encoder_roll = int16_t(result.data[2] + (result.data[3] << 8)) * 0.02197265625;
              platform_infos.encoder_pitch = int16_t(result.data[4] + (result.data[5] << 8)) * 0.02197265625;
              platform_infos.encoder_yaw = int16_t(result.data[6] + (result.data[7] << 8)) * 0.02197265625;
              if (config.debug_output_encoder)
                ROS_INFO_STREAM("roll: " << platform_infos.encoder_roll << ", pitch: " << platform_infos.encoder_pitch
                                         << ", yaw: " << platform_infos.encoder_yaw);
              return true;
            }
            else
            {
              ROS_INFO("Crc check failed in encoder reading...");
            }
          }
        }
        else
          ROS_INFO("Check sum failed in encoder reading...");
      }
    }
    result.data.clear();
  }
  return false;
}

bool toCtr::read_data_imu()
{
  std_msgs::UInt8MultiArray result;
  uint8_t buffer[11], crc[2];
  buffer[0] = 0x58;
  buffer[1] = 0x08;
  buffer[2] = 0x60;

  ros_serial.flush();
  read_data_request("imu");
  delay_ms(1.0);
  // read
  while (ros_serial.available() != 0)
  {
    // header check
    if (ros_serial.read(result.data, 1) && result.data[0] == 0x24)
    {
      result.data.clear();
      // ROS_INFO("data1: %x", result.data[0]);
      if (ros_serial.read(result.data, 1) && result.data[0] == 0x58)

      {
        result.data.clear();
        // Check sum test
        if (ros_serial.read(result.data, 2) && (0x58 + result.data[0] == result.data[1]))
        {
          // ROS_INFO("data3: %x", result.data[0]);
          // ROS_INFO("data4: %x", result.data[1]);
          result.data.clear();
          if (ros_serial.read(result.data, 10) == 10)
          {
            for (int i = 3; i < 11; i++)
              buffer[i] = result.data[i - 3];
            //crc check
            crc16_calculate(11, buffer, crc);
            if (crc[0] == result.data[8] && crc[1] == result.data[9])
            {
              platform_infos.roll = int16_t(result.data[2] + (result.data[3] << 8)) * 0.02197265625;
              platform_infos.pitch = int16_t(result.data[4] + (result.data[5] << 8)) * 0.02197265625;
              platform_infos.yaw = int16_t(result.data[6] + (result.data[7] << 8)) * 0.02197265625;
              // ROS_INFO("data3: %x", result.data[6]);
              // ROS_INFO("data4: %x", result.data[7]);
              return true;
            }
            else
            {
              ROS_INFO("Crc check failed in imu reading...");
            }
          }
        }
        else
          ROS_INFO("Check sum failed in imu reading...");
      }
    }
    result.data.clear();
  }
  return false;
}

bool toCtr::read_data_all()
{
  std_msgs::UInt8MultiArray result;
  uint8_t buffer[17], crc[2];
  buffer[0] = 0x58;

  ros_serial.flush();
  read_data_request("all");
  delay_ms(1.0);
  // read
  while (ros_serial.available() != 0)
  {
    // header check
    if (ros_serial.read(result.data, 1) && result.data[0] == 0x24)
    {
      result.data.clear();
      // ROS_INFO("data1: %x", result.data[0]);
      if (ros_serial.read(result.data, 1) && result.data[0] == 0x58)
      {
        result.data.clear();
        // Check sum test
        if (ros_serial.read(result.data, 2) && (0x58 + result.data[0] == result.data[1]))
        {
          buffer[1] = result.data[0];
          buffer[2] = result.data[1];
          // ROS_INFO("data3: %x", result.data[0]);
          // ROS_INFO("data4: %x", result.data[1]);
          result.data.clear();
          if (ros_serial.read(result.data, 16) == 16)
          {
            for (int i = 3; i < 17; i++)
              buffer[i] = result.data[i - 3];
            //crc check
            crc16_calculate(17, buffer, crc);
            if (crc[0] == result.data[14] && crc[1] == result.data[15])
            {
              platform_infos.roll = int16_t(result.data[2] + (result.data[3] << 8)) * 0.02197265625;
              platform_infos.pitch = int16_t(result.data[4] + (result.data[5] << 8)) * 0.02197265625;
              platform_infos.yaw = int16_t(result.data[6] + (result.data[7] << 8)) * 0.02197265625;
              platform_infos.encoder_roll = int16_t(result.data[8] + (result.data[9] << 8)) * 0.02197265625;
              platform_infos.encoder_pitch = int16_t(result.data[10] + (result.data[11] << 8)) * 0.02197265625;
              platform_infos.encoder_yaw = int16_t(result.data[12] + (result.data[13] << 8)) * 0.02197265625;
              return true;
            }
            else
            {
              ROS_INFO("Crc check failed in all data reading...");
            }
          }
        }
        else
          ROS_INFO("Check sum failed in all data reading...");
      }
    }
    result.data.clear();
  }
  return false;
}

void toCtr::encoder_calibration()
{
  uint8_t encoder_cal_buffer[16];
  encoder_cal_buffer[0] = 0x24;
  encoder_cal_buffer[1] = 0x1a;
  encoder_cal_buffer[2] = 0x00;
  encoder_cal_buffer[3] = 0x1a;

  uint8_t crc[2];
  crc16_calculate(3, encoder_cal_buffer + 1, crc);
  encoder_cal_buffer[4] = crc[0];
  encoder_cal_buffer[5] = crc[1];
  ros_serial.write(encoder_cal_buffer, sizeof(encoder_cal_buffer));
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "toCtr");
  toCtr m_ctrobj;
  m_ctrobj.init_system();
  ros::NodeHandle nh_p("~");
  ros::Timer timer = nh_p.createTimer(ros::Duration(config.cycle_time_second), &toCtr::timerCb, &m_ctrobj);
  ros::Publisher platform_yaw_pub = nh_p.advertise<std_msgs::Float32>("platform_yaw", 1);
  ros::Rate r(100); // 100Hz
  while (ros::ok())
  {
    std_msgs::Float32 current_yaw;
    current_yaw.data = -(m_ctrobj.platform_infos.encoder_yaw -
                         m_ctrobj.platform_infos.encoder_yaw_init); // reverse the sign for convenience
    platform_yaw_pub.publish(current_yaw);
    ros::spinOnce();
    r.sleep();
  }
}
