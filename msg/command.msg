uint8 command #0:on; 1:off; 2:angle; 11-13:set roll pid; 21-23 set pitch pid; 31-33 set yaw pid
float32 roll_angle # 0.02197265625 degree 
float32 pitch_angle
float32 yaw_angle
float32 roll_speed # 0.1220740379 deg/sec
float32 pitch_speed
float32 yaw_speed
uint8 roll_p #(0-255)
uint8 roll_i
uint8 roll_d 
uint8 pitch_p #(0-255)
uint8 pitch_i
uint8 pitch_d 
uint8 yaw_p #(0-255)
uint8 yaw_i
uint8 yaw_d

byte POWER_ON = 0
byte POWER_OFF = 1
byte ANGLE_CONTROL = 2
byte SET_ROLL_P = 11
byte SET_ROLL_I = 12
byte SET_ROLL_D = 13
byte SET_PITCH_P = 21
byte SET_PITCH_I = 22
byte SET_PITCH_D = 23
byte SET_YAW_P = 31
byte SET_YAW_I = 32
byte SET_YAW_D = 33
