#include "Variables.h"
#include "pid.h"
#include "can_recv.h"

/********************************** global variable ****************************************/
/********************************** Motor variable ****************************************/
s_pid_absolute_t M3508_pid_speed;

s_pid_absolute_t chassis_pid_angle;

s_motor_data_t DM4340_Date[3]; // DM4340回传数据结构体
s_motor_data_t DM8006_Date[1]; // DM4340回传数据结构体
s_motor_data_t HT04_Data;      // HT04回传数据结构体

