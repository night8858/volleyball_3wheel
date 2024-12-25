#include "Variables.h"
#include "pid.h"
#include "can_recv.h"
#include "Monitor.h"
#include "chassis.h"
#include "bat_control.h"


/********************************** global variable *************************************/
s_robo_Mode_Setting robot_StateMode;
s_task_flags task_flags;

chassis_control_t motor_control;
bat_control_t bat_control;


/********************************** PID variable ****************************************/
s_pid_absolute_t chassis_M3508_pid_speed;
s_pid_absolute_t chassis_M3508_pid_angle;
s_pid_absolute_t chassis_pid_angle;


/********************************** DATA variable ****************************************/
s_Dji_motor_data_t motor_Date[4];      // RM电机回传数据结构体
s_motor_data_t DM4340_Date[3];         // DM4340回传数据结构体
s_motor_data_t DM8006_Date[1];         // DM4340回传数据结构体
s_motor_data_t HT04_Data;              // HT04回传数据结构体

