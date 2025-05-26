  
  // ██╗    ██╗  ██╗  ██╗ ███████╗ ███████╗ ██╗     
  // ██║    ██║  ██║  ██║ ██╔════╝ ██╔════╝ ██║     
  // ██║ ██╗ ██║ ███████║ ██████╗  ██████╗  ██║     
  // ██║ ╚██╗██║ ██╔══██║ ██╔══╝   ██╔══╝   ██║     
  // ╚██╗╚████║  ██║  ██║ ███████╗ ███████╗ ███████╗
  //  ╚═╝ ╚═══╝  ╚═╝  ╚═╝ ╚══════╝ ╚══════╝ ╚══════╝
  
  
  
  /*
  *   █████████████╗   ███████████╗     ████████████╗
  *   ╚════██╔═════╝  ██╔════════██╗    ██╔═════════╝
  *        ██║        ██║        ██║    ██║
  *        ██║        ██║        ██║    ██║
  *        ██║        ██║        ██║    ██║█████████╗
  *        ██║        ██║        ██║    ██╔═════════╝    
  *        ██║        ██║        ██║    ██║    
  *        ██║        ██║        ██║    ██║ 
  *        ██║        ╚███████████╔╝    ████████████╗
  *        ╚═╝         ╚══════════╝     ╚═══════════╝
  *  Version    Date            Author          Modification
  *  V1.0.0     2024            night8858       
  */

#include "Variables.h"
#include "pid.h"
#include "can_recv.h"
#include "Monitor.h"
#include "chassis.h"
#include "bat_control.h"
#include "pc_interface.h"
#include "tracking.h"

/********************************** global variable *************************************/
s_robo_Mode_Setting robot_StateMode;       // 机器人状态模式
s_robo_Mode_Setting robot_StateMode_last;       // 机器人状态模式

s_task_flags task_flags;                   // 任务标志位
roboError_flag_t VolleyRobot_R1 = {0};     // 机器人错误标志位
chassis_control_t chassis_control;           // 底盘控制结构体
bat_control_t bat_control;                // 球拍控制结构体

su_PC_DATA pcData;       //pc数据
visInf_t s_visionInform; //视觉数据结构体

kalman1_state kalman_hik_x;
kalman1_state kalman_hik_y;    
kalman1_state kalman_hik_z;  
               
/********************************** PID variable ****************************************/
s_pid_absolute_t volleyball_track_X_PID_pos;          //排球远处定位跟踪PID结构体，位置环，击球杆处相机排球位置为参考数据
s_pid_absolute_t volleyball_track_keepY_PID;        //排球远处定位跟踪PID结构体，速度环，车辆y速度为参考数据

s_pid_absolute_t volleyball_track_Y_PID_pos;        //排球近处定位跟踪PID结构体，位置环，球拍下相机排球位置为参考数据
s_pid_absolute_t volleyball_track_close_PID_speed;      //排球近处定位跟踪PID结构体，速度环，车辆方向速度为参考数据

s_pid_absolute_t serveing_tracking_pid;          //发球定位跟踪PID结构体

/********************************** DATA variable ****************************************/
 s_Dji_motor_data_t motor_Date[6];    // RM电机回传数据结构体
s_motor_data_t DM4340_Date[3];         // DM4340回传数据结构体
s_motor_data_t DM8006_Date[1];         // DM8006回传数据结构体
s_motor_data_t HT04_Data;              // HT04回传数据结构体  null

/********************************** FLAG variable ****************************************/

volatile uint8_t imu_flag = 0;        // IMU数据标志位
volatile uint8_t skriker_flag = 0;        //击球杆控制标志位
volatile uint8_t robot_start_flag = 0;        //机器人启动标志位