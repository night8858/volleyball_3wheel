#ifndef MONITOR_H_
#define MONITOR_H_
#include "stm32f4xx.h"

typedef struct
{

  volatile enum roboMode {
    MODE_STOP = 0,      // 停止模式
    MODE_ARTIFICAL = 1, // 人工模式
    MODE_AUTO = 2,      // 自动模式

    ARTIFICAL_CHASSIS = 3, // 人工模式下，机器人底盘
    ARTIFICAL_BAT = 4,     // 人工模式下，机器人球拍
    ARTIFICAL_STRIKER = 5, // 人工模式下，机器人击球杆

    ROBOT_DEBUG = 6,   // 机器人调试模式
    AUTO_RECEIVE_BALL = 7, // 自动模式下，机器人接发球
    AUTO_PASS_BALL = 9,    // 自动模式下，机器人传球

  } roboMode; // 机器控制模式

} s_robo_Mode_Setting;

typedef struct
{
  volatile int8_t DBUS_ERROR;
  volatile int8_t CHASSIS_MOTOR_ERROR;      // 电机错误标志位
  volatile int8_t BAT_MOTOR_ERROR;
  volatile int8_t Pitch_MOTOR_ERROR;
  volatile int8_t Striker_MOTOR_ERROR;
  volatile int8_t PC_ERROR;
  volatile int8_t board_imu_ERROR;


}roboError_flag_t;     // 机器人错误标志位标记结构体


typedef struct
{

  volatile uint8_t Init_complete_flag;
  volatile uint8_t chassis_Init_flag;
  volatile uint8_t bat_control_Init_flag;
  volatile uint8_t striker_Init_flag;
  volatile char sensor_is_blocked;
  volatile uint8_t mode_switched_flag;    //模式切换标志位

  volatile uint8_t skeiker_hit_allow_flag;         //击球杆击球标志位，1为击球杆击球 ， 0为击球杆没有击球

  volatile uint8_t bat_auto_hit_flag;                       //光电硬触发标志位，1为光电硬触发需要击球 ， 0为不需要击球
  volatile uint8_t hit_ball_launch_flag;                    //球拍击球标志位用于判断运动阶段
  volatile uint8_t Underhand_serve_action_flag;             //发球标志位用于下手球判断运动阶段

  volatile uint8_t chassis_get_ball_center_flag;            //机器人底盘获取球中心标志位用于判断运动阶段
  volatile uint8_t bat_running_flag;                        //球拍击球标志位 ， 1为球拍正在运动 ， 0为球拍没有运动
  volatile uint8_t ball_soaring_flag;                       //球被击起标志位 ， 1为球被击起过一次 ，0为球没有被击起过
  volatile uint8_t ball_need_hit_flag;                      //需要发球标志位 ， 1为需要发球 ，0为不需要发球
  volatile uint8_t hit_ball_chassis_move_flag;              //机器人底盘移动标志位用于判断运动阶段
  volatile uint8_t hit_ball_chassis_stop_flag;              //机器人底盘stop标志位用于判断运动阶段
  volatile uint8_t hit_ball_chassis_star_remenber_flag;     //机器人底盘stop标志位用于判断运动阶段

  volatile int sriker_delay_flag;                       //击球杆延时标志位

} s_task_flags;

typedef struct
{
  uint16_t dbus;            //DT7的遥控器数据
  uint16_t M3508_M1;
  uint16_t M3508_M2;
  uint16_t M3508_M3;
  uint16_t DM4340_M1;
  uint16_t DM4340_M2;
  uint16_t DM4340_M3;
  uint16_t Pitch_DM8006;
  uint16_t Striker_3508;
  uint16_t PC;
  uint16_t board_imu;

} s_FPS_monitor;

extern const s_robo_Mode_Setting *get_robot_mode_pint(void);
void mode_state_check(void);
void robo_init_complete(void);
void start_Monitor(void);
void final_Monitor(void);

void mode_switch(void);

#endif /* MONITOR_H_ */
