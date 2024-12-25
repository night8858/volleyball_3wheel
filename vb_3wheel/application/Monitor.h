#ifndef MONITOR_H_
#define MONITOR_H_
#include "stm32f4xx.h"

typedef struct
{

  volatile enum roboState {
    DEAD = 0,            // 机器人死亡
    INITIALIZING = 1,    // 初始化
    ON_PROCESSING = 2,   // 运行中
    WELL_PROCESSING = 3, // 处理完毕
    DBUS_ERROR = 4,      // 遥控器出错
    MOTOR_ERROR = 5,     // 电机出错
    JUDGE_ERROR = 6,     // 裁判系统出错
    CAP_ERROR = 7,       // 电容控制板出错
    PC_ERROR = 8,        // PC出错
    NOMAL = 9,           // 一切正常
  } roboState;           // 机器人状态

  volatile enum roboMode {
    MODE_STOP = 0,      // 停止模式
    MODE_ARTIFICAL = 1, // 人工模式
    MODE_AUTO = 2,      // 自动模式

    ARTIFICAL_CHASSIS = 3, // 人工模式下，机器人底盘
    ARTIFICAL_BAT = 4,     // 人工模式下，机器人球拍
    ARTIFICAL_STRIKER = 5, // 人工模式下，机器人击球杆

  } roboMode; // 机器控制模式

} s_robo_Mode_Setting;

typedef struct
{

  volatile uint8_t Init_complete_flag;
  volatile uint8_t chassis_Init_flag;
  volatile uint8_t bat_control_Init_flag;
  volatile char sensor_is_blocked;

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

void mode_switch(s_robo_Mode_Setting *robot_Mode);

#endif /* MONITOR_H_ */
