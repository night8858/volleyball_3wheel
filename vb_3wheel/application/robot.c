/**
  ******************************TOE******************************
  * @file       robot.c/h
  * @brief      用于机器人初始化
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025            night8858       
  *
  @verbatim
  ===============================================================

  ===============================================================
  @endverbatim
  ******************************TOE******************************
  */
#include "stm32f407xx.h"

#include "robot.h"
#include "chassis.h"
#include "bat_control.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"

extern volatile uint8_t imu_flag;
extern volatile uint8_t robot_start_flag;        //机器人启动标志位
extern chassis_control_t chassis_control;           // 底盘控制结构体
extern bat_control_t bat_control;                 // 球拍控制结构体

/**
 * @brief 机器人初始化函数
 *
 * 执行机器人系统的初始化操作，包括：
 * - 模式切换初始化
 * - 底盘系统初始化
 * - 击打电机初始化
 */
void robot_init(void)
{

    mode_switch();              // 模式切换初始化,确认初始模式      
    chassis_init(&chassis_control);// 底盘初始化
    bat_motor_Init(&bat_control);//球拍初始化
    robo_init_complete();
    robot_start_flag = 1;
}
