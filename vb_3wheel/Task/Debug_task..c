/**
  ******************************TOE******************************
  * @file       debug_task.c/h
  * @brief      用于Debug的任务线程
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2024            night8858
  *
  @verbatim
  ===============================================================

  ===============================================================
  @endverbatim
  ******************************TOE******************************
  */

#include "main.h"
#include "cmsis_os.h"

#include "debug_task.h"
#include "bsp_usart.h"
#include "bat_control.h"
#include "Variables.h"
#include "INS_task.h"
#include "pc_interface.h"

extern s_IMU_all_Value IMU_All_Value;    // 用于计算欧拉角连续化的结构体
extern s_Dji_motor_data_t motor_Date[4]; // RM电机回传数据结构体
extern s_motor_data_t DM4340_Date[3];    // DM4340回传数据结构体
extern chassis_control_t chassis_control;
extern bat_control_t bat_control;
extern s_motor_data_t DM8006_Date[1];     // DM8006回传数据结构体
extern volatile uint8_t robot_start_flag; // 机器人启动标志位
extern s_task_flags task_flags;           // 引用任务标志结构体
extern visInf_t s_visionInform;           // 视觉数据结构体

void debug_task(void const *argument)
{
  static TickType_t debugLastWakeTime;
  while (1)
  {
    debugLastWakeTime = xTaskGetTickCount();

    // 调试8006使用
    //  uart_dma_printf(&huart1, "%4.3f, %4.3f ,%4.3f ,%4.3f\n",
    //                  DM8006_Date[0].real_angle,
    //                  DM8006_Date[0].target_angle,
    //                  DM8006_Date[0].out_current,
    //                  DM8006_Date[0].esc_back_speed);

    // 调试击球电机
    //  uart_dma_printf(&huart1, "%4.3d, %4.3f ,%4.3d ,%4.3f\n",
    //                  motor_Date[3].back_motor_speed,
    //                  motor_Date[3].target_motor_ang,
    //                  motor_Date[3].out_current,
    //                  motor_Date[3].serial_motor_ang);

    // 调试底盘电机
    //  uart_dma_printf(&huart1, "%4.3f, %4.3f ,%4.3d\n",
    //                  motor_Date[1].serial_motor_ang,
    //                  motor_Date[1].target_motor_ang,
    //                  motor_Date[1].out_current);

    // uart_dma_printf(&huart1, "%4.3f, %4.3f ,%4.3f, %4.3f ,%4.3f ,%4.3f\n",
    //                 DM4340_Date[0].real_angle,
    //                 DM4340_Date[1].real_angle,
    //                 DM4340_Date[2].real_angle,
    //                 bat_control.CurrentPoint.x,
    //                 bat_control.CurrentPoint.y,
    //                 bat_control.CurrentPoint.z);

    // 调试视觉数据
    // uart_dma_printf(&huart1, "%4.3f, %4.3f ,%4.3f, %4.3f ,%4.3f ,%4.3f\n",
    //                 s_visionInform.x_hikvision,
    //                 s_visionInform.y_hikvision,
    //                 s_visionInform.z_hikvision,
    //                 s_visionInform.x_usbcam,
    //                 s_visionInform.y_usbcam,
    //                 s_visionInform.z_usbcam);
    // 调试接发球视觉数据
    uart_dma_printf(&huart1, "%4.3f, %4.3f ,%4.3f ,%4.3f\n",
                    s_visionInform.x_hikvision,
                    s_visionInform.y_hikvision,
                    s_visionInform.z_hikvision,
                    chassis_control.vy_set);

    // 调试all_flag
    // uart_dma_printf(&huart1, "%4.3d, %4.3d ,%4.3d\n",
    //                 task_flags.bat_running_flag,
    //                 task_flags.sensor_is_blocked,
    //                 task_flags.ball_soaring_flag);

    // 调试陀螺仪补正
    // uart_dma_printf(&huart1, "%4.3f, %4.3f ,%4.3f\n",
    //                 IMU_All_Value.yaw.yawAngV,
    //                 chassis_control.vx,
    //                 chassis_control.vy);

    // 球拍位置调试
    // uart_dma_printf(&huart1, "%4.3f, %4.3f ,%4.3f, %4.3f ,%4.3f ,%4.3f , %d\n",
    //                 bat_control.DesiredPoint.x,
    //                 bat_control.DesiredPoint.y,
    //                 bat_control.DesiredPoint.z,
    //                 bat_control.CurrentPoint.x,
    //                 bat_control.CurrentPoint.y,
    //                 bat_control.CurrentPoint.z,
    //                 bat_control.bat_running_flag);

    // uart_dma_printf(&huart1, "%4.3d , %4.3f ,%4.3f ,%4.3f\n",
    //                     motor_Date[3].back_motor_speed,
    //                     motor_Date[3].target_motor_speed,
    //                     motor_Date[3].serial_motor_ang,
    //                     bat_control.striker_start_angle);
    vTaskDelayUntil(&debugLastWakeTime, 2); // 200Hz
  }
}
