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

extern s_Dji_motor_data_t motor_Date[4];      // RM电机回传数据结构体
extern s_motor_data_t DM4340_Date[3]; // DM4340回传数据结构体
extern chassis_control_t motor_control;
extern bat_control_t bat_control;

void debug_task(void const *argument)
{
    static TickType_t debugLastWakeTime;
    while (1)
    {
        debugLastWakeTime = xTaskGetTickCount();

       uart_dma_printf(&huart1, "%4.3f ,%4.3f ,%4.3f ,%4.3f, %4.3f ,%4.3f\n",
                      DM4340_Date[0].real_angle,
                      DM4340_Date[1].real_angle,
                      DM4340_Date[2].real_angle,
                      bat_control.pos_angle_data.CurrentPoint.x,
                      bat_control.pos_angle_data.CurrentPoint.y,
                      bat_control.pos_angle_data.CurrentPoint.z);

        // uart_dma_printf(&huart1, "%4.3d , %4.3f ,%4.3f ,%4.3f\n",
        //                     motor_Date[3].back_motor_speed,
        //                     motor_Date[3].target_motor_speed,
        //                     motor_Date[3].serial_motor_ang,
        //                     bat_control.striker_start_angle);
        vTaskDelayUntil(&debugLastWakeTime, 5);//200Hz
    }
}
