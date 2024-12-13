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

extern s_motor_data_t DM4340_Date[3]; // DM4340回传数据结构体
extern chassis_control_t motor_control;
extern bat_control_t bat_control;

void debug_task(void const * argument)
{
        static TickType_t debugLastWakeTime;
    while(1)
    {
        debugLastWakeTime = xTaskGetTickCount();

            uart_dma_printf(&huart1, "%4.3f ,%4.3f ,%4.3f ,%4.3f, %4.3f ,%4.3f\n",
                            DM4340_Date[0].real_angle,
                            DM4340_Date[1].target_angle,
                            DM4340_Date[2].esc_back_position,
                            bat_control.pos_angle_data.BallCurrentPoint.x,
                            bat_control.pos_angle_data.BallCurrentPoint.y,
                            bat_control.pos_angle_data.BallCurrentPoint.z
                            );
        vTaskDelayUntil(&debugLastWakeTime, 4);//250Hz

    }
}

