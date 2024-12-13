/**
  ******************************TOE******************************
  * @file       chassis_task.c/h
  * @brief      用于底盘控制
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

#include "Chassis_task.h"
#include "chassis.h"
#include "Monitor.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

extern chassis_control_t motor_control;
extern s_robo_Mode_Setting robot_StateMode;

void chassis_task(void const *argument)
{
    vTaskDelay(4000);

    motor_init(&motor_control);

    static TickType_t chassisLastWakeTime;
    while (1)
    {
        // uart_dma_printf(&huart1,"%4.3f ,%1.1f ,%4.3f\n",
        // rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1], rc_ctrl.rc.ch[2]);
        // 选择进入控制模式
        chassisLastWakeTime = xTaskGetTickCount();

        /////防止任务切换////
        taskENTER_CRITICAL();
        chassis_feedback_update(&motor_control);

        mode_switch(&robot_StateMode);

        rc_to_motor_set(&motor_control);

        chassis_movement_calc(&motor_control);

        motor_control_send(&motor_control);
        taskEXIT_CRITICAL();

        //设定调度频率
        vTaskDelayUntil(&chassisLastWakeTime , 2); //200Hz
    }
}