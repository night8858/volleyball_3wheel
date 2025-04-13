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

extern chassis_control_t chassis_control;
extern s_robo_Mode_Setting robot_StateMode;
extern volatile uint8_t robot_start_flag;        //机器人启动标志位

void chassis_task(void const *argument)
{
    vTaskDelay(2000);

    chassis_init(&chassis_control);

    static TickType_t chassisLastWakeTime;

    while (1)
    {
        // uart_dma_printf(&huart1,"%4.3f ,%1.1f ,%4.3f\n",
        // rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1], rc_ctrl.rc.ch[2]);
        // 选择进入控制模式
        chassisLastWakeTime = xTaskGetTickCount();

          chassis_feedback_update(&chassis_control);

          mode_switch();
  
          rc_to_motor_set(&chassis_control);
  
          chassis_movement_calc(&chassis_control);
  
        //设定调度频率
        vTaskDelayUntil(&chassisLastWakeTime , 2); //200Hz
    }
}
