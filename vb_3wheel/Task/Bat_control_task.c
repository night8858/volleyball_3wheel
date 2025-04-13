/**
  ******************************TOE******************************
  * @file       functional_zone_task.c/h
  * @brief      用于控制机器人顶部功能区的任务
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

#include "Bat_control_task.h"
#include "bat_control.h"
#include "Variables.h"
#include "Monitor.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

extern bat_control_t bat_control;
extern s_robo_Mode_Setting robot_StateMode ;
extern volatile uint8_t robot_start_flag;        //机器人启动标志位

void functional_zone_task(void const *argument)
{
    vTaskDelay(2000);
    ///////////////初始化函数///////////////
    bat_motor_Init(&bat_control);
    ///////////////初始化函数///////////////

    robo_init_complete();
    static TickType_t 	batLastWakeTime;
    while (1)
    {
        // HT04_motor_PID_Control(&hcan1 , 0x50 , 0.0f);
        batLastWakeTime = xTaskGetTickCount();

        // taskENTER_CRITICAL();
        bat_data_update(&bat_control);
        mode_switch();
        bat_action(&bat_control);
        motor_pid_clac(&bat_control);
        bat_motor_control(&bat_control);
        // taskEXIT_CRITICAL();
        
        vTaskDelayUntil(&batLastWakeTime, 2);//500Hz

       
    }
}


/*
当前的顶部解算和插值方式有问题，需要重新设计,明天尽快进行改动

*/
