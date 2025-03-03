/**
  ******************************TOE******************************
  * @file       Monitor_task.c/h
  * @brief      用于监控所有的程序,数据等,确保正常的运行,在出现异常时
  *             及时杀死线程
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

#include "Monitor_task.h"
#include "Monitor.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "Variables.h"
extern s_robo_Mode_Setting robot_StateMode;
extern s_task_flags task_flags ;


void Monitor_task(void const *argument)
{
	
    vTaskDelay(500);
    while (task_flags.Init_complete_flag != 1)
    {
        if (task_flags.chassis_Init_flag && task_flags.bat_control_Init_flag == 1)
        {
            task_flags.Init_complete_flag = 1;
        }
        osDelay(100);
    }
	robo_init_complete();
    static TickType_t monitorLastWakeTime;
    mode_switch(&robot_StateMode);

    while (1)
    {   //mode_state_check();
        start_Monitor();//FPS赋值给startFPS
		monitorLastWakeTime = xTaskGetTickCount();
		vTaskDelayUntil(&monitorLastWakeTime, 1000);//在此阻塞1000ms，FPS依旧在加，startFPS赋值被阻塞，以便下一次计算差值
		final_Monitor();//计算每秒进入CAN的次数
        mode_state_check();
        //osDelay(2);
        vTaskDelayUntil(&monitorLastWakeTime, 5);//200Hz

    }
}