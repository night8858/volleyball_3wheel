// ██╗    ██╗  ██╗  ██╗ ███████╗ ███████╗ ██╗
// ██║    ██║  ██║  ██║ ██╔════╝ ██╔════╝ ██║
// ██║ ██╗ ██║ ███████║ ██████╗  ██████╗  ██║
// ██║ ╚██╗██║ ██╔══██║ ██╔══╝   ██╔══╝   ██║
// ╚██╗╚████║  ██║  ██║ ███████╗ ███████╗ ███████╗
//  ╚═╝ ╚═══╝  ╚═╝  ╚═╝ ╚══════╝ ╚══════╝ ╚══════╝

/*
 *   █████████████╗   ███████████╗     ████████████╗
 *   ╚════██╔═════╝  ██╔════════██╗    ██╔═════════╝
 *        ██║        ██║        ██║    ██║
 *        ██║        ██║        ██║    ██║
 *        ██║        ██║        ██║    ██║█████████╗
 *        ██║        ██║        ██║    ██╔═════════╝
 *        ██║        ██║        ██║    ██║
 *        ██║        ██║        ██║    ██║
 *        ██║        ╚███████████╔╝    ████████████╗
 *        ╚═╝         ╚══════════╝     ╚═══════════╝
 *  Version    Date            Author          Modification
 *  V1.0.0     2024            night8858
 */

#include "striker_task.h"
#include "Bat_control_task.h"
#include "bat_control.h"
#include "Variables.h"
#include "Monitor.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

extern bat_control_t bat_control;
extern s_robo_Mode_Setting robot_StateMode;
extern volatile uint8_t robot_start_flag; // 机器人启动标志位
extern s_task_flags task_flags;             // 引用任务标志结构体

void striker_task(void const *argument)
{
    osDelay(2000);
    striker_new_init();

    for (;;)
    {
        if (task_flags.bat_control_Init_flag  == 1)
        {
            if (robot_StateMode.roboMode == ARTIFICAL_STRIKER) // 机器人模式为ARTIFICAL_STRIKER时，使用遥控器控制击球杆
            {

                // hit_ball_launch(bat_control);
                ACTION_striker_move();
                // int control_falg = 0;
                // // 击球杆运行状态判断
            }

            striker_motor_control(&bat_control);
        }
        osDelay(2);
    }
}
