#include "Monitor_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "remote_control.h"

// 确定机器人状态，同时监控所有的错误

s_robo_Mode_Setting robot_StateMode;

extern RC_ctrl_t rc_ctrl;

static void mode_switch(s_robo_Mode_Setting *robot_Mode);

void Monitor_task(void const *argument)
{
    vTaskDelay(3000);
    while (1)
    {
        mode_switch(&robot_StateMode);
        osDelay(2);
    }
}

void mode_state_check(void)
{
}

static void mode_switch(s_robo_Mode_Setting *robot_Mode)
{
    if (rc_ctrl.rc.s[0] == 1)
    {
        robot_Mode->roboMode = MODE_ARTIFICAL;

        if (rc_ctrl.rc.s[1] == 1)
        {
            robot_Mode->roboMode = ARTIFICAL_CHASSIS;
        }

        else if (rc_ctrl.rc.s[1] == 2)
        {
            robot_Mode->roboMode =ARTIFICAL_BAT;
        }
        else if ( rc_ctrl.rc.s[1] == 3)
        {
            robot_Mode->roboMode = ARTIFICAL_STRIKER;
        }
    }

    else if (rc_ctrl.rc.s[0] == 2)
    {
        robot_Mode->roboMode = MODE_AUTO;
    }
    else if (rc_ctrl.rc.s[0] == 3)
    {
        robot_Mode->roboMode = MODE_STOP;
    }
}

const s_robo_Mode_Setting *get_robot_mode_pint(void)
{
    return &robot_StateMode;
}
