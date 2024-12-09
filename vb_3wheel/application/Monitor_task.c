#include "Monitor_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "remote_control.h"
#include "bsp_buzzer.h"

// 确定机器人状态，同时监控所有的错误

s_robo_Mode_Setting robot_StateMode;

extern RC_ctrl_t rc_ctrl;
void mode_state_check(void);
static void mode_switch(s_robo_Mode_Setting *robot_Mode);

void Monitor_task(void const *argument)
{
    vTaskDelay(500);
    
    while (1)
    {   //mode_state_check();
        mode_switch(&robot_StateMode);
        osDelay(2);
    }
}

void mode_state_check(void)
{
	static uint16_t psc_motor=0;//电机蜂鸣器分频值
	static uint16_t psc_dbus=0;//遥控器DBUS蜂鸣器分频值
	static uint16_t psc_judge=0;//裁判系统蜂鸣器分频值
	static uint16_t psc_cap=0;//超级电容控制板蜂鸣器分频值
	static uint16_t psc_pc=0;//pc蜂鸣器分频值
	static uint16_t pwm_motor=0;//电机蜂鸣器重载值
	static uint16_t pwm_dbus=0;//遥控器DBUS蜂鸣器重载值
	static uint16_t pwm_judge=0;//裁判系统蜂鸣器重载值
	static uint16_t pwm_cap=0;//超级电容控制板蜂鸣器重载值
	static uint16_t pwm_pc=0;//pc蜂鸣器重载值

    	if(pwm_dbus==0) pwm_dbus=20000;
		else pwm_dbus=0;
		buzzer_on(psc_dbus, pwm_dbus);
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
