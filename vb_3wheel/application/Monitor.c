#include "Monitor.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "remote_control.h"
#include "bsp_buzzer.h"

// 本进程负责确定机器人控制状态，同时监控所有的错误

s_FPS_monitor FPS = {0};
s_FPS_monitor startFPS = {0};
s_FPS_monitor finalFPS = {0};

extern uint16_t fps_remote_count;
extern RC_ctrl_t rc_ctrl;

extern s_task_flags task_flags ;
extern s_robo_Mode_Setting robot_StateMode ;



static void mode_state_check(void)
{

}


//初始化完成提示音di_di_di
 void robo_init_complete(void)
{
    static uint16_t psc_init = 0 ;
    static uint16_t pwm_init = 0;
    for (size_t i = 0; i < 3; i++)
    {
       		psc_init=0;
			pwm_init=200;
			buzzer_on(psc_init, pwm_init);
            osDelay(100);
            buzzer_off();
            osDelay(200);
    }
}

void start_Monitor(void)
{
	FPS.dbus                        = fps_remote_count;
	startFPS.M3508_M1		    	= FPS.M3508_M1;
	startFPS.M3508_M2		    	= FPS.M3508_M2;
	startFPS.M3508_M3		    	= FPS.M3508_M3;
	startFPS.DM4340_M1		    	= FPS.DM4340_M1;
	startFPS.DM4340_M2				= FPS.DM4340_M2;
	startFPS.DM4340_M3				= FPS.DM4340_M3;
	startFPS.Pitch_DM8006 	        = FPS.Pitch_DM8006;
	startFPS.Striker_HT04			= FPS.Striker_HT04;
	startFPS.PC				        = FPS.PC;
	startFPS.board_imu				= FPS.board_imu;
    startFPS.dbus		            = FPS.dbus;    
	
}

/**
  * @brief          二次计算FPS，在任务延时一秒后计算
  * @param[in]      none
  * @retval         none
  */
void final_Monitor(void)
{
    finalFPS.dbus               = fps_remote_count      - startFPS.dbus;            //正常70左右
    finalFPS.M3508_M1           = FPS.M3508_M1          - startFPS.M3508_M1;        //正常1200左右（未降过频率）
    finalFPS.M3508_M2           = FPS.M3508_M2          - startFPS.M3508_M2;
    finalFPS.M3508_M3           = FPS.M3508_M3          - startFPS.M3508_M3;
    finalFPS.DM4340_M1          = FPS.DM4340_M1         - startFPS.DM4340_M1;
    finalFPS.DM4340_M2          = FPS.DM4340_M2         - startFPS.DM4340_M2;
    finalFPS.DM4340_M3          = FPS.DM4340_M3         - startFPS.DM4340_M3;
    finalFPS.Pitch_DM8006       = FPS.Pitch_DM8006      - startFPS.Pitch_DM8006;
    finalFPS.Striker_HT04        = FPS.Striker_HT04     - startFPS.Striker_HT04;
    finalFPS.PC                 = FPS.PC                - startFPS.PC;
	
 }

void mode_switch(s_robo_Mode_Setting *robot_Mode)
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
