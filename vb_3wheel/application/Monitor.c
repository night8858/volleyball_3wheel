//*********************************************//
//监控和判断当前遥控器所处的转台
//
//
//
//
//
//
//*********************************************//


#include "Monitor.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "remote_control.h"
#include "bsp_buzzer.h"


s_FPS_monitor FPS = {0};
s_FPS_monitor startFPS = {0};
s_FPS_monitor finalFPS = {0};

extern uint16_t fps_remote_count;
extern RC_ctrl_t rc_ctrl;

extern s_task_flags task_flags ;
extern s_robo_Mode_Setting robot_StateMode ;
extern s_robo_Mode_Setting robot_StateMode_last ;
extern roboError_flag_t VolleyRobot_R1;

 void mode_state_check(void)
{
    //判断遥控器是否掉线
    if (finalFPS.dbus < 30)
    {
        VolleyRobot_R1.DBUS_ERROR = 1;
    }else
    {
        VolleyRobot_R1.DBUS_ERROR = 0;
    }

    //判断底盘电机是否掉线
    if (finalFPS.M3508_M1 < 300  || finalFPS.M3508_M2 < 300 || finalFPS.M3508_M3 < 300)
    {
        VolleyRobot_R1.CHASSIS_MOTOR_ERROR = 1;

    }else
    {
        VolleyRobot_R1.CHASSIS_MOTOR_ERROR = 0;
    }

    //判断球拍电机是否掉线
    if (finalFPS.DM4340_M1 < 200 || finalFPS.DM4340_M2 < 200 || finalFPS.DM4340_M3 < 200) 
    {
        VolleyRobot_R1.BAT_MOTOR_ERROR = 1;

    }else
    {
        VolleyRobot_R1.BAT_MOTOR_ERROR = 0;
    }

    //判断击球电机是否掉线
    if (finalFPS.Striker_3508 < 300)
    {
        VolleyRobot_R1.Striker_MOTOR_ERROR = 1;
    }else
    {
        VolleyRobot_R1.Striker_MOTOR_ERROR = 0;
    }

    //判断pitch电机是否掉线
    if (finalFPS.Pitch_DM8006 < 200)
    {
        VolleyRobot_R1.Pitch_MOTOR_ERROR = 1;
    }else
    {
        VolleyRobot_R1.Pitch_MOTOR_ERROR = 0;
    }

    
}


//初始化完成提示音di_di_di
 void robo_init_complete(void)
{
     uint16_t psc_init = 0;
     uint16_t pwm_init = 0;
    for (size_t i = 0; i < 3; i++)
    {
       		psc_init=0;
			pwm_init=200;
			buzzer_on(psc_init, pwm_init);
            osDelay(200);
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
	startFPS.Striker_3508			= FPS.Striker_3508;
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
    finalFPS.Striker_3508       = FPS.Striker_3508      - startFPS.Striker_3508;
    finalFPS.PC                 = FPS.PC                - startFPS.PC;
	
}

void mode_switch(void)
{   
    robot_StateMode_last.roboMode = robot_StateMode.roboMode;
    
    if (rc_ctrl.rc.s[1] == 3)
    {
        // robot_StateMode.roboMode = MODE_ARTIFICAL;

        if (rc_ctrl.rc.s[0] == 1)
        {
            //手动模式底盘
            robot_StateMode.roboMode = ARTIFICAL_CHASSIS;
        } 

        else if (rc_ctrl.rc.s[0] == 2)
        {
            //手动模式球拍
            robot_StateMode.roboMode =ARTIFICAL_BAT;
        }
        else if ( rc_ctrl.rc.s[0] == 3)
        {
            //手动模式击球
            robot_StateMode.roboMode = ARTIFICAL_STRIKER;
        }
    }

    else if (rc_ctrl.rc.s[1] == 1)
    {
        // robot_StateMode.roboMode = MODE_AUTO;
        if (rc_ctrl.rc.s[0] == 1)
        {
            //自动模式下，机器人发球
            robot_StateMode.roboMode = ROBOT_DEBUG;
        }

        else if (rc_ctrl.rc.s[0] == 2)
        {
            //自动模式下，机器人接发球
            robot_StateMode.roboMode = AUTO_RECEIVE_BALL;
        }

        else if ( rc_ctrl.rc.s[0] == 3)
        {
            //手动模式击球
            robot_StateMode.roboMode = MODE_STOP;
        }
    }
    else if (rc_ctrl.rc.s[1] == 2)
    {
        robot_StateMode.roboMode = MODE_STOP;
    }

//检测模式切换
   if(robot_StateMode_last.roboMode == robot_StateMode.roboMode)
   {
    task_flags.mode_switched_flag = 0;
   } else
   {
    task_flags.mode_switched_flag = 1;
   }

   if(VolleyRobot_R1.DBUS_ERROR == 1)
   {
       
       robot_StateMode.roboMode = MODE_STOP;
       return;
   }

}

const s_robo_Mode_Setting *get_robot_mode_pint(void)
{
    return &robot_StateMode;
}
