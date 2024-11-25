#ifndef BAT_CONTROL_H
#define BAT_CONTROL_H

#include "can_recv.h"
#include "bsp_can.h"
#include "chassis.h"
#include "pid.h"

#define PI 3.141592653824f //圆周率

#define R1 89.0236f
#define L1 150.0f         //连杆1，mm
#define La 150.0f         //连杆2
#define R2 94.9741f         //连接点到球拍中心的距离

#define DM_MOTOR_KP     72.0f //71.0f    //位置增益  67
#define DM_MOTOR_KD     3.3f //3.2f   //速度增益    可以尝试绑定发球速
#define DM_MOTOR_t_ff   -1.2f//-1.3f前馈力矩

#define DM4340_ANGLE_PID_KP   1.3f    //
#define DM4340_ANGLE_PID_KI   0.0f    //
#define DM4340_ANGLE_PID_KD   7.0f
#define DM4340_ANGLE_PID_MAX_IOUT 5000.0f //  
#define DM4340_ANGLE_PID_MAX_OUT 8000.0f //  

#define DM4340_SPEED_PID_KP   3.2f    //
#define DM4340_SPEED_PID_KI   0.0f    //
#define DM4340_SPEED_PID_KD   0.0f
#define DM4340_SPEED_PID_MAX_IOUT 5000.0f //  
#define DM4340_SPEED_PID_MAX_OUT 8000.0f // 

#define HT04_ANGLE_PID_KP   5.0f    //
#define HT04_ANGLE_PID_KI   0.0f    //
#define HT04_ANGLE_PID_KD   0.0f
#define HT04_ANGLE_PID_MAX_IOUT 0.0f //  
#define HT04_ANGLE_PID_MAX_OUT 0.0f //  

#define HT04_SPEED_PID_KP   0.0f    //
#define HT04_SPEED_PID_KI   0.0f    //
#define HT04_SPEED_PID_KD   0.0f
#define HT04_SPEED_PID_MAX_IOUT 0.0f //  
#define HT04_SPEED_PID_MAX_OUT 0.0f // 

//过高5.6

//点坐标xyz
struct Point
{
    int x;
    int y;
    int z;
};
//角度θ
struct Angle
{
    float theta1;
    float theta2;
    float theta3;
};
struct Data
{
    struct Point CurrentPoint;       //球拍当前点坐标
    struct Point DesiredPoint;       //球拍期望点坐标
    struct Point BallCurrentPoint;   //球当前点坐标
    struct Point BallLastPoint;      //球上一次点坐标
    struct Angle CurrentAngle;       //球拍当前角度
};

extern struct Data Data;
//从上位机接收的球位置
typedef struct
{
	float ball_pos_x;//画面x

	float ball_pos_y;//画面y

	float ball_pos_z;//画面z（深度）

	float White_ratio;        //白色占比
}Ball_Pos;

//球拍的当前位置
typedef struct
{
    float real_x;
    float real_y;
    float real_z;
}bat_space_pos_t;


//三个电机的解算角度
typedef struct 
{
	float angle[3];
	
}top_inverse_calculation_angle;



/*球拍控制部分结构体*/
typedef struct 
{
    const RC_ctrl_t *control_RC;      
    s_pid_absolute_t DM_Motor_PID_speed[3];
    s_pid_absolute_t DM_Motor_PID_angle[3];

    s_pid_absolute_t HT_Motor_PID_speed;
    s_pid_absolute_t HT_Motor_PID_angle;

    top_inverse_calculation_angle top_inverse_angle;
    bat_space_pos_t bat_space_pos;



    float pid_out[3]; 

    float set_x;
    float set_y;
    float set_z;

}bat_control_t;

void functional_zone_task(void const * argument);

void start_motor(CAN_HandleTypeDef *Target_hcan, uint16_t id);
void bat_motor_Init(bat_control_t *bat_control);
void DM_motor_control(bat_control_t *bat_control);
void DM_motor_pid_clac(bat_control_t *bat_control);
void top_RC_control_set(bat_control_t *bat_control);
void bat_data_update(bat_control_t *bat_control);
float float_constrain(float Value, float minValue, float maxValue);
void delta_arm_inverse_calculation(bat_control_t *bat_control ,float x, float y, float z);
void delay(int count);
void Forward_Kinematics(bat_control_t *bat_control,float theta1, float theta2, float theta3);
//static void POS_PID_init(M6020_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
//static fp32 POS_PID_calc(M6020_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);



#endif /* BAT_CONTROL_H_ */
