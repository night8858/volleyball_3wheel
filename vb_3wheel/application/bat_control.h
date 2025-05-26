#ifndef BAT_CONTROL_H
#define BAT_CONTROL_H

#include "can_recv.h"
#include "bsp_can.h"
#include "chassis.h"
#include "pid.h"
#include "Monitor.h"

#define PI 3.141592653824f //圆周率

#define WIGTH_of_BAT 3.4f //球拍重量(kg)
#define R1 89.0236f
#define L1 150.0f         //连杆1，mm
#define La 150.0f         //连杆2
#define R2 94.9741f         //连接点到球拍中心的距离

#define DM_MOTOR_KP     72.0f //71.0f    //位置增益  67
#define DM_MOTOR_KD     3.3f //3.2f   //速度增益    可以尝试绑定发球速
#define DM_MOTOR_t_ff   -1.2f//-1.3f前馈力矩

//控球拍（bat）的PID参数//
#define DM4340_ANGLE_PID_KP   1.4f    //
#define DM4340_ANGLE_PID_KI   0.0f    //
#define DM4340_ANGLE_PID_KD   4.0f
#define DM4340_ANGLE_PID_MAX_IOUT 100.0f //  
#define DM4340_ANGLE_PID_MAX_OUT 25.0f //  

#define DM4340_SPEED_PID_KP   0.4f    //
#define DM4340_SPEED_PID_KI   0.0f    //
#define DM4340_SPEED_PID_KD   0.0f
#define DM4340_SPEED_PID_MAX_IOUT 100.0f //  
#define DM4340_SPEED_PID_MAX_OUT 7.5f //   给10是限制扭矩输出，4340只有9n的额定



//pitch俯仰角的PID参数//
#define DM8006_ANGLE_PID_KP   3.88f    //
#define DM8006_ANGLE_PID_KI   0.0f    //
#define DM8006_ANGLE_PID_KD   5.2f
#define DM8006_ANGLE_PID_MAX_IOUT 100.0f //  
#define DM8006_ANGLE_PID_MAX_OUT 80.0f //  

#define DM8006_SPEED_PID_KP   0.11f    //不知道为什么大于0.3就震荡
#define DM8006_SPEED_PID_KI   0.0f    //
#define DM8006_SPEED_PID_KD   0.0f
#define DM8006_SPEED_PID_MAX_IOUT 100.0f //  
#define DM8006_SPEED_PID_MAX_OUT 8.0f // 给20是限制扭矩输出，8006只有8n的额定



//击球拍（striker）的PID参数//
#define HT04_ANGLE_PID_KP   2.0f    //
#define HT04_ANGLE_PID_KI   0.0f    //
#define HT04_ANGLE_PID_KD   0.0f
#define HT04_ANGLE_PID_MAX_IOUT 1000.0f //  
#define HT04_ANGLE_PID_MAX_OUT 30.0f //  

#define HT04_SPEED_PID_KP   1.0f    //
#define HT04_SPEED_PID_KI   0.0f    //
#define HT04_SPEED_PID_KD   0.0f
#define HT04_SPEED_PID_MAX_IOUT 1000.0f //  
#define HT04_SPEED_PID_MAX_OUT 30.0f // 


//击球拍击打用
#define STRIKER_3508_ANGLE_PID_KP   17.0f    //
#define STRIKER_3508_ANGLE_PID_KI   0.0f    //
#define STRIKER_3508_ANGLE_PID_KD   2.1f    //
#define STRIKER_3508_ANGLE_PID_MAX_IOUT 12000.0f //  
#define STRIKER_3508_ANGLE_PID_MAX_OUT 18000.0f //  

#define STRIKER_3508_SPEED_PID_KP   3.4f    //
#define STRIKER_3508_SPEED_PID_KI   0.0f    //
#define STRIKER_3508_SPEED_PID_KD   4.7f
#define STRIKER_3508_SPEED_PID_MAX_IOUT 12000.0f //  
#define STRIKER_3508_SPEED_PID_MAX_OUT 15000.0f // 

//击球拍初始化用
#define STRIKER_3508_SPEED_INIT_PID_KP   42.0f    //
#define STRIKER_3508_SPEED_INIT_PID_KI   0.0f    //
#define STRIKER_3508_SPEED_INIT_PID_KD   8.0f
#define STRIKER_3508_SPEED_INIT_PID_MAX_IOUT 8000.0f //  
#define STRIKER_3508_SPEED_INIT_PID_MAX_OUT 8000.0f // 



typedef enum {
    BAT_IDLE,
    BAT_HITTING,
    BAT_COOLDOWN
} bat_state_t;           //球拍打球状态机

typedef enum 
{
    SERVE_IDLE,          // 空闲状态，等待指令
    BAT_HITTING_UP1,      // 球拍向上击球
    WITE_FOR_BALL,            // 准备冷却    
    BAT_HITTING_UP2,      // 球拍向上击球
    WAIT_FOR_BALL_DOWN,  // 等待球下落
    STRIKER_HITTING,     // 击球杆击球
    STRIKER_RETURNING,   // 击球杆复位
    SERVE_COOLDOWN,       // 冷却状态
    OVERTIME_RETURNING     // 超时复位
}serve_ball_t;           //击球发球状态机

typedef enum 
{
    STRIKER_START,      // 准备击球
    STRIKER_MOVEING,      // 
    SERVE_COMPLETE          // 空闲状态，等待指令

}striker_reading_t;           //击球发球状态机


// typedef enum{

//     STRIKER_3508_PID_DOUBLE = 0,   //串级环
//     STRIKER_3508_PID_SIGLE = 1     //单速度环

// }striker_pid_choose_e;

//过高5.6

//点坐标xyz
typedef struct 
{
    float x;
    float y;
    float z;
}Point;
//角度θ
typedef struct 
{
    float theta1;
    float theta2;
    float theta3;
}Angle;

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

typedef enum
{
    STRIKER_is_READY = 0,
    STRIKER_is_RUNNING = 1,
}e_striker_flags;

typedef enum
{
    no_move = 0,
    Underhand_serve = 1,     ///下手球发球
    Overhand_serve = 2,      ///上手球发球
  
}e_striker_action_flags;

typedef enum
{
    BAT_is_READY = 0,    //球拍准备好接受指令
    BAT_is_RUNNING = 1,  //球拍正在运行指令
    BAT_in_zero = 2,     //球拍停止

}e_bat_flags;
typedef struct
{

    float striker_start_angle;
    int striker_runing_num;

    float striker_trget_speed;

}s_seriker_control_t;

/*球拍控制部分结构体*/
typedef struct 
{
    const RC_ctrl_t *control_RC;

    ///电机PID参数
    s_pid_absolute_t DM_Motor_PID_speed[3];
    s_pid_absolute_t DM_Motor_PID_angle[3];

    s_pid_absolute_t DM_Motor_8006_PID_speed;
    s_pid_absolute_t DM_Motor_8006_PID_angle;

    s_pid_absolute_t HT_Motor_PID_speed;
    s_pid_absolute_t HT_Motor_PID_angle;
    
    s_pid_absolute_t STRIKER_3508_PID_speed;
    s_pid_absolute_t STRIKER_3508_PID_angle;

    s_pid_absolute_t STRIKER_3508_INIT_PID_speed;
    ///电机PID参数

    top_inverse_calculation_angle top_inverse_angle;  //已废弃

    e_striker_flags striker_state;
    e_bat_flags bat_state;
    e_striker_action_flags striker_action_state;
    
    first_order_filter_type_t input_pitch_pos_filter; // 使用一阶低通滤波器对输入位置进行平滑处理
    first_order_filter_type_t input_set_X_filter; // 使用一阶低通滤波器对输入位置进行平滑处理
    first_order_filter_type_t input_set_Y_filter; // 使用一阶低通滤波器对输入位置进行平滑处理
    first_order_filter_type_t input_set_Z_filter; // 使用一阶低通滤波器对输入位置进行平滑处理
    first_order_filter_type_t input_set_striker_filter; // 使用一阶低通滤波器对输入位置进行平滑处理

    first_order_filter_type_t DesiredPoint_X_filter; // 使用一阶低通滤波器对期望位置进行平滑处理
    first_order_filter_type_t DesiredPoint_Z_filter; // 使用一阶低通滤波器对期望位置进行平滑处理
    first_order_filter_type_t DesiredPoint_Y_filter; // 使用一阶低通滤波器对期望位置进行平滑处理

    Point CurrentPoint;       //球拍当前点坐标
    Point DesiredPoint;       //球拍期望点坐标
    Point DesiredPoint_filter;       //球拍期望点坐标

    Point BallCurrentPoint;   //球当前点坐标
    Point BallLastPoint;      //球上一次点坐标
    Angle CurrentAngle;       //球拍当前角度
    Angle DesireAngle;        //球拍期望角度


    float pitch_init_angle;

    float pid_out[3]; 

    float set_x;
    float set_y;
    float set_z;
    float set_pitch;

    float real_x;
    float real_y;
    float real_z;
    float real_pitch;
    
    float striker_angle;
    float striker_start_angle;
    float set_striker_angle;
    
}bat_control_t;
/*用来储存所有的机器人参数的总结构体，可通过修改部分参数来更改机器人项目*/

void striker_motor_control(bat_control_t *bat_control);
void striker_new_init(void);
void ACTION_striker_move(void);


void bat_motor_Init(bat_control_t *bat_control);
void bat_motor_control(bat_control_t *bat_control);
void motor_pid_clac(bat_control_t *bat_control);
void bat_action(bat_control_t *bat_control);

void bat_data_update(bat_control_t *bat_control);
float float_constrain(float Value, float minValue, float maxValue);
void delta_arm_inverse_calculation(bat_control_t *bat_control, float x, float y, float z);
void Forward_Kinematics(bat_control_t *bat_control, float theta1, float theta2, float theta3);

void bat_posion_set(float x, float y, float z);
void judge_bat_pos(bat_control_t *bat_control);
// float CalDistance2Point( Point point1,  Point point2);
// struct Point GetPointInLine( Point currentP,  Point desiredP, float t);


#endif /* BAT_CONTROL_H_ */
