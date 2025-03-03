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
#define DM4340_SPEED_PID_MAX_OUT 9.0f //   给10是限制扭矩输出，4340只有9n的额定



//pitch俯仰角的PID参数//
#define DM8006_ANGLE_PID_KP   2.95f    //
#define DM8006_ANGLE_PID_KI   0.0f    //
#define DM8006_ANGLE_PID_KD   1.2f
#define DM8006_ANGLE_PID_MAX_IOUT 100.0f //  
#define DM8006_ANGLE_PID_MAX_OUT 60.0f //  

#define DM8006_SPEED_PID_KP   0.25f    //不知道为什么大于0.3就震荡
#define DM8006_SPEED_PID_KI   0.0f    //
#define DM8006_SPEED_PID_KD   0.0f
#define DM8006_SPEED_PID_MAX_IOUT 100.0f //  
#define DM8006_SPEED_PID_MAX_OUT 18.0f // 给20是限制扭矩输出，8006只有20n的额定



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


#define STRIKER_3508_ANGLE_PID_KP   13.0f    //
#define STRIKER_3508_ANGLE_PID_KI   0.0f    //
#define STRIKER_3508_ANGLE_PID_KD   2.0f    //
#define STRIKER_3508_ANGLE_PID_MAX_IOUT 12000.0f //  
#define STRIKER_3508_ANGLE_PID_MAX_OUT 18000.0f //  

#define STRIKER_3508_SPEED_PID_KP   1.5f    //
#define STRIKER_3508_SPEED_PID_KI   0.0f    //
#define STRIKER_3508_SPEED_PID_KD   0.0f
#define STRIKER_3508_SPEED_PID_MAX_IOUT 12000.0f //  
#define STRIKER_3508_SPEED_PID_MAX_OUT 18000.0f // 

#define STRIKER_3508_SPEED_INIT_PID_KP   21.0f    //
#define STRIKER_3508_SPEED_INIT_PID_KI   3.5f    //
#define STRIKER_3508_SPEED_INIT_PID_KD   0.0f
#define STRIKER_3508_SPEED_INIT_PID_MAX_IOUT 8000.0f //  
#define STRIKER_3508_SPEED_INIT_PID_MAX_OUT 12000.0f // 


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
typedef struct 
{
    struct Point CurrentPoint;       //球拍当前点坐标
    struct Point DesiredPoint;       //球拍期望点坐标
    struct Point BallCurrentPoint;   //球当前点坐标
    struct Point BallLastPoint;      //球上一次点坐标
    struct Angle CurrentAngle;       //球拍当前角度
    struct Angle DesireAngle;        //球拍期望角度

}s_Data_t;

//从上位机接收的球位置

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
    BAT_is_READY = 0,    //球拍准备好接受指令
    BAT_is_RUNNING = 1,  //球拍正在运行指令
    BAT_in_zero = 2,     //球拍停止


}e_bat_flags;

/*球拍控制部分结构体*/
typedef struct 
{
    const RC_ctrl_t *control_RC;

    s_pid_absolute_t DM_Motor_PID_speed[3];
    s_pid_absolute_t DM_Motor_PID_angle[3];

    s_pid_absolute_t DM_Motor_8006_PID_speed;
    s_pid_absolute_t DM_Motor_8006_PID_angle;

    s_pid_absolute_t HT_Motor_PID_speed;
    s_pid_absolute_t HT_Motor_PID_angle;
    
    s_pid_absolute_t STRIKER_3508_PID_speed;
    s_pid_absolute_t STRIKER_3508_PID_angle;

    s_pid_absolute_t STRIKER_3508_INIT_PID_speed;

    top_inverse_calculation_angle top_inverse_angle;
    s_Data_t pos_angle_data;

    e_striker_flags striker_state;
    e_bat_flags bat_state;

    first_order_filter_type_t input_pitch_pos_filter; // 使用一阶低通滤波器对输入位置进行平滑处理
    first_order_filter_type_t input_set_X_filter; // 使用一阶低通滤波器对输入位置进行平滑处理
    first_order_filter_type_t input_set_Y_filter; // 使用一阶低通滤波器对输入位置进行平滑处理
    first_order_filter_type_t input_set_Z_filter; // 使用一阶低通滤波器对输入位置进行平滑处理
    first_order_filter_type_t input_set_striker_filter; // 使用一阶低通滤波器对输入位置进行平滑处理


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
    
    float striker_start_angle;
    float set_striker_angle;
    
}bat_control_t;



void bat_motor_Init(bat_control_t *bat_control);
void bat_motor_control(bat_control_t *bat_control);
void motor_pid_clac(bat_control_t *bat_control);
void top_RC_control_set(bat_control_t *bat_control);
void bat_data_update(bat_control_t *bat_control);
float float_constrain(float Value, float minValue, float maxValue);
void delta_arm_inverse_calculation(struct Angle *angle, float x, float y, float z);
void Forward_Kinematics(struct Point *Point, float theta1, float theta2, float theta3);

float CalDistance2Point(struct Point point1, struct Point point2);
struct Point GetPointInLine(struct Point currentP, struct Point desiredP, float t);


#endif /* BAT_CONTROL_H_ */
