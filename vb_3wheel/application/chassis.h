#ifndef CHASSIS_H
#define CHASSIS_H

#include "struct_typedef.h"
#include "can_recv.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"


#define PI 3.141592653824f

// 3508电机的挂载数
#define motor_3505_num 3

// PID宏定义参数
#define M3505_MOTOR_MOVE_SPEED_PID_KP 16.0f
#define M3505_MOTOR_MOVE_SPEED_PID_KI 1.3f
#define M3505_MOTOR_MOVE_SPEED_PID_KD 0.0f
#define M3505_MOTOR_MOVE_SPEED_PID_MAX_OUT 14000.0f
#define M3505_MOTOR_MOVE_SPEED_PID_MAX_IOUT 4000.0f

#define M3505_MOTOR_SPEED_PID_KP 2.5f
#define M3505_MOTOR_SPEED_PID_KI 0.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 14000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 4000.0f

#define M3505_MOTOR_ANGLE_PID_KP 17.0f
#define M3505_MOTOR_ANGLE_PID_KI 0.0f
#define M3505_MOTOR_ANGLE_PID_KD 3.7f
#define M3505_MOTOR_ANGLE_PID_MAX_OUT 14000.0f
#define M3505_MOTOR_ANGLE_PID_MAX_IOUT 4000.0f

// #define CHASSIS_ANGLE_PID_KP -0.0f
// #define CHASSIS_ANGLE_PID_KI 0.0f
// #define CHASSIS_ANGLE_PID_KD 45.0f
// #define CHASSIS_ANGLE_PID_MAX_OUT 12000.0f
// #define CHASSIS_ANGLE_PID_MAX_IOUT 8000.0f

#define CHASSIS_ANGLE_SPEED_PID_KP 390.0f
#define CHASSIS_ANGLE_SPEED_PID_KI 0.6f
#define CHASSIS_ANGLE_SPEED_PID_KD 0.0f
#define CHASSIS_ANGLE_SPEED_PID_MAX_OUT 2000.f
#define CHASSIS_ANGLE_SPEED_PID_MAX_IOUT 0.4f

#define M6020_MOTOR_SPEED_PID_KP 2.0f
#define M6020_MOTOR_SPEED_PID_KI 0.0f
#define M6020_MOTOR_SPEED_PID_KD 1.2f
#define M6020_MOTOR_SPEED_PID_MAX_OUT 30000.0f
#define M6020_MOTOR_SPEED_PID_MAX_IOUT 20000.0f

#define M6020_MOTOR_POSION_PID_KP 25000.0f
#define M6020_MOTOR_POSION_PID_KI 0.0f
#define M6020_MOTOR_POSION_PID_KD 7.0f
#define M6020_MOTOR_POSION_PID_MAX_OUT 30000.0f
#define M6020_MOTOR_POSION_PID_MAX_IOUT 20000.0f

// 控制参数宏定义
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#define CHASSIS_VX_KP 16                 // x速度比例
#define CHASSIS_VY_KP 6                 // y速度比例
#define CHASSIS_WZ_KP 30            // 角速度比例
#define CHASSIS_STOP_LINE 12             // 停止线位置


#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.1666666667f

typedef struct
{
    float kp;
    float ki;
    float kd;

    float set;
    float get;
    float err;

    float max_out;
    float max_iout;

    float Pout;
    float Iout;
    float Dout;

    float out;

} M6020_PID_t;

typedef struct
{
    float kp;
    float ki;
    float kd;

    float set;
    float get;
    float err;

    float max_out;
    float max_iout;

    float Pout;
    float Iout;
    float Dout;

    float out;

} M3508_PID_t;

typedef struct
{

    const motor_measure_t *gimbal_motor_measure; // 电机数据结构体
    M6020_PID_t gimbal_motor_relative_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;
    uint16_t offset_ecd;
    float max_relative_angle; // rad
    float min_relative_angle; // rad

    float relative_angle;     // rad
    float relative_angle_set; // rad

    float motor_gyro; // rad/s
    float motor_gyro_set;
    float motor_speed;
    float raw_cmd_current;
    float current_set;
    int16_t given_current;

    int16_t last_angle;
    int circle_num;
    float serial_angle;
    float position_angle;

} motor_6020_t;

typedef struct
{

    const s_Dji_motor_data_t *chassis_motor_measure; // 电机数据结构体
    s_pid_absolute_t M3508_pid_speed;
    s_pid_absolute_t M3508_pid_angle;  
    uint16_t offset_ecd;

    int64_t serial_position;//电机连续编码值（刻度）
    int64_t serial_position_set; // 电机连续编码值设定值

    float motor_speed;
    float motor_speed_set;
    float raw_cmd_current;
    float current_set;
    float accel;

    int16_t given_current;

} motor_3508_t;

typedef struct
{

    motor_3508_t M3508[motor_3505_num];

} chassis_motor_t;


typedef struct
{
    const RC_ctrl_t *chassis_RC;      // 底盘使用的遥控器指针, the point to remote control
    const fp32 *chassis_INS_angle;    // the point to the euler angle of gyro sensor.获取陀螺仪解算出的欧拉角指针
    // chassis_motor_t chassis_motor;    // chassis motor data.底盘电机数据

    s_pid_absolute_t chassis_pid_anglespeed;   //底盘角度pid

    s_pid_absolute_t M3508_pid_stop_speed[3];
    s_pid_absolute_t M3508_pid_stop_angle[3];
    
    s_pid_absolute_t M3508_pid_move_speed[3];

    first_order_filter_type_t chassis_cmd_slow_set_vx; // use first order filter to slow set-point.使用一阶低通滤波减缓设定值
    first_order_filter_type_t chassis_cmd_slow_set_vy; // use first order filter to slow set-point.使用一阶低通滤波减缓设定值
    first_order_filter_type_t chassis_cmd_slow_set_vw;
    fp32 vx;     // chassis vertical speed, positive means forward,unit m/s. 底盘速度 前进方向 前为正，单位 m/s
    fp32 vy;     // chassis horizontal speed, positive means letf,unit m/s.底盘速度 左右方向 左为正  单位 m/s
    fp32 wz;     // chassis rotation speed, positive means counterclockwise,unit rad/s.底盘旋转角速度，逆时针为正 单位 rad/s
    fp32 vx_set; // chassis set vertical speed,positive means forward,unit m/s.底盘设定速度 前进方向 前为正，单位 m/s
    fp32 vy_set; // chassis set horizontal speed,positive means left,unit m/s.底盘设定速度 左右方向 左为正，单位 m/s
    fp32 wz_set; // chassis set rotation speed,positive means counterclockwise,unit rad/s.底盘设定旋转角速度，逆时针为正 单位 rad/s

    int16_t chassis_vx_ch;
    int16_t chassis_vy_ch;
    int16_t chassis_wz_ch;

    float ops9_pos_y_set;
    float ops9_pos_x_set;
    
    float chassis_forward_postion; // chassis forward speed, unit m/s.底盘前进距离 单位m
    float chassis_shift_postion; // chassis forward speed, unit m/s.底盘平移距离 单位m
    
    int64_t chassis_keep_position[3]; // 底盘连续编码值（刻度）

    fp32 chassis_yaw_set;
    fp32 chassis_yaw;
    fp32 chassis_yaw_last;

    fp32 vx_max_speed; // max forward speed, unit m/s.前进方向最大速度 单位m/s
    fp32 vx_min_speed; // max backward speed, unit m/s.后退方向最大速度 单位m/s
    fp32 vy_max_speed; // max letf speed, unit m/s.左方向最大速度 单位m/s
    fp32 vy_min_speed; // max right speed, unit m/s.右方向最大速度 单位m/s

    float chassis_start_ang[3];
    float stop_angle[3];

} chassis_control_t;

// extern void chasis_Task(void const * argument);

void chassis_init(chassis_control_t *init);
void motor_control_send(chassis_control_t *control_loop);
void chassis_feedback_update(chassis_control_t *feedback_update);
void chassis_movement_calc(chassis_control_t *motor_control);
void rc_to_motor_set(chassis_control_t *motor_control);
void chassis_stop_pid_calc(chassis_control_t *control_loop);
void chassis_speed_pid_calc(chassis_control_t *control_loop);


#endif
