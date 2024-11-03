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
#define M3505_MOTOR_SPEED_PID_KP 12.0f
#define M3505_MOTOR_SPEED_PID_KI 0.0f
#define M3505_MOTOR_SPEED_PID_KD 3.4f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 15000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 14000.0f

#define CHASSIS_ANGLE_PID_KP 700.0f
#define CHASSIS_ANGLE_PID_KI 0.0f
#define CHASSIS_ANGLE_PID_KD 1.2f
#define CHASSIS_ANGLE_PID_MAX_OUT 12000.0f
#define CHASSIS_ANGLE_PID_MAX_IOUT 8000.0f


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
#define CHASSIS_VX_KP 12                 // x速度比例
#define CHASSIS_VY_KP 10                 // y速度比例
#define CHASSIS_WZ_KP 0.005f             // 角速度比例
#define CHASSIS_STOP_LINE 12             // 停止线位置


#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

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

    const motor_measure_t *chassis_motor_measure; // 电机数据结构体
    pid_type_def chassis_motor_gyro_pid;
    uint16_t offset_ecd;

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

typedef enum
{
    CHASSIS_MODE_ARTIFICAL,
    CHASSIS_MODE_AUTO,
    CHASSIS_MODE_STOP,
    
} chassis_mode_t;

typedef struct
{
    const RC_ctrl_t *chassis_RC;      // 底盘使用的遥控器指针, the point to remote control
    const fp32 *chassis_INS_angle;    // the point to the euler angle of gyro sensor.获取陀螺仪解算出的欧拉角指针
    chassis_motor_t chassis_motor;    // chassis motor data.底盘电机数据
    pid_type_def motor_speed_pid[4];  // motor speed PID.底盘电机速度pid
    pid_type_def chassis_angle_pid;   // follow angle PID.底盘跟随角度pid
    chassis_mode_t chassis_mode;      // chassis mode.底盘模式
    chassis_mode_t chassis_mode_last; // chassis mode set.底盘的上一次模式

    first_order_filter_type_t chassis_cmd_slow_set_vx; // use first order filter to slow set-point.使用一阶低通滤波减缓设定值
    first_order_filter_type_t chassis_cmd_slow_set_vy; // use first order filter to slow set-point.使用一阶低通滤波减缓设定值

    fp32 vx;     // chassis vertical speed, positive means forward,unit m/s. 底盘速度 前进方向 前为正，单位 m/s
    fp32 vy;     // chassis horizontal speed, positive means letf,unit m/s.底盘速度 左右方向 左为正  单位 m/s
    fp32 wz;     // chassis rotation speed, positive means counterclockwise,unit rad/s.底盘旋转角速度，逆时针为正 单位 rad/s
    fp32 vx_set; // chassis set vertical speed,positive means forward,unit m/s.底盘设定速度 前进方向 前为正，单位 m/s
    fp32 vy_set; // chassis set horizontal speed,positive means left,unit m/s.底盘设定速度 左右方向 左为正，单位 m/s
    fp32 wz_set; // chassis set rotation speed,positive means counterclockwise,unit rad/s.底盘设定旋转角速度，逆时针为正 单位 rad/s

    fp32 chassis_yaw_set;
    fp32 chassis_yaw;

    fp32 vx_max_speed; // max forward speed, unit m/s.前进方向最大速度 单位m/s
    fp32 vx_min_speed; // max backward speed, unit m/s.后退方向最大速度 单位m/s
    fp32 vy_max_speed; // max letf speed, unit m/s.左方向最大速度 单位m/s
    fp32 vy_min_speed; // max right speed, unit m/s.右方向最大速度 单位m/s

} chassis_control_t;

// extern void chasis_Task(void const * argument);

#endif
