#include "main.h"

#include "pid.h"
#include "chassis.h"
#include "can_recv.h"
#include "user_lib.h"
#include "remote_control.h"
#include "bsp_usart.h"
#include "bsp_buzzer.h"
#include "math.h"
#include "INS_task.h"
#include "Variables.h"
#include "Monitor.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"
/*对应电机数据,0~2 为1~3号电机3508

     3508（0） ----------------------  3508（1）
        \                               /
         \              ^x             /
          \             |             /
           \     y      |            /
            \    <------+           /
             \                     /
              \                   /
               \                 /
                \               /
                 \---3508(2)---/

*/

//
//
//        < S1 >                                             < S0 >
//
//
//
//
//
//
//            |                                              |
//            |                                              |
//            |                                              |
//            |                                              |
//   -------------------ch2                        -------------------ch0
//            |                                              |
//            |                                              |
//            |                                              |
//            |                                              |
//           ch3                                            ch1
//

#define gen_3 1.73205f
#define RofCenter 0.4067f // 轮子中心距

extern RC_ctrl_t rc_ctrl;
extern UART_HandleTypeDef huart1;
extern s_IMU_all_Value IMU_All_Value;       // 储存imu数据结构体
extern s_robo_Mode_Setting robot_StateMode; // 储存机器人当前模式
extern s_task_flags task_flags;             // 引用任务标志结构体
extern s_Dji_motor_data_t motor_Date[4];    // 储存电机数据结构体

extern chassis_control_t motor_control;
extern s_pid_absolute_t chassis_M3508_pid_speed;
extern s_pid_absolute_t chassis_M3508_pid_angle;

#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

static int16_t rc_dead_zone(int16_t rc_data);

// static fp32 M6020_PID_calc(M6020_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
// static void M6020_PID_init(M6020_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
// static void M6020_motor_relative_angle_control(motor_6020_t *gimbal_motor);
// static void motor_feedback_update(chassis_control_t *feedback_update);
// static void M6020_PID_clear(M6020_PID_t *M6020_pid_clear);

// 电机数据的初始化
void motor_init(chassis_control_t *init)
{
    // 底盘速度环pid值

    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    const static fp32 chassis_w_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    // 电机数据指针获取
    init->chassis_motor.M3508[0].chassis_motor_measure = get_3508_M1_motor_measure_point();
    init->chassis_motor.M3508[1].chassis_motor_measure = get_3508_M2_motor_measure_point();
    init->chassis_motor.M3508[2].chassis_motor_measure = get_3508_M3_motor_measure_point();

    // 遥控器数据指针获取
    init->chassis_RC = get_remote_control_point();

    // 陀螺仪数据获取
    init->chassis_INS_angle = get_INS_angle_point();

    for (uint8_t i = 0; i < motor_3505_num; i++)
    {
        pid_abs_param_init(&init->chassis_motor.M3508[i].M3508_pid_angle, M3505_MOTOR_ANGLE_PID_KP, M3505_MOTOR_ANGLE_PID_KI, M3505_MOTOR_ANGLE_PID_KD, M3505_MOTOR_ANGLE_PID_MAX_IOUT, M3505_MOTOR_ANGLE_PID_MAX_OUT);
        pid_abs_param_init(&init->chassis_motor.M3508[i].M3508_pid_speed, M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD, M3505_MOTOR_SPEED_PID_MAX_IOUT, M3505_MOTOR_SPEED_PID_MAX_OUT);
    }

    first_order_filter_init(&init->chassis_cmd_slow_set_vx, 0.002f, chassis_x_order_filter);
    first_order_filter_init(&init->chassis_cmd_slow_set_vy, 0.002f, chassis_y_order_filter);
    first_order_filter_init(&init->chassis_cmd_slow_set_vw, 0.002f, chassis_w_order_filter);

    // 底盘速度数据初始化
    for (uint8_t i = 0; i < motor_3505_num; i++)
    {
        init->chassis_motor.M3508[i].motor_speed_set = init->chassis_motor.M3508[i].motor_speed;
    }

    init->chassis_yaw = *(init->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET); // 底盘姿态角度初始化

    // 底盘初始化完成标志
    task_flags.chassis_Init_flag = 1;
}

// 3508的pid计算

// 速度环pid

// 底盘的数据反馈
void chassis_feedback_update(chassis_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    // 数据更新,各个电机的速度数据
    for (uint8_t i = 0; i < motor_3505_num; i++)
    {
        feedback_update->chassis_motor.M3508[i].motor_speed = feedback_update->chassis_motor.M3508[i].chassis_motor_measure->back_motor_speed;
        feedback_update->chassis_motor.M3508[i].serial_position = feedback_update->chassis_motor.M3508[i].chassis_motor_measure->serial_position;
    }

    // 更新底盘纵向速度 x， 平移速度y，旋转速度wz，坐标系为右手系
    feedback_update->vx = feedback_update->chassis_motor.M3508[0].motor_speed * sqrt(3) / 2 - feedback_update->chassis_motor.M3508[1].motor_speed * sqrt(3) / 2;
    feedback_update->vy = feedback_update->chassis_motor.M3508[2].motor_speed - 0.5f * (feedback_update->chassis_motor.M3508[1].motor_speed + feedback_update->chassis_motor.M3508[1].motor_speed);
    feedback_update->wz = (feedback_update->chassis_motor.M3508[0].motor_speed + feedback_update->chassis_motor.M3508[1].motor_speed + feedback_update->chassis_motor.M3508[2].motor_speed) / 0.5f;

    feedback_update->chassis_yaw_last = feedback_update->chassis_yaw;
    // 计算底盘姿态角度, 底盘上有陀螺仪
    feedback_update->chassis_yaw = rad_format(*(feedback_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET)); // - chassis_move_update->chassis_yaw_motor->relative_angle);
}

// 电机主控制循环
void motor_control_send(chassis_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
    if (robot_StateMode.roboMode == 3)
    {
        // 计算所有3508电机pid
        for (uint8_t i = 0; i < motor_3505_num; i++)
        {
            motor_single_loop_PID(&control_loop->chassis_motor.M3508[i].M3508_pid_speed, control_loop->chassis_motor.M3508[i].motor_speed_set, control_loop->chassis_motor.M3508[i].motor_speed);
            control_loop->chassis_motor.M3508[i].given_current = control_loop->chassis_motor.M3508[i].M3508_pid_speed.PIDout;
            control_loop->chassis_motor.M3508[i].current_set = (int16_t)motor_control.chassis_motor.M3508[i].given_current;
        }
    }
        // 非底盘模式进行制动
    if (robot_StateMode.roboMode != 3 && robot_StateMode.roboMode != 2)
    {
        for (uint8_t i = 0; i < motor_3505_num; i++)
        {
            control_loop->chassis_motor.M3508[i].M3508_pid_angle.NowError = 
            control_loop->chassis_motor.M3508[i].serial_position_set - control_loop->chassis_motor.M3508[i].serial_position;
            PID_AbsoluteMode(&control_loop->chassis_motor.M3508[i].M3508_pid_angle);
            control_loop->chassis_motor.M3508[i].M3508_pid_speed.NowError = 
            control_loop->chassis_motor.M3508[i].M3508_pid_angle.PIDout - control_loop->chassis_motor.M3508[i].motor_speed;
            PID_AbsoluteMode(&control_loop->chassis_motor.M3508[i].M3508_pid_speed);
            control_loop->chassis_motor.M3508[i].given_current = control_loop->chassis_motor.M3508[i].M3508_pid_speed.PIDout;
        }
    }

    // 为每个电机增加前馈
    // control_loop->chassis_motor.M3508[0].current_set += 700;
    // control_loop->chassis_motor.M3508[1].current_set += 700;
    // control_loop->chassis_motor.M3508[2].current_set += 700;
    //  发送给电机数据

    //此处出现的4号电机是
    CAN_cmd_3508(control_loop->chassis_motor.M3508[0].current_set, control_loop->chassis_motor.M3508[1].current_set, control_loop->chassis_motor.M3508[2].current_set, motor_Date[3].out_current);
}

// 遥控器数据转电机数据
void rc_to_motor_set(chassis_control_t *motor_control)
{

    if (motor_control == NULL)
    {
        return;
    }

    // 手动模式的底盘操作
    if (robot_StateMode.roboMode == 3)
    {

        motor_control->chassis_vx_ch = 0.0f;
        motor_control->chassis_vy_ch = 0.0f;
        motor_control->chassis_wz_ch = 0.0f;

        motor_control->chassis_vx_ch = rc_dead_zone(motor_control->chassis_RC->rc.ch[3]) * CHASSIS_VX_KP;
        motor_control->chassis_vy_ch = rc_dead_zone(-(motor_control->chassis_RC->rc.ch[2])) * CHASSIS_VY_KP;
        motor_control->chassis_wz_ch = rc_dead_zone(motor_control->chassis_RC->rc.ch[0]) * CHASSIS_WZ_KP;

        first_order_filter_cali(&motor_control->chassis_cmd_slow_set_vx, motor_control->chassis_vx_ch);
        first_order_filter_cali(&motor_control->chassis_cmd_slow_set_vy, motor_control->chassis_vy_ch);
        first_order_filter_cali(&motor_control->chassis_cmd_slow_set_vw, motor_control->chassis_wz_ch);

        // 停止区间
        if (motor_control->chassis_vx_ch < 12 * CHASSIS_VX_KP && motor_control->chassis_vx_ch < -12 * CHASSIS_VY_KP)
        {
            motor_control->chassis_vx_ch = 0.0f;
        }

        if (motor_control->chassis_vy_ch < 12 * CHASSIS_VX_KP && motor_control->chassis_vy_ch < -12 * CHASSIS_VY_KP)
        {
            motor_control->chassis_vy_ch = 0.0f;
        }

        motor_control->vx_set = motor_control->chassis_cmd_slow_set_vx.out;
        motor_control->vy_set = motor_control->chassis_cmd_slow_set_vy.out;
    }
}

// 运动计算
void chassis_movement_calc(chassis_control_t *chassis_control)
{
    // 此处为遥控器控制的方式
    if (chassis_control == NULL)
    {
        return;
    }

    fp32 angle_set = 0.0f;
    float motor_speed_calc[motor_3505_num] = {0.0f, 0.0f, 0.0f};
    fp32 delat_angle = 0.0f;
    // set chassis yaw angle set-point
    angle_set = chassis_control->chassis_yaw;
    // 设置底盘控制的角度
    chassis_control->chassis_yaw_set = chassis_control->chassis_yaw;
    // 计算底盘角度差
    delat_angle = chassis_control->chassis_yaw_set - chassis_control->chassis_yaw_last;

    if (robot_StateMode.roboMode == 3)
    {

        // calculate rotation speed
        // 计算旋转的角速度,输入值与角度补正的和
        chassis_control->wz_set = chassis_control->chassis_wz_ch - motor_single_loop_PID(&chassis_control->chassis_pid_angle, 0.0f, delat_angle);
        // speed limit

        // 这里换成串级pid，角度值内环，角速度未外环

        // 计算各个电机速度,三全向解算
        //        motor_speed_calc[0] = RofCenter * chassis_control->wz_set / 3 + chassis_control->vx_set * sqrt(3) / 3 - chassis_control->vy_set / 3;
        //        motor_speed_calc[1] = RofCenter * chassis_control->wz_set / 3 - chassis_control->vx_set * sqrt(3) / 3 - chassis_control->vy_set / 3;
        //        motor_speed_calc[2] = RofCenter * chassis_control->wz_set / 3 + chassis_control->vy_set * 2 / 3;

        // 计算各个电机速度,三全向解算
        motor_speed_calc[0] = RofCenter * chassis_control->wz_set + chassis_control->vx_set * sqrt(3) / 2 - chassis_control->vy_set / 2;
        motor_speed_calc[1] = RofCenter * chassis_control->wz_set - chassis_control->vx_set * sqrt(3) / 2 - chassis_control->vy_set / 2;
        motor_speed_calc[2] = RofCenter * chassis_control->wz_set + chassis_control->vy_set;

        for (uint8_t i = 0; i < motor_3505_num; i++)
        {
            chassis_control->chassis_motor.M3508[i].motor_speed_set = motor_speed_calc[i];
        }
        // 制动时保持位置使用的参数
        for (uint8_t i = 0; i < motor_3505_num; i++)
        {chassis_control->chassis_keep_position[i] = chassis_control->chassis_motor.M3508[i].serial_position;}    

    }

    else if (robot_StateMode.roboMode != 3)
    {

        // 非自动模式进行制动
        if (robot_StateMode.roboMode != 2)
        {
            chassis_control->chassis_motor.M3508[0].serial_position_set = chassis_control->chassis_keep_position[0];
            chassis_control->chassis_motor.M3508[1].serial_position_set = chassis_control->chassis_keep_position[1];
            chassis_control->chassis_motor.M3508[2].serial_position_set = chassis_control->chassis_keep_position[2];

/*             // 计算旋转的角速度
            chassis_control->wz_set -= motor_single_loop_PID(&chassis_control->chassis_pid_angle, 0.0f, delat_angle);
            chassis_control->vx_set = 0.0f;
            chassis_control->vy_set = 0.0f;

            // 计算各个电机速度,三全向解算
            motor_speed_calc[0] = RofCenter * chassis_control->wz_set + chassis_control->vx_set * sqrt(3) / 2 - chassis_control->vy_set / 2;
            motor_speed_calc[1] = RofCenter * chassis_control->wz_set - chassis_control->vx_set * sqrt(3) / 2 - chassis_control->vy_set / 2;
            motor_speed_calc[2] = RofCenter * chassis_control->wz_set + chassis_control->vy_set;

            for (uint8_t i = 0; i < motor_3505_num; i++)
            {
                chassis_control->chassis_motor.M3508[i].motor_speed_set = motor_speed_calc[i];
            } */
        }
    }
}

// 设定摇杆死区为10
static int16_t rc_dead_zone(int16_t rc_data)
{
    int16_t temp = rc_data;
    if (fabs(rc_data) < 10)
    {
        return 0;
    }
    else
        rc_data = temp;
    return rc_data;
}

// 自动模式运动计算
/*
不适用但有用的代码区

// 编码值转为弧度制
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    //if (relative_ecd > 4096)
    //{
    //   relative_ecd -= 8191;
    //}
    //else if (relative_ecd < -4096)
    //{
    //   relative_ecd += 8191;
    //}
    return relative_ecd * MOTOR_ECD_TO_RAD;
}

// 6020的PID初始化
static void M6020_PID_init(M6020_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

// 6020的PID计算
static fp32 M6020_PID_calc(M6020_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

// 清除6020的PID
static void M6020_PID_clear(M6020_PID_t *M6020_pid_clear)
{
    if (M6020_pid_clear == NULL)
    {
        return;
    }

    M6020_pid_clear->err = M6020_pid_clear->set = M6020_pid_clear->get = 0.0f;
    M6020_pid_clear->out = M6020_pid_clear->Pout = M6020_pid_clear->Iout = M6020_pid_clear->Dout = 0.0f;
}


// 6020的pid计算
static void M6020_motor_relative_angle_control(motor_6020_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    int flag = 1;
    // 角度环，速度环串级pid

    gimbal_motor->motor_gyro_set = M6020_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->serial_angle ,gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set); // 控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
*/
