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
#include "pc_interface.h"
#include "tracking.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"
/*对应电机数据,0~2 为1~3号电机3508

     3508（0） ----------------------  3508（1）
        \                               /
         \              ^y             /
          \             |             /
           \            |            /
            \           +----->y    /
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
extern s_tracking_data_t tracking_data;

extern chassis_control_t chassis_control;
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
void chassis_init(chassis_control_t *init)
{
    // 底盘速度环pid值
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    const static fp32 chassis_w_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    // 遥控器数据指针获取
    init->chassis_RC = get_remote_control_point();

    // 陀螺仪数据获取
    init->chassis_INS_angle = get_INS_angle_point();

    for (uint8_t i = 0; i < motor_3505_num; i++)
    {
        pid_abs_param_init(&init->M3508_pid_stop_angle[i], M3505_MOTOR_ANGLE_PID_KP, M3505_MOTOR_ANGLE_PID_KI, M3505_MOTOR_ANGLE_PID_KD, M3505_MOTOR_ANGLE_PID_MAX_IOUT, M3505_MOTOR_ANGLE_PID_MAX_OUT);
        pid_abs_param_init(&init->M3508_pid_stop_speed[i], M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD, M3505_MOTOR_SPEED_PID_MAX_IOUT, M3505_MOTOR_SPEED_PID_MAX_OUT);
        pid_abs_param_init(&init->M3508_pid_move_speed[i], M3505_MOTOR_MOVE_SPEED_PID_KP, M3505_MOTOR_MOVE_SPEED_PID_KI, M3505_MOTOR_MOVE_SPEED_PID_KD, M3505_MOTOR_MOVE_SPEED_PID_MAX_IOUT, M3505_MOTOR_MOVE_SPEED_PID_MAX_OUT);
    }
    pid_abs_param_init(&init->chassis_pid_anglespeed, CHASSIS_ANGLE_SPEED_PID_KP, CHASSIS_ANGLE_SPEED_PID_KI, CHASSIS_ANGLE_SPEED_PID_KD, CHASSIS_ANGLE_SPEED_PID_MAX_IOUT, CHASSIS_ANGLE_SPEED_PID_MAX_OUT);

    tracking_init(&tracking_data);

    first_order_filter_init(&init->chassis_cmd_slow_set_vx, 0.002f, chassis_x_order_filter);
    first_order_filter_init(&init->chassis_cmd_slow_set_vy, 0.002f, chassis_y_order_filter);
    first_order_filter_init(&init->chassis_cmd_slow_set_vw, 0.002f, chassis_w_order_filter);

    // 底盘速度位置数据初始化
    for (uint8_t i = 0; i < motor_3505_num; i++)
    {
        motor_Date[i].target_motor_speed = 0;
        motor_Date[i].target_motor_ang = 0;

        // init->chassis_motor.M3508[i].motor_speed_set = init->chassis_motor.M3508[i].motor_speed;
    }

    // 制动时保持位置使用的参数
    for (uint8_t i = 0; i < motor_3505_num; i++)
    {
        motor_Date[i].target_motor_ang = motor_Date[i].serial_motor_ang;
    }

    task_flags.hit_ball_chassis_stop_flag == 1;

    init->chassis_yaw = *(init->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET); // 底盘姿态角度初始化

    // 底盘初始化完成标志
    task_flags.chassis_Init_flag = 1;
}

// 3508的pid计算

// 速度环pid

// 底盘的数据反馈
/**
 * @brief 更新底盘反馈数据
 * @param feedback_update 底盘控制结构体指针，包含电机和姿态数据
 * @note 该函数用于:
 *       1. 计算底盘的三轴运动速度(vx,vy,wz)
 *       2. 更新底盘航向角数据
 *       3. 使用右手坐标系
 */
void chassis_feedback_update(chassis_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    // 数据更新,各个电机的速度数据
    // for (uint8_t i = 0; i < motor_3505_num; i++)
    // {
    //     feedback_update->chassis_motor.M3508[i].motor_speed = feedback_update->chassis_motor.M3508[i].chassis_motor_measure->back_motor_speed;
    //     feedback_update->chassis_motor.M3508[i].serial_position = feedback_update->chassis_motor.M3508[i].chassis_motor_measure->serial_position;
    // }

    // 更新底盘纵向速度 x， 平移速度y，旋转速度wz，坐标系为右手系
    feedback_update->vx = motor_Date[0].back_motor_speed * sqrt(3) / 2 - motor_Date[1].back_motor_speed * sqrt(3) / 2;
    feedback_update->vy = motor_Date[2].back_motor_speed - 0.5f * (motor_Date[1].back_motor_speed + motor_Date[1].back_motor_speed);
    feedback_update->wz = (motor_Date[0].back_motor_speed + motor_Date[1].back_motor_speed + motor_Date[1].back_motor_speed) / 0.5f;

    feedback_update->chassis_yaw_last = feedback_update->chassis_yaw;
    // 计算底盘姿态角度, 底盘上有陀螺仪
    feedback_update->chassis_yaw = rad_format(*(feedback_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET)); // - chassis_move_update->chassis_yaw_motor->relative_angle);
}

/**
 * @brief 计算底盘电机停止时的PID控制
 * @param control_loop 底盘控制结构体指针，包含PID参数和控制数据
 * @note 该函数对三个M3508电机进行串级PID控制，包括角度环和速度环
 *       外环为角度环，内环为速度环，最终输出电机控制电流值
 */
void chassis_stop_pid_calc(chassis_control_t *control_loop)
{
    // 角度环运算
    for (uint8_t i = 0; i < motor_3505_num; i++)
    {
        control_loop->M3508_pid_stop_angle[i].NowError =
            motor_Date[i].target_motor_ang - motor_Date[i].serial_motor_ang;
        PID_AbsoluteMode(&control_loop->M3508_pid_stop_angle[i]);
        control_loop->M3508_pid_stop_speed[i].NowError =
            control_loop->M3508_pid_stop_angle[i].PIDout - motor_Date[i].back_motor_speed;
        PID_AbsoluteMode(&control_loop->M3508_pid_stop_speed[i]);

        motor_Date[i].out_current = (int16_t)control_loop->M3508_pid_stop_speed[i].PIDout;
    }
}

/**
 * @brief 计算底盘电机速度PID控制
 * @param control_loop 底盘控制结构体指针
 * @note 对每个M3508电机进行速度环PID计算，根据目标速度和反馈速度计算输出电流值
 */
void chassis_speed_pid_calc(chassis_control_t *control_loop)
{
    // 速度环运算
    for (uint8_t i = 0; i < motor_3505_num; i++)
    {
        control_loop->M3508_pid_move_speed[i].NowError =
            motor_Date[i].target_motor_speed - motor_Date[i].back_motor_speed;
        PID_AbsoluteMode(&control_loop->M3508_pid_move_speed[i]);

        motor_Date[i].out_current = (int16_t)control_loop->M3508_pid_move_speed[i].PIDout;
    }
}

// 电机主控制循环
/**
 * @brief 发送电机控制指令，处理底盘电机的PID控制和电流输出
 * @param control_loop 底盘控制结构体指针，包含电机控制参数
 * @note 根据机器人状态模式(roboMode)执行不同的控制逻辑:
 *       - 模式3: 执行速度环PID控制
 *       - 非模式2/3: 执行位置环制动控制
 *       最后通过CAN总线发送控制指令给电机
 */
void motor_control_send(chassis_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
    // 此处出现的4号电机是击球电机
    CAN_cmd_3508(motor_Date[0].out_current, motor_Date[1].out_current, motor_Date[2].out_current, motor_Date[3].out_current);
}

// 遥控器数据转电机数据
void rc_to_motor_set(chassis_control_t *chassis_control)
{

    if (chassis_control == NULL)
    {
        return;
    }

    // 手动模式的底盘操作
    if (robot_StateMode.roboMode == 3)
    {

        chassis_control->chassis_vx_ch = 0.0f;
        chassis_control->chassis_vy_ch = 0.0f;
        chassis_control->chassis_wz_ch = 0.0f;

        chassis_control->chassis_vx_ch = rc_dead_zone(chassis_control->chassis_RC->rc.ch[3]) * CHASSIS_VX_KP;
        chassis_control->chassis_vy_ch = rc_dead_zone(-(chassis_control->chassis_RC->rc.ch[2])) * CHASSIS_VY_KP;
        chassis_control->chassis_wz_ch = rc_dead_zone(chassis_control->chassis_RC->rc.ch[0]) * CHASSIS_WZ_KP;

        first_order_filter_cali(&chassis_control->chassis_cmd_slow_set_vx, chassis_control->chassis_vx_ch);
        first_order_filter_cali(&chassis_control->chassis_cmd_slow_set_vy, chassis_control->chassis_vy_ch);
        first_order_filter_cali(&chassis_control->chassis_cmd_slow_set_vw, chassis_control->chassis_wz_ch);

        // 停止区间
        if (chassis_control->chassis_vx_ch < 12 * CHASSIS_VX_KP && chassis_control->chassis_vx_ch < -12 * CHASSIS_VY_KP)
        {
            chassis_control->chassis_vx_ch = 0.0f;
        }

        if (chassis_control->chassis_vy_ch < 12 * CHASSIS_VX_KP && chassis_control->chassis_vy_ch < -12 * CHASSIS_VY_KP)
        {
            chassis_control->chassis_vy_ch = 0.0f;
        }

        chassis_control->vx_set = chassis_control->chassis_cmd_slow_set_vx.out;
        chassis_control->vy_set = chassis_control->chassis_cmd_slow_set_vy.out;
    }
}

// 运动计算
void chassis_movement_calc(chassis_control_t *chassis_control)
{
    float stop_angle[3] = 0.0f;
    // 此处为遥控器控制的方式
    if (chassis_control == NULL)
    {
        return;
    }
    for (uint8_t i = 0; i < motor_3505_num; i++)
    {
        stop_angle[i] = motor_Date[i].serial_motor_ang;
    }

    fp32 angle_set = 0.0f;
    float motor_speed_calc[motor_3505_num] = {0.0f, 0.0f, 0.0f};
    fp32 delat_angle = 0.0f;

    chassis_control->chassis_pid_anglespeed.NowError = 0 - IMU_All_Value.yaw.yawAngV;
    PID_AbsoluteMode(&chassis_control->chassis_pid_anglespeed);
    chassis_control->wz_set = chassis_control->chassis_cmd_slow_set_vw.out + chassis_control->chassis_pid_anglespeed.PIDout;

    ////////////////////////////////////////////////////////////////////////////////
    if (robot_StateMode.roboMode == ARTIFICAL_CHASSIS)
    {
        // calculate rotation speed
        // 计算旋转的角速度,输入值与角度补正的和
        // speed limit

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
            motor_Date[i].target_motor_speed = motor_speed_calc[i];
        }

        chassis_speed_pid_calc(chassis_control);
        CAN_cmd_3508(motor_Date[0].out_current, motor_Date[1].out_current, motor_Date[2].out_current, motor_Date[3].out_current);
        task_flags.hit_ball_chassis_stop_flag == 1;

        // 制动时保持位置使用的参数
        for (uint8_t i = 0; i < motor_3505_num; i++)
        {
            motor_Date[i].target_motor_ang = motor_Date[i].serial_motor_ang;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    else if (robot_StateMode.roboMode == ARTIFICAL_STRIKER)
    {

    }

    else if (robot_StateMode.roboMode == AUTO_RECEIVE_BALL)
    {
        chassis_volleyball_track();
    }
    ////////////////////////////////////////////////////////////////////////////////////
    else if (robot_StateMode.roboMode == MODE_STOP || robot_StateMode.roboMode == ARTIFICAL_BAT)
    {

        // 非自动模式进行制动
        task_flags.hit_ball_chassis_stop_flag == 1;
        chassis_stop_pid_calc(chassis_control);
        CAN_cmd_3508(motor_Date[0].out_current, motor_Date[1].out_current, motor_Date[2].out_current, motor_Date[3].out_current);
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

/**
 * @brief 执行机器人底盘发球动作控制
 * @param chassis_control 底盘控制结构体指针
 * @details 该函数实现了底盘发球时的运动控制:等待球拍任务执行完毕后进行控制
 *          
 */
void ACTION_chassis_serve_a_ball(chassis_control_t *chassis_control)
{
    // 进行发球运动动作,由球拍发球位置决定
    if (task_flags.hit_ball_chassis_move_flag == 1)
    {
        float start_angle[3] = 0.0f;
        // 记录初始位置
        for (uint8_t i = 0; i < motor_3505_num; i++)
        {
            start_angle[i] = motor_Date[i].serial_motor_ang;
        }
        float nowspeed = 0.0f;
        float MAXspeed = 6000.0f;

        float target_distance_x = 10.0f;
        float current_distance_x = 0.0f; // 当前位置与目标位置的差值
        while (current_distance_x < target_distance_x)
        {
            float motor_speed_calc[3] = {0};
            if (robot_StateMode.roboMode != ARTIFICAL_STRIKER)
            {
                break;
            }

            if (nowspeed < MAXspeed)
            {
                nowspeed += 50;
            }

            chassis_control->chassis_pid_anglespeed.NowError = 0 - IMU_All_Value.yaw.yawAngV;
            PID_AbsoluteMode(&chassis_control->chassis_pid_anglespeed);
            chassis_control->wz_set = chassis_control->chassis_cmd_slow_set_vw.out + chassis_control->chassis_pid_anglespeed.PIDout;
            chassis_control->vx_set = nowspeed;

            motor_speed_calc[0] = RofCenter * chassis_control->wz_set + chassis_control->vx_set * sqrt(3) / 2 - chassis_control->vy_set / 2;
            motor_speed_calc[1] = RofCenter * chassis_control->wz_set - chassis_control->vx_set * sqrt(3) / 2 - chassis_control->vy_set / 2;
            motor_speed_calc[2] = RofCenter * chassis_control->wz_set + chassis_control->vy_set;

            for (uint8_t i = 0; i < motor_3505_num; i++)
            {
                motor_Date[i].target_motor_speed = motor_speed_calc[i];
            }

            current_distance_x = 0.2067731f * ((motor_Date[0].serial_motor_ang - start_angle[0]) / 360 - (motor_Date[1].serial_motor_ang - start_angle[1]) / 360);

            chassis_speed_pid_calc(chassis_control);
            CAN_cmd_3508(motor_Date[0].out_current, motor_Date[1].out_current, motor_Date[2].out_current, motor_Date[3].out_current);
            osDelay(1);
        }

        task_flags.hit_ball_chassis_stop_flag = 1;
        task_flags.hit_ball_chassis_move_flag = 0;
        // for (uint8_t i = 0; i < motor_3505_num; i++)
        // {
        //     motor_Date[i].target_motor_ang = motor_Date[i].serial_motor_ang;
        // }
        // motor_Date[3].target_motor_ang = motor_Date[3].serial_motor_ang + 360.0f * 19;
    }
    else
    {

        chassis_stop_pid_calc(chassis_control);
        CAN_cmd_3508(motor_Date[0].out_current, motor_Date[1].out_current, motor_Date[2].out_current, motor_Date[3].out_current);
    }
    for (uint8_t i = 0; i < motor_3505_num; i++)
    {
        motor_Date[i].target_motor_ang = motor_Date[i].serial_motor_ang;
    }
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
