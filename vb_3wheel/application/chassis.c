
// ██╗    ██╗  ██╗  ██╗ ███████╗ ███████╗ ██╗
// ██║    ██║  ██║  ██║ ██╔════╝ ██╔════╝ ██║
// ██║ ██╗ ██║ ███████║ ██████╗  ██████╗  ██║
// ██║ ╚██╗██║ ██╔══██║ ██╔══╝   ██╔══╝   ██║
// ╚██╗╚████║  ██║  ██║ ███████╗ ███████╗ ███████╗
//  ╚═╝ ╚═══╝  ╚═╝  ╚═╝ ╚══════╝ ╚══════╝ ╚══════╝

/*
 *   █████████████╗   ███████████╗     ████████████╗
 *   ╚════██╔═════╝  ██╔════════██╗    ██╔═════════╝
 *        ██║        ██║        ██║    ██║
 *        ██║        ██║        ██║    ██║
 *        ██║        ██║        ██║    ██║█████████╗
 *        ██║        ██║        ██║    ██╔═════════╝
 *        ██║        ██║        ██║    ██║
 *        ██║        ██║        ██║    ██║
 *        ██║        ╚███████████╔╝    ████████████╗
 *        ╚═╝         ╚══════════╝     ╚═══════════╝
 *  Version    Date            Author          Modification
 *  V1.0.0     2024            night8858
 */

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
#include "sensor.h"

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

extern volatile uint8_t imu_flag;
extern int ops9_flag;

extern RC_ctrl_t rc_ctrl;
extern UART_HandleTypeDef huart1;
extern s_IMU_all_Value IMU_All_Value;       // 储存imu数据结构体
extern s_robo_Mode_Setting robot_StateMode; // 储存机器人当前模式
extern s_task_flags task_flags;             // 引用任务标志结构体
extern s_Dji_motor_data_t motor_Date[6];    // RM电机回传数据结构体
extern s_tracking_data_t tracking_data;
extern s_ops9_data_t ops9_data; // ops9数据结构体

extern chassis_control_t chassis_control;
extern s_pid_absolute_t chassis_M3508_pid_speed;
extern s_pid_absolute_t chassis_M3508_pid_angle;

extern serve_ball_t serve_state; // 发球状态机

s_pid_absolute_t chassis_serve_pos_pid;

static int chassis_stop_flag = 0; // 底盘停止标志位
static uint8_t ops9_clear_flag = 0;

static int16_t rc_dead_zone(int16_t rc_data);
static void ops9_data_clear_check(void);
static void chassis_keep_straight(int ops9_flag);

static void ACTION_chassis_recive_ball(chassis_control_t *chassis_control);
static void ACTION_chassis_serve_a_ball(chassis_control_t *chassis_control);
static void ACTION_keep_ball_in_center(chassis_control_t *chassis_control);
static void ACTION_chassis_stop(chassis_control_t *chassis_control);

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
    pid_abs_param_init(&chassis_serve_pos_pid, 6, 0, 0, 200, 4000); ////发球追踪的pid

    tracking_init(&tracking_data);

    first_order_filter_init(&init->chassis_cmd_slow_set_vx, 0.008f, chassis_x_order_filter);
    first_order_filter_init(&init->chassis_cmd_slow_set_vy, 0.008f, chassis_y_order_filter);
    first_order_filter_init(&init->chassis_cmd_slow_set_vw, 0.008f, chassis_w_order_filter);

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
    // 设定底盘角度不变
    chassis_control.chassis_yaw_set = 0;
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

        chassis_control->chassis_vx_ch = rc_dead_zone(chassis_control->chassis_RC->rc.ch[3]) * 12;
        chassis_control->chassis_vy_ch = rc_dead_zone(-(chassis_control->chassis_RC->rc.ch[2])) * 12;
        chassis_control->chassis_wz_ch = rc_dead_zone(chassis_control->chassis_RC->rc.ch[0]) * 10;

        first_order_filter_cali(&chassis_control->chassis_cmd_slow_set_vx, chassis_control->chassis_vx_ch);
        first_order_filter_cali(&chassis_control->chassis_cmd_slow_set_vy, chassis_control->chassis_vy_ch);
        first_order_filter_cali(&chassis_control->chassis_cmd_slow_set_vw, chassis_control->chassis_wz_ch);

        // 停止区间
        if (chassis_control->chassis_vx_ch < 12 * 12 && chassis_control->chassis_vx_ch < -12 * 12)
        {
            chassis_control->chassis_vx_ch = 0.0f;
        }

        if (chassis_control->chassis_vy_ch < 12 * 12 && chassis_control->chassis_vy_ch < -12 * 12)
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
    // 此处为遥控器控制的方式
    if (chassis_control == NULL)
    {
        return;
    }

    if (chassis_stop_flag == 0)
    {
        for (uint8_t i = 0; i < motor_3505_num; i++)
        {
            chassis_control->stop_angle[i] = motor_Date[i].serial_motor_ang;
        }
    }

    float motor_speed_calc[motor_3505_num] = {0.0f, 0.0f, 0.0f};
    ops9_data_clear_check();          // ops9有数据后给yaw角度赋值为0，设为初始位置
    chassis_keep_straight(ops9_flag); // 保持底盘水平
    ////////////////////////////////////////////////////////////////////////////////
    if (robot_StateMode.roboMode == ARTIFICAL_CHASSIS)
    {
        // 计算各个电机速度,三全向解算
        //        motor_speed_calc[0] = RofCenter * chassis_control->wz_set / 3 + chassis_control->vx_set * sqrt(3) / 3 - chassis_control->vy_set / 3;
        //        motor_speed_calc[1] = RofCenter * chassis_control->wz_set / 3 - chassis_control->vx_set * sqrt(3) / 3 - chassis_control->vy_set / 3;
        //        motor_speed_calc[2] = RofCenter * chassis_control->wz_set / 3 + chassis_control->vy_set * 2 / 3;
        chassis_stop_flag = 0;
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
        // task_flags.hit_ball_chassis_stop_flag == 1;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    else if (robot_StateMode.roboMode == ARTIFICAL_STRIKER)
    {
        ACTION_chassis_serve_a_ball(chassis_control);
    }

    else if (robot_StateMode.roboMode == AUTO_RECEIVE_BALL)
    {
        for (uint8_t i = 0; i < motor_3505_num; i++)
        {
            motor_Date[i].target_motor_speed = motor_speed_calc[i];
        }
        ACTION_chassis_recive_ball(chassis_control);
    }

    else if (robot_StateMode.roboMode == ROBOT_DEBUG)
    {
        ACTION_keep_ball_in_center(chassis_control);
    }

    ////////////////////////////////////////////////////////////////////////////////////
    else if (robot_StateMode.roboMode == MODE_STOP || robot_StateMode.roboMode == ARTIFICAL_BAT)
    {
        // 非自动模式进行制动
        // chassis_control->chassis_yaw_set = IMU_All_Value.yaw.yawAng;
        task_flags.hit_ball_chassis_stop_flag == 1;
        ACTION_chassis_stop(chassis_control);
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
 * @brief 底盘保持直线运动的控制函数
 *
 * @param ops9_flag ops9标志位，用于判断是否启动直线控制
 *
 * @details
 * 当ops9_flag为1时，启动底盘直线控制：
 * 1. 如果有旋转指令(chassis_wz_ch不为0)，更新目标航向角
 * 2. 计算当前航向角误差并进行PID控制
 * 3. 最终输出角速度为手动控制量与PID修正量之和
 */
void chassis_keep_straight(int ops9_flag)
{
    if (ops9_flag == 1) // ops9启动后
    {
        if (chassis_control.chassis_wz_ch != 0)
        {
            chassis_control.chassis_yaw_set = ops9_data.yaw;
        }
        // chassis_control.chassis_yaw_last = ops9_data.yaw;
        chassis_control.chassis_pid_anglespeed.NowError = chassis_control.chassis_yaw_set - ops9_data.yaw;
        PID_AbsoluteMode(&chassis_control.chassis_pid_anglespeed);
    }

    chassis_control.wz_set = chassis_control.chassis_wz_ch - chassis_control.chassis_pid_anglespeed.PIDout;
}

void ops9_data_clear_check(void)
{
    if (ops9_clear_flag == 1)
    {
        return;
    }

    if (ops9_flag == 1)
    {
        ops9_Zero_Clearing();
        ops9_clear_flag = 1;
    }
}

/**
 * @brief 执行机器人底盘发球动作控制
 * @param chassis_control 底盘控制结构体指针
 * @details 该函数实现了底盘发球时的运动控制:等待球拍任务执行完毕后进行控制
 *
 */
void ACTION_chassis_serve_a_ball(chassis_control_t *chassis_control)
{
    if (serve_state > 0 && serve_state < 5)
    {
        ACTION_keep_ball_in_center(chassis_control);
    }
    else
    {
        ACTION_chassis_stop(chassis_control);
    }
}

void ACTION_chassis_recive_ball(chassis_control_t *chassis_control)
{
    chassis_stop_flag = 0;   
    chassis_volleyball_track();
    chassis_speed_pid_calc(chassis_control);
    CAN_cmd_3508(motor_Date[0].out_current, motor_Date[1].out_current, motor_Date[2].out_current, motor_Date[3].out_current);
}

void ACTION_keep_ball_in_center(chassis_control_t *chassis_control)
{
    chassis_stop_flag = 0;
    keep_ball_in_center_track();
    chassis_speed_pid_calc(chassis_control);
    CAN_cmd_3508(motor_Date[0].out_current, motor_Date[1].out_current, motor_Date[2].out_current, motor_Date[3].out_current);
}

void ACTION_chassis_stop(chassis_control_t *chassis_control)
{
    chassis_stop_flag = 1;
    for (size_t i = 0; i < 3; i++)
    {
        motor_Date[i].target_motor_ang = chassis_control->stop_angle[i];
    }

    chassis_stop_pid_calc(chassis_control);
    CAN_cmd_3508(motor_Date[0].out_current, motor_Date[1].out_current, motor_Date[2].out_current, motor_Date[3].out_current);
}
// 自动模式运动计算

// 似乎击球后球拍控制有问题了，得大改
