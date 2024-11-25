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
#define gen_3 1.73205f
#define RofCenter 0.4067f // 轮子中心距

extern RC_ctrl_t rc_ctrl;
extern UART_HandleTypeDef huart1;
extern motor_measure_t motor_Date[8];
extern s_IMU_all_Value IMU_All_Value; // 储存imu数据结构体

chassis_control_t motor_control;
s_pid_absolute_t M3508_speed_pid;


#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

static void motor_init(chassis_control_t *init);
static void motor_control_send(chassis_control_t *control_loop);
static void chassis_feedback_update(chassis_control_t *feedback_update);
static void chassis_movement_calc(chassis_control_t *motor_control);
static void rc_to_motor_set(chassis_control_t *motor_control);
static void chassis_mode_set(chassis_control_t *chassis_control);
static int16_t rc_dead_zone(int16_t rc_data);
static void chassis_mode_switch(chassis_control_t *chassis_mode_change);

// static fp32 M6020_PID_calc(M6020_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
// static void M6020_PID_init(M6020_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
// static void M6020_motor_relative_angle_control(motor_6020_t *gimbal_motor);
// static void motor_feedback_update(chassis_control_t *feedback_update);
// static void M6020_PID_clear(M6020_PID_t *M6020_pid_clear);
//  主线程
void chassis_task(void const *argument)
{
    vTaskDelay(5000);

    motor_init(&motor_control);
    Di_Di();
    while (1)
    {
        // uart_dma_printf(&huart1,"%4.3f ,%1.1f ,%4.3f\n",
        // rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1], rc_ctrl.rc.ch[2]);
        // 选择进入控制模式
        chassis_mode_set(&motor_control);

        chassis_feedback_update(&motor_control);

        chassis_mode_switch(&motor_control);

        rc_to_motor_set(&motor_control);

        chassis_movement_calc(&motor_control);

        motor_control_send(&motor_control);

        osDelay(1);
    }
}

// 电机数据的初始化
static void motor_init(chassis_control_t *init)
{
    // 底盘速度环pid值

    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

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
        pid_abs_param_init(&init->chassis_motor.M3508[i].M3508_pid_speed, M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD, M3505_MOTOR_SPEED_PID_MAX_IOUT, M3505_MOTOR_SPEED_PID_MAX_OUT);
    }

    first_order_filter_init(&init->chassis_cmd_slow_set_vx, 0.002f, chassis_x_order_filter);
    first_order_filter_init(&init->chassis_cmd_slow_set_vy, 0.002f, chassis_y_order_filter);
    // 底盘速度数据初始化
    for (uint8_t i = 0; i < motor_3505_num; i++)
    {
        init->chassis_motor.M3508[i].motor_speed_set = init->chassis_motor.M3508[i].motor_speed;
    }

    init->chassis_yaw = *(init->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET); // 底盘姿态角度初始化
}


// 底盘运动模式设定
static void chassis_mode_set(chassis_control_t *chassis_control)
{
    if (chassis_control == NULL)
    {
        return;
    }

    // 手动模式
    if (chassis_control->chassis_RC->rc.s[0] == 1)
    {
        chassis_control->chassis_mode = MODE_ARTIFICAL;

        if (chassis_control->chassis_RC->rc.s[1] == 1)
        {
            chassis_control->control_mode_everone == ARTIFICAL_CHASSIS;
        }
        if (chassis_control->chassis_RC->rc.s[1] == 2)
        {
            chassis_control->control_mode_everone == ARTIFICAL_BAT;
        }
        if (chassis_control->chassis_RC->rc.s[1] == 0)
        {
            chassis_control->control_mode_everone == ARTIFICAL_STRIKER;
        }
    }

    else if (chassis_control->chassis_RC->rc.s[0] == 2)
    {
        chassis_control->chassis_mode = MODE_AUTO;
    }
    else if (chassis_control->chassis_RC->rc.s[0] == 3)
    {
        chassis_control->chassis_mode = MODE_STOP;
    }
}

// 底盘模式更变数据处理
static void chassis_mode_switch(chassis_control_t *chassis_mode_change)
{
    // 无输入，退出函数
    if (chassis_mode_change == NULL)
    {
        return;
    }
    // 模式无变更，退出函数
    if (chassis_mode_change->chassis_mode == chassis_mode_change->chassis_mode_last)
    {
        return;
    }

    // 手动转自动处理
    if (chassis_mode_change->chassis_mode_last == MODE_ARTIFICAL && chassis_mode_change->chassis_mode == MODE_AUTO)
    {
        /* code */
    }

    // 自动转手动处理
    if (chassis_mode_change->chassis_mode_last == MODE_AUTO && chassis_mode_change->chassis_mode == MODE_ARTIFICAL)
    {
        /* code */
    }

    chassis_mode_change->chassis_mode_last = chassis_mode_change->chassis_mode;
}

// 3508的pid计算

// 速度环pid

// 底盘的数据反馈
static void chassis_feedback_update(chassis_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    // 数据更新,各个电机的速度数据
    for (uint8_t i = 0; i < motor_3505_num; i++)
    {
        feedback_update->chassis_motor.M3508[i].motor_speed = feedback_update->chassis_motor.M3508[i].chassis_motor_measure->speed_rpm;
    }

    // 更新底盘纵向速度 x， 平移速度y，旋转速度wz，坐标系为右手系
    feedback_update->vx = feedback_update->chassis_motor.M3508[0].motor_speed * sqrt(3) / 2 - feedback_update->chassis_motor.M3508[1].motor_speed * sqrt(3) / 2;
    feedback_update->vy = feedback_update->chassis_motor.M3508[2].motor_speed - 0.5f * (feedback_update->chassis_motor.M3508[1].motor_speed + feedback_update->chassis_motor.M3508[1].motor_speed);
    feedback_update->wz = (feedback_update->chassis_motor.M3508[0].motor_speed + feedback_update->chassis_motor.M3508[1].motor_speed + feedback_update->chassis_motor.M3508[2].motor_speed) / 0.5f;

    // 计算底盘姿态角度, 底盘上有陀螺仪
    feedback_update->chassis_yaw = rad_format(*(feedback_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET)); // - chassis_move_update->chassis_yaw_motor->relative_angle);
}

// 电机主控制循环
static void motor_control_send(chassis_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
    // 计算所有6020电机pid

    // 计算所有3508电机pid
    for (uint8_t i = 0; i < motor_3505_num; i++)
    {
        motor_single_loop_PID(&control_loop->chassis_motor.M3508[i].M3508_pid_speed, control_loop->chassis_motor.M3508[i].motor_speed_set, control_loop->chassis_motor.M3508[i].motor_speed);
        motor_control.chassis_motor.M3508[i].given_current = control_loop->chassis_motor.M3508[i].M3508_pid_speed.PIDout;
        motor_control.chassis_motor.M3508[i].current_set = (int16_t)motor_control.chassis_motor.M3508[i].given_current;
    }

    // 发送给电机数据
    CAN_cmd_3508(motor_control.chassis_motor.M3508[0].given_current, motor_control.chassis_motor.M3508[1].given_current, motor_control.chassis_motor.M3508[2].given_current, 0x00);
}

// 遥控器数据转电机数据
static void rc_to_motor_set(chassis_control_t *motor_control)
{

    if (motor_control == NULL)
    {
        return;
    }

    // 手动模式的遥控器赋值
    if (motor_control->chassis_mode == MODE_ARTIFICAL)
    {

        motor_control->chassis_vx_ch = 0.0f;
        motor_control->chassis_vy_ch = 0.0f;
        motor_control->chassis_wz_ch = 0.0f;

        motor_control->chassis_vx_ch = rc_dead_zone(motor_control->chassis_RC->rc.ch[3]) * CHASSIS_VX_KP;
        motor_control->chassis_vy_ch = rc_dead_zone(-(motor_control->chassis_RC->rc.ch[2])) * CHASSIS_VY_KP;
        motor_control->chassis_wz_ch = rc_dead_zone(motor_control->chassis_RC->rc.ch[0]) * CHASSIS_WZ_KP;

        first_order_filter_cali(&motor_control->chassis_cmd_slow_set_vx, motor_control->chassis_vx_ch);
        first_order_filter_cali(&motor_control->chassis_cmd_slow_set_vy, motor_control->chassis_vy_ch);

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
static void chassis_movement_calc(chassis_control_t *chassis_control)
{
    // 此处为遥控器控制的方式
    if (chassis_control == NULL)
    {
        return;
    }

    fp32 angle_set = 0.0f;
    float motor_speed_calc[motor_3505_num] = {0.0f, 0.0f, 0.0f};

    if (chassis_control->chassis_mode == MODE_ARTIFICAL)
    {

        fp32 delat_angle = 0.0f;
        // set chassis yaw angle set-point
        angle_set = chassis_control->chassis_yaw;
        // 设置底盘控制的角度
        chassis_control->chassis_yaw_set = chassis_control->chassis_yaw;
        // 计算底盘角度差
        delat_angle = rad_format(chassis_control->chassis_yaw_set - chassis_control->chassis_yaw);
        // calculate rotation speed
        // 计算旋转的角速度,输入值与角度补正的和
        chassis_control->wz_set = chassis_control->chassis_wz_ch + motor_single_loop_PID(&chassis_control->chassis_pid_angle, 0.0f, delat_angle);
        // speed limit

        // 计算各个电机速度,三全向解算
        motor_speed_calc[0] = RofCenter * chassis_control->wz_set / 3 + chassis_control->vx_set * sqrt(3) / 3 - chassis_control->vy_set / 3;
        motor_speed_calc[1] = RofCenter * chassis_control->wz_set / 3 - chassis_control->vx_set * sqrt(3) / 3 - chassis_control->vy_set / 3;
        motor_speed_calc[2] = RofCenter * chassis_control->wz_set / 3 + chassis_control->vy_set * 2 / 3;

        for (uint8_t i = 0; i < motor_3505_num; i++)
        {
            chassis_control->chassis_motor.M3508[i].motor_speed_set = motor_speed_calc[i];
        }
    }

    if (chassis_control->chassis_mode == MODE_STOP)
    {
        chassis_control->chassis_yaw_set = rad_format(angle_set);
        fp32 delat_angle = 0.0f;
        // calculate rotation speed
        // 计算旋转的角速度
        chassis_control->wz_set = 0.0;
        chassis_control->vx_set = 0.0f;
        chassis_control->vy_set = 0.0f;

        for (uint8_t i = 0; i < motor_3505_num; i++)
        {
            chassis_control->chassis_motor.M3508[i].motor_speed_set = motor_speed_calc[i];
        }
    }

    // vofa调试用的代码
    // uart_dma_printf(&huart1,"%4.3d ,%4.3d\n",line_speed_x , line_speed_y);
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
