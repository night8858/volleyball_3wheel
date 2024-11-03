#include "main.h"

#include "pid.h"
#include "chassis.h"
#include "can_recv.h"
#include "user_lib.h"
#include "remote_control.h"
#include "bsp_usart.h"
#include "math.h"
#include "INS_task.h"

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

chassis_control_t motor_control;

#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

static void motor_init(chassis_control_t *init);

static void M3508_motor_speed_control(motor_3508_t *chassis_motor);
static void motor_control_send(chassis_control_t *control_loop);
static void chassis_feedback_update(chassis_control_t *feedback_update);
static void chassis_movement_calc(chassis_control_t *motor_control);
static void rc_to_motor_set(chassis_control_t *motor_control);
static void chassis_wheel_speed_control(chassis_control_t *chassis_control);
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
    vTaskDelay(2000);

    motor_init(&motor_control);
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

        chassis_wheel_speed_control(&motor_control);

        motor_control_send(&motor_control);
        osDelay(1);
    }
}

// 电机数据的初始化
static void motor_init(chassis_control_t *init)
{
    // 底盘速度环pid值

    // chassis angle PID
    // 底盘角度pid值
    static const fp32 M3508_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    static const fp32 chassiss_angle_pid[3] = {CHASSIS_ANGLE_PID_KP, CHASSIS_ANGLE_PID_KI, CHASSIS_ANGLE_PID_KD};

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
        PID_clear(&init->chassis_motor.M3508[i].chassis_motor_gyro_pid);
    }

    for (uint8_t i = 0; i < motor_3505_num; i++)
    {
        PID_init(&init->chassis_motor.M3508[i].chassis_motor_gyro_pid, PID_POSITION, M3508_speed_pid, M3505_MOTOR_SPEED_PID_MAX_IOUT, M3505_MOTOR_SPEED_PID_MAX_OUT);
    }
    PID_init(&init->chassis_angle_pid, PID_POSITION, chassiss_angle_pid, CHASSIS_ANGLE_PID_MAX_IOUT, CHASSIS_ANGLE_PID_MAX_OUT);
    // 清除所有PID

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

    if (chassis_control->chassis_RC->rc.s[0] == 1)
    {
        chassis_control->chassis_mode = CHASSIS_MODE_ARTIFICAL;
    }
    else if (chassis_control->chassis_RC->rc.s[0] == 2)
    {
        chassis_control->chassis_mode = CHASSIS_MODE_AUTO;
    }
    else if (chassis_control->chassis_RC->rc.s[0] == 3)
    {
        chassis_control->chassis_mode = CHASSIS_MODE_STOP;
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
    if (chassis_mode_change->chassis_mode_last == CHASSIS_MODE_ARTIFICAL && chassis_mode_change->chassis_mode == CHASSIS_MODE_AUTO)
    {
        /* code */
    }

    // 自动转手动处理
    if (chassis_mode_change->chassis_mode_last == CHASSIS_MODE_AUTO && chassis_mode_change->chassis_mode == CHASSIS_MODE_ARTIFICAL)
    {
        /* code */
    }

    chassis_mode_change->chassis_mode_last = chassis_mode_change->chassis_mode;
}

// 3508的pid计算
static void M3508_motor_speed_control(motor_3508_t *chassis_motor)
{
    if (chassis_motor == NULL)
    {
        return;
    }

    // 速度环pid
    chassis_motor->current_set = PID_calc(&chassis_motor->chassis_motor_gyro_pid, chassis_motor->motor_speed, chassis_motor->motor_speed_set); // 控制值赋值
    chassis_motor->given_current = (int16_t)(chassis_motor->current_set);
}

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

    // calculate chassis euler angle, if chassis add a new gyro sensor,please change this code
    // 计算底盘姿态角度, 如果底盘上有陀螺仪请更改这部分代码
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
        M3508_motor_speed_control(&control_loop->chassis_motor.M3508[i]);
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
    motor_control->vx_set = 0.0f;
    motor_control->vy_set = 0.0f;
    motor_control->wz_set = 0.0f;

        motor_control->vx_set = rc_dead_zone(motor_control->chassis_RC->rc.ch[3]) * CHASSIS_VX_KP;
        motor_control->vy_set = rc_dead_zone(-(motor_control->chassis_RC->rc.ch[2])) * CHASSIS_VY_KP;
        motor_control->wz_set = rc_dead_zone(motor_control->chassis_RC->rc.ch[0]) * CHASSIS_WZ_KP;

        first_order_filter_cali(&motor_control->chassis_cmd_slow_set_vx, motor_control->vx_set);
        first_order_filter_cali(&motor_control->chassis_cmd_slow_set_vy, motor_control->vy_set);

        // 停止区间
        if (motor_control->vx_set < 12 * CHASSIS_VX_KP && motor_control->vx_set < -12 * CHASSIS_VY_KP)
        {
            motor_control->vx_set = 0.0f;
        }

        if (motor_control->vy_set < 12 * CHASSIS_VX_KP && motor_control->vy_set < -12 * CHASSIS_VY_KP)
        {
            motor_control->vy_set = 0.0f;
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
    
    if (chassis_control->chassis_mode == CHASSIS_MODE_ARTIFICAL)
    {
    
    fp32 delat_angle = 0.0f;
    // set chassis yaw angle set-point
    angle_set = chassis_control->chassis_yaw;
    // 设置底盘控制的角度
    chassis_control->chassis_yaw_set = chassis_control->chassis_yaw;
    // 计算底盘角度差
    delat_angle = rad_format(chassis_control->chassis_yaw_set - chassis_control->chassis_yaw);
    // calculate rotation speed
    // 计算旋转的角速度
    chassis_control->wz_set += PID_calc(&chassis_control->chassis_angle_pid, 0.0f, delat_angle);
    // speed limit

    }

    if (chassis_control->chassis_mode == CHASSIS_MODE_STOP)
    {
        chassis_control->chassis_yaw_set = rad_format(angle_set);
        fp32 delat_angle = 0.0f;
    // calculate rotation speed
    // 计算旋转的角速度
        chassis_control->wz_set = 0.0;
        chassis_control->vx_set = 0.0f;
        chassis_control->vy_set = 0.0f;
    }
    

    // vofa调试用的代码
    // uart_dma_printf(&huart1,"%4.3d ,%4.3d\n",line_speed_x , line_speed_y);
}

static void chassis_wheel_speed_control(chassis_control_t *chassis_control)
{
    // 电机速度计算数组
    float motor_speed_calc[motor_3505_num] = {0.0f, 0.0f, 0.0f};

    // 计算各个电机速度,三全向解算
    motor_speed_calc[0] = RofCenter * chassis_control->wz_set / 3 + chassis_control->vx_set * sqrt(3) / 3 + chassis_control->vy_set / 3;
    motor_speed_calc[1] = RofCenter * chassis_control->wz_set / 3 - chassis_control->vx_set * sqrt(3) / 3 - chassis_control->vy_set / 3;
    motor_speed_calc[2] = RofCenter * chassis_control->wz_set / 3 + chassis_control->vy_set * 2 / 3;

    for (uint8_t i = 0; i < motor_3505_num; i++)
    {
        chassis_control->chassis_motor.M3508[i].motor_speed_set = motor_speed_calc[i];
    }

}

// 设定摇杆死区为10
static int16_t rc_dead_zone(int16_t rc_data)
{
    int16_t temp = rc_data;
    if ( fabs(rc_data) < 10)
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
