#include "bat_control.h"
#include "can_recv.h"
#include "can.h"
#include "bsp_usart.h"
#include "Variables.h"
#include "math.h"
#include "pc_interface.h"

#include "cmsis_os.h"

#define DEBUG 1 // 调试开关
/*
//——————————————————————————————————————————————————————————————————————————————
//               1
//        < S1 > 3                                            < S0 >
//               2
//
//
//
//
//
//            |                                              |
//            |                                              |
//            |                                              |
//            |                                              |
//    ————————+———————ch2                            ————————+———————ch0
//            |                                              |
//            |                                              |
//            |                                              |
//            |                                              |
//           ch3                                            ch1
//————————————————————————————————————————————————————————————————————————————————
*/
extern s_Dji_motor_data_t motor_Date[4];    // RM电机回传数据结构体
extern s_motor_data_t DM4340_Date[3];       // DM4340回传数据结构体
extern s_motor_data_t DM8006_Date[1];       // DM4340回传数据结构体
extern s_motor_data_t HT04_Data;            // 引用HT04回传数据结构体
extern s_robo_Mode_Setting robot_StateMode; // 引用机器人模式结构体
extern s_task_flags task_flags;             // 引用任务标志结构体
extern bat_control_t bat_control;
extern chassis_control_t motor_control;
extern visInf_t s_visionInform; // 视觉数据结构体

first_order_filter_type_t filter_angle;
top_inverse_calculation_angle inverse_calculation_angle;

int MMPerLinearSegment = 6;

extern volatile uint8_t skriker_flag; // 击球杆控制标志位

void striker_init(void);
void all_target_clac(bat_control_t *bat_control);
void ACTION_serve_a_ball(void);
void judge_bat_pos(bat_control_t *bat_control);
void bat_posion_set(float x, float y, float z);



void ACTION_artifical_bat(bat_control_t *bat_control);
void ACTION_up_receive_a_ball(void);
void ACTION_receive_a_ball(void);
bool_t ACTION_bat_MOVE(float aim_x, float aim_y, float aim_z,
                       float end_x, float end_y, float end_z,
                       float need_pitch);
//////////////////////////////////////////////////////
/*                                                  */
//////////////////////////////////////////////////////

/**
 * @brief 初始化球拍控制系统
 * @param bat_control 球拍控制结构体指针
 * @note 该函数完成以下初始化工作:
 *       1. 启动DM系列电机
 *       2. 初始化一阶滤波器
 *       3. 设置电机初始目标角度
 *       4. 初始化各电机PID控制参数
 *       5. 初始化击球器
 *       完成后设置初始化完成标志位
 */
void bat_motor_Init(bat_control_t *bat_control)
{
    /*
    //球拍初始化流程//
    */
    const static fp32 input_pitch_pos_num[1] = {0.16666666f};
    const static fp32 input_set_X_num[1] = {0.8f};
    const static fp32 input_set_Y_num[1] = {0.8f};
    const static fp32 input_set_Z_num[1] = {0.8f};
    // 初始化电机
    for (uint8_t i = 0; i < 5; i++)
    {
        DM_motor_start(&hcan2, i); // 启动DM电机
        osDelay(200);
    }

    first_order_filter_init(&bat_control->input_pitch_pos_filter, 0.008f, input_pitch_pos_num);
    // first_order_filter_init(&bat_control->input_set_X_filter, 0.008f, input_set_X_num);
    // first_order_filter_init(&bat_control->input_set_Y_filter, 0.008f, input_set_Y_num);
    // first_order_filter_init(&bat_control->input_set_Z_filter, 0.008f, input_set_Z_num);
    first_order_filter_init(&bat_control->input_set_striker_filter, 0.008f, input_set_Z_num);
    // first_order_filter_init(&bat_control->pitch_pos_filter , 0.002f , pitch_pos_filter);
    first_order_filter_init(&bat_control->DesiredPoint_X_filter, 0.6f, input_set_X_num);
    first_order_filter_init(&bat_control->DesiredPoint_Y_filter, 0.6f, input_set_Y_num);
    first_order_filter_init(&bat_control->DesiredPoint_Z_filter, 0.6f, input_set_Z_num);

    // 获取当前电机空间坐标
    Forward_Kinematics(bat_control, DM4340_Date[0].real_angle, DM4340_Date[1].real_angle, DM4340_Date[2].real_angle);

    // 获取遥控器指针
    bat_control->control_RC = get_remote_control_point();
    // bat_control->robot_StateMode = get_robot_mode_pint();
    //  设定初始目标位置
    bat_posion_set(0.0f, 0.0f, 0.0f);

    delta_arm_inverse_calculation(bat_control, bat_control->DesiredPoint.x, bat_control->DesiredPoint.y, bat_control->DesiredPoint.z);

    DM4340_Date[0].target_angle = float_constrain(bat_control->DesireAngle.theta1, 9.367, 54.367);
    DM4340_Date[1].target_angle = float_constrain(bat_control->DesireAngle.theta2, 8.809, 53.809);
    DM4340_Date[2].target_angle = float_constrain(bat_control->DesireAngle.theta3, 9.903, 54.903);

    // 这里尝试写一个编码器初始值校准函数，针对球拍的pitch电机
    // bat_pitch_init();
    bat_control->set_pitch = 100.0f;
    DM8006_Date[0].target_angle = bat_control->set_pitch;

    // HT04_Data.target_angle = 0.0f;

    /***********初始化PID参数***********/
    for (int i = 0; i < 3; i++)
    {
        pid_abs_param_init(&bat_control->DM_Motor_PID_angle[i], DM4340_ANGLE_PID_KP, DM4340_ANGLE_PID_KI, DM4340_ANGLE_PID_KD, DM4340_ANGLE_PID_MAX_IOUT, DM4340_ANGLE_PID_MAX_OUT);
        pid_abs_param_init(&bat_control->DM_Motor_PID_speed[i], DM4340_SPEED_PID_KP, DM4340_SPEED_PID_KI, DM4340_SPEED_PID_KD, DM4340_SPEED_PID_MAX_IOUT, DM4340_SPEED_PID_MAX_OUT);
    }

    pid_abs_param_init(&bat_control->DM_Motor_8006_PID_angle, DM8006_ANGLE_PID_KP, DM8006_ANGLE_PID_KI, DM8006_ANGLE_PID_KD, DM8006_ANGLE_PID_MAX_IOUT, DM8006_ANGLE_PID_MAX_OUT);
    pid_abs_param_init(&bat_control->DM_Motor_8006_PID_speed, DM8006_SPEED_PID_KP, DM8006_SPEED_PID_KI, DM8006_SPEED_PID_KD, DM8006_SPEED_PID_MAX_IOUT, DM8006_SPEED_PID_MAX_OUT);

    pid_abs_param_init(&bat_control->STRIKER_3508_INIT_PID_speed, STRIKER_3508_SPEED_INIT_PID_KP, STRIKER_3508_SPEED_INIT_PID_KI, STRIKER_3508_SPEED_INIT_PID_KD, STRIKER_3508_SPEED_INIT_PID_MAX_IOUT, STRIKER_3508_SPEED_INIT_PID_MAX_OUT);

    pid_abs_param_init(&bat_control->STRIKER_3508_PID_angle, STRIKER_3508_ANGLE_PID_KP, STRIKER_3508_ANGLE_PID_KI, STRIKER_3508_ANGLE_PID_KD, STRIKER_3508_ANGLE_PID_MAX_IOUT, STRIKER_3508_ANGLE_PID_MAX_OUT);
    pid_abs_param_init(&bat_control->STRIKER_3508_PID_speed, STRIKER_3508_SPEED_PID_KP, STRIKER_3508_SPEED_PID_KI, STRIKER_3508_SPEED_PID_KD, STRIKER_3508_SPEED_PID_MAX_IOUT, STRIKER_3508_SPEED_PID_MAX_OUT);
    /*************************************/

    striker_init();

    // 初始化完成标志位设定
    task_flags.bat_control_Init_flag = 1;
}

/**
 * @brief 击球电机初始化函数
 *
 * 该函数执行击球电机的初始化过程:
 * 1. 缓慢加速电机到指定速度
 * 2. 检测电机是否被阻塞
 * 3. 记录初始角度位置
 * 4. 使用PID控制器调节电机速度
 *
 * @note 函数会等待直到传感器检测到阻塞或超时
 */
void striker_init(void)
{
    motor_Date[3].target_motor_speed = 0.0f;

    int motor_runing_falg = 0; // 电机运行标志位
    int cout = 0;
    while (task_flags.sensor_is_blocked == 0)
    {
        if (motor_Date[3].target_motor_speed < 85)
        {
            motor_Date[3].target_motor_speed += 0.5f;
        }

        bat_control.STRIKER_3508_INIT_PID_speed.NowError =
            motor_Date[3].target_motor_speed - motor_Date[3].back_motor_speed;
        PID_AbsoluteMode(&bat_control.STRIKER_3508_INIT_PID_speed);
        motor_Date[3].out_current = (uint16_t)bat_control.STRIKER_3508_INIT_PID_speed.PIDout;

        CAN_cmd_3508(motor_Date[0].out_current, motor_Date[1].out_current,
                     motor_Date[2].out_current, motor_Date[3].out_current);
        osDelay(1);
        if (motor_Date[3].back_pos_last == motor_Date[3].back_position)
        {
            motor_runing_falg = 1;
        }
        else
        {
            motor_runing_falg = 0;
        }
        if (motor_runing_falg == 0)
        {
            cout++;
            if (cout > 5000)
            {
                break;
            }
        }
    }

    /// motor_Date[3].circle_num = 0;
    motor_Date[3].out_current = 0;
    bat_control.striker_start_angle = motor_Date[3].serial_motor_ang;
    motor_Date[3].target_motor_ang = bat_control.striker_start_angle;
}

/// @brief 电机的pid计算
/// @param bat_control
void motor_pid_clac(bat_control_t *bat_control)
{

    for (int i = 0; i < 3; i++)
    {
        /***********DM4340电机pid计算***********/
        bat_control->DM_Motor_PID_angle[i].NowError = DM4340_Date[i].target_angle - DM4340_Date[i].real_angle; // bat_control->pitch_pos_filter.out
        PID_AbsoluteMode(&bat_control->DM_Motor_PID_angle[i]);
        bat_control->DM_Motor_PID_speed[i].NowError = bat_control->DM_Motor_PID_angle[i].PIDout - DM4340_Date[i].esc_back_speed;
        PID_AbsoluteMode(&bat_control->DM_Motor_PID_speed[i]);
        DM4340_Date[i].out_current = bat_control->DM_Motor_PID_speed[i].PIDout;
    }

    /***********striker电机pid计算***********/
    bat_control->STRIKER_3508_PID_angle.NowError = motor_Date[3].target_motor_ang - motor_Date[3].serial_motor_ang;
    PID_AbsoluteMode(&bat_control->STRIKER_3508_PID_angle);
    bat_control->STRIKER_3508_PID_speed.NowError = bat_control->STRIKER_3508_PID_angle.PIDout - motor_Date[3].back_motor_speed;
    PID_AbsoluteMode(&bat_control->STRIKER_3508_PID_speed);
    motor_Date[3].out_current = bat_control->STRIKER_3508_PID_speed.PIDout;

    /***********8006电机pid计算***********/
    bat_control->DM_Motor_8006_PID_angle.NowError = DM8006_Date[0].target_angle - DM8006_Date[0].real_angle; // DM8006_Date[0].real_angle;
    PID_AbsoluteMode(&bat_control->DM_Motor_8006_PID_angle);
    bat_control->DM_Motor_8006_PID_speed.NowError = bat_control->DM_Motor_8006_PID_angle.PIDout - DM8006_Date[0].esc_back_speed;
    PID_AbsoluteMode(&bat_control->DM_Motor_8006_PID_speed);
    DM8006_Date[0].out_current = bat_control->DM_Motor_8006_PID_speed.PIDout;
}

/// @brief 更新球拍的数据
/// @param bat_control
void bat_data_update(bat_control_t *bat_control)
{
    Forward_Kinematics(bat_control, DM4340_Date[0].real_angle, DM4340_Date[1].real_angle, DM4340_Date[2].real_angle);
    // Forward_Kinematics(bat_control ,DM4340_Date[0].target_angle, DM4340_Date[1].target_angle,DM4340_Date[2].target_angle);
}

/**
 * @brief 控制击球机构的电机运动
 * @param bat_control 击球控制结构体指针
 *
 * @details 通过CAN2总线控制4个电机:
 *          - 3个DM4340电机(ID:0x01-0x03)用于击球机构
 *          - 1个DM8006电机(ID:0x04)用于底盘运动
 *          每个电机通过PID控制输出电流,并在每次发送后延时1ms
 */
void bat_motor_control(bat_control_t *bat_control)
{

    // MD4340_motor_PID_Control(&hcan2, 0x02, 0);
    // osDelay(1);
    // MD4340_motor_PID_Control(&hcan2, 0x01, 0);
    // osDelay(1);
    // MD4340_motor_PID_Control(&hcan2, 0x03, 0);
    // osDelay(1);

    MD4340_motor_PID_Control(&hcan2, 0x02, DM4340_Date[1].out_current);
    osDelay(1);
    MD4340_motor_PID_Control(&hcan2, 0x01, DM4340_Date[0].out_current);
    osDelay(1);
    MD4340_motor_PID_Control(&hcan2, 0x03, DM4340_Date[2].out_current);
    osDelay(1);
    // DM8006_Date[0].out_current  += 4.72f * cos(DM8006_Date[0].real_angle / ROUND2RAD);
    DM8006_motor_PID_Control(&hcan2, 0x04, DM8006_Date[0].out_current); // +(4.998f * sin(-(DM8006_Date->real_angle) ))));
    osDelay(1);

    // 击球电机的被并入到底盘中了//
}

/**
 * @brief 设置球拍控制参数和运动控制
 * @param bat_control 球拍控制结构体指针
 *
 * @details 根据机器人状态模式(roboMode)执行不同的控制逻辑:
 *          - ARTIFICAL_BAT模式: 遥控器控制球拍位置和姿态
 *          - ARTIFICAL_STRIKER模式: 遥控器控制击球杆
 *          - MODE_AUTO模式: 自动控制(待实现)
 *          - MODE_STOP模式: 回到初始位置
 *          最后更新电机目标角度
 */
void bat_action(bat_control_t *bat_control)
{
    // delta_arm_inverse_calculation(bat_control , 0 , 0, 0);
    if (robot_StateMode.roboMode == ARTIFICAL_BAT) // 机器人模式为4时，使用遥控器控制球拍
    {
        ACTION_artifical_bat(bat_control);
    }
    else if (robot_StateMode.roboMode == ARTIFICAL_CHASSIS) // 机器人模式为AUTO_RECEIVE_BALL时，使用遥控器控制球拍
    {
        //使用球拍自动接球
        ACTION_receive_a_ball();
    }
    else if (robot_StateMode.roboMode == ARTIFICAL_STRIKER) // 机器人模式为ARTIFICAL_STRIKER时，使用遥控器控制击球杆
    {

        // hit_ball_launch(bat_control);

        int control_falg = 0;
        // 击球杆运行状态判断
        if (fabs(motor_Date[3].target_motor_ang - motor_Date[3].serial_motor_ang < 3.0f))
        {
            skriker_flag = 0;
            bat_control->striker_state = STRIKER_is_READY;
        }
        else
        {
            bat_control->striker_state == STRIKER_is_RUNNING;
            control_falg = 1;
        }

        if (bat_control->striker_state == STRIKER_is_RUNNING)
        {
            // 击球杆正在运行状态
            return;
        }
        else if (bat_control->striker_state == STRIKER_is_READY)
        {
            // 击球杆准备状态

            if (bat_control->control_RC->rc.ch[2] > 500)
            {
                // osDelay(200);
                 motor_Date[3].target_motor_ang = motor_Date[3].serial_motor_ang - 360.0f * 19;
                osDelay(2000);
            }
            // if (bat_control->control_RC->rc.ch[0] > 400)
            // {
            //     ACTION_serve_a_ball();
            // }
        }
    }
    else if (robot_StateMode.roboMode == AUTO_RECEIVE_BALL)
    {
        ACTION_receive_a_ball();
    }
    else if (robot_StateMode.roboMode == AUTO_SERVE_BALL)
    {
        ACTION_up_receive_a_ball();
    }
    else if (robot_StateMode.roboMode == MODE_STOP)
    {
        // 否则全部回0点待命
        bat_control->set_x = 0.0f;
        bat_control->set_y = 0.0f;
        bat_control->set_z = 0.0f;
        bat_posion_set(bat_control->set_x, bat_control->set_y, bat_control->set_z);
        bat_control->set_pitch = 100.0f;
        bat_control->set_striker_angle = 0.0f;
    }
    // 计算所有目标位置和角度，输入参量
    all_target_clac(bat_control);
}

/**
 * @brief 计算所有目标位置和角度
 * @param bat_control 机械臂控制结构体指针
 * @details 该函数执行以下操作:
 *          1. 对期望位置 X,Y,Z 和俯仰角进行一阶滤波
 *          2. 通过逆运动学计算三个关节角度
 *          3. 对三个 DM4340 电机的目标角度进行限位
 *          4. 设置 DM8006 电机的目标俯仰角
 * @note DM4340 电机角度限位范围:
 *       电机1: 4.367° ~ 59.367°
 *       电机2: 3.809° ~ 58.809°
 *       电机3: 4.903° ~ 59.903°
 */
void all_target_clac(bat_control_t *bat_control)
{
    first_order_filter_cali(&bat_control->DesiredPoint_X_filter, bat_control->DesiredPoint.x);
    first_order_filter_cali(&bat_control->DesiredPoint_Y_filter, bat_control->DesiredPoint.y);
    first_order_filter_cali(&bat_control->DesiredPoint_Z_filter, bat_control->DesiredPoint.z);
    first_order_filter_cali(&bat_control->input_pitch_pos_filter, bat_control->set_pitch);

    // 计算输出量
    delta_arm_inverse_calculation(bat_control, bat_control->DesiredPoint_X_filter.out, bat_control->DesiredPoint_Y_filter.out, bat_control->DesiredPoint_Z_filter.out);

    DM4340_Date[0].target_angle = float_constrain(bat_control->DesireAngle.theta1, 4.367, 59.367);
    DM4340_Date[1].target_angle = float_constrain(bat_control->DesireAngle.theta2, 3.809, 58.809);
    DM4340_Date[2].target_angle = float_constrain(bat_control->DesireAngle.theta3, 4.903, 59.903);

    DM8006_Date[0].target_angle = float_constrain(bat_control->set_pitch, 35, 100);
}

/// @brief    限幅函数
/// @details  限幅函数，将输入值限幅到指定的最小值和最大值之间。
/// @param Value   输入值
/// @param minValue 最小值
/// @param maxValue 最大值
/// @return
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

// 球拍俯仰角的限幅函数
float limit_bat_pitch(float Value)
{
    if (Value < -50)
        return -50.0f;
    else if (Value > 0.0f)
        return 0.0f;
    else
        return Value;
}

/**
 * @brief 设置机器人末端执行器的目标位置,赋值给DesiredPoint
 * @param x 目标位置的 X 坐标值
 * @param y 目标位置的 Y 坐标值
 * @param z 目标位置的 Z 坐标值
 */
void bat_posion_set(float x, float y, float z)
{
    bat_control.DesiredPoint.x = x;
    bat_control.DesiredPoint.y = y;
    bat_control.DesiredPoint.z = z;
}

/**
 * @brief 判断球拍是否到达目标位置
 * @param bat_control 球拍控制结构体指针
 * @return 返回球拍位置判断结果T/F
 * @note 当球拍当前位置与目标位置在x、y、z三个方向的误差均小于error时，认为到达目标位置
 */
void judge_bat_pos(bat_control_t *bat_control)
{
    // 或许用bool返回判断更好
    float error = 5.0f; // 允许误差

    // 判断球拍是否到达目标位置
    if ((fabs(bat_control->CurrentPoint.x - bat_control->DesiredPoint.x) < error) &&
        (fabs(bat_control->CurrentPoint.y - bat_control->DesiredPoint.y) < error) &&
        (fabs(bat_control->CurrentPoint.z - bat_control->DesiredPoint.z) < error))
    {
        task_flags.bat_running_flag = 0;
    }
    else
    {
        task_flags.bat_running_flag = 1;
    }
}


/**
 * @brief 控制机器人击球动作的运动轨迹
 * @param aim_x 击球点x坐标
 * @param aim_y 击球点y坐标
 * @param aim_z 击球点z坐标
 * @param end_x 结束点x坐标
 * @param end_y 结束点y坐标
 * @param end_z 结束点z坐标
 * @param need_pitch 需要的俯仰角度
 * @return bool_t 动作执行完成返回1
 *
 * @details 该函数控制机器人从当前位置运动到击球点,完成击球动作后运动到结束点。
 * 整个过程分为两段运动,每段运动都通过位置闭环控制实现。
 * 运动过程中会实时计算运动学和PID控制。
 */
bool_t ACTION_bat_MOVE(float aim_x, float aim_y, float aim_z,
                       float end_x, float end_y, float end_z,
                       float need_pitch)
{

    bat_posion_set(aim_x, aim_y, aim_z);
    bat_control.set_pitch = need_pitch;

    task_flags.bat_running_flag = 1;
    while (task_flags.bat_running_flag == 1)
    {
        all_target_clac(&bat_control);
        motor_pid_clac(&bat_control);
        bat_motor_control(&bat_control);
        Forward_Kinematics(&bat_control, DM4340_Date[0].real_angle, DM4340_Date[1].real_angle, DM4340_Date[2].real_angle);
        if (robot_StateMode.roboMode == MODE_STOP)
        {
            break;
        }
        judge_bat_pos(&bat_control);
        osDelay(1);
    }

    bat_posion_set(end_x, end_y, end_z);
    task_flags.bat_running_flag = 1;
    while (task_flags.bat_running_flag == 1)
    {
        all_target_clac(&bat_control);
        motor_pid_clac(&bat_control);
        bat_motor_control(&bat_control);
        Forward_Kinematics(&bat_control, DM4340_Date[0].real_angle, DM4340_Date[1].real_angle, DM4340_Date[2].real_angle);
        if (robot_StateMode.roboMode == MODE_STOP)
        {
            break;
        }
        judge_bat_pos(&bat_control);
        osDelay(1);
    }

    // task_flags.ball_soaring_flag = 1;
    return 1;
}

/**
 * @brief 执行发球动作的控制函数
 *
 * @details 该函数控制机器人执行完整的发球动作序列:
 *          1. 首先将球拍向下摆至-30位置
 *          2. 然后向上摆至50位置
 *          3. 回到0度初始位置并短暂延时
 *          4. 快速向上摆至80击球
 *          5. 最后回到0初始位置
 *
 * @note 每个动作都会等待球拍到达指定位置后才执行下一步
 */

// void ACTION_serve_a_ball(void)
// {
//     int need_hit = 0;
//     // if (s_visionInform.z_usbcam < 35)
//     // {
//         // need_hit = 1;
//         ACTION_bat_MOVE(0, 0, 70, 0, 0, 0, 90);
//     // }
//     // bat_posion_set(0, 0, 70);
// }

void ACTION_up_receive_a_ball(void)
{
    ACTION_bat_MOVE(0, 0, 0, 0, 0, 0, 95);

    if (s_visionInform.z_usbcam < 0.40)
    {
        //-1 即为未识别到球
        if (s_visionInform.z_usbcam == -1)
        {
            return;
        }
        // need_hit = 1;
        if (s_visionInform.usbcam_ball_judje == come)
        {
            ACTION_bat_MOVE(0, 0, 70, 0, 0, 0, 95);
            // osDelay(100);
        }
    }
    all_target_clac(&bat_control);
    motor_pid_clac(&bat_control);
    // bat_posion_set(0, 0, 70);
}




/////////////////球拍手操控制////////////////////////
/**
 * @brief 手动控制球拍动作的函数
 * @param bat_control 球拍控制结构体指针
 * @note 该函数通过遥控器通道值计算球拍的目标位置和姿态，
 *       并执行相应的运动控制
 *       - CH0: X轴位置
 *       - CH1: Y轴位置
 *       - CH2: PITCH角度
 *       - CH3: Z轴位置
 */
void ACTION_artifical_bat(bat_control_t *bat_control)
{
    // 球拍获得目标位置，进行手动运动
    bat_control->set_x = (bat_control->control_RC->rc.ch[0] / 6);
    bat_control->set_y = (bat_control->control_RC->rc.ch[1] / 6);
    bat_control->set_z = fabs(bat_control->control_RC->rc.ch[3] / 3);
    bat_control->set_pitch = float_constrain(bat_control->set_pitch - (bat_control->control_RC->rc.ch[2] / 660), 35, 100);

    bat_control->DesiredPoint.x = bat_control->set_x;
    bat_control->DesiredPoint.y = bat_control->set_y;
    bat_control->DesiredPoint.z = bat_control->set_z;

    all_target_clac(&bat_control);
    motor_pid_clac(&bat_control);
    bat_motor_control(&bat_control);
}
////////////////////球拍手操控制////////////////////////

///////////////////自动接发球//////////////////////////////
/**
 * @brief 执行接球动作的控制函数
 *
 * 该函数实现机器人接球动作的控制逻辑:
 * 1. 初始化机器人姿态
 * 2. 根据USB摄像头获取的球距离信息调整击球高度
 * 3. 计算目标位置并执行电机PID控制
 *
 * @note 当z_usbcam < 0.45时会触发击球动作
 * @note z_usbcam = -1 表示未识别到球
 */
void ACTION_receive_a_ball(void)
{
    ACTION_bat_MOVE(0, 0, 0, 0, 0, 0, 60);

    if (s_visionInform.z_usbcam < 0.45) // 0.45可可能还是有问题；适当降低，此外在特殊角度有存在距离有问题的现象
    {
        //-1 即为未识别到球
        if (s_visionInform.z_usbcam == -1)
        {
            return;
        }
        // need_hit = 1;
        ACTION_bat_MOVE(0, 0, 70, 0, 0, 0, 60);
        osDelay(200);
    }
    all_target_clac(&bat_control);
    motor_pid_clac(&bat_control);
    bat_motor_control(&bat_control);

    // bat_posion_set(0, 0, 70);
}

//////////////////////发球动作//////////////////////////
/**
 * @brief 执行发球动作的函数
 * 
 * 该函数控制机器人执行完整的发球动作序列:
 * 1. 将击球装置移动到准备位置(-30)
 * 2. 抬升击球装置(70)
 * 3. 回到中间位置(0)
 * 4. 设置击球俯仰角度(40)
 * 5. 底盘前移，击球杆启动
 * 6. 执行击球动作
 * 
 * 在每个动作过程中会实时计算运动学、执行PID控制,并检查机器人状态。
 * 如果机器人模式改变,则提前结束动作序列。
 */
void ACTION_bat_serve_a_ball(void)
{
    // 机器人执行击球动作,将球运起
    bat_posion_set(0.0f, 0.0f, -30.0f);
    task_flags.bat_running_flag = 1;
    while (task_flags.bat_running_flag == 1)
    {
        all_target_clac(&bat_control);
        motor_pid_clac(&bat_control);
        bat_motor_control(&bat_control);
        Forward_Kinematics(&bat_control, DM4340_Date[0].real_angle, DM4340_Date[1].real_angle, DM4340_Date[2].real_angle);
        if (robot_StateMode.roboMode != ARTIFICAL_STRIKER)
        {
            break;
        }
        judge_bat_pos(&bat_control);
    }

    if (robot_StateMode.roboMode != ARTIFICAL_STRIKER)
    {
        goto end;
    }
    bat_posion_set(0.0f, 0.0f, 70.0f);
    task_flags.bat_running_flag = 1;
    while (task_flags.bat_running_flag == 1)
    {
        all_target_clac(&bat_control);
        motor_pid_clac(&bat_control);
        bat_motor_control(&bat_control);
        Forward_Kinematics(&bat_control, DM4340_Date[0].real_angle, DM4340_Date[1].real_angle, DM4340_Date[2].real_angle);

        if (robot_StateMode.roboMode != ARTIFICAL_STRIKER)
        {
            break;
        }
        judge_bat_pos(&bat_control);
    }
    if (robot_StateMode.roboMode != ARTIFICAL_STRIKER)
    {
        goto end;
    }

    bat_posion_set(0.0f, 0.0f, 0.0f);
    task_flags.bat_running_flag = 1;
    while (task_flags.bat_running_flag == 1)
    {
        all_target_clac(&bat_control);
        motor_pid_clac(&bat_control);
        bat_motor_control(&bat_control);
        Forward_Kinematics(&bat_control, DM4340_Date[0].real_angle, DM4340_Date[1].real_angle, DM4340_Date[2].real_angle);

        if (robot_StateMode.roboMode != ARTIFICAL_STRIKER)
        {
            break;
        }
        judge_bat_pos(&bat_control);
    }
    if (robot_StateMode.roboMode != ARTIFICAL_STRIKER)
    {
        goto end;
    }
    osDelay(50);

    bat_control.set_pitch = 40.0f;
    motor_pid_clac(&bat_control);
    bat_motor_control(&bat_control);

    task_flags.hit_ball_chassis_move_flag = 1;
    osDelay(200);
    motor_Date[3].target_motor_ang = motor_Date[3].serial_motor_ang - 360.0f * 19;
    task_flags.ball_need_hit_flag == 0;
end:
    return;
}
//////////////////////发球动作//////////////////////////




// 计算力矩

/*
// 球拍姿态解算
void delta_arm_inverse_calculation(float x, float y, float z,  top_inverse_calculation_angle *angle)
{
    // 不使用全局变量
    double A1 = (x * x + y * y + z* z+ L1 * L1 - La * La + (R1 - R2) * (R1 - R2) - 2 * x * (R1 - R2)) / (2 * L1);
    double B1 = -(R1 - R2 - x);
    double C1 = z;

    double A2 = (x * x + y * y + z* z+ L1 * L1 - La * La + (R1 - R2) * (R1 - R2) + (x - sqrt(3) * y) * (R1 - R2)) / L1;
    double B2 = -2 * (R1 - R2) - (x - sqrt(3) * y);
    double C2 = 2 * z;

    double A3 = (x * x + y * y + z* z+ L1 * L1 - La * La + (R1 - R2) * (R1 - R2) + (x - sqrt(3) * y) * (R1 - R2)) / L1;
    double B3 = -2 * (R1 - R2) - (x + sqrt(3) * y);
    double C3 = 2 * z;

    double K1 = A1 + B1;
    double U1 = 2 * C1;
    double V1 = A1 - B1;

    double K2 = A2 + B2;
    double U2 = 2 * C2;
    double V2 = A2 - B2;

    double K3 = A3 + B3;
    double U3 = 2 * C3;
    double V3 = A3 - B3;

    double T1 = (-U1 - sqrt(U1 * U1 - 4 * K1 * V1)) / (2 * K1);
    double T2 = (-U2 - sqrt(U2 * U2 - 4 * K2 * V2)) / (2 * K2);
    double T3 = (-U3 - sqrt(U3 * U3 - 4 * K3 * V3)) / (2 * K3);

    angle->angle01 = (180 * (2 * atan(T1))) / PI;
    angle->angle02 = (180 * (2 * atan(T2))) / PI;
    angle->angle03 = (180 * (2 * atan(T3))) / PI;
}
*/

/**
 * @brief 计算三角洲机械臂的逆运动学
 *
 * @param bat_control 机械臂控制结构体指针
 * @param x 目标位置X坐标
 * @param y 目标位置Y坐标
 * @param z 目标位置Z坐标(相对于基座高度235mm)
 *
 * @note 使用几何法计算三个舵机的角度，将笛卡尔坐标(x,y,z)转换为三个舵机角度(theta1,theta2,theta3)
 *       计算结果存储在bat_control结构体的DesireAngle成员中
 */
void delta_arm_inverse_calculation(bat_control_t *bat_control, float x, float y, float z)
{
    // 不使用全局变量
    z -= 240;
    double A1 = (x * x + y * y + z * z + 35.4084f - 2 * x * (-5.9505f)) / (300.0f);
    double B1 = -((-5.9505f) - x);
    double C1 = z;

    double A2 = (x * x + y * y + z * z + 35.4084f + (x - 1.73205f * y) * (-5.9505f)) / L1;
    double B2 = -2 * ((-5.9505f)) - (x - 1.73205f * y);
    double C2 = 2 * z;

    double A3 = (x * x + y * y + z * z + 35.4084f + (x - 1.73205f * y) * (-5.9505f)) / L1;
    double B3 = -2 * ((-5.9505f)) - (x + 1.73205f * y);
    double C3 = 2 * z;

    double K1 = A1 + B1;
    double U1 = 2 * C1;
    double V1 = A1 - B1;

    double K2 = A2 + B2;
    double U2 = 2 * C2;
    double V2 = A2 - B2;

    double K3 = A3 + B3;
    double U3 = 2 * C3;
    double V3 = A3 - B3;

    double T1 = (-U1 - sqrt(U1 * U1 - 4 * K1 * V1)) / (2 * K1);
    double T2 = (-U2 - sqrt(U2 * U2 - 4 * K2 * V2)) / (2 * K2);
    double T3 = (-U3 - sqrt(U3 * U3 - 4 * K3 * V3)) / (2 * K3);

    bat_control->DesireAngle.theta3 = ((180 * (2 * atan(T1))) / PI);
    bat_control->DesireAngle.theta2 = ((180 * (2 * atan(T2))) / PI);
    bat_control->DesireAngle.theta1 = ((180 * (2 * atan(T3))) / PI);

    // bat_control->DesireAngle.theta3 = ((180 * (2 * atan(T1))) / PI) - 0.186f;
    // bat_control->DesireAngle.theta2 = ((180 * (2 * atan(T2))) / PI) + 0.208f;
    // bat_control->DesireAngle.theta1 = ((180 * (2 * atan(T3))) / PI) - 0.448f;
}

/**
 * @brief 计算三轮并联机器人的正向运动学
 *
 * @param bat_control 机器人控制结构体指针
 * @param theta1 第一个关节角度(度)
 * @param theta2 第二个关节角度(度)
 * @param theta3 第三个关节角度(度)
 *
 * @details 根据三个关节角度计算末端执行器的空间位置(X,Y,Z)。
 *          使用几何法求解三轮并联机器人的正向运动学方程。
 *          计算结果存储在bat_control结构体的CurrentPoint成员中。
 */
void Forward_Kinematics(bat_control_t *bat_control, float theta1, float theta2, float theta3)
{

    // 输入角度加上数据是为了消除初始角度的些许不同
    // theta1 = (PI * (theta1 - 0.208f) / 180);
    // theta2 = (PI * (theta2 + 0.448f) / 180);
    // theta3 = (PI * (theta3 + 0.186f) / 180);

    theta1 = (PI * (theta1) / 180);
    theta2 = (PI * (theta2) / 180);
    theta3 = (PI * (theta3) / 180);
    double A1 = R1 + L1 * cos(theta3) - R2;
    // double B1 = 1;
    double C1 = L1 * sin(theta3);
    double A2 = -(1 / 2.0) * (R1 + L1 * cos(theta1) - R2);
    double B2 = (sqrt(3) / 2.0) * (R1 + L1 * cos(theta1) - R2);
    double C2 = L1 * sin(theta1);
    double A3 = -(1 / 2.0) * (R1 + L1 * cos(theta2) - R2);
    double B3 = -(sqrt(3) / 2.0) * (R1 + L1 * cos(theta2) - R2);
    double C3 = L1 * sin(theta2);
    double D1 = (1 / 2.0) * (A1 * A1 - A2 * A2 + C1 * C1 - C2 * C2 - B2 * B2);
    double A21 = A2 - A1;
    double C21 = C2 - C1;
    double D2 = (1 / 2.0) * (A1 * A1 - A3 * A3 + C1 * C1 - C3 * C3 - B3 * B3);
    double A31 = A3 - A1;
    double C31 = C3 - C1;
    double E1 = (B3 * C21 - B2 * C31) / (A21 * B3 - A31 * B2);
    double F1 = (B2 * D2 - B3 * D1) / (A21 * B3 - A31 * B2);
    double E2 = (A31 * C21 - A21 * C31) / (A31 * B2 - A21 * B3);
    double F2 = (A21 * D2 - A31 * D1) / (A31 * B2 - A21 * B3);
    double a = E1 * E1 + E2 * E2 + 1;
    double b = 2 * E2 * F2 + 2 * C1 - 2 * E1 * (A1 - F1);
    double c = (A1 - F1) * (A1 - F1) + F2 * F2 + C1 * C1 - La * La;
    double Z = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
    double X = E1 * Z + F1;
    double Y = E2 * Z + F2;

    bat_control->CurrentPoint.x = X;
    bat_control->CurrentPoint.y = Y;
    bat_control->CurrentPoint.z = Z + 240;
}

// /// @brief // 计算移动距离
// /// @param point1
// /// @param point2
// /// @return 返回两点之间的距离
// float CalDistance2Point(struct Point point1, struct Point point2)
// {
//     float x_Offset = point1.x - point2.x;
//     float y_Offset = point1.y - point2.y;
//     float z_Offset = point1.z - point2.z;

//     float distance = sqrt(pow(x_Offset, 2) + pow(y_Offset, 2) + pow(z_Offset, 2));

//     if (distance < 0.2 && distance > -0.2)
//         distance = 0;

//     return distance;
// }

// /// @brief 计算两点的插值
// /// @param currentP 当前点
// /// @param desiredP 目标点
// /// @param t 百分比
// /// @return point 返回插值点
// struct Point GetPointInLine(struct Point currentP, struct Point desiredP, float t)
// {
//     struct Point buffer;

//     buffer.x = currentP.x - ((currentP.x - desiredP.x) * t);
//     buffer.y = currentP.y - ((currentP.y - desiredP.y) * t);
//     buffer.z = currentP.z - ((currentP.z - desiredP.z) * t);

//     return buffer;
// }

/// 基础测试已经通过

// 周三任务是测试球拍刚度，性能如何，以及机械臂的刚度，
// 性能如何准备施加一下配合视觉的效果还有位置运算
// 尝试要不要在上位机加滤波
