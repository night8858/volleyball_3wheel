#include "bat_control.h"
#include "can_recv.h"
#include "can.h"
#include "bsp_usart.h"
#include "Variables.h"
#include "math.h"

#include "cmsis_os.h"

#define DEBUG 1 // 调试开关
/*
//——————————————————————————————————————————————————————————————————————————————
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

first_order_filter_type_t filter_angle;
top_inverse_calculation_angle inverse_calculation_angle;

float angle[3];

int MMPerLinearSegment = 6;

float limit_bat_pitch(float Value);
void bat_pitch_init(void);
void striker_init(void);
//////////////////////////////////////////////////////
/*                                                  */
//////////////////////////////////////////////////////

/// @brief 球拍部分初始化
/// @param bat_control
void bat_motor_Init(bat_control_t *bat_control)
{
    /*
    //球拍初始化流程//





    */
    const static fp32 input_pitch_pos_num[1] = {0.16666666f};
    const static fp32 input_set_X_num[1] = {0.16666666f};
    const static fp32 input_set_Y_num[1] = {0.16666666f};
    const static fp32 input_set_Z_num[1] = {0.16666666f};
    // 初始化电机
    for (uint8_t i = 0; i < 5; i++)
    {
        DM_motor_start(&hcan2, i); // 启动DM电机
        osDelay(500);
    }

    // HT04_motor_start(&hcan1, HT8115_M1); // 启动HT04电机
    // osDelay(100);

    first_order_filter_init(&bat_control->input_pitch_pos_filter, 0.002f, input_pitch_pos_num);
    first_order_filter_init(&bat_control->input_set_X_filter, 0.002f, input_set_X_num);
    first_order_filter_init(&bat_control->input_set_Y_filter, 0.002f, input_set_Y_num);
    first_order_filter_init(&bat_control->input_set_Z_filter, 0.002f, input_set_Z_num);
    first_order_filter_init(&bat_control->input_set_striker_filter, 0.002f, input_set_Z_num);
    // first_order_filter_init(&bat_control->pitch_pos_filter , 0.002f , pitch_pos_filter);

    // 获取当前电机空间坐标
    Forward_Kinematics(&bat_control->pos_angle_data.CurrentPoint, DM4340_Date[0].real_angle, DM4340_Date[1].real_angle, DM4340_Date[2].real_angle);

    // 获取遥控器指针
    bat_control->control_RC = get_remote_control_point();
    // bat_control->robot_StateMode = get_robot_mode_pint();
    //  设定初始目标角度
    DM4340_Date[0].target_angle = float_constrain(50, 9.367, 54.367);
    DM4340_Date[1].target_angle = float_constrain(50, 8.809, 53.809);
    DM4340_Date[2].target_angle = float_constrain(50, 9.903, 54.903);

    // 这里尝试写一个编码器初始值校准函数，针对球拍的pitch电机
    // bat_pitch_init();

    DM8006_Date[0].target_angle = limit_bat_pitch(0.0f);

    HT04_Data.target_angle = 0.0f;

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

    // for (int i = 0; i < 3; i++)
    //{
    //     MD_motor_SendCurrent(&hcan2, i + 1, bat_control->pid_out[i]);
    // }
    // 此处初始化击球3508电机的位置参数

    striker_init();
    // 初始化完成标志位设定
    task_flags.bat_control_Init_flag = 1;
}

// 初始化pitch电机角度
void bat_pitch_init(void)
{
    int count = 100;
    // 电机pitch初始位置校准,摁怼1s确定初始值
    while (count--)
    {
        DM8006_motor_PID_Control(&hcan2, 0x04, 3.4f);
        osDelay(10);
    }

    bat_control.pitch_init_angle = DM8006_Date[0].real_angle;
}

//击球3508电机的初始化
void striker_init(void)
{
    motor_Date[3].target_motor_speed = 190.0f;

    int cout = 0; 
    while (task_flags.sensor_is_blocked == 0)
    {
        bat_control.STRIKER_3508_INIT_PID_speed.NowError =
            motor_Date[3].target_motor_speed - motor_Date[3].back_motor_speed;
        PID_AbsoluteMode(&bat_control.STRIKER_3508_INIT_PID_speed);
        motor_Date[3].out_current = (uint16_t)bat_control.STRIKER_3508_INIT_PID_speed.PIDout;

        CAN_cmd_3508(motor_control.chassis_motor.M3508[0].current_set, motor_control.chassis_motor.M3508[1].current_set,
                     motor_control.chassis_motor.M3508[2].current_set, motor_Date[3].out_current);
        osDelay(2);
        cout++;
        if (cout > 8000)
        {
            break;
        }
    }

    ///motor_Date[3].circle_num = 0;
    bat_control.striker_start_angle = motor_Date[3].serial_motor_ang;
    motor_Date[3].target_motor_ang = bat_control.striker_start_angle;

}

/// @brief 达妙电机的pid计算
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

    /***********HT04电机pid计算***********/
    /*
        bat_control->HT_Motor_PID_angle.NowError = HT04_Data.target_angle - HT04_Data.real_angle;
        PID_AbsoluteMode(&bat_control->HT_Motor_PID_angle);
        bat_control->HT_Motor_PID_speed.NowError = bat_control->HT_Motor_PID_angle.PIDout - HT04_Data.esc_back_speed;
        PID_AbsoluteMode(&bat_control->HT_Motor_PID_speed);
        HT04_Data.out_current = bat_control->HT_Motor_PID_speed.PIDout;
     */
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
    Forward_Kinematics(&bat_control->pos_angle_data.CurrentPoint, DM4340_Date[0].real_angle, DM4340_Date[1].real_angle, DM4340_Date[2].real_angle);
    // Forward_Kinematics(bat_control ,DM4340_Date[0].target_angle, DM4340_Date[1].target_angle,DM4340_Date[2].target_angle);
}
/// @brief 发送电机控制信号
/// @param bat_control
void bat_motor_control(bat_control_t *bat_control)
{

    // MD4340_motor_PID_Control(&hcan2, 0x02, DM4340_Date[1].out_current);
    // osDelay(1);
    // MD4340_motor_PID_Control(&hcan2, 0x01, DM4340_Date[0].out_current);
    // osDelay(1);
    // MD4340_motor_PID_Control(&hcan2, 0x03, DM4340_Date[2].out_current);
    // osDelay(1);

    MD4340_motor_PID_Control(&hcan2, 0x02, 0);
    osDelay(1);
    MD4340_motor_PID_Control(&hcan2, 0x01, 0);
    osDelay(1);
    MD4340_motor_PID_Control(&hcan2, 0x03, 0);
    osDelay(1);
    DM8006_motor_PID_Control(&hcan2, 0x04, DM8006_Date[0].out_current); // +(4.998f * sin(-(DM8006_Date->real_angle) ))));
    // osDelay(1);

    
    //击球电机的被并入到底盘中了//

}

void top_RC_control_set(bat_control_t *bat_control)
{
    // delta_arm_inverse_calculation(bat_control , 0 , 0, 0);
    if (robot_StateMode.roboMode == 4) // 机器人模式为4时，使用遥控器控制球拍
    {
        bat_control->set_x = (bat_control->control_RC->rc.ch[0] / 6);
        bat_control->set_y = (bat_control->control_RC->rc.ch[1] / 6);
        bat_control->set_z = fabs(bat_control->control_RC->rc.ch[3] / 3);
        bat_control->set_pitch = -(bat_control->control_RC->rc.ch[4] / 2000);

        first_order_filter_cali(&bat_control->input_set_X_filter, bat_control->set_x);
        first_order_filter_cali(&bat_control->input_set_Y_filter, bat_control->set_y);
        first_order_filter_cali(&bat_control->input_set_Z_filter, bat_control->set_z);
        first_order_filter_cali(&bat_control->input_pitch_pos_filter, bat_control->set_pitch);

        bat_control->pos_angle_data.DesiredPoint.x = bat_control->input_set_X_filter.out;
        bat_control->pos_angle_data.DesiredPoint.y = bat_control->input_set_Y_filter.out;
        bat_control->pos_angle_data.DesiredPoint.z = bat_control->input_set_Z_filter.out;


        delta_arm_inverse_calculation(&bat_control->pos_angle_data.DesireAngle, bat_control->input_set_X_filter.out, bat_control->input_set_Y_filter.out, bat_control->input_set_Z_filter.out);
    }

    else if (robot_StateMode.roboMode == 5) // 机器人模式为5时，使用遥控器控制击球杆
    {

        if (bat_control->control_RC->rc.ch[2] > 400)
        //// 此处设定击球值，还需要编码处理，进行编码值绝对化，尽可能保证击球速度，尝试一下
        //// 或者是利用车速加上击球杆的冲量（可能要加以配重处理）来控制击球速度
        {
            bat_control->striker_state = BAT_is_RUNNING;
            bat_control->set_striker_angle = 0.0f;

        }
    }
    else
    {
        // 否则全部回0点待命
        bat_control->set_x = 0.0f;
        bat_control->set_y = 0.0f;
        bat_control->set_z = 0.0f;
        bat_control->set_pitch = 0.0f;
        bat_control->set_striker_angle = 0.0f;

        // first_order_filter_cali(&bat_control->input_set_X_filter, bat_control->set_x);
        // first_order_filter_cali(&bat_control->input_set_Y_filter, bat_control->set_y);
        // first_order_filter_cali(&bat_control->input_set_Z_filter, bat_control->set_z);
        // first_order_filter_cali(&bat_control->input_pitch_pos_filter, bat_control->set_pitch);

        delta_arm_inverse_calculation(&bat_control->pos_angle_data.DesireAngle, bat_control->set_x, bat_control->set_y, bat_control->set_z);
    }

    DM8006_Date->target_angle = (limit_bat_pitch(DM8006_Date->target_angle + bat_control->set_pitch));

    HT04_Data.target_angle = bat_control->set_striker_angle;

    // auto_hit_ball_loop(bat_control);
    DM4340_Date[0].target_angle = float_constrain(bat_control->pos_angle_data.DesireAngle.theta1, 9.367, 54.367);
    DM4340_Date[1].target_angle = float_constrain(bat_control->pos_angle_data.DesireAngle.theta2, 8.809, 53.809);
    DM4340_Date[2].target_angle = float_constrain(bat_control->pos_angle_data.DesireAngle.theta3, 9.903, 54.903);

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

float RUD_DirAngle_c(float Angle)
{
    while (Angle > 18000 || Angle < 0)
    {
        if (Angle < 0)
        {
            Angle += 360;
        }
        if (Angle > 360)
        {
            Angle -= 360;
        }
    }
    return (float)Angle;
}

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

/// @brief delta_arm机械臂逆解算
/// @param bat_control 输入的机械臂控制结构体
/// @param x 目标位置x
/// @param y 目标位置y
/// @param z 目标位置z
void delta_arm_inverse_calculation(struct Angle *angle, float x, float y, float z)
{
    // 不使用全局变量
    z -= 235;
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

    angle->theta1 = ((180 * (2 * atan(T1))) / PI) - 0.186f;
    angle->theta2 = ((180 * (2 * atan(T2))) / PI) + 0.208f;
    angle->theta3 = ((180 * (2 * atan(T3))) / PI) - 0.448f;
}

/// @brief dleta机械臂正解出当前中心点坐标
/// @param bat_control 机械臂控制结构体
/// @param theta1 输入角度1
/// @param theta2 输入角度2
/// @param theta3 输入角度3
void Forward_Kinematics(struct Point *Point, float theta1, float theta2, float theta3)
{

    // 输入角度加上数据是为了消除初始角度的些许不同
    theta1 = (PI * (theta1 - 0.208f) / 180);
    theta2 = (PI * (theta2 + 0.448f) / 180);
    theta3 = (PI * (theta3 + 0.186f) / 180);
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

    Point->x = X;
    Point->y = Y;
    Point->z = Z + 235;
}

/// @brief // 计算移动距离
/// @param point1
/// @param point2
/// @return 返回两点之间的距离
float CalDistance2Point(struct Point point1, struct Point point2)
{
    float x_Offset = point1.x - point2.x;
    float y_Offset = point1.y - point2.y;
    float z_Offset = point1.z - point2.z;

    float distance = sqrt(pow(x_Offset, 2) + pow(y_Offset, 2) + pow(z_Offset, 2));

    if (distance < 0.2 && distance > -0.2)
        distance = 0;

    return distance;
}

/// @brief 计算两点的插值
/// @param currentP 当前点
/// @param desiredP 目标点
/// @param t 百分比
/// @return point 返回插值点
struct Point GetPointInLine(struct Point currentP, struct Point desiredP, float t)
{
    struct Point buffer;

    buffer.x = currentP.x - ((currentP.x - desiredP.x) * t);
    buffer.y = currentP.y - ((currentP.y - desiredP.y) * t);
    buffer.z = currentP.z - ((currentP.z - desiredP.z) * t);

    return buffer;
}

