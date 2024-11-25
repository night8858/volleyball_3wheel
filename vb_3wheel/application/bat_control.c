#include "bat_control.h"
#include "can_recv.h"
#include "can.h"
#include "bsp_usart.h"

#include "math.h"

#include "cmsis_os.h"

#define DEBUG 1 // 调试开关

extern DM4340_motor_data_t DM4340_Date[3];    // 引用DM4340回传数据结构体
extern ht_motor_data_t HT04_Data;    // 引用HT04回传数据结构体

struct Data Data;

bat_control_t bat_control;
Ball_Pos RX_ball_pos;
first_order_filter_type_t filter_angle;
top_inverse_calculation_angle inverse_calculation_angle;

float angle[3];

int MMPerLinearSegment = 6;


// 球拍控制主循环
void functional_zone_task(void const * argument)
{
    //vTaskDelay(4000);
    ///////////////初始化函数///////////////
    bat_motor_Init(&bat_control);
    // ball_track_pid_init();
    ///////////////初始化函数///////////////

    while (1)
    {
        //HT04_motor_PID_Control(&hcan1 , 0x50 ,0.3);

        bat_data_update(&bat_control);
        top_RC_control_set(&bat_control);
        DM_motor_pid_clac(&bat_control);
        DM_motor_control(&bat_control);
        osDelay(1);
        //////////////////调试函数///////////////
        if (DEBUG)
        {
            uart_dma_printf(&huart1, "%4.3f ,%4.3f ,%4.3f,%4.3f ,%4.3f ,%4.3f\n",
                            DM4340_Date[1].real_angle,
                            DM4340_Date[1].target_angle,
                            DM4340_Date[1].out_current,
                            DM4340_Date[1].esc_back_speed,
                            HT04_Data.esc_back_speed,
                            DM4340_Date[2].target_angle);
                            /*                                                        bat_control.top_inverse_angle.angle[0],
                            bat_control.top_inverse_angle.angle[1],
                            bat_control.top_inverse_angle.angle[2]);*/

        } 
        //Forward_Kinematics(DM4340_Date[0].real_angle, DM4340_Date[1].real_angle,DM4340_Date[2].real_angle);
        //delta_arm_inverse_calculation(0,0,0, &inverse_calculation_angle);
        // 电机角度更新
    }
}


/// @brief 球拍部分初始化
/// @param bat_control 
void bat_motor_Init(bat_control_t *bat_control)
{
    //初始化电机
    for (int i = 0; i < 3; i++)
    {
        DM_motor_start(&hcan2 , i);
        osDelay(500);
    }
    
    HT04_motor_start(&hcan1, HT8115_M1);
    osDelay(100);
    //获取当前电机空间坐标
    Forward_Kinematics(bat_control ,DM4340_Date[0].real_angle, DM4340_Date[1].real_angle,DM4340_Date[2].real_angle);

    //获取遥控器指针
    bat_control->control_RC = get_remote_control_point();

    //设定初始目标角度
    DM4340_Date[0].target_angle = float_constrain(50, 9.367, 54.367);
    DM4340_Date[1].target_angle = float_constrain(50, 8.809, 53.809);
    DM4340_Date[2].target_angle = float_constrain(50, 9.903, 54.903);

    /***********初始化双环PID参数***********/
    for ( int i = 0; i < 3; i++)
    {
        pid_abs_param_init(&bat_control->DM_Motor_PID_angle[i] ,DM4340_ANGLE_PID_KP , DM4340_ANGLE_PID_KI , DM4340_ANGLE_PID_KD , DM4340_ANGLE_PID_MAX_IOUT , DM4340_ANGLE_PID_MAX_OUT );
        pid_abs_param_init(&bat_control->DM_Motor_PID_speed[i] ,DM4340_SPEED_PID_KP , DM4340_SPEED_PID_KI , DM4340_SPEED_PID_KD , DM4340_SPEED_PID_MAX_IOUT , DM4340_SPEED_PID_MAX_OUT );
        
    }

        pid_abs_param_init(&bat_control->HT_Motor_PID_angle ,HT04_ANGLE_PID_KP , HT04_ANGLE_PID_KI , HT04_ANGLE_PID_KD , HT04_ANGLE_PID_MAX_IOUT , HT04_ANGLE_PID_MAX_OUT );
        pid_abs_param_init(&bat_control->HT_Motor_PID_speed ,HT04_SPEED_PID_KP , HT04_SPEED_PID_KI , HT04_SPEED_PID_KD , HT04_SPEED_PID_MAX_IOUT , HT04_SPEED_PID_MAX_OUT );
        
    //for (int i = 0; i < 3; i++)
    //{
    //    MD_motor_SendCurrent(&hcan2, i + 1, bat_control->pid_out[i]);
    //}
}

/// @brief 达妙电机的pid计算
/// @param bat_control 
void DM_motor_pid_clac(bat_control_t *bat_control)
{
    static float pid_out[3];
    
    for (int i = 0; i < 3; i++)
    {
        pid_out[i] = motor_double_loop_PID(&bat_control->DM_Motor_PID_angle[i] , &bat_control->DM_Motor_PID_speed[i] , DM4340_Date[i].real_angle, DM4340_Date[i].target_angle ,DM4340_Date[i].esc_back_speed);
        DM4340_Date[i].out_current = pid_out[i];
    }

        HT04_Data.out_current = motor_double_loop_PID(&bat_control->HT_Motor_PID_angle , &bat_control->HT_Motor_PID_speed , HT04_Data.real_angle , HT04_Data.target_position , HT04_Data.esc_back_speed);
}

/// @brief 更新球拍的数据
/// @param bat_control 
void bat_data_update(bat_control_t *bat_control)
{
    Forward_Kinematics(bat_control ,DM4340_Date[0].real_angle, DM4340_Date[1].real_angle,DM4340_Date[2].real_angle);

}
/// @brief 发送电机控制信号
/// @param bat_control 
void DM_motor_control(bat_control_t *bat_control)
{

        MD_motor_SendCurrent(&hcan2 , 0x01 , DM4340_Date[0].out_current);
        osDelay(1);
        MD_motor_SendCurrent(&hcan2 , 0x02 , DM4340_Date[1].out_current);
        osDelay(1);
        MD_motor_SendCurrent(&hcan2 , 0x03 , DM4340_Date[2].out_current);
        osDelay(1);
    
}



void top_RC_control_set(bat_control_t *bat_control)
{
    delta_arm_inverse_calculation(bat_control , 0 , 0, 0);

    DM4340_Date[0].target_angle = float_constrain(bat_control->top_inverse_angle.angle[0], 9.367, 54.367);
    DM4340_Date[1].target_angle = float_constrain(bat_control->top_inverse_angle.angle[1], 8.809, 53.809);
    DM4340_Date[2].target_angle = float_constrain(bat_control->top_inverse_angle.angle[2], 9.903, 54.903);

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
void froce_calculate(void)
{
    for (int i = 0; i < 3; i++)
    {
            DM4340_Date[i].T_calc = DM_MOTOR_t_ff + DM_MOTOR_KP * (DM4340_Date[i].target_angle - DM4340_Date[i].real_angle) + DM_MOTOR_KD * (0 - DM4340_Date[i].real_speed);
 
    }
    
}

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
void delta_arm_inverse_calculation(bat_control_t *bat_control ,float x, float y, float z)
{
    // 不使用全局变量
    z -= 235;
    double A1 = (x * x + y * y + z * z + 35.4084f - 2 * x * (-5.9505f)) / (300.0f);
    double B1 = -((-5.9505f) - x);
    double C1 = z;

    double A2 = (x * x + y * y + z* z + 35.4084f + (x - 1.73205f * y) * (-5.9505f)) / L1;
    double B2 = -2 * ((-5.9505f)) - (x - 1.73205f * y);
    double C2 = 2 * z;

    double A3 = (x * x + y * y + z* z + 35.4084f + (x - 1.73205f * y) * (-5.9505f)) / L1;
    double B3 = -2 * ((-5.9505f)) - (x + 1.73205f  * y);
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

    bat_control->top_inverse_angle.angle[0] = ((180 * (2 * atan(T1))) / PI) - 0.186f;
    bat_control->top_inverse_angle.angle[1] = ((180 * (2 * atan(T2))) / PI) + 0.208f;
    bat_control->top_inverse_angle.angle[2] = ((180 * (2 * atan(T3))) / PI) - 0.448f;
}


/// @brief dleta机械臂正解出当前中心点坐标
/// @param bat_control 机械臂控制结构体
/// @param theta1 输入角度1
/// @param theta2 输入角度2
/// @param theta3 输入角度3
void Forward_Kinematics(bat_control_t *bat_control, float theta1, float theta2, float theta3)
{

    //输入角度加上数据是为了消除初始角度的些许不同
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

    bat_control->bat_space_pos.real_x = X;
    bat_control->bat_space_pos.real_y = Y;
    bat_control->bat_space_pos.real_z = Z + 235;

}
//// pos的PID初始化
// static void POS_PID_init(M6020_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
//{
//     if (pid == NULL)
//     {
//         return;
//     }
//     pid->kp = kp;
//     pid->ki = ki;
//     pid->kd = kd;
//
//     pid->err = 0.0f;
//     pid->get = 0.0f;
//
//     pid->max_iout = max_iout;
//     pid->max_out = maxout;
// }
//
//// 6020的PID计算
// static fp32 POS_PID_calc(M6020_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
//{
//     fp32 err;
//     if (pid == NULL)
//     {
//         return 0.0f;
//     }
//     pid->get = get;
//     pid->set = set;
//
//     err = set - get;
//     pid->err = rad_format(err);
//     pid->Pout = pid->kp * pid->err;
//     pid->Iout += pid->ki * pid->err;
//     pid->Dout = pid->kd * error_delta;
//     abs_limit(&pid->Iout, pid->max_iout);
//     pid->out = pid->Pout + pid->Iout + pid->Dout;
//     abs_limit(&pid->out, pid->max_out);
//     return pid->out;
// }

// 延时函数
void delay(int count)
{
    int i;
    for (i = 1; i <= count; i++)
        ;
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
struct Point GetPointInLine(struct Point currentP, struct Point desiredP, float t) 
{
    struct Point buffer;

    buffer.x = currentP.x - ((currentP.x - desiredP.x) * t);
    buffer.y = currentP.y - ((currentP.y - desiredP.y) * t);
    buffer.z = currentP.z - ((currentP.z - desiredP.z) * t);

    return buffer;
}

bool_t auto_hit_ball_loop(int speed)
{
    float distance2Point = CalDistance2Point(Data.CurrentPoint, Data.DesiredPoint); // 计算移动距离
        if (distance2Point == 0) // 距离=0
    {
        // Data.IsExecutedGcode = true;
        return 0;
    }

    struct Angle angle_; // 目标位置逆解
    //Inverse_Kinematics(Data.DesiredPoint.x, Data.DesiredPoint.y, Data.DesiredPoint.z, &angle_);

        int NumberSegment = floorf(distance2Point / MMPerLinearSegment); // 求段数
    if (NumberSegment < 1)
        NumberSegment = 1;

    float tbuffer;
    // struct Angle lastAngle = Data.CurrentAngle;//有新的目标角度时，当前角度即为过去
    struct Angle currentAngle;
    float mm_per_seg = distance2Point / NumberSegment; // 每段实际长度

        for (uint16_t i = 1; i <= NumberSegment; i++) // 遍历每一段
    {
        tbuffer = (float)i / NumberSegment;
        struct Point pointBuffer = GetPointInLine(Data.CurrentPoint, Data.DesiredPoint, tbuffer); // 求插值点坐标
        //Inverse_Kinematics(pointBuffer.x, pointBuffer.y, pointBuffer.z, &currentAngle);           // 插值点坐标逆解

        //电机控制部分


        ///////////

        // UploadSegment(lastAngle, currentAngle, mm_per_seg, i - 1);
        // lastAngle = currentAngle;
    }

    Data.CurrentPoint = Data.DesiredPoint; // 遍历到最后，目标位置即为当前位置
    // Data.CurrentAngle = currentAngle;      // 目标角度即为当前角度
    Data.CurrentAngle = currentAngle;

    return 1;
}
