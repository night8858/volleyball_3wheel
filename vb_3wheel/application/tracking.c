#include "tracking.h"
#include "Variables.h"
#include "math.h"
#include "pc_interface.h"
#include "pid.h"
#include "chassis.h"
#include "bat_control.h"
#include "user_lib.h"

s_tracking_data_t tracking_data;

extern kalman1_state s_kalman1_hik_x;                   //卡尔曼滤波器_海康x数据
extern s_pid_absolute_t volleyball_track_far_PID_pos;   // 排球跟踪PID结构体，位置环，排球位置为参考数据
extern s_pid_absolute_t volleyball_track_far_PID_speed; // 排球跟踪PID结构体，速度环，车辆y速度为参考数据
extern s_pid_absolute_t volleyball_track_close_PID_pos;        //排球近处定位跟踪PID结构体，位置环，球拍下相机排球位置为参考数据
extern s_pid_absolute_t volleyball_track_close_PID_speed;      //排球近处定位跟踪PID结构体，速度环，车辆方向速度为参考数据


extern visInf_t s_visionInform;                         // 视觉数据结构体
extern chassis_control_t chassis_control;               // 底盘控制结构体
extern bat_control_t bat_control;                       // 球拍控制结构体
extern s_Dji_motor_data_t motor_Date[4];                // 储存电机数据结构体

/**
 * @brief  初始化一维卡尔曼滤波器状态
 * @param  state    卡尔曼滤波器状态结构体指针
 * @param  init_x   状态初始值
 * @param  init_p   初始协方差
 * @param  init_q   过程噪声协方差
 * @param  init_r   测量噪声协方差
 * @note   状态转移矩阵A和观测矩阵H默认设为1
 */
void kalman1_init(kalman1_state *state, float init_x, float init_p, float init_q, float init_r)
{
    state->x = init_x;
    state->p = init_p;
    state->A = 1;
    state->H = 1;
    state->q = init_q;//10e-6;  /* 预测噪声 */
    state->r = init_r;//10e-5;  /*数据误差？ */  // 155 300
}

/**
 * @brief 一维卡尔曼滤波器
 * @param state 卡尔曼滤波器状态结构体指针
 * @param z_measure 测量值
 * @return 滤波后的估计值
 *
 * @details 执行一维卡尔曼滤波算法:
 *          1. 预测阶段：计算先验状态估计和误差协方差
 *          2. 更新阶段：计算卡尔曼增益，更新状态估计和误差协方差
 */
float kalman1_filter(kalman1_state *state, float z_measure)
{
    /* 预测计算 */
    state->x = state->A * state->x;
    state->p = state->A * state->A * state->p + state->q;  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* 数据 */
    state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);
    state->x = state->x + state->gain * (z_measure - state->H * state->x);
    state->p = (1 - state->gain * state->H) * state->p;

    return state->x;
}


/*
 * @brief   
 *   Init fields of structure @kalman1_state.
 *   I make some defaults in this init function:
 *     A = {{1, 0.1}, {0, 1}};
 *     H = {1,0}; 
 *   and @q,@r are valued after prior tests. 
 *
 *   NOTES: Please change A,H,q,r according to your application.
 *
 * @inputs  
 * @outputs 
 * @retval  
 */
void kalman2_init(kalman2_state *state, const float *init_x, float (*init_p)[2])
{
    state->x[0]    = init_x[0];
    state->x[1]    = init_x[1];
    state->p[0][0] = init_p[0][0];
    state->p[0][1] = init_p[0][1];
    state->p[1][0] = init_p[1][0];
    state->p[1][1] = init_p[1][1];
    //state->A       = {{1, 0.1}, {0, 1}};
    state->A[0][0] = 1;
    state->A[0][1] = 0.1;
    state->A[1][0] = 0;
    state->A[1][1] = 1;
    //state->H       = {1,0};
    state->H[0]    = 1;
    state->H[1]    = 0;
    //state->q       = {{10e-6,0}, {0,10e-6}};  /* measure noise convariance */
    state->q[0]    = 10e-7;
    state->q[1]    = 10e-7;
    state->r       = 10e-7;  /* estimated error convariance */
}

/*
 * @brief   
 *   2 Dimension kalman filter
 * @inputs  
 *   state - Klaman filter structure
 *   z_measure - Measure value
 * @outputs 
 *   state->x[0] - Updated state value, Such as angle,velocity
 *   state->x[1] - Updated state value, Such as diffrence angle, acceleration
 *   state->p    - Updated estimated error convatiance matrix
 * @retval  
 *   Return value is equals to state->x[0], so maybe angle or velocity.
 */
float kalman2_filter(kalman2_state *state, float z_measure)
{
    float temp0 = 0.0f;
    float temp1 = 0.0f;
    float temp = 0.0f;

    /* Step1: Predict */
    state->x[0] = state->A[0][0] * state->x[0] + state->A[0][1] * state->x[1];
    state->x[1] = state->A[1][0] * state->x[0] + state->A[1][1] * state->x[1];
    /* p(n|n-1)=A^2*p(n-1|n-1)+q */
    state->p[0][0] = state->A[0][0] * state->p[0][0] + state->A[0][1] * state->p[1][0] + state->q[0];
    state->p[0][1] = state->A[0][0] * state->p[0][1] + state->A[1][1] * state->p[1][1];
    state->p[1][0] = state->A[1][0] * state->p[0][0] + state->A[0][1] * state->p[1][0];
    state->p[1][1] = state->A[1][0] * state->p[0][1] + state->A[1][1] * state->p[1][1] + state->q[1];

    /* Step2: Measurement */
    /* gain = p * H^T * [r + H * p * H^T]^(-1), H^T means transpose. */
    temp0 = state->p[0][0] * state->H[0] + state->p[0][1] * state->H[1];
    temp1 = state->p[1][0] * state->H[0] + state->p[1][1] * state->H[1];
    temp  = state->r + state->H[0] * temp0 + state->H[1] * temp1;
    state->gain[0] = temp0 / temp;
    state->gain[1] = temp1 / temp;
    /* x(n|n) = x(n|n-1) + gain(n) * [z_measure - H(n)*x(n|n-1)]*/
    temp = state->H[0] * state->x[0] + state->H[1] * state->x[1];
    state->x[0] = state->x[0] + state->gain[0] * (z_measure - temp); 
    state->x[1] = state->x[1] + state->gain[1] * (z_measure - temp);

    /* Update @p: p(n|n) = [I - gain * H] * p(n|n-1) */
    state->p[0][0] = (1 - state->gain[0] * state->H[0]) * state->p[0][0];
    state->p[0][1] = (1 - state->gain[0] * state->H[1]) * state->p[0][1];
    state->p[1][0] = (1 - state->gain[1] * state->H[0]) * state->p[1][0];
    state->p[1][1] = (1 - state->gain[1] * state->H[1]) * state->p[1][1];

    return state->x[0];
}


void pc_data_check(void)
{

    // 由于视觉数据还不稳定，所以当视觉数据为0时，将视觉数据置为-1，防止PID控制出错
    if (s_visionInform.x_hikvision == 0 && s_visionInform.y_hikvision == 0)
    {
        s_visionInform.z_hikvision = -1;
    }
    if (s_visionInform.x_usbcam == 0 && s_visionInform.y_usbcam == 0)
    {
        s_visionInform.z_usbcam = -1;
    }
}

/**
 * @brief 初始化追踪数据结构体和PID控制器参数
 *
 * @param tracking_data 追踪数据结构体指针
 *
 * @details 设置海康摄像头和USB摄像头的中心目标值，
 *          并初始化排球追踪PID控制器的位置环参数
 */
void tracking_init(s_tracking_data_t *tracking_data)
{
    fp32 unm={0.8};

    first_order_filter_init(&tracking_data->HIKVISION_x_filter , 0.9 , &unm);
    first_order_filter_init(&tracking_data->HIKVISION_y_filter , 0.9 , &unm);

    first_order_filter_init(&tracking_data->USBCAM_x_filter , 0.9 , &unm);
    first_order_filter_init(&tracking_data->USBCAM_y_filter , 0.9 , &unm);

    //kalman1_init(&s_kalman1_hik_x, 0, 0.4 , 0.0001 ,0.0001);
    tracking_data->mid_traget_HIKVISION_x = 720;
    tracking_data->mid_traget_HIKVISION_y = 540;
    tracking_data->mid_traget_USBCAM_x = 320;
    tracking_data->mid_traget_USBCAM_y = 240;

    // 初始化追踪pid参数
    pid_abs_param_init(&volleyball_track_far_PID_pos,   2, 0, 0, 1000, 6000);
    pid_abs_param_init(&volleyball_track_far_PID_speed, 1, 0, 0, 1000, 6000);

    pid_abs_param_init(&volleyball_track_close_PID_pos,   10, 0, 0, 1000, 6000);
    pid_abs_param_init(&volleyball_track_close_PID_speed, 1, 0, 0, 1000, 6000);
}


void pc_data_filter(void)
{
    // 由于视觉数据还不稳定，进行一次滤波
    first_order_filter_cali(&tracking_data.HIKVISION_x_filter , s_visionInform.x_hikvision);
    first_order_filter_cali(&tracking_data.HIKVISION_y_filter , s_visionInform.y_hikvision);
    first_order_filter_cali(&tracking_data.USBCAM_x_filter    , s_visionInform.x_usbcam);
    first_order_filter_cali(&tracking_data.USBCAM_y_filter    , s_visionInform.y_usbcam);

}

void ball_real_pos_calc(void)
{

}

/**
 * @brief 排球追踪控制函数
 *
 * 该函数实现排球的位置追踪控制，通过PID控制器计算底盘速度：
 * 1. 计算目标位置与当前位置的误差
 * 2. 通过位置PID控制器计算期望速度
 * 3. 通过速度PID控制器计算最终输出
 * 4. 设置底盘横向速度
 */
void chassis_volleyball_track(void)
{
    pd_data_filter();
    if (s_visionInform.z_hikvision != -1)
    {
        float motor_speed_calc[3] = 0;
        volleyball_track_far_PID_pos.NowError = tracking_data.HIKVISION_x_filter.out - tracking_data.mid_traget_HIKVISION_x ; 
        PID_AbsoluteMode(&volleyball_track_far_PID_pos);
        volleyball_track_far_PID_speed.NowError = volleyball_track_far_PID_pos.PIDout - chassis_control.vy;
        PID_AbsoluteMode(&volleyball_track_far_PID_speed);
        // 相当于平行移动寻找排球位置
        if(s_visionInform.z_hikvision < 3)
        {
            //在x方向上靠近排球//
            // if(s_visionInform.z_usbcam != 0)
            // {
            //     volleyball_track_close_PID_pos.NowError =s_visionInform.y_usbcam -  tracking_data.mid_traget_USBCAM_y ; 
            //     PID_AbsoluteMode(&volleyball_track_close_PID_pos);
            //     volleyball_track_close_PID_speed.NowError = volleyball_track_close_PID_pos.PIDout - chassis_control.vx;
            //     PID_AbsoluteMode(&volleyball_track_close_PID_speed);
                
            //     //chassis_control.vx_set = volleyball_track_close_PID_speed.PIDout;
            // }
        }
        // 决定底盘速度
        chassis_control.vy_set = volleyball_track_far_PID_speed.PIDout;
        // 计算各个电机速度,三全向解算
        motor_speed_calc[0] = 0.4067f * chassis_control.wz_set + chassis_control.vx_set * sqrt(3) / 2 - chassis_control.vy_set / 2;
        motor_speed_calc[1] = 0.4067f * chassis_control.wz_set - chassis_control.vx_set * sqrt(3) / 2 - chassis_control.vy_set / 2;
        motor_speed_calc[2] = 0.4067f * chassis_control.wz_set + chassis_control.vy_set;
        
        for (uint8_t i = 0; i < motor_3505_num; i++)
        {
            motor_Date[i].target_motor_speed = motor_speed_calc[i];
        }
        chassis_speed_pid_calc(&chassis_control);
        CAN_cmd_3508(motor_Date[0].out_current, motor_Date[1].out_current, motor_Date[2].out_current, motor_Date[3].out_current);

     }
     //else
    // {
    //     chassis_stop_pid_calc(&chassis_control);
    //     CAN_cmd_3508(motor_Date[0].out_current, motor_Date[1].out_current, motor_Date[2].out_current, motor_Date[3].out_current);
    // }

    for (uint8_t i = 0; i < motor_3505_num; i++)
    {
        motor_Date[i].target_motor_ang = motor_Date[i].serial_motor_ang;
    }
}

/*
当海康识别的距离大于某一值（目前只好调参确定）后，不在x方向追踪排球（即不向前靠近追踪排球），
当排球距离小于给定值时，向前移动尝试去接球，
在通过球拍向前倾斜的45度大致判断球的位置，使用球拍击球
*/