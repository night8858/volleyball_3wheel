/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "struct_typedef.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;


/********increase mode*******/
typedef struct 
{
  float kp;
	float ki;
	float kd;
	
	float dErrP;
	float dErrI;
	float dErrD;
	
	float errNow;
	float errOld1;
	float errOld2;

	float dCtrOut;
	float dOutMAX;
	float ctrOut;
	float OutMAX;
	
}s_pid_increase_t;//增量式
/********absolute mode*******/
//PID参数结构体
typedef struct
{
  float Kp;//比例环节参数
	float Ki;//积分环节参数
	float Kd;//微分环节参数

	float Perror;//比例偏差值
	float Ierror;//积分偏差值
	float Derror;//微分偏差值
	
	float Pout;//比例环节输出量
	float Iout;//积分环节输出量
	float Dout;//微分环节输出量
	
	float NowError;   //当前偏差
	float LastError;  //上次偏差
  float IerrorApartVal;//积分分离限制值
	float IerrorLim;  //积分偏差上限
	float PIDout;     //PID运算后输出量
	float PIDoutMAX;  //PID运算后输出量上限
}s_pid_absolute_t; //绝对式

//实验室传统使用的pid函数
void PID_IncrementMode(s_pid_increase_t *pid);
void PID_AbsoluteMode(s_pid_absolute_t *pid);
void pid_abs_param_init(s_pid_absolute_t *pid, float kp, float ki, float kd, float errILim, float MaxOutCur);
void pid_abs_evaluation(s_pid_absolute_t *pid, float kp, float ki, float kd, float errILim, float MaxOutCur);
int16_t motor_single_loop_PID(s_pid_absolute_t *single_pid, float target , float get);
float motor_double_loop_PID(s_pid_absolute_t *pos_pid, s_pid_absolute_t *spd_pid, float externGet, float externSet, float internGet);
float motor_double_loop_PID_integral_apart(s_pid_absolute_t *pos_pid, s_pid_absolute_t *spd_pid, float externGet, float externSet, \
float internGet,float integral_apart_val);




/////////////////////////////////////////////////////
//以下为官方使用的pid函数
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

#endif
