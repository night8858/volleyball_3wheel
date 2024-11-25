/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pid瀹為敓琛楃尨鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹濮嬮敓鏂ゆ嫹閿熸枻鎷稰ID閿熸枻鎷烽敓濮愬嚱閿熸枻鎷烽敓鏂ゆ嫹
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 閿熸枻鎷烽敓锟�
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"
#include "main.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

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
  * @param[out]     pid: PID閿熺粨鏋勯敓鏂ゆ嫹閿熸枻鎷锋寚閿熸枻鎷�
  * @param[in]      mode: PID_POSITION:閿熸枻鎷烽€歅ID
  *                 PID_DELTA: 閿熸枻鎷烽敓绲嘔D
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷�
  * @param[in]      max_iout: pid閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿燂拷
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid閿熸枻鎷烽敓鏂ゆ嫹
  * @param[out]     pid: PID閿熺粨鏋勯敓鏂ゆ嫹閿熸枻鎷锋寚閿熸枻鎷�
  * @param[in]      ref: 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
  * @param[in]      set: 閿熷€熷畾鍊�
  * @retval         pid閿熸枻鎷烽敓锟�
  */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷�
  * @param[out]     pid: PID閿熺粨鏋勯敓鏂ゆ嫹閿熸枻鎷锋寚閿熸枻鎷�
  * @retval         none
  */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

////////////////////////////////////////////////////////////////////////////////////

/*********************澧為噺寮廝ID鎺у埗***********************/
void PID_IncrementMode(s_pid_increase_t *pid)
{
	 if(pid->kp<0) pid->kp=-pid->kp;
	 if(pid->ki<0) pid->ki=-pid->ki;
	 if(pid->kd<0) pid->kd=-pid->kd;
	
	 if(pid->errNow >5 || pid->errNow<-5)pid->errNow=0;

	 pid->dErrP=pid->errNow-pid->errOld1;
	 pid->dErrI=pid->errNow;
	 pid->dErrD=pid->errNow-2*pid->errOld1+pid->errOld2;
	
	 pid->errOld2=pid->errOld1;
	 pid->errOld1=pid->errNow;
	
	 pid->dCtrOut=pid->kp*pid->dErrP+pid->ki*pid->dErrI+pid->kd*pid->dErrD;
	
	 if(pid->dCtrOut>pid->dOutMAX) pid->dCtrOut=pid->dOutMAX;
     else if(pid->dCtrOut<-pid->dOutMAX) pid->dCtrOut=-pid->dOutMAX;

	
	 if(pid->kp==0 && pid->ki==0 && pid->kd==0) pid->ctrOut=0;
	 else pid->ctrOut+=pid->dCtrOut;
	 
	 if(pid->ctrOut>pid->OutMAX) pid->ctrOut=pid->OutMAX;
     else if(pid->ctrOut<-pid->OutMAX)
   
	 pid->ctrOut=-pid->OutMAX; 
}
/********************缁濆寮廝ID鎺у埗**************************/

/**
 * @brief 缁濆寮廝ID璁＄畻
 * @param s_pid_absolute_t *pid
 * @return float PIDout
 */
void PID_AbsoluteMode(s_pid_absolute_t *pid)
{
    //PID鍚勭幆鑺傚亸宸�
	pid->Perror = pid->NowError;                  //P鐜妭鍋忓樊鏄綋鍓嶅亸宸�
	pid->Ierror += pid->NowError;                 //I鐜妭鍋忓樊鏄笂鐢靛悗涓€鐩存寔缁埌鐜板湪鐨勫亸宸�
	pid->Derror = pid->NowError - pid->LastError; //D鐜妭鍋忓樊鏄綋鍓嶅亸宸笌涓婃鍋忓樊鐨勫樊鍊硷紝鍗冲亸宸閲�
	pid->LastError = pid->NowError;               //鏇存柊鍋忓樊	
	//闄愬埗绉垎鍘嗗彶鍋忓樊
	if( pid->Ierror >= pid->IerrorLim) pid->Ierror =  pid->IerrorLim;
	else if( pid->Ierror <= -pid->IerrorLim)  pid->Ierror =  -pid->IerrorLim;
	//PID鍚勭幆鑺傝緭鍑洪噺
	pid->Pout = pid->Kp * pid->Perror;
	pid->Iout = pid->Ki * pid->Ierror;
	pid->Dout = pid->Kd * pid->Derror;
	//PID鎬昏緭鍑洪噺
	pid->PIDout = pid->Pout + pid->Iout + pid->Dout;
	//闄愬埗PID鎬昏緭鍑洪噺
	if(pid->PIDout > pid->PIDoutMAX) pid->PIDout = pid->PIDoutMAX;
	else if(pid->PIDout < -pid->PIDoutMAX) pid->PIDout = -pid->PIDoutMAX;
}
/**
 * @brief 缁濆寮廝ID璁＄畻(绉垎鍒嗙)
 * @param s_pid_absolute_t *pid
 * @param float integral_apart_val
 * @return float PIDout
 */
void PID_AbsoluteMode_integral_apart(s_pid_absolute_t *pid,float integral_apart_val)
{
    //PID鍚勭幆鑺傚亸宸�
	pid->Perror = pid->NowError;                  //P鐜妭鍋忓樊鏄綋鍓嶅亸宸�
    if(fabs(pid->NowError)<integral_apart_val)
	    pid->Ierror += pid->NowError;                 //I鐜妭鍋忓樊鏄笂鐢靛悗涓€鐩存寔缁埌鐜板湪鐨勫亸宸�
    else pid->Ierror = 0;
	pid->Derror = pid->NowError - pid->LastError; //D鐜妭鍋忓樊鏄綋鍓嶅亸宸笌涓婃鍋忓樊鐨勫樊鍊硷紝鍗冲亸宸閲�
	pid->LastError = pid->NowError;               //鏇存柊鍋忓樊
	//闄愬埗绉垎鍘嗗彶鍋忓樊
	if( pid->Ierror >= pid->IerrorLim) pid->Ierror =  pid->IerrorLim;
	else if( pid->Ierror <= -pid->IerrorLim)  pid->Ierror =  -pid->IerrorLim;
	//PID鍚勭幆鑺傝緭鍑洪噺
	pid->Pout = pid->Kp * pid->Perror;
	pid->Iout = pid->Ki * pid->Ierror;
	pid->Dout = pid->Kd * pid->Derror;
	//PID鎬昏緭鍑洪噺
	pid->PIDout = pid->Pout + pid->Iout + pid->Dout;
	//闄愬埗PID鎬昏緭鍑洪噺
	if(pid->PIDout > pid->PIDoutMAX) pid->PIDout = pid->PIDoutMAX;
	else if(pid->PIDout < -pid->PIDoutMAX) pid->PIDout = -pid->PIDoutMAX;
}
/**
 * @brief   PID鍙傛暟鍒濆鍖栵紝鍙互鏀惧湪鍒濆鍖栧嚱鏁颁腑锛屼篃鍙互鏀惧湪寰幆閲�
 * @param 	PID_AbsoluteType *pid
 * @param   float kp
 * @param   float ki
 * @param   float kd
 * @param   float errILim
 * @param   float MaxOutCur		
 * @return None
 */
void pid_abs_param_init(s_pid_absolute_t *pid, float kp, float ki, float kd, float errILim, float MaxOutCur)
{
	memset(pid,0,sizeof(s_pid_absolute_t));
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->IerrorLim = errILim;
	pid->PIDoutMAX = MaxOutCur;
}
/**
 * @brief   PID鍙傛暟璧嬪€硷紝鍙互鏀惧湪鍒濆鍖栧嚱鏁颁腑锛屼篃鍙互鏀惧湪寰幆閲岋紝鎴戠敤鏉ユ斁鍒板惊鐜噷璋冭瘯鍙傛暟
 * @param 	PID_AbsoluteType *pid
 * @param   float kp
 * @param   float ki
 * @param   float kd
 * @param   float errILim
 * @param   float MaxOutCur		
 * @return None
 */
void pid_abs_evaluation(s_pid_absolute_t *pid, float kp, float ki, float kd, float errILim, float MaxOutCur)
{
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->IerrorLim = errILim;
	pid->PIDoutMAX = MaxOutCur;
}

/**
 * @brief   鍗曠幆PID
 * @param 	s_pid_absolute_t *single_pid
 * @param   float get
 * @param   float targeti		
 * @return  float pid_output
 * @attention None
 */
int16_t motor_single_loop_PID(s_pid_absolute_t *single_pid , float target , float get)
{
	static float pid_output;
	single_pid->NowError = (float)(target - get);
	PID_AbsoluteMode(single_pid);
	pid_output = single_pid->PIDout;

	return pid_output;
}
/**
 * @brief   涓茬骇PID
 * @param 	s_pid_absolute_t *pos_pid
 * @param   s_pid_absolute_t *spd_pid
 * @param   float externGet
 * @param   float externSet
 * @param   float internGet		
 * @return  pid_output(float)
 * @attention None
 */
float motor_double_loop_PID(s_pid_absolute_t *pos_pid, s_pid_absolute_t *spd_pid, float externGet, float externSet, float internGet)
{
	static float pid_output;
	static float out_st;

	pos_pid->NowError = (float)externSet - (float)externGet;
	PID_AbsoluteMode(pos_pid);
	out_st = pos_pid->PIDout;

	spd_pid->NowError = out_st - (float)internGet;
	PID_AbsoluteMode(spd_pid);
	pid_output = spd_pid->PIDout;

	return pid_output;
}
/**
 * @brief 涓茬骇PID(閫熷害鐜Н鍒嗗垎绂�)
 * @param s_pid_absolute_t *pos_pid
 * @param s_pid_absolute_t *spd_pid
 * @param float externGet
 * @param float externSet
 * @param float internGet
 * @param float integral_apart_val
 * @return pid_output(float)
 * @attention None
 */
float motor_double_loop_PID_integral_apart(s_pid_absolute_t *pos_pid, s_pid_absolute_t *spd_pid, float externGet, float externSet, \
                                            float internGet,float integral_apart_val)
{
	static float pid_output;
	static float out_st;

	pos_pid->NowError = (float)externSet - (float)externGet;
    PID_AbsoluteMode(pos_pid);
	out_st = pos_pid->PIDout;

	spd_pid->NowError = out_st - (float)internGet;
	PID_AbsoluteMode_integral_apart(spd_pid,integral_apart_val);
	pid_output = spd_pid->PIDout;

	return pid_output;
}

