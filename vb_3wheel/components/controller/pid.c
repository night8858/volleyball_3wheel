/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pid鐎圭偤鏁撶悰妤冨皑閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗婵鏁撻弬銈嗗闁跨喐鏋婚幏绋癐D闁跨喐鏋婚幏鐑芥晸婵劕鍤遍柨鐔告灮閹风兘鏁撻弬銈嗗
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 闁跨喐鏋婚幏鐑芥晸閿燂拷
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
  * @param[out]     pid: PID闁跨喓绮ㄩ弸鍕晸閺傘倖瀚归柨鐔告灮閹烽攱瀵氶柨鐔告灮閹凤拷
  * @param[in]      mode: PID_POSITION:闁跨喐鏋婚幏鐑解偓姝匢D
  *                 PID_DELTA: 闁跨喐鏋婚幏鐑芥晸缁插様D
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹凤拷
  * @param[in]      max_iout: pid闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨噦鎷�
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
  * @brief          pid闁跨喐鏋婚幏鐑芥晸閺傘倖瀚�
  * @param[out]     pid: PID闁跨喓绮ㄩ弸鍕晸閺傘倖瀚归柨鐔告灮閹烽攱瀵氶柨鐔告灮閹凤拷
  * @param[in]      ref: 闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗
  * @param[in]      set: 闁跨喎鈧喎鐣鹃崐锟�
  * @retval         pid闁跨喐鏋婚幏鐑芥晸閿燂拷
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
  * @brief          pid 闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹凤拷
  * @param[out]     pid: PID闁跨喓绮ㄩ弸鍕晸閺傘倖瀚归柨鐔告灮閹烽攱瀵氶柨鐔告灮閹凤拷
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

/*********************婢х偤鍣哄寤滻D閹貉冨煑***********************/
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
/********************缂佹繂顕寤滻D閹貉冨煑**************************/

/**
 * @brief 缂佹繂顕寤滻D鐠侊紕鐣�
 * @param s_pid_absolute_t *pid
 * @return float PIDout
 */
void PID_AbsoluteMode(s_pid_absolute_t *pid)
{
    //PID閸氬嫮骞嗛懞鍌氫焊瀹革拷
	pid->Perror = pid->NowError;                  //P閻滎垵濡崑蹇撴▕閺勵垰缍嬮崜宥呬焊瀹革拷
	pid->Ierror += pid->NowError;                 //I閻滎垵濡崑蹇撴▕閺勵垯绗傞悽闈涙倵娑撯偓閻╁瓨瀵旂紒顓炲煂閻滄澘婀惃鍕焊瀹革拷
	pid->Derror = pid->NowError - pid->LastError; //D閻滎垵濡崑蹇撴▕閺勵垰缍嬮崜宥呬焊瀹割喕绗屾稉濠冾偧閸嬪繐妯婇惃鍕▕閸婄》绱濋崡鍐蹭焊瀹割喖顤冮柌锟�
	pid->LastError = pid->NowError;               //閺囧瓨鏌婇崑蹇撴▕	
	//闂勬劕鍩楃粔顖氬瀻閸樺棗褰堕崑蹇撴▕
	if( pid->Ierror >= pid->IerrorLim) pid->Ierror =  pid->IerrorLim;
	else if( pid->Ierror <= -pid->IerrorLim)  pid->Ierror =  -pid->IerrorLim;
	//PID閸氬嫮骞嗛懞鍌濈翻閸戞椽鍣�
	pid->Pout = pid->Kp * pid->Perror;
	pid->Iout = pid->Ki * pid->Ierror;
	pid->Dout = pid->Kd * pid->Derror;
	//PID閹槒绶崙娲櫤
	pid->PIDout = pid->Pout + pid->Iout + pid->Dout;
	//闂勬劕鍩桺ID閹槒绶崙娲櫤
	if(pid->PIDout > pid->PIDoutMAX) pid->PIDout = pid->PIDoutMAX;
	else if(pid->PIDout < -pid->PIDoutMAX) pid->PIDout = -pid->PIDoutMAX;
}
/**
 * @brief 缂佹繂顕寤滻D鐠侊紕鐣�(缁夘垰鍨庨崚鍡欘瀲)
 * @param s_pid_absolute_t *pid
 * @param float integral_apart_val
 * @return float PIDout
 */
void PID_AbsoluteMode_integral_apart(s_pid_absolute_t *pid,float integral_apart_val)
{
    //PID閸氬嫮骞嗛懞鍌氫焊瀹革拷
	pid->Perror = pid->NowError;                  //P閻滎垵濡崑蹇撴▕閺勵垰缍嬮崜宥呬焊瀹革拷
    if(fabs(pid->NowError)<integral_apart_val)
	    pid->Ierror += pid->NowError;                 //I閻滎垵濡崑蹇撴▕閺勵垯绗傞悽闈涙倵娑撯偓閻╁瓨瀵旂紒顓炲煂閻滄澘婀惃鍕焊瀹革拷
    else pid->Ierror = 0;
	pid->Derror = pid->NowError - pid->LastError; //D閻滎垵濡崑蹇撴▕閺勵垰缍嬮崜宥呬焊瀹割喕绗屾稉濠冾偧閸嬪繐妯婇惃鍕▕閸婄》绱濋崡鍐蹭焊瀹割喖顤冮柌锟�
	pid->LastError = pid->NowError;               //閺囧瓨鏌婇崑蹇撴▕
	//闂勬劕鍩楃粔顖氬瀻閸樺棗褰堕崑蹇撴▕
	if( pid->Ierror >= pid->IerrorLim) pid->Ierror =  pid->IerrorLim;
	else if( pid->Ierror <= -pid->IerrorLim)  pid->Ierror =  -pid->IerrorLim;
	//PID閸氬嫮骞嗛懞鍌濈翻閸戞椽鍣�
	pid->Pout = pid->Kp * pid->Perror;
	pid->Iout = pid->Ki * pid->Ierror;
	pid->Dout = pid->Kd * pid->Derror;
	//PID閹槒绶崙娲櫤
	pid->PIDout = pid->Pout + pid->Iout + pid->Dout;
	//闂勬劕鍩桺ID閹槒绶崙娲櫤
	if(pid->PIDout > pid->PIDoutMAX) pid->PIDout = pid->PIDoutMAX;
	else if(pid->PIDout < -pid->PIDoutMAX) pid->PIDout = -pid->PIDoutMAX;
}
/**
 * @brief   PID閸欏倹鏆熼崚婵嗩潗閸栨牭绱濋崣顖欎簰閺€鎯ф躬閸掓繂顫愰崠鏍у毐閺侀鑵戦敍灞肩瘍閸欘垯浜掗弨鎯ф躬瀵邦亞骞嗛柌锟�
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
 * @brief   PID閸欏倹鏆熺挧瀣偓纭风礉閸欘垯浜掗弨鎯ф躬閸掓繂顫愰崠鏍у毐閺侀鑵戦敍灞肩瘍閸欘垯浜掗弨鎯ф躬瀵邦亞骞嗛柌宀嬬礉閹存垹鏁ら弶銉︽杹閸掓澘鎯婇悳顖炲櫡鐠嬪啳鐦崣鍌涙殶
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
 * @brief   閸楁洜骞哖ID
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
 * @brief   娑撹尙楠嘝ID
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
 * @brief 娑撹尙楠嘝ID(闁喎瀹抽悳顖溞濋崚鍡楀瀻缁傦拷)
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

