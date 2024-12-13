/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pid閻庡湱鍋ら弫鎾舵偘濡ゅ啫鐨戦柟椋庡厴閺佹捇寮妶鍡楊伓闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏跺┑顔碱儔閺佹捇寮妶鍡楊伓闂佽法鍠愰弸濠氬箯缁嬬檺D闂佽法鍠愰弸濠氬箯閻戣姤鏅稿┑顔藉姇閸ら亶鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柨鐕傛嫹
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
  * @param[out]     pid: PID闂佽法鍠撶划銊╁几閸曨垱鏅搁柡鍌樺€栫€氬綊鏌ㄩ悢鍛婄伄闁圭兘鏀辩€垫岸鏌ㄩ悢鍛婄伄闁瑰嚖鎷�
  * @param[in]      mode: PID_POSITION:闂佽法鍠愰弸濠氬箯閻戣В鍋撳鍖
  *                 PID_DELTA: 闂佽法鍠愰弸濠氬箯閻戣姤鏅哥紒鎻掓D
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氬綊鏌ㄩ悢鍛婄伄闁瑰嚖鎷�
  * @param[in]      max_iout: pid闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ櫐閹凤拷
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
  * @brief          pid闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氾拷
  * @param[out]     pid: PID闂佽法鍠撶划銊╁几閸曨垱鏅搁柡鍌樺€栫€氬綊鏌ㄩ悢鍛婄伄闁圭兘鏀辩€垫岸鏌ㄩ悢鍛婄伄闁瑰嚖鎷�
  * @param[in]      ref: 闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏�
  * @param[in]      set: 闂佽法鍠庨埀顒傚枎閻ｉ箖宕愰敓锟�
  * @retval         pid闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柨鐕傛嫹
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
  * @brief          pid 闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氬綊鏌ㄩ悢鍛婄伄闁瑰嚖鎷�
  * @param[out]     pid: PID闂佽法鍠撶划銊╁几閸曨垱鏅搁柡鍌樺€栫€氬綊鏌ㄩ悢鍛婄伄闁圭兘鏀辩€垫岸鏌ㄩ悢鍛婄伄闁瑰嚖鎷�
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

/*********************濠⒀呭仱閸ｅ搫顕ｅ婊籇闁硅矇鍐ㄧ厬***********************/
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
/********************缂備焦绻傞顔碱嚕瀵ゆ换D闁硅矇鍐ㄧ厬**************************/

/**
 * @brief 缂備焦绻傞顔碱嚕瀵ゆ换D閻犱緤绱曢悾锟�
 * @param s_pid_absolute_t *pid
 * @return float PIDout
 */
void PID_AbsoluteMode(s_pid_absolute_t *pid)
{
    //PID闁告艾瀚獮鍡涙嚍閸屾矮鐒婄€归潻鎷�
	pid->Perror = pid->NowError;                  //P闁绘粠鍨垫俊顓㈠磻韫囨挻鈻曢柡鍕靛灠缂嶅宕滃鍛剨鐎归潻鎷�
	pid->Ierror += pid->NowError;                 //I闁绘粠鍨垫俊顓㈠磻韫囨挻鈻曢柡鍕靛灟缁楀倿鎮介棃娑欏€靛☉鎾亾闁烩晛鐡ㄧ€垫梻绱掗鐐茬厒闁绘粍婢樺﹢顏堟儍閸曨偂鐒婄€归潻鎷�
	pid->Derror = pid->NowError - pid->LastError; //D闁绘粠鍨垫俊顓㈠磻韫囨挻鈻曢柡鍕靛灠缂嶅宕滃鍛剨鐎瑰壊鍠曠粭灞剧▔婵犲喚鍋ч柛瀣箰濡﹪鎯冮崟顐ｂ枙闁稿﹦銆嬬槐婵嬪础閸愯弓鐒婄€瑰壊鍠栭·鍐煂閿燂拷
	pid->LastError = pid->NowError;               //闁哄洤鐡ㄩ弻濠囧磻韫囨挻鈻�	
	//闂傚嫭鍔曢崺妤冪矓椤栨艾鐎婚柛妯烘瑜板爼宕戣箛鎾粹枙
	if( pid->Ierror >= pid->IerrorLim) pid->Ierror =  pid->IerrorLim;
	else if( pid->Ierror <= -pid->IerrorLim)  pid->Ierror =  -pid->IerrorLim;
	//PID闁告艾瀚獮鍡涙嚍閸屾繄缈婚柛鎴炴そ閸ｏ拷
	pid->Pout = pid->Kp * pid->Perror;
	pid->Iout = pid->Ki * pid->Ierror;
	pid->Dout = pid->Kd * pid->Derror;
	//PID闁诡剚妲掔欢顓㈠礄濞差亜娅�
	pid->PIDout = pid->Pout + pid->Iout + pid->Dout;
	//闂傚嫭鍔曢崺妗篒D闁诡剚妲掔欢顓㈠礄濞差亜娅�
	if(pid->PIDout > pid->PIDoutMAX) pid->PIDout = pid->PIDoutMAX;
	else if(pid->PIDout < -pid->PIDoutMAX) pid->PIDout = -pid->PIDoutMAX;
}
/**
 * @brief 缂備焦绻傞顔碱嚕瀵ゆ换D閻犱緤绱曢悾锟�(缂佸鍨伴崹搴ㄥ礆閸℃瑯鐎�)
 * @param s_pid_absolute_t *pid
 * @param float integral_apart_val
 * @return float PIDout
 */
void PID_AbsoluteMode_integral_apart(s_pid_absolute_t *pid,float integral_apart_val)
{
    //PID闁告艾瀚獮鍡涙嚍閸屾矮鐒婄€归潻鎷�
	pid->Perror = pid->NowError;                  //P闁绘粠鍨垫俊顓㈠磻韫囨挻鈻曢柡鍕靛灠缂嶅宕滃鍛剨鐎归潻鎷�
    if(fabs(pid->NowError)<integral_apart_val)
	    pid->Ierror += pid->NowError;                 //I闁绘粠鍨垫俊顓㈠磻韫囨挻鈻曢柡鍕靛灟缁楀倿鎮介棃娑欏€靛☉鎾亾闁烩晛鐡ㄧ€垫梻绱掗鐐茬厒闁绘粍婢樺﹢顏堟儍閸曨偂鐒婄€归潻鎷�
    else pid->Ierror = 0;
	pid->Derror = pid->NowError - pid->LastError; //D闁绘粠鍨垫俊顓㈠磻韫囨挻鈻曢柡鍕靛灠缂嶅宕滃鍛剨鐎瑰壊鍠曠粭灞剧▔婵犲喚鍋ч柛瀣箰濡﹪鎯冮崟顐ｂ枙闁稿﹦銆嬬槐婵嬪础閸愯弓鐒婄€瑰壊鍠栭·鍐煂閿燂拷
	pid->LastError = pid->NowError;               //闁哄洤鐡ㄩ弻濠囧磻韫囨挻鈻�
	//闂傚嫭鍔曢崺妤冪矓椤栨艾鐎婚柛妯烘瑜板爼宕戣箛鎾粹枙
	if( pid->Ierror >= pid->IerrorLim) pid->Ierror =  pid->IerrorLim;
	else if( pid->Ierror <= -pid->IerrorLim)  pid->Ierror =  -pid->IerrorLim;
	//PID闁告艾瀚獮鍡涙嚍閸屾繄缈婚柛鎴炴そ閸ｏ拷
	pid->Pout = pid->Kp * pid->Perror;
	pid->Iout = pid->Ki * pid->Ierror;
	pid->Dout = pid->Kd * pid->Derror;
	//PID闁诡剚妲掔欢顓㈠礄濞差亜娅�
	pid->PIDout = pid->Pout + pid->Iout + pid->Dout;
	//闂傚嫭鍔曢崺妗篒D闁诡剚妲掔欢顓㈠礄濞差亜娅�
	if(pid->PIDout > pid->PIDoutMAX) pid->PIDout = pid->PIDoutMAX;
	else if(pid->PIDout < -pid->PIDoutMAX) pid->PIDout = -pid->PIDoutMAX;
}
/**
 * @brief   PID闁告瑥鍊归弳鐔煎礆濠靛棭娼楅柛鏍ㄧ壄缁辨繈宕ｉ娆庣鞍闁衡偓閹勮含闁告帗绻傞～鎰板礌閺嵮冩瘣闁轰線顣﹂懙鎴︽晬鐏炶偐鐦嶉柛娆樺灟娴滄帡寮ㄩ幆褎韬€甸偊浜為獮鍡涙煂閿燂拷
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
 * @brief   PID闁告瑥鍊归弳鐔烘導鐎ｎ亖鍋撶涵椋庣闁告瑯鍨禍鎺楀绩閹勮含闁告帗绻傞～鎰板礌閺嵮冩瘣闁轰線顣﹂懙鎴︽晬鐏炶偐鐦嶉柛娆樺灟娴滄帡寮ㄩ幆褎韬€甸偊浜為獮鍡涙煂瀹€瀣闁瑰瓨鍨归弫銈夊级閵夛附鏉归柛鎺撴緲閹﹪鎮抽鐐叉閻犲鍟抽惁顖炲矗閸屾稒娈�
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
 * @brief   闁告娲滈獮鍝朓D
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
 * @brief   濞戞捁灏欐鍢滻D
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
 * @brief 濞戞捁灏欐鍢滻D(闂侇偆鍠庣€规娊鎮抽婧炴繈宕氶崱妤€鐎荤紒鍌︽嫹)
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

