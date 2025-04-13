#include 	<string.h>
#include <math.h>
#include <stdbool.h>
#include "stm32f407xx.h"

#include "pc_interface.h"
#include "bat_control.h" 
#include "Variables.h"

extern su_PC_DATA pcData;       //pc数据
extern visInf_t s_visionInform; //视觉数据结构体

void RobotDealUSBData(uint8_t *Buf)
{
    DealPcData(&pcData,Buf);
}
/**
  * @brief          处理PC传输过来的数据，为Python视觉的数据格式，放到USB接收中，480取流帧率260
  * @param[in]      su_PC_DATA *pcdata ：自定义的pc数据数组
  * @param[in]      uint8_t *visionBuf ：串口或USB获取的数据数组
  * @retval         none
  */
 void DealPcData(su_PC_DATA *pcdata, uint8_t *visionBuf)
 {

	//先把从串口或者USB获取的数据存到定义的pc共用体中
	pcdata->x_hikvision.bit[0] = visionBuf[0];
	pcdata->x_hikvision.bit[1] = visionBuf[1];
	pcdata->x_hikvision.bit[2] = visionBuf[2];
	pcdata->x_hikvision.bit[3] = visionBuf[3];

	pcdata->y_hikvision.bit[0] = visionBuf[4];
	pcdata->y_hikvision.bit[1] = visionBuf[5];
	pcdata->y_hikvision.bit[2] = visionBuf[6];
	pcdata->y_hikvision.bit[3] = visionBuf[7];

	pcdata->z_hikvision.bit[0] = visionBuf[8];
	pcdata->z_hikvision.bit[1] = visionBuf[9];
	pcdata->z_hikvision.bit[2] = visionBuf[10];
	pcdata->z_hikvision.bit[3] = visionBuf[11];

	pcdata->x_usbcam.bit[0] = visionBuf[12];
	pcdata->x_usbcam.bit[1] = visionBuf[13];
	pcdata->x_usbcam.bit[2] = visionBuf[14];
	pcdata->x_usbcam.bit[3] = visionBuf[15];

	pcdata->y_usbcam.bit[0] = visionBuf[16];
	pcdata->y_usbcam.bit[1] = visionBuf[17];
	pcdata->y_usbcam.bit[2] = visionBuf[18];
	pcdata->y_usbcam.bit[3] = visionBuf[19];

	pcdata->z_usbcam.bit[0] = visionBuf[20];
	pcdata->z_usbcam.bit[1] = visionBuf[21];
	pcdata->z_usbcam.bit[2] = visionBuf[22];
	pcdata->z_usbcam.bit[3] = visionBuf[23];

	s_visionInform.z_hikvision_last = s_visionInform.z_hikvision;
	s_visionInform.z_usbcam_last = s_visionInform.z_usbcam;

	s_visionInform.x_hikvision = pcdata->x_hikvision.data;
	s_visionInform.y_hikvision = pcdata->y_hikvision.data;
	s_visionInform.z_hikvision = pcdata->z_hikvision.data;

	s_visionInform.x_usbcam = pcdata->x_usbcam.data;
	s_visionInform.y_usbcam = pcdata->y_usbcam.data;
	s_visionInform.z_usbcam = pcdata->z_usbcam.data;

	//判断球是否在视觉内，为靠近还是远离
	if (s_visionInform.x_hikvision == 0 && s_visionInform.y_hikvision == 0)
	{
		s_visionInform.z_hikvision = -1;
		s_visionInform.hikcam_ball_judje = empty;
	}
	if (s_visionInform.x_usbcam == 0 && s_visionInform.y_usbcam == 0)
	{
		s_visionInform.z_usbcam = -1;
		s_visionInform.usbcam_ball_judje = empty;
	}
	
	if (s_visionInform.z_hikvision != -1 && s_visionInform.z_hikvision_last != -1)
	{
		if (s_visionInform.z_hikvision > s_visionInform.z_hikvision_last)
		{
			s_visionInform.hikcam_ball_judje = gone;
		}else
		{
			s_visionInform.hikcam_ball_judje = come;
		}
	}
	
	if (s_visionInform.z_usbcam != -1 && s_visionInform.z_usbcam_last != -1)
	{
		if (s_visionInform.z_usbcam> s_visionInform.z_usbcam_last)
		{
			s_visionInform.usbcam_ball_judje = gone;
		}else
		{
			s_visionInform.usbcam_ball_judje = come;
		}
	}
	// if (s_visionInform.z_usbcam == INFINITY)
	// {
	// 	s_visionInform.z_usbcam = 100;
	// }
	// if (s_visionInform.z_hikvision == INFINITY)
	// {
	// 	s_visionInform.z_hikvision = 100;
	// }
	
 }

 /**
  * @brief 将相机坐标系下的坐标转换为机器人坐标系下的坐标
  *
  * @param x 相机坐标系下的 x 坐标指针
  * @param y 相机坐标系下的 y 坐标指针
  * @param z 相机坐标系下的 z 坐标指针
  * @return Point 机器人坐标系下的排球相对坐标点
  */
 Point cam_space2bot_speace(float *x, float *y, float *z)
 {

	float Cam_offset_Angle = 30.0f;        //相机与机器人坐标系之间的偏角
	float cam_distanceofbat = 0.0f;        //相机到球拍的距离
	float cam_hightofbat = 0.0f;           //相机到球拍的高度
	float cam_offsetofbat = 0.0f;          //相机到球拍的偏置

	

 }


 