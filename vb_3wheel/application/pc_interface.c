  
  // ██╗    ██╗  ██╗  ██╗ ███████╗ ███████╗ ██╗     
  // ██║    ██║  ██║  ██║ ██╔════╝ ██╔════╝ ██║     
  // ██║ ██╗ ██║ ███████║ ██████╗  ██████╗  ██║     
  // ██║ ╚██╗██║ ██╔══██║ ██╔══╝   ██╔══╝   ██║     
  // ╚██╗╚████║  ██║  ██║ ███████╗ ███████╗ ███████╗
  //  ╚═╝ ╚═══╝  ╚═╝  ╚═╝ ╚══════╝ ╚══════╝ ╚══════╝
  
  
  
  /*
  *   █████████████╗   ███████████╗     ████████████╗
  *   ╚════██╔═════╝  ██╔════════██╗    ██╔═════════╝
  *        ██║        ██║        ██║    ██║
  *        ██║        ██║        ██║    ██║
  *        ██║        ██║        ██║    ██║█████████╗
  *        ██║        ██║        ██║    ██╔═════════╝    
  *        ██║        ██║        ██║    ██║    
  *        ██║        ██║        ██║    ██║ 
  *        ██║        ╚███████████╔╝    ████████████╗
  *        ╚═╝         ╚══════════╝     ╚═══════════╝
  *  Version    Date            Author          Modification
  *  V1.0.0     2024            night8858       
  */

#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "stm32f407xx.h"

#include "pc_interface.h"
#include "bat_control.h"
#include "Variables.h"
#include "tracking.h"

extern su_PC_DATA pcData;		// pc数据
extern visInf_t s_visionInform; // 视觉数据结构体
extern s_motor_data_t DM8006_Date[1];       // DM4340回传数据结构体

int cout_x = 0;
int cout_y = 0;

static Point cam_space2bot_speace(float x, float y, float z);
static Point cam_space2bot_speace_60(float x, float y, float z );


void RobotDealUSBData(uint8_t *Buf)
{
	DealPcData(&pcData, Buf);
}
/**
 * @brief          处理PC传输过来的数据，为Python视觉的数据格式，放到USB接收中，480取流帧率260
 * @param[in]      su_PC_DATA *pcdata ：自定义的pc数据数组
 * @param[in]      uint8_t *visionBuf ：串口或USB获取的数据数组
 * @retval         none
 */
void DealPcData(su_PC_DATA *pcdata, uint8_t *visionBuf)
{

	// 先把从串口或者USB获取的数据存到定义的pc共用体中
	pcdata->x.bit[0] = visionBuf[0];
	pcdata->x.bit[1] = visionBuf[1];
	pcdata->x.bit[2] = visionBuf[2];
	pcdata->x.bit[3] = visionBuf[3];

	pcdata->y.bit[0] = visionBuf[4];
	pcdata->y.bit[1] = visionBuf[5];
	pcdata->y.bit[2] = visionBuf[6];
	pcdata->y.bit[3] = visionBuf[7];

	pcdata->z.bit[0] = visionBuf[8];
	pcdata->z.bit[1] = visionBuf[9];
	pcdata->z.bit[2] = visionBuf[10];
	pcdata->z.bit[3] = visionBuf[11];

	s_visionInform.pos_ball_x = pcdata->x.data;
	s_visionInform.pos_ball_y = pcdata->y.data;
	s_visionInform.pos_ball_z = pcdata->z.data;
	
	s_visionInform.ball_pos_bat = cam_space2bot_speace(s_visionInform.pos_ball_x, s_visionInform.pos_ball_y, s_visionInform.pos_ball_z);
	s_visionInform.ball_pos_bat_60 = cam_space2bot_speace_60(s_visionInform.pos_ball_x, s_visionInform.pos_ball_y, s_visionInform.pos_ball_z);
	if (s_visionInform.pos_ball_x == 0 && s_visionInform.pos_ball_y == 0 && s_visionInform.pos_ball_z == 0)
	{
		s_visionInform.ball_pos_bat.x = 0;
		s_visionInform.ball_pos_bat.y = 0;
		s_visionInform.ball_pos_bat.z = 0;
		s_visionInform.ball_pos_bat_60.x = 0;
		s_visionInform.ball_pos_bat_60.y = 0;
		s_visionInform.ball_pos_bat_60.z = 0;
		s_visionInform.hasvaild = 0;
		//视觉数据无效
	}else 
	{
		s_visionInform.hasvaild = 1;   //视觉数据有效
	}
	
}

/**
 * @brief 将相机坐标系下的坐标转换为机器人坐标系下的坐标
 *
 * @param x 相机坐标系下的 x 坐标指针
 * @param y 相机坐标系下的 y 坐标指针
 * @param z 相机坐标系下的 z 坐标指针
 * @return Point 机器人坐标系下的排球相对坐标点
 */
Point cam_space2bot_speace(float x, float y, float z )
{

	float theta = -90.0f;     
	float alpha = -(90.0f - 6.5f);  //10度为安装角度  bat为球拍与竖直90度的夹角


	float x1, y1, z1 ;
	float tx , ty ,yz;

	float cos_theta = cosf(theta * 0.017453292);
	float sin_theta = sinf(theta * 0.017453292);
	float cos_alpha = cosf(alpha * 0.017453292);
	float sin_alpha = sinf(alpha * 0.017453292);

	//ty = 0.3869f;
    // 先应用Z轴旋转
    double z_rotated_x = x * cos_theta - y * sin_theta;
    double z_rotated_y = x * sin_theta + y * cos_theta;
    double z_rotated_z = z;
    
    // 再应用X轴旋转
    x1 = z_rotated_x  - 0.021;
    y1 = z_rotated_y * cos_alpha - z_rotated_z * sin_alpha  -0.272;
    z1 = z_rotated_y * sin_alpha + z_rotated_z * cos_alpha  +0.007;

	Point temp;	
	temp.x = x1;
	temp.y = y1;
	temp.z = z1;

	return temp;
}


/**
 * @brief 将相机坐标系下的坐标转换为机器人坐标系下的坐标,球拍前倾的说时候
 *
 * @param x 相机坐标系下的 x 坐标指针
 * @param y 相机坐标系下的 y 坐标指针
 * @param z 相机坐标系下的 z 坐标指针
 * @return Point 机器人坐标系下的排球相对坐标点
 */
Point cam_space2bot_speace_60(float x, float y, float z )
{

	float theta = -90.0f;     
	float alpha = -(90.0f - 6.5f - 30.0f);  //10度为安装角度  bat为球拍与竖直90度的夹角


	float x1, y1, z1 ;
	float tx , ty ,yz;

	float cos_theta = cosf(theta * 0.017453292);
	float sin_theta = sinf(theta * 0.017453292);
	float cos_alpha = cosf(alpha * 0.017453292);
	float sin_alpha = sinf(alpha * 0.017453292);

	//ty = 0.3869f;
    // 先应用Z轴旋转
    double z_rotated_x = x * cos_theta - y * sin_theta;
    double z_rotated_y = x * sin_theta + y * cos_theta;
    double z_rotated_z = z;
    
    // 再应用X轴旋转
    x1 = z_rotated_x  - 0.021;
    y1 = z_rotated_y * cos_alpha - z_rotated_z * sin_alpha  -0.272;
    z1 = z_rotated_y * sin_alpha + z_rotated_z * cos_alpha  +0.007;

	Point temp;	
	temp.x = x1;
	temp.y = y1;
	temp.z = z1;

	return temp;
}
