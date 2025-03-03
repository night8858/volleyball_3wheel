#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "pc_interface.h"


su_PC_DATA pcData;       //pc数据
visInf_t s_visionInform; //视觉数据结构体

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

	s_visionInform.x = pcdata->x.data;
	s_visionInform.y = pcdata->y.data;
	s_visionInform.z = pcdata->z.data;
 }    
