#ifndef PC_INTERFACE_H
#define PC_INTERFACE_H

#include "stm32f4xx_hal.h"
#include "stdio.h"

typedef struct
{
    union
    {
        uint8_t          bit[4];
        float            data;
    }x;
    union
    {
        uint8_t          bit[4];
        float            data;
    }y;
    union
    {
        uint8_t          bit[4];
        float            data;
    }z;
    union
    {
        uint8_t          bit[2];
        unsigned char    data;
    }id;
    union
    {
        uint8_t          bit[2];
        unsigned char    data;
    }checkCode;

}su_PC_DATA;


typedef struct
{
    float x;   
    float y;
    float z;

    float pitchCenter;
    float pitchOffset;
    float pitPos;

    float disOffset;
    float shootOffset;
    float distance;

    float yawCenter;
    float filterOffset;
    float yawPos;
	
	float yawPre;
	float yawRelV;
	
}visInf_t;

void DealPcData(su_PC_DATA*pcdata, uint8_t *visionBuf);
void RobotDealUSBData(uint8_t *Buf);

#endif
