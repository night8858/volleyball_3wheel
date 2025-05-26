#ifndef PC_INTERFACE_H
#define PC_INTERFACE_H

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "bat_control.h"

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

typedef enum
{
    
    empty = 0, // 无排球
    come = 1,  // 排球靠近
    gone = 2,  // 排球远离

} cam_ball_state_e;

/**
 * @brief Structure containing vision information from both Hikvision and USB cameras
 *
 * @details This structure stores the 3D coordinates (x, y, z) of detected objects
 * from both Hikvision and USB cameras, along with their previous z values and
 * ball detection states.
 *
 * @note x_hikvision, y_hikvision, z_hikvision: Position data from Hikvision camera
 * @note x_usbcam, y_usbcam, z_usbcam: Position data from USB camera
 * @note z_hikvision_last, z_usbcam_last: Previous z-axis positions
 * @note hikcam_ball_judje: Ball detection state from Hikvision camera
 * @note usbcam_ball_judje: Ball detection state from USB camera
 */
typedef struct
{

    float pos_ball_x;
    float pos_ball_y;
    float pos_ball_z;

    Point ball_pos_bat;
    Point ball_pos_bat_60;

    float judeg_hit;
    float ball_hight_;
    uint8_t  hasvaild;     //确定是否有球
    
    cam_ball_state_e hikcam_ball_judje;
    cam_ball_state_e usbcam_ball_judje;

} visInf_t;

void DealPcData(su_PC_DATA*pcdata, uint8_t *visionBuf);
void RobotDealUSBData(uint8_t *Buf);

#endif
