#include "main.h"
#include "can_recv.h"
#include "remote_control.h"
#include "chassis.h"
#include "variables.h"
#include "Monitor.h"

#include "math.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl;
extern s_FPS_monitor FPS;

extern s_Dji_motor_data_t motor_Date[6];    // RM电机回传数据结构

extern s_motor_data_t DM4340_Date[3]; // DM4340回传数据结构体
extern s_motor_data_t DM8006_Date[1]; // DM4340回传数据结构体
extern s_motor_data_t HT04_Data;

int DM_circle_num[4];

static CAN_TxHeaderTypeDef RM6020_tx_message; // can_6020发送邮箱
static CAN_TxHeaderTypeDef RM3508_tx_message; // can_3508发送邮箱RM
static CAN_TxHeaderTypeDef RM3508_tx_message2; // can_3508发送邮箱RM


static CAN_TxHeaderTypeDef CAN_DMstart_TxHeader;
static CAN_TxHeaderTypeDef CAN_DM4340_msg_TxHeader;
static CAN_TxHeaderTypeDef CAN_DM8006_msg_TxHeader;

static CAN_TxHeaderTypeDef CAN_HTstart_TxHeader;
static CAN_TxHeaderTypeDef CAN_HTmsg_TxHeader;

static uint8_t can_6020_send_data[8];
static uint8_t can_3508_send_data[8];
static uint8_t can_3508_send_data2[8];
#define get_motor_measure(ptr, data)                                   \
    {                                                                  \
        (ptr)->last_ecd = (ptr)->ecd;                                  \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temperate = (data)[6];                                  \
    }

// 所有6020电机的指令发送
// void CAN_cmd_6020(int16_t CMD_ID_1, int16_t CMD_ID_2, int16_t CMD_ID_3, int16_t CMD_ID_4)
// {
//     uint32_t send_mail_box;
//     RM6020_tx_message.StdId = CAN_6020_ALL_ID;
//     RM6020_tx_message.IDE = CAN_ID_STD;
//     RM6020_tx_message.RTR = CAN_RTR_DATA;
//     RM6020_tx_message.DLC = 0x08;
//     can_6020_send_data[0] = (CMD_ID_1 >> 8);
//     can_6020_send_data[1] = CMD_ID_1;
//     can_6020_send_data[2] = (CMD_ID_2 >> 8);
//     can_6020_send_data[3] = CMD_ID_2;
//     can_6020_send_data[4] = (CMD_ID_3 >> 8);
//     can_6020_send_data[5] = CMD_ID_3;
//     can_6020_send_data[6] = (CMD_ID_4 >> 8);
//     can_6020_send_data[7] = CMD_ID_4;
//     HAL_CAN_AddTxMessage(&hcan2, &RM6020_tx_message, can_6020_send_data, &send_mail_box);
// }

// 所有3508电机（动力电机）的指令发送
void CAN_cmd_3508(int16_t CMD_ID_1, int16_t CMD_ID_2, int16_t CMD_ID_3, int16_t CMD_ID_4)
{
    uint32_t send_mail_box01;
    RM3508_tx_message.StdId = CAN_3508_ALL_ID;
    RM3508_tx_message.IDE = CAN_ID_STD;
    RM3508_tx_message.RTR = CAN_RTR_DATA;
    RM3508_tx_message.DLC = 0x08;
    can_3508_send_data[0] = (CMD_ID_1 >> 8);
    can_3508_send_data[1] = CMD_ID_1;
    can_3508_send_data[2] = (CMD_ID_2 >> 8);
    can_3508_send_data[3] = CMD_ID_2;
    can_3508_send_data[4] = (CMD_ID_3 >> 8);
    can_3508_send_data[5] = CMD_ID_3;
    can_3508_send_data[6] = (CMD_ID_4 >> 8);
    can_3508_send_data[7] = CMD_ID_4;
    HAL_CAN_AddTxMessage(&hcan1, &RM3508_tx_message, can_3508_send_data, &send_mail_box01);
}


void CAN_cmd_striker(int16_t CMD_ID_5, int16_t CMD_ID_6)
{
    uint32_t send_mail_box01;
    RM3508_tx_message2.StdId = CAN_3508_ALL_2ID;
    RM3508_tx_message2.IDE = CAN_ID_STD;
    RM3508_tx_message2.RTR = CAN_RTR_DATA;
    RM3508_tx_message2.DLC = 0x08;
    can_3508_send_data2[0] = (CMD_ID_5 >> 8);
    can_3508_send_data2[1] = CMD_ID_5;
    can_3508_send_data2[2] = (CMD_ID_6 >> 8);
    can_3508_send_data2[3] = CMD_ID_6;
    can_3508_send_data2[4] = 0;
    can_3508_send_data2[5] = 0;
    can_3508_send_data2[6] = 0;
    can_3508_send_data2[7] = 0;
    HAL_CAN_AddTxMessage(&hcan1, &RM3508_tx_message2, can_3508_send_data2, &send_mail_box01);
}
/// @brief HT04电机启动函数
/// @param Target_hcan can输出句柄
/// @param id 电机id号
void HT04_motor_start(CAN_HandleTypeDef *Target_hcan, uint8_t id)
{
    uint8_t TxData[8];
    CAN_HTstart_TxHeader.StdId = id;
    CAN_HTstart_TxHeader.IDE = CAN_ID_STD;
    CAN_HTstart_TxHeader.RTR = CAN_RTR_DATA;
    CAN_HTstart_TxHeader.DLC = 0x08;
    TxData[0] = 0xFF;
    TxData[1] = 0xFF;
    TxData[2] = 0xFF;
    TxData[3] = 0xFF;
    TxData[4] = 0xFF;
    TxData[5] = 0xFF;
    TxData[6] = 0xFF;
    TxData[7] = 0xFC;
    HAL_CAN_AddTxMessage(Target_hcan, &CAN_HTstart_TxHeader, TxData, (uint32_t *)CAN_TX_MAILBOX0);
}

/// @brief DM电机启动函数
/// @param Target_hcan CAN输出句柄
/// @param id DM电机的id号
void DM_motor_start(CAN_HandleTypeDef *Target_hcan, uint16_t id)
{
    // uint32_t send_mail_box;
    uint8_t TxData[8];
    CAN_DMstart_TxHeader.StdId = id;
    CAN_DMstart_TxHeader.IDE = CAN_ID_STD;
    CAN_DMstart_TxHeader.RTR = CAN_RTR_DATA;
    CAN_DMstart_TxHeader.DLC = 0x08;
    TxData[0] = 0xFF;
    TxData[1] = 0xFF;
    TxData[2] = 0xFF;
    TxData[3] = 0xFF;
    TxData[4] = 0xFF;
    TxData[5] = 0xFF;
    TxData[6] = 0xFF;
    TxData[7] = 0xFC;

    HAL_CAN_AddTxMessage(Target_hcan, &CAN_DMstart_TxHeader, TxData, (uint32_t *)CAN_TX_MAILBOX0);
}

/// @brief 使用pid输出力矩的方式控制，即电流环控制
/// @param hcan can输出句柄
/// @param id HT电机的id号
/// @param _torq 输入的前馈力矩
void HT04_motor_PID_Control(CAN_HandleTypeDef *hcan, uint32_t id, float _torq)
{
    uint8_t TxData[8];
    uint16_t p, v, kp, kd, t;

    /* 限制输入的参数在定义的范围内 */
    p = 0;
    v = 0;
    kp = 0;
    kd = 0;
    t = float_to_uint(_torq, HT04_T_MIN, HT04_T_MAX, 12);

    /* 根据传输协议，把数据转换为CAN命令数据字段 */
    CAN_HTmsg_TxHeader.StdId = id;
    CAN_HTmsg_TxHeader.IDE = CAN_ID_STD;
    CAN_HTmsg_TxHeader.RTR = CAN_RTR_DATA;
    CAN_HTmsg_TxHeader.DLC = 0x08;
    TxData[0] = p >> 8;
    TxData[1] = p & 0xFF;
    TxData[2] = v >> 4;
    TxData[3] = ((v & 0xF) << 4) | (kp >> 8);
    TxData[4] = kp & 0xFF;
    TxData[5] = kd >> 4;
    TxData[6] = ((kd & 0xF) << 4) | (t >> 8);
    TxData[7] = t & 0xff;

    HAL_CAN_AddTxMessage(hcan, &CAN_HTmsg_TxHeader, TxData, (uint32_t *)CAN_TX_MAILBOX0);
}

/// @brief 使用pid输出力矩的方式控制，即电流环控制
/// @param hcan can输出句柄
/// @param id   DM电机的id号
/// @param _torq 输出的前馈力矩
void MD4340_motor_PID_Control(CAN_HandleTypeDef *hcan, uint32_t id, float _torq)
{
    uint8_t txData[8];
    // uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp; // 声明临时变量
    uint16_t tor_tmp;
    float _pos, _vel, _KP, _KD = {0};

    //_pos = fminf(fmaxf(DM4340_P_MIN, _pos), DM4340_P_MAX);
    // pos_tmp = float_to_uint(_pos, DM4340_P_MIN, DM4340_P_MAX, 16);
    // vel_tmp = float_to_uint(_vel, DM4340_V_MIN, DM4340_V_MAX, 12);
    // kp_tmp = float_to_uint(_KP, DM4340_KP_MIN, DM4340_KP_MAX, 12);
    // kd_tmp = float_to_uint(_KD, DM4340_KD_MIN, DM4340_KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, DM4340_T_MIN, DM4340_T_MAX, 12);

    CAN_DM4340_msg_TxHeader.StdId = id;
    CAN_DM4340_msg_TxHeader.IDE = CAN_ID_STD;
    CAN_DM4340_msg_TxHeader.RTR = CAN_RTR_DATA;
    CAN_DM4340_msg_TxHeader.DLC = 0x08;
    txData[0] = 0;
    txData[1] = 0;
    txData[2] = 0;
    txData[3] = 0;
    txData[4] = 0;
    txData[5] = 0;
    txData[6] = ((0 & 0xf) << 4) | (tor_tmp >> 8);
    txData[7] = tor_tmp & 0xff;

    HAL_CAN_AddTxMessage(hcan, &CAN_DM4340_msg_TxHeader, txData, (uint32_t *)CAN_TX_MAILBOX0);
}

/// @brief 使用pid输出力矩的方式控制，即电流环控制
/// @param hcan can输出句柄
/// @param id   DM电机的id号
/// @param _torq 输出的前馈力矩
void DM8006_motor_PID_Control(CAN_HandleTypeDef *hcan, uint32_t id, float _torq)
{
    uint8_t txData[8];
    // uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp; // 声明临时变量
    uint16_t tor_tmp;
    float _pos, _vel, _KP, _KD = {0};

    //_pos = fminf(fmaxf(DM4340_P_MIN, _pos), DM4340_P_MAX);
    // pos_tmp = float_to_uint(_pos, DM8006_P_MIN, DM8006_P_MAX, 16);
    // vel_tmp = float_to_uint(_vel, DM8006_V_MIN, DM8006_V_MAX, 12);
    // kp_tmp = float_to_uint(_KP, DM8006_KP_MIN, DM8006_KP_MAX, 12);
    // kd_tmp = float_to_uint(_KD, DM8006_KD_MIN, DM8006_KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, DM8006_T_MIN, DM8006_T_MAX, 12);

    CAN_DM8006_msg_TxHeader.StdId = id;
    CAN_DM8006_msg_TxHeader.IDE = CAN_ID_STD;
    CAN_DM8006_msg_TxHeader.RTR = CAN_RTR_DATA;
    CAN_DM8006_msg_TxHeader.DLC = 0x08;
    txData[0] = 0;
    txData[1] = 0;
    txData[2] = 0;
    txData[3] = 0;
    txData[4] = 0;
    txData[5] = 0;
    txData[6] = ((0 & 0xf) << 4) | (tor_tmp >> 8);
    txData[7] = tor_tmp & 0xff;

    HAL_CAN_AddTxMessage(hcan, &CAN_DM8006_msg_TxHeader, txData, (uint32_t *)CAN_TX_MAILBOX0);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header_can1, rx_header_can2;
    uint8_t rx_data_can1[8];
    uint8_t rx_data_can2[8];
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header_can1, rx_data_can1);

        switch (rx_header_can1.StdId)
        {
        case CAN_3508_M1_ID:
        {
			motor_Date[0].back_position    =		rx_data_can1[0]<<8 | rx_data_can1[1];
			motor_Date[0].back_motor_speed =		rx_data_can1[2]<<8 | rx_data_can1[3];
			motor_Date[0].back_current     = 	    rx_data_can1[4]<<8 | rx_data_can1[5];
            continue_motor_pos(&motor_Date[0]);
            FPS.M3508_M1++;
            break;
        }
        case CAN_3508_M2_ID:
        {
            motor_Date[1].back_position    =		rx_data_can1[0]<<8 | rx_data_can1[1];
			motor_Date[1].back_motor_speed =		rx_data_can1[2]<<8 | rx_data_can1[3];
			motor_Date[1].back_current     = 	    rx_data_can1[4]<<8 | rx_data_can1[5];
            continue_motor_pos(&motor_Date[1]);
            FPS.M3508_M2++;
            break;
        }
        case CAN_3508_M3_ID:
        {
			motor_Date[2].back_position    =		rx_data_can1[0]<<8 | rx_data_can1[1];
			motor_Date[2].back_motor_speed =		rx_data_can1[2]<<8 | rx_data_can1[3];
			motor_Date[2].back_current     = 	    rx_data_can1[4]<<8 | rx_data_can1[5];
            continue_motor_pos(&motor_Date[2]);
            FPS.M3508_M3++;
            break;
        }
        case CAN_3508_M5_ID:
        {
            motor_Date[4].back_position    =		rx_data_can1[0]<<8 | rx_data_can1[1];
			motor_Date[4].back_motor_speed =		rx_data_can1[2]<<8 | rx_data_can1[3];
			motor_Date[4].back_current     = 	    rx_data_can1[4]<<8 | rx_data_can1[5];
            continue_motor_pos(&motor_Date[4]);            
            FPS.Striker_3508++;
            break;
        }
        case CAN_3508_M6_ID:
        {
            motor_Date[5].back_position    =		rx_data_can1[0]<<8 | rx_data_can1[1];
			motor_Date[5].back_motor_speed =		rx_data_can1[2]<<8 | rx_data_can1[3];
			motor_Date[5].back_current     = 	    rx_data_can1[4]<<8 | rx_data_can1[5];
            continue_motor_pos(&motor_Date[5]);            
            //FPS.Striker_3508++;
            break;
        }
        default:
        {
            break;
        }
        }
    }

    ////////CAN2接收数据处理//////////
    /*            */
    if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header_can2, rx_data_can2); // 从FIFO中接收消息至rx_header_can2
        switch (rx_header_can2.StdId)
        {

        case DM4340_M1:
        {

            DM4340_Date[0].id = (rx_data_can2[0]) & 0x0F;
            MD_CanReceive(&DM4340_Date[0], rx_data_can2);

            DM4340_Date[0].esc_back_position_last = DM4340_Date[0].esc_back_position;
            DM4340_Date[0].real_angle = DM4340_Date[0].esc_back_position * 57.29577951308f; // RUD_DirAngle_Proc(DM4340_Date[0].serial_angle);
            FPS.DM4340_M1++;
            break;
        }
        case DM4340_M2:
        {
            DM4340_Date[1].id = (rx_data_can2[0]) & 0x0F;
            MD_CanReceive(&DM4340_Date[1], rx_data_can2);

            DM4340_Date[1].esc_back_position_last = DM4340_Date[1].esc_back_position;
            DM4340_Date[1].real_angle = DM4340_Date[1].esc_back_position * 57.29577951308f;
            FPS.DM4340_M2++;
            break;
        }
        case DM4340_M3:
        {
            DM4340_Date[2].id = (rx_data_can2[0]) & 0x0F;
            MD_CanReceive(&DM4340_Date[2], rx_data_can2);

            DM4340_Date[2].esc_back_position_last = DM4340_Date[2].esc_back_position;
            DM4340_Date[2].real_angle = DM4340_Date[2].esc_back_position * 57.29577951308f;
            FPS.DM4340_M3++;
            break;
        }
        case DM8006_M1:
        {
            DM8006_Date[0].id = (rx_data_can2[0]) & 0x0F;
            MD_CanReceive(&DM8006_Date[0], rx_data_can2);

            DM8006_Date[0].esc_back_position_last = DM8006_Date[0].esc_back_position;
            DM8006_Date[0].real_angle = (DM8006_Date[0].esc_back_position * 57.29577951308f)  + 35.0f;
            //这里加上35度偏移量，是加上机械方面的限制最靠下的倾角是35
            FPS.Pitch_DM8006++;
            break;
        }

        default:
        {
            break;
        }
        }
    }
}

// 达妙电机的数据解包和赋值
void MD_CanReceive(s_motor_data_t *motor, uint8_t RxDate[8])
{

    int p_int = (RxDate[1] << 8) | RxDate[2];
    int v_int = (RxDate[3] << 4) | (RxDate[4] >> 4);
    int i_int = ((RxDate[4] & 0xf) << 8) | (RxDate[5]);
    int T_int = RxDate[6];
    if (motor->id == 0x01)
    {
        motor->state = (RxDate[0]) >> 4;
        motor->esc_back_position = uint_to_float(p_int, DM4340_P_MIN, DM4340_P_MAX, 16); // 电机位置
        motor->esc_back_speed = uint_to_float(v_int, DM4340_V_MIN, DM4340_V_MAX, 12);    // 电机速度
        motor->esc_back_current = uint_to_float(i_int, DM4340_T_MIN, DM4340_T_MAX, 12);  //	电机扭矩/电流
        motor->Tmos = (float)(RxDate[6]);
        motor->Tcoil = (float)(RxDate[7]);
    }
    if (motor->id == 0x02)
    {
        motor->state = (RxDate[0]) >> 4;
        motor->esc_back_position = uint_to_float(p_int, DM4340_P_MIN, DM4340_P_MAX, 16); // 电机位置
        motor->esc_back_speed = uint_to_float(v_int, DM4340_V_MIN, DM4340_V_MAX, 12);    // 电机速度
        motor->esc_back_current = uint_to_float(i_int, DM4340_T_MIN, DM4340_T_MAX, 12);  //	电机扭矩/电流
        motor->Tmos = (float)(RxDate[6]);
        motor->Tcoil = (float)(RxDate[7]);
    }
    if (motor->id == 0x03)
    {
        motor->state = (RxDate[0]) >> 4;
        motor->esc_back_position = uint_to_float(p_int, DM4340_P_MIN, DM4340_P_MAX, 16); // 电机位置
        motor->esc_back_speed = uint_to_float(v_int, DM4340_V_MIN, DM4340_V_MAX, 12);    // 电机速度
        motor->esc_back_current = uint_to_float(i_int, DM4340_T_MIN, DM4340_T_MAX, 12);  //	电机扭矩/电流
        motor->Tmos = (float)(RxDate[6]);
        motor->Tcoil = (float)(RxDate[7]);
    }
    if (motor->id == 0x04)
    {
        motor->state = (RxDate[0]) >> 4;
        motor->esc_back_position = uint_to_float(p_int, DM8006_P_MIN, DM8006_P_MAX, 16); // 电机位置
        motor->esc_back_speed = uint_to_float(v_int, DM8006_V_MIN, DM8006_V_MAX, 12);    // 电机速度
        motor->esc_back_current = uint_to_float(i_int, DM8006_T_MIN, DM8006_T_MIN, 12);  //	电机扭矩/电流
        motor->Tmos = (float)(RxDate[6]);
        motor->Tcoil = (float)(RxDate[7]);
    }
}

void HT04_CanReceive(s_motor_data_t *motor, uint8_t *RxData)
{
    int p_int = (RxData[1] << 8) | RxData[2];
    int v_int = (RxData[3] << 4) | (RxData[4] >> 4);
    int i_int = ((RxData[4] & 0xf) << 8) | (RxData[5]);
    if (motor->id == 0x50)
    {
        motor->esc_back_position = uint_to_float(p_int, HT04_P_MIN, HT04_P_MAX, 16); //
        motor->esc_back_speed = uint_to_float(v_int, HT04_V_MIN, HT04_V_MAX, 12);    //
        motor->esc_back_current = uint_to_float(i_int, HT04_T_MIN, HT04_T_MAX, 12);  //
    }
}

//dji电机连续编码器数据处理 
void continue_motor_pos(s_Dji_motor_data_t *s_motor)
{
    if (s_motor->is_pos_ready == 1) // 如果电机第一次上电后记录了那时的电机编码器值并将预备标志位置一了的话，进入此判断
    {
        // 如果（当前电机返回值-上一次电机返回值）值大于4096，因为电机不可能在几毫秒内转过半圈
        if (s_motor->back_position - s_motor->back_pos_last > 4096)
        {
            s_motor->circle_num--; // 圈数--
        }
        else if (s_motor->back_position - s_motor->back_pos_last < -4096) // 同上，只不过方向是反的
        {
            s_motor->circle_num++; // 圈数++
        }
    }
    else
    {
        s_motor->target_pos = s_motor->back_position; // 如果电机预备标志位不为1，也就是电机第一次上电
        s_motor->is_pos_ready = 1;                    // 电机预备标志位赋值为一，也就是说电机已经准备好
    }
    s_motor->back_pos_last = s_motor->back_position;                                // 将上一次进入该函数的电机返回值赋值，方便计算连续值
    s_motor->serial_position = s_motor->back_position + s_motor->circle_num * 8191; // 返回的电机连续编码器值
    s_motor->back_motor_ang = s_motor->back_position / 8191.0f * 360.0f;            // 返回的电机绝对角度
    s_motor->serial_motor_ang = s_motor->serial_position / 8191.0f * 360.0f;        // 返回的电机连续角度
}

typedef union
{
    float fdata;
    unsigned long ldata;
} FloatLongType;

/*
将4个字节数据byte[4]转化为浮点数存放在*f中
*/
void Byte_to_Float(float *date1, float *date2, unsigned char byte[])
{
    FloatLongType fl, f2;
    fl.ldata = 0;
    f2.ldata = 0;
    fl.ldata = byte[3];
    fl.ldata = (fl.ldata << 8) | byte[2];
    fl.ldata = (fl.ldata << 8) | byte[1];
    fl.ldata = (fl.ldata << 8) | byte[0];
    f2.ldata = byte[7];
    f2.ldata = (f2.ldata << 8) | byte[6];
    f2.ldata = (f2.ldata << 8) | byte[5];
    f2.ldata = (f2.ldata << 8) | byte[4];
    *date1 = fl.fdata;
    *date2 = f2.fdata;
}

// 过零检测
float RUD_DirAngle_Proc(float Angle)
{
    while (Angle > 360 || Angle < 0)
    {
        if (Angle < 0)
        {
            Angle += 360;
        }
        if (Angle > 360)
        {
            Angle -= 360;
        }
    }
    return (float)Angle;
}

// 浮点数转整形,同时限制输入范围
int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
    float span = x_max - x_min;
    if (x < x_min)
        x = x_min;
    else if (x > x_max)
        x = x_max;

    return (int)((x - x_min) * ((float)((1 << bits) / span)));
}
// 整形转浮点数
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

const s_Dji_motor_data_t *get_3508_M1_motor_measure_point(void)
{
    return &motor_Date[0];
}
const s_Dji_motor_data_t *get_3508_M2_motor_measure_point(void)
{
    return &motor_Date[1];
}
const s_Dji_motor_data_t *get_3508_M3_motor_measure_point(void)
{
    return &motor_Date[2];
}
const s_Dji_motor_data_t *get_3508_M4_motor_measure_point(void)
{
    return &motor_Date[3];
}
/* const motor_measure_t *get_6020_M1_motor_measure_point(void)
{
    return &motor_Date[4];
}
const motor_measure_t *get_6020_M2_motor_measure_point(void)
{
    return &motor_Date[5];
}
const motor_measure_t *get_6020_M3_motor_measure_point(void)
{
    return &motor_Date[6];
}
const motor_measure_t *get_6020_M4_motor_measure_point(void)
{
    return &motor_Date[7];
} */
