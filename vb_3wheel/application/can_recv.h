#ifndef CAN_RECV_H
#define CAN_RECV_H

#include "struct_typedef.h"
#include "can.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2


#define DM4340_P_MIN -12.5f // 位置最小值
#define DM4340_P_MAX 12.5f  // 位置最大值
#define DM4340_V_MIN -10.0f   // 速度最小值
#define DM4340_V_MAX 10.0f    // 速度最大值
#define DM4340_KP_MIN 0.0f    // Kp最小值
#define DM4340_KP_MAX 500.0f  // Kp最大值
#define DM4340_KD_MIN 0.0f    // Kd最小值
#define DM4340_KD_MAX 5.0f    // Kd最大值
#define DM4340_T_MIN -10.0f   // 转矩最大值
#define DM4340_T_MAX 10.0f    // 转矩最小值

#define DM8006_P_MIN -12.5f // 位置最小值
#define DM8006_P_MAX 12.5f  // 位置最大值
#define DM8006_V_MIN -45.0f   // 速度最小值
#define DM8006_V_MAX 45.0f    // 速度最大值
#define DM8006_KP_MIN 0.0f    // Kp最小值
#define DM8006_KP_MAX 500.0f  // Kp最大值
#define DM8006_KD_MIN 0.0f    // Kd最小值
#define DM8006_KD_MAX 5.0f    // Kd最大值
#define DM8006_T_MIN -18.0f   // 转矩最大值
#define DM8006_T_MAX 18.0f    // 转矩最小值


#define HT04_P_MIN   -95.5f
#define HT04_P_MAX   95.5f
#define HT04_V_MIN   -45.0f
#define HT04_V_MAX   45.0f
#define HT04_KP_MIN   0.0f
#define HT04_KP_MAX   500.0f
#define HT04_KD_MIN   0.0f
#define HT04_KD_MAX   5.0f
#define HT04_T_MIN   -20.0f
#define HT04_T_MAX   20.0f


/* CAN send and receive ID */
/*所有can消息的ID*/
typedef enum
{
    /*3508电机的CAN ID*/
    CAN_3508_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_3508_ALL_2ID = 0x1FF,
    CAN_3508_M5_ID = 0x205,    //击球电机
    CAN_3508_M6_ID = 0x206,    //击球电机


    /*6020电机的CAN ID*/
    // CAN_6020_ALL_ID = 0x1FF,
    // CAN_6020_M1_ID = 0X205,
    // CAN_6020_M2_ID = 0x206,
    // CAN_6020_M3_ID = 0x207,
    // CAN_6020_M4_ID = 0x208,

    /*板间通讯的CAN ID*/
    DT7_RX_ch = 0x301,
    DT7_RX_S  = 0x302,
    AUTO_MODE_CMD = 0x303,

    /*DM4340电机的CAN master ID*/
    DM4340_M1_MATER  = 0x01,
    DM4340_M2_MATER  = 0x02,
    DM4340_M3_MATER  = 0x03,

    /*DM4340电机的CAN slave ID*/
    DM4340_M1 = 0x11,
    DM4340_M2 = 0x12,
    DM4340_M3 = 0x13,

    /*DM8006电机的CAN master ID*/
    DM8006_M1_MATER = 0x04,

    /*DM8006电机的CAN slave ID*/
    DM8006_M1 = 0x14,

    /*HT8115电机的CAN ID*/
    HT8115_ALL = 0x00,
    HT8115_M1 = 0x50,

} can_msg_id_e;


typedef struct 
{
	int id;                // 帧ID
    int state;             // 状态

    float f_kp;            // 位置环增益
    float f_p;            // 位置环偏差
    float f_kd;            // 速度环增益
    float f_v;            // 速度环偏差
    float f_t;            // 转矩

    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float esc_back_position; // 返回的位置
    float esc_back_speed;    // 反馈速度
    float esc_back_current;  // 反馈电流/扭矩
    float esc_back_angle;    // 反馈电流/扭矩
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;

    /*处理连续码盘值*/
    float esc_back_position_last; // 上一次返回的位置
    int64_t circle_num;           // 旋转圈数
    float serial_position;        // 总码盘值
    float serial_angle;           // 总角度
    float serial_angle_last;      // 上一次的总角度
    float real_angle;         // 真实角度

    /*目标值*/
    float target_speed; // 目标速度
    float set_speed;    // 设置速度
    double target_position; // 目标位置
    float target_angle;     // 目标角度
    int target_state;

    /*电机输出电流*/
    float out_current; // 输出电流
} s_motor_data_t;


typedef struct
{
	/**电机基础信息**/
    int     ID;//电机所连电调CAN通信ID
	int16_t temperature;//电机温度
	/**连续圈数变量**/
	int64_t circle_num;//电机连续编码圈数
    uint8_t is_pos_ready;//电机上电时目标位置为返回的绝对编码值，只用在了连续编码函数里
	/**刻度转角度**/
	float	back_motor_ang;//电机当前编码器转换成角度（刻度转角度）
	float   serial_motor_ang;//电机连续编码转换成角度（刻度转角度）
	double	target_motor_ang;//电机目标角度（用在PID位置环）（刻度转角度）
	/**刻度**/
	int64_t serial_position;//电机连续编码值（刻度）
	int16_t back_position;//电机返回的编码器值（刻度）
	int16_t back_pos_last;//电机连续编码上一次值，只用在电机连续编码函数里了（刻度）
	int16_t back_motor_speed;//电机当前速度（刻度）
    double  target_pos;//电机目标编码器值（用在PID位置环）（刻度）
	float   target_motor_speed;//目标电机速度（用在PID单环速度环）（刻度）
	/**motor+IMU**/
	float   back_ang_imu;//返回的imu角度（motor+IMU）
	float   back_ang_last_imu;//返回的上一次imu角度（motor+IMU）
	float   back_ang_speed_imu;//返回的imu角速度（motor+IMU）
	float   back_ang_speed_last_imu;//返回的上一次imu角速度（motor+IMU）
	float   target_ang_imu;//目标imu角度（motor+IMU）
    float   target_ang_speed_imu;//目标imu角速度（motor+IMU）
	/**电流**/
    int16_t out_current;//输出电流值
    int16_t back_current;//返回电流值
	
} s_Dji_motor_data_t;//电机信息结构体类型

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    int circle_num;
    float serial_position;
    float position_angle;

} motor_measure_t;


/*RM电机的控制函数*/

void CAN_cmd_6020(int16_t CMD_ID_1, int16_t CMD_ID_2, int16_t CMD_ID_3, int16_t CMD_ID_4);
void CAN_cmd_3508(int16_t CMD_ID_1, int16_t CMD_ID_2, int16_t CMD_ID_3, int16_t CMD_ID_4);
void CAN_cmd_striker(int16_t CMD_ID_5, int16_t CMD_ID_6);


/*达妙电机的控制函数*/

extern void MD_CanReceive(s_motor_data_t *motor, uint8_t RxDate[8]);

void DM_motor_start(CAN_HandleTypeDef *Target_hcan, uint16_t id);

void MD4340_motor_PID_Control(CAN_HandleTypeDef *hcan, uint32_t id, float _torq);

void DM8006_motor_PID_Control(CAN_HandleTypeDef *hcan, uint32_t id, float _torq);


/*海泰电机的控制函数*/

void HT04_motor_start(CAN_HandleTypeDef *Target_hcan, uint8_t id);

void HT04_CanReceive(s_motor_data_t *motor, uint8_t *RxData);

void HT04_motor_PID_Control(CAN_HandleTypeDef *hcan, uint32_t id, float _torq);


/*通讯协议的处理函数*/
void continue_motor_pos(s_Dji_motor_data_t *s_motor);

float RUD_DirAngle_Proc(float Angle);

int float_to_uint(float x, float x_min, float x_max, unsigned int bits);

static float uint_to_float(int x_int, float x_min, float x_max, int bits);

void Byte_to_Float(float *date1, float *date2, unsigned char byte[]);

extern const s_Dji_motor_data_t *get_3508_M1_motor_measure_point(void);
extern const s_Dji_motor_data_t *get_3508_M2_motor_measure_point(void);
extern const s_Dji_motor_data_t *get_3508_M3_motor_measure_point(void);
extern const s_Dji_motor_data_t *get_3508_M4_motor_measure_point(void);

#endif
