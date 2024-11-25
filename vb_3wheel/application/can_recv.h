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
#define DM4340_T_MIN -28.0f   // 转矩最大值
#define DM4340_T_MAX 28.0f    // 转矩最小值

#define HT04_P_MIN   -95.5f
#define HT04_P_MAX   95.5f
#define HT04_V_MIN   -45.0f
#define HT04_V_MAX   45.0f
#define HT04_KP_MIN   0.0f
#define HT04_KP_MAX   500.0f
#define HT04_KD_MIN   0.0f
#define HT04_KD_MAX   5.0f
#define HT04_T_MIN   -18.0f
#define HT04_T_MAX   18.0f
#define HT04_T_MIN_04   -50.0f
#define HT04_T_MAX_04   50.0f

/* CAN send and receive ID */
/*所有can消息的ID*/
typedef enum
{
    /*RM3508电机的CAN ID*/
    CAN_3508_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    /*6020电机的CAN ID*/
    CAN_6020_ALL_ID = 0x1FF,
    CAN_6020_M1_ID = 0X205,
    CAN_6020_M2_ID = 0x206,
    CAN_6020_M3_ID = 0x207,
    CAN_6020_M4_ID = 0x208,

    /*板间通讯的CAN ID*/
    DT7_RX_ch = 0x301,
    DT7_RX_S  = 0x302,
    AUTO_MODE_CMD = 0x303,

    /*DM4340电机的CAN ID*/
    DM4340_M1 = 0x11,
    DM4340_M2 = 0x12,
    DM4340_M3 = 0x13,

    /*HT8115电机的CAN ID*/
    HT8115_ALL = 0x0,
    HT8115_M1 = 0x50,

} can_msg_id_e;

typedef struct
{
    int id;        // ID
    int state;     // 状态
    int p_int;     // 位置
    int v_int;     // 速度
    int t_int;     // 转矩
    int kp_int;    // Kp
    int kd_int;    // Kd
    float esc_back_position; // 返回的位置-12.5~12.5
    float esc_back_speed;    // 反馈速度-45.0~45.0
    float esc_back_current;  // 反馈电流/扭矩
    float esc_back_angle;    // 反馈电流/扭矩
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;

    float T_calc; // 计算得到的转矩

    /*处理连续码盘值*/
    float esc_back_position_last; // 上一次返回的位置
    int64_t circle_num;           // 旋转圈数
    float serial_position;        // 总码盘值
    float serial_angle;           // 总角度
    float serial_angle_last;
    float real_angle; // 真实角度
    float real_speed; // 真实速度
    
    /*目标值*/
    float target_speed; // 目标速度
    float set_speed;
    double target_position; // 目标位置
    float target_angle;     // 目标角度
    int target_state;

    /*电机输出电流*/
    float out_current; // 输出电流

} DM4340_motor_data_t;

typedef struct 
{
	int id;
    int state;

    float f_kp;
    float f_p;
    float f_kd;
    float f_v;
    float f_t;

    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float esc_back_position; // 返回的位置
    float esc_back_speed;    // 反馈速度-45.0~45.0
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
    float serial_angle_last;
    float real_angle; // 真实角度

    /*目标值*/
    float target_speed; // 目标速度
    float set_speed;
    double target_position; // 目标位置
    float target_angle;     // 目标角度
    int target_state;

    /*电机输出电流*/
    float out_current; // 输出电流
} ht_motor_data_t;

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
	
} s_motor_data_t;//电机信息结构体类型

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

void CAN_cmd_6020(int16_t CMD_ID_1, int16_t CMD_ID_2, int16_t CMD_ID_3, int16_t CMD_ID_4);
void CAN_cmd_3508(int16_t CMD_ID_1, int16_t CMD_ID_2, int16_t CMD_ID_3, int16_t CMD_ID_4);


/*达妙电机的控制函数*/

extern void MD_CanReceive(DM4340_motor_data_t *motor, uint8_t RxDate[8]);

void DM_motor_start(CAN_HandleTypeDef *Target_hcan, uint16_t id);

void MD_motor_SendCurrent(CAN_HandleTypeDef *hcan, uint32_t id, float _torq);





void HT04_motor_start(CAN_HandleTypeDef *Target_hcan, uint8_t id);

void HT04_CanReceive(ht_motor_data_t *motor, uint8_t *RxData);

void HT04_motor_PID_Control(CAN_HandleTypeDef *hcan, uint32_t id, float _torq);



float RUD_DirAngle_Proc(float Angle);


int float_to_uint(float x, float x_min, float x_max, unsigned int bits);

static float uint_to_float(int x_int, float x_min, float x_max, int bits);

void Byte_to_Float(float *date1, float *date2, unsigned char byte[]);

void Float_to_Byte(float a, float b, unsigned char byte[]);


extern const motor_measure_t *get_3508_M1_motor_measure_point(void);
extern const motor_measure_t *get_3508_M2_motor_measure_point(void);
extern const motor_measure_t *get_3508_M3_motor_measure_point(void);
extern const motor_measure_t *get_3508_M4_motor_measure_point(void);
extern const motor_measure_t *get_6020_M1_motor_measure_point(void);
extern const motor_measure_t *get_6020_M2_motor_measure_point(void);
extern const motor_measure_t *get_6020_M3_motor_measure_point(void);
extern const motor_measure_t *get_6020_M4_motor_measure_point(void);

#endif
