#ifndef CAN_RECV_H
#define CAN_RECV_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

/* CAN send and receive ID */
typedef enum
{
    CAN_3508_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_6020_ALL_ID = 0x1FF,
    CAN_6020_M1_ID = 0X205,
    CAN_6020_M2_ID = 0x206,
    CAN_6020_M3_ID = 0x207,
    CAN_6020_M4_ID = 0x208,

    DT7_RX_ch = 0x301,
    DT7_RX_S  = 0x302,
    AUTO_MODE_CMD = 0x303,

} can_msg_id_e;

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

void Byte_to_Float(float *date1, float *date2, unsigned char byte[]);

extern const motor_measure_t *get_3508_M1_motor_measure_point(void);
extern const motor_measure_t *get_3508_M2_motor_measure_point(void);
extern const motor_measure_t *get_3508_M3_motor_measure_point(void);
extern const motor_measure_t *get_3508_M4_motor_measure_point(void);
extern const motor_measure_t *get_6020_M1_motor_measure_point(void);
extern const motor_measure_t *get_6020_M2_motor_measure_point(void);
extern const motor_measure_t *get_6020_M3_motor_measure_point(void);
extern const motor_measure_t *get_6020_M4_motor_measure_point(void);

#endif
