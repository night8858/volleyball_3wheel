#ifndef SENSOR_H_
#define SENSOR_H_
#include "stm32f4xx.h"

typedef struct
{
    float yaw   ;
    float pitch ;
    float roll  ;
    float pos_x ;
    float pos_y ;
    float w_z   ;

} s_ops9_data_t;

void ops9_init(void);

void usart6_send(char *data, uint8_t num);

void ops9_Zero_Clearing(void);
void Update_A(float angle);
void Update_X(float posx);
void Update_Y(float posy);
void Update_XY(float posx, float posy);

#endif /* SENSOR_H_ */
