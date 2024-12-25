#include "sensor.h"
#include "stm32f407xx.h"
#include "Variables.h"
#include "Monitor.h"
#include "gpio.h"

extern s_robo_Mode_Setting robot_StateMode;
extern s_task_flags task_flags;

void Sensor_Init(void)
{
    

}


//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    if (GPIO_Pin == E3Z_L61_Pin)
//    {
//        
//        task_flags.sensor_is_blocked = 1;
//
//    }
//    
//
//}