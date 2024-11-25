#include "bsp_buzzer.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;//结构体定义在stm32f4xx_hal_tim.h文件中

/*定义蜂鸣器开启函数，设置分频值和重载值*/
void buzzer_on(uint16_t psc, uint16_t pwm)
{
	__HAL_TIM_PRESCALER(&htim4, psc);//设置分频值
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);//设置比较值=pwm，进而修改占空比
}


/*蜂鸣器关闭函数，把比较值设为0*/
void buzzer_off(void)
{
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);//设置比较值为0，即高电平为0
}

//
void Di_Di(void)
{
	static uint16_t count = 0;
	count ++;
	if(count > 0 && count < 2 )
	buzzer_on(1, 50);
	if(count > 2 && count < 6 )
	buzzer_off();
	if(count > 6 && count < 8 )
	buzzer_on(1, 50);
	if(count > 8 && count < 68)
	buzzer_off();
	else if(count > 68)
	count = 0;
}
