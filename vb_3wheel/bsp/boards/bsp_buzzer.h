#ifndef BSP_BUZZER_H

#include "stm32f4xx.h"
#include "tim.h"

extern void buzzer_on(uint16_t psc, uint16_t pwm);
extern void buzzer_off(void);

#endif
