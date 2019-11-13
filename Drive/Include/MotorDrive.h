#ifndef __MOTORDRIVE_H
#define __MOTORDRIVE_H
#include "stm32f10x.h"

# define FirAIN1 GPIO_Pin_12
# define FirAIN2 GPIO_Pin_13
# define FirBIN1 GPIO_Pin_14
# define FirBIN2 GPIO_Pin_15

# define SecAIN1 GPIO_Pin_8
# define SecAIN2 GPIO_Pin_9
# define SecBIN1 GPIO_Pin_4
# define SecBIN2 GPIO_Pin_5

void  GPIO_Configuration(void);
void TIM1_UP_IRQHandler(void);
void TIM8_PWM_Init(u16 arr, u16 psc);
int speed_limit(int speed);
void Motor_go(int v_1, int v_2, int v_3);

#endif
