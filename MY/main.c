/***
	***************************************************************************
	*	@file  	 main.c
	*	@version V1.0
	* @date    2017-8-1		
	*	@author  Henry
	*	@brief   团队万能板控制
   ***************************************************************************
   *  @description
	***************************************************************************
***/

#include "stdAfx.h"

int Encoder_1, Encoder_2, Encoder_3, Encoder_4;

int main(void)
{
	LED_Init();                      //一共有两个灯(LED1_ON, LED2_OFF)
	Delay_Init();     							 //延时控制
  Usart_Config();   							 //串口打开，支持直接使用官方的DAP线
  KEY_Init();       							 //按键输入
	GPIO_Configuration();           //GPIO输出初始化
	TIM8_PWM_Init(7199, 0);         //初始化PWM，极限值为7200
	Encoder_Init_TIM2();             //=====编码器接口
	Encoder_Init_TIM3();             //=====编码器接口
	Encoder_Init_TIM4();             //=====编码器接口
	Encoder_Init_TIM5();             //=====编码器接口
	RC_Init();
	IMU_Init(115200);
	uart4_init(9600);
	NVIC_Configuration();
	Delay_ms(100);
	TIM6_Int_Init(49, 7199); 				 //定时器6，10ms

	while(1)
	{		
//		LED1_ON;
//		LED2_OFF;	
//		Delay_ms(250);	
//		LED1_OFF;
//		LED2_ON;	
//		Delay_ms(250);	
		printf("%d, %d, %d, %d, %d\r\n", dis_1, dis_2, dis_3, dis_4, (int)(angle_z+angle_rate));//rate抵消那个线性误差的问题
	}
}

