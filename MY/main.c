/***
	***************************************************************************
	*	@file  	 main.c
	*	@version V1.0
	* @date    2017-8-1		
	*	@author  Henry
	*	@brief   �Ŷ����ܰ����
   ***************************************************************************
   *  @description
	***************************************************************************
***/

#include "stdAfx.h"

int Encoder_1, Encoder_2, Encoder_3, Encoder_4;

int main(void)
{
	LED_Init();                      //һ����������(LED1_ON, LED2_OFF)
	Delay_Init();     							 //��ʱ����
  Usart_Config();   							 //���ڴ򿪣�֧��ֱ��ʹ�ùٷ���DAP��
  KEY_Init();       							 //��������
	GPIO_Configuration();           //GPIO�����ʼ��
	TIM8_PWM_Init(7199, 0);         //��ʼ��PWM������ֵΪ7200
	Encoder_Init_TIM2();             //=====�������ӿ�
	Encoder_Init_TIM3();             //=====�������ӿ�
	Encoder_Init_TIM4();             //=====�������ӿ�
	Encoder_Init_TIM5();             //=====�������ӿ�
	RC_Init();
	IMU_Init(115200);
	uart4_init(9600);
	NVIC_Configuration();
	Delay_ms(100);
	TIM6_Int_Init(49, 7199); 				 //��ʱ��6��10ms

	while(1)
	{		
//		LED1_ON;
//		LED2_OFF;	
//		Delay_ms(250);	
//		LED1_OFF;
//		LED2_ON;	
//		Delay_ms(250);	
		printf("%d, %d, %d, %d, %d\r\n", dis_1, dis_2, dis_3, dis_4, (int)(angle_z+angle_rate));//rate�����Ǹ�������������
	}
}

