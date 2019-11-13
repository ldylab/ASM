/***
	***************************************************************************
	*	@file  	key.c
	*	@version V1.0.0
	*	@brief   �����ӿ���غ���
   ***************************************************************************
   *  @description
	*
	*  ��ʼ���������ţ�����Ϊ�������롢�ٶȵȼ�2M��
	* 	
	***************************************************************************
***/

#include "stdAfx.h"

// ����������IO�ڳ�ʼ��
//
void KEY_Init(void)
{		
	GPIO_InitTypeDef GPIO_InitStructure; //����ṹ��
	RCC_APB2PeriphClockCmd ( KEY_CLK, ENABLE); 	//��ʼ��KEYʱ��	
	
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; //�ٶ�ѡ��
	GPIO_InitStructure.GPIO_Pin   = KEY_PIN;	 
	
	GPIO_Init(KEY_PORT, &GPIO_InitStructure);	

}

// ����������ɨ��
//	���أ�KEY_ON - �������£�KEY_OFF - �����ſ� 
//			
u8	KEY_Scan(void)
{
	if( GPIO_ReadInputDataBit ( KEY_PORT,KEY_PIN) == 0 )	//��ⰴ���Ƿ񱻰���
	{	
		Delay_ms(10);	//��ʱ����
		if(GPIO_ReadInputDataBit ( KEY_PORT,KEY_PIN) == 0)	//�ٴμ���Ƿ�Ϊ�͵�ƽ
		{
			while(GPIO_ReadInputDataBit ( KEY_PORT,KEY_PIN) == 0);	//�ȴ������ſ�
			return KEY_ON;	//���ذ������±�־
		}
	}
	return KEY_OFF;	
}

