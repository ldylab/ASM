#include "stdAfx.h"

#define IO1_0 	  GPIO_ResetBits(GPIOB, GPIO_Pin_2);	// 输出低电平，点亮LED1	
#define IO1_1 	  GPIO_SetBits(GPIOB, GPIO_Pin_2);		// 输出高电平，关闭LED1	
#define IO3_0 	  GPIO_ResetBits(GPIOB, GPIO_Pin_0);	// 输出低电平，点亮LED1	
#define IO3_1 	  GPIO_SetBits(GPIOB, GPIO_Pin_0);		// 输出高电平，关闭LED1	
#define IO2_0 	  GPIO_ResetBits(GPIOA, GPIO_Pin_4);	// 输出低电平，点亮LED1	
#define IO2_1 	  GPIO_SetBits(GPIOA, GPIO_Pin_4);		// 输出高电平，关闭LED1	

void SendCmd_Configuration(void)
{
	GPIO_InitTypeDef GPIOB_InitStructure;//定义两个GPIO_InitTypeDef类型的结构体
	GPIO_InitTypeDef GPIOA_InitStructure;//定义两个GPIO_InitTypeDef类型的结构体
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //开启GPIOB外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
	
	GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2;
	GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       //配置引脚模式为推挽式输出
	GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //设置引脚速度为50MHz
	GPIO_Init(GPIOB, &GPIOB_InitStructure);                 //调用初始化GPIOB       

	GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       //配置引脚模式为推挽式输出
	GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //设置引脚速度为50MHz
	GPIO_Init(GPIOA, &GPIOA_InitStructure);                 //调用初始化GPIOB       
}

void SendCmd(int Cmd)
{
	switch(Cmd)
	{
		case 0:
			IO1_0;
			IO2_0;
			IO3_0;
			break;
		case 2:
			IO1_1;
			IO2_0;
			IO3_0;
			break;
		case 1:
			IO1_0;
			IO2_1;
			IO3_0;
			break;
		case 3:
			IO1_1;
			IO2_1;
			IO3_0;
			break;
		case 4:
			IO1_0;
			IO2_0;
			IO3_1;
			break;
		case 6:
			IO1_1;
			IO2_0;
			IO3_1;
			break;
		case 5:
			IO1_0;
			IO2_1;
			IO3_1;
			break;
		case 7:
			IO1_1;
			IO2_1;
			IO3_1;
			break;
	}
}