#ifndef __RECE_H
#define __RECE_H
#include "stdio.h"	
#include "stdAfx.h"

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define MAX_DATA_LENS       9
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
#define USART1_BASEADDR       0X40013800
#define USART1_DR_ARR         0X40013804
	
extern int16_t SBUS_Ch[11];
extern unsigned char dr16_rbuff[25];	//数据包
void RC_Init(void);
void NVIC_Configuration(void);
#endif


