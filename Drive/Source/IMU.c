#include "stdAfx.h"
/*串口3初始化(mpu6050) < uart3_init(填写波特率); >*/
void IMU_Init(u32 bound){
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	//USART3_TX   GPIOB.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_Init(GPIOB, &GPIO_InitStructure);  
	   
	//USART3_RX	  GPIOB.11初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
    //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
 
	USART_Init(USART3, &USART_InitStructure); //初始化串口3
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(USART3, ENABLE);                    //使能串口3
}

#if EN_USART3_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART3_RX_BUF[USART3_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART3_RX_STA=0;       //接收状态标记	  
struct GYROs gyro;
float angle_x, angle_y, angle_z;
long IMU_inner_times = 0;
void USART3_IRQHandler(void)
{
	uint8_t ucTemp;
	IMU_inner_times++;
	u8 i;
	u8 res;
	u8 sum=0;
	//printf("COME\r\n");
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	//printf("COME____2\r\n");
	if(USART3->SR&(1<<5))	//接收到数据
	{	
		res=USART3->DR; 
		if((USART3_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART3_RX_STA&0x4000)//接收到了0x55
			{
				USART3_RX_BUF[USART3_RX_STA&0X3FFF]=res;
				if((USART3_RX_STA&0X3FFF)==10)
				{
					//printf("RIGHT");
					for(i=0;i<10;i++) 
					{
						//printf("BUF = %x\r\n", UART4_RX_BUF[i]);
						sum+=USART3_RX_BUF[i];
					}

					if(sum == USART3_RX_BUF[10])
					{
						
						if(USART3_RX_BUF[1]==0x51) 	//加速度
						{
							gyro.ax = 156.8*(((short)(USART3_RX_BUF[3]<<8))|USART3_RX_BUF[2])/32768;
							gyro.ay = 156.8*(((short)(USART3_RX_BUF[5]<<8))|USART3_RX_BUF[4])/32768;
							gyro.az = 156.8*(((short)(USART3_RX_BUF[7]<<8))|USART3_RX_BUF[6])/32768;
							USART3_RX_STA=0;
						}
						else if(USART3_RX_BUF[1]==0x53)	//角度
						{
							gyro.roll = 180.0*((USART3_RX_BUF[3]<<8)|USART3_RX_BUF[2])/32768;
							gyro.pitch = 180.0*((USART3_RX_BUF[5]<<8)|USART3_RX_BUF[4])/32768;
							gyro.yaw = 180.0*((USART3_RX_BUF[7]<<8)|USART3_RX_BUF[6])/32768;
							angle_x = gyro.roll;
							angle_y = gyro.pitch;
							angle_z = gyro.yaw;
							USART3_RX_STA=0;
						}
						else USART3_RX_STA=0;
					}
					else USART3_RX_STA=0; 	//校验错误
				}
				else USART3_RX_STA++;
			} 
			else //还没收到0X55
			{
				if(res==0x55)
				{
					USART3_RX_STA|=0x4000;
					USART3_RX_BUF[USART3_RX_STA&0X3FFF]=res;
					USART3_RX_STA++;
				}
			}
		} 									     
	}
	if(USART_GetITStatus(USART3,USART_IT_RXNE)!=RESET) 
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE); //清除一下下
		ucTemp=USART_ReceiveData(USART3);
	}
	if(USART_GetFlagStatus(USART3,USART_FLAG_ORE) == SET) //清除一下下
	{
		USART_ClearFlag(USART3,USART_FLAG_ORE);
		USART_ReceiveData(USART3);
	}
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 
#endif		
