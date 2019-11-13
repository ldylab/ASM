#include "stdAfx.h"

DMA_InitTypeDef DMA_InitStructure;
u8 sw;
u8 mode;
u8 sw1,sw2;
u16 DMA1_MEM_LEN; //保存DMA每次的传送长度
unsigned char dr16_rbuff[25];	//数据包
int16_t SBUS_Ch[11];              //解算以后的遥控器数据

//DMA1的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_CHx:DMA通道CHx
//cpar:外设地址
//cmar:存储器地址
//cndtr:数据传输量 

void RC_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/*--------------------Configure GPIO---------------------*/
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure); 
		
	}
	/*--------------------Configure USART---------------------*/
	{
		USART_InitTypeDef USART_InitStructure;
		
		USART_InitStructure.USART_BaudRate = 100000;//串口波特率
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_Even;//偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
		
		USART_Init(USART2, &USART_InitStructure); //初始化串口1
		
		USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//开启串口接受中断
		USART_Cmd(USART2, ENABLE);                    //使能串口1 
		USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	}
	/*--------------------Configure DMA---------------------*/
	{
		DMA_InitTypeDef DMA_InitStructure;
		
		DMA_DeInit(DMA1_Channel6);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(USART2->DR);  //DMA外设基地址
		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(dr16_rbuff);  //DMA内存基地址
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //数据传输方向，从外设到内存
		DMA_InitStructure.DMA_BufferSize = 25;  //DMA通道的DMA缓存的大小//25
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //工作在正常缓存模式   (现在设的是循环采集，试试再换回来)
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //DMA通道 x拥有中优先级 
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
		
		DMA_Init(DMA1_Channel6, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器
		
		DMA_ITConfig(DMA1_Channel6,DMA_IT_TC,ENABLE);
		//MY_NVIC_Init(0,2,DMA1_Channel5_IRQn,2);
		DMA_Cmd(DMA1_Channel6,ENABLE); 
	}
}

void DMA1_Channel6_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_IT_TC6)==SET)//DMA1 Channel5 transfer complete flag.
	{
		//USART_SendData(USART1,dr16_rbuff[0]);
		
		//USART_SendData(USART1,i);
		SBUS_Ch[0]=(dr16_rbuff[1]|(dr16_rbuff[2])<<8)&0x07FF;
		SBUS_Ch[0]-=1024;//right left and right
		SBUS_Ch[1]=(dr16_rbuff[2]>>3|dr16_rbuff[3]<<5)&0x07FF;
		SBUS_Ch[1]-=1024;//left up and down 
		SBUS_Ch[2]=(dr16_rbuff[3]>>6|(dr16_rbuff[4])<<2|dr16_rbuff[5]<<10)&0x07FF;
		SBUS_Ch[2]-=1024;//left up and down
		SBUS_Ch[3]=(dr16_rbuff[5]>>1|(dr16_rbuff[6])<<7)&0x07FF;
		SBUS_Ch[3]-=1024;//left left and right
		
		
		if(SBUS_Ch[0] <= 5 && SBUS_Ch[0] >= -5)//右横
			SBUS_Ch[0] = 0;
		if(SBUS_Ch[1] <= 5 && SBUS_Ch[1] >= -5)//右竖
			SBUS_Ch[1] = 0;
		if(SBUS_Ch[2] <= 5 && SBUS_Ch[2] >= -5)//左竖
			SBUS_Ch[2] = 0;
		if(SBUS_Ch[3] <= 5 && SBUS_Ch[3] >= -5)//左横
			SBUS_Ch[3] = 0;
		
		DMA_ClearFlag(DMA1_IT_TC6);			
	}
}

void NVIC_Configuration(void)
{
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
    NVIC_SetPriority (SysTick_IRQn, 0);
   
		//定时器中断优先级
//    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
//		NVIC_Init(&NVIC_InitStructure);  
//   
//    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
//		NVIC_Init(&NVIC_InitStructure); 

//		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&NVIC_InitStructure);          
		
		NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;         
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;   
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;    
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     
		NVIC_Init(&NVIC_InitStructure);  
		
		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;         
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     
		NVIC_Init(&NVIC_InitStructure);   

		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;         
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;    
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     
		NVIC_Init(&NVIC_InitStructure);                            

		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

//    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn ;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);

//    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);   
}
