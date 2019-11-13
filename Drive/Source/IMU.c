#include "stdAfx.h"
/*����3��ʼ��(mpu6050) < uart3_init(��д������); >*/
void IMU_Init(u32 bound){
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	//USART3_TX   GPIOB.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_Init(GPIOB, &GPIO_InitStructure);  
	   
	//USART3_RX	  GPIOB.11��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
    //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
 
	USART_Init(USART3, &USART_InitStructure); //��ʼ������3
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���3
}

#if EN_USART3_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART3_RX_BUF[USART3_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART3_RX_STA=0;       //����״̬���	  
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
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();    
#endif
	//printf("COME____2\r\n");
	if(USART3->SR&(1<<5))	//���յ�����
	{	
		res=USART3->DR; 
		if((USART3_RX_STA&0x8000)==0)//����δ���
		{
			if(USART3_RX_STA&0x4000)//���յ���0x55
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
						
						if(USART3_RX_BUF[1]==0x51) 	//���ٶ�
						{
							gyro.ax = 156.8*(((short)(USART3_RX_BUF[3]<<8))|USART3_RX_BUF[2])/32768;
							gyro.ay = 156.8*(((short)(USART3_RX_BUF[5]<<8))|USART3_RX_BUF[4])/32768;
							gyro.az = 156.8*(((short)(USART3_RX_BUF[7]<<8))|USART3_RX_BUF[6])/32768;
							USART3_RX_STA=0;
						}
						else if(USART3_RX_BUF[1]==0x53)	//�Ƕ�
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
					else USART3_RX_STA=0; 	//У�����
				}
				else USART3_RX_STA++;
			} 
			else //��û�յ�0X55
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
		USART_ClearITPendingBit(USART3,USART_IT_RXNE); //���һ����
		ucTemp=USART_ReceiveData(USART3);
	}
	if(USART_GetFlagStatus(USART3,USART_FLAG_ORE) == SET) //���һ����
	{
		USART_ClearFlag(USART3,USART_FLAG_ORE);
		USART_ReceiveData(USART3);
	}
#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();  											 
#endif
} 
#endif		
