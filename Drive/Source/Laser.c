#include "stdAfx.h"

/*串口4初始化(Tof) < uart4_init(填写波特率); >*/
void uart4_init(u32 bound){
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	
	//USART4_TX   GPIOC.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PC.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC.10
	   
	//USART1_RX	  GPIOC.11初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PC.11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC.11
	
    //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

	USART_Init(UART4, &USART_InitStructure); //初始化串口4
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(UART4, ENABLE);                    //使能串口4
}

char Servo_y[3];
int numbit_y = 0;
int num_int_y;

int temp_rece = 0;

char Servo_x[6];
int numbit_x = 0;
int num_int_x;
char Cam_rec[20];
int Cam_numbit = 0;
int Cam_times = 0;
int Cam_num_int = 0;
int Last_times = 0;
int Cam_times_dp = 0;
int Rece_EN = 0;
char Return_servo[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int Comma_bit;
int Semi_bit;
char W_speed_char[4] = {0, 0, 0, 0};
char Servo_angle_char[4] = {0, 0, 0, 0};
char Catcher_char[1] = {0};
int W_speed_int;
int Servo_angle_int;
int Semi_rate;
int i_last = 0;
int Comma_times = 0;
int Last_comma_bit = 0;
int Last_comma_times;

char Rece_array[30]; //用于接收数据的数组
int Rece_data_int[10]; //最多可以接收十组数据
int Posi_array[10]; //放置'[' ',' ']' 位置的数组
char Temp_int[10]; //临时接收，准备转换为整数的数组
int Temp_count;

int Rece_count = 0; //用于接收数据的计算次数
int Reset_state = 1; //用于在最开始的时候对Rece_array进行清零
int Posi_count = 0; //有多少次

int Comma_posi = 1; // 用于统计逗号的个数（在数组中的运算）
int Analy_state = 0; //是否进入截取分析数组的函数中，在接收到 ']' 就是说明已经可以开始处理的了

int dis_1, dis_2, dis_3, dis_4, sum;

int Catcher_state;

// 返回一个特定的字符在数组中的位置
void FindPosi(void)
{
    int i;
    for(i = 0; i <= Rece_count; i++)
    {
        if(Rece_array[i] == '[')
        {
            Posi_array[0] = i;
        }

        else if(Rece_array[i] == ',')
        {
            Posi_array[Comma_posi] = i;
            Comma_posi++;
        }

        else if(Rece_array[i] == ']')
        {
            Posi_array[Comma_posi] = i;
            Posi_count =  Comma_posi;
            Comma_posi = 1;
        }
    }
}

void ResetArray(char array[])
{
    memset(array,'\0',sizeof(array));
}


void UART4_IRQHandler(void)
{
	if(Reset_state == 1)
    {
        ResetArray(Rece_array);
        ResetArray(Temp_int);
        Rece_count = 0;
        Reset_state = 0;
        Analy_state = 0;
        Comma_posi = 1;
    }
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)//判断接收标志
	{
		temp_rece++;
			//接收到最开始的标志 '[' 后开始把接收到的数据放在数组里面
			if(USART_ReceiveData(UART4) == '[' || Rece_EN == 1)
			{
					Rece_array[Rece_count] = UART4->DR;
					Rece_array[Rece_count] = USART_ReceiveData(UART4);
				 
					if(Rece_array[Rece_count] == ']')
					{
							Rece_EN = 0;
							Analy_state = 1;
					}
					else
					{
							Rece_EN = 1;         
							Rece_count++; //能够反映一共接收到了多少数据
							Analy_state = 0;
					}
			}

			//开始对所接收到的数据进行处理
			if(Analy_state == 1)
			{
				int i, n;
				// 寻找 '[' ',' ']' 在数组里面的相对位置，储存在Posi_array[]数值里面
				FindPosi();

				//把两个符号之间的数给拿出来
				for(i = 0; i <= Posi_count; i++)
				{
						Temp_count = 0;
						for(n = Posi_array[i]; n < Posi_array[i + 1]; n++)
						{
								Temp_int[Temp_count] = Rece_array[n + 1];
								Temp_count++;
						}

						Rece_data_int[i] = atoi(Temp_int);
				}
				Reset_state = 1;
			}
			
			if(Rece_data_int[4] == (Rece_data_int[0] + Rece_data_int[1] + Rece_data_int[3] + Rece_data_int[2]))
			{
				dis_1 = Rece_data_int[0];
				dis_2 = Rece_data_int[1];
				dis_3 = Rece_data_int[2];
				dis_4 = Rece_data_int[3];
			}
		}
}
