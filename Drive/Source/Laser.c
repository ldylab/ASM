#include "stdAfx.h"

/*����4��ʼ��(Tof) < uart4_init(��д������); >*/
void uart4_init(u32 bound){
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	
	//USART4_TX   GPIOC.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PC.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIOC.10
	   
	//USART1_RX	  GPIOC.11��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PC.11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIOC.11
	
    //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

	USART_Init(UART4, &USART_InitStructure); //��ʼ������4
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	USART_Cmd(UART4, ENABLE);                    //ʹ�ܴ���4
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

char Rece_array[30]; //���ڽ������ݵ�����
int Rece_data_int[10]; //�����Խ���ʮ������
int Posi_array[10]; //����'[' ',' ']' λ�õ�����
char Temp_int[10]; //��ʱ���գ�׼��ת��Ϊ����������
int Temp_count;

int Rece_count = 0; //���ڽ������ݵļ������
int Reset_state = 1; //�������ʼ��ʱ���Rece_array��������
int Posi_count = 0; //�ж��ٴ�

int Comma_posi = 1; // ����ͳ�ƶ��ŵĸ������������е����㣩
int Analy_state = 0; //�Ƿ�����ȡ��������ĺ����У��ڽ��յ� ']' ����˵���Ѿ����Կ�ʼ�������

int dis_1, dis_2, dis_3, dis_4, sum;

int Catcher_state;

// ����һ���ض����ַ��������е�λ��
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
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)//�жϽ��ձ�־
	{
		temp_rece++;
			//���յ��ʼ�ı�־ '[' ��ʼ�ѽ��յ������ݷ�����������
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
							Rece_count++; //�ܹ���ӳһ�����յ��˶�������
							Analy_state = 0;
					}
			}

			//��ʼ�������յ������ݽ��д���
			if(Analy_state == 1)
			{
				int i, n;
				// Ѱ�� '[' ',' ']' ��������������λ�ã�������Posi_array[]��ֵ����
				FindPosi();

				//����������֮��������ó���
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
