#include "stdAfx.h"

# define FirAIN1 GPIO_Pin_12
# define FirAIN2 GPIO_Pin_13
# define FirBIN1 GPIO_Pin_14
# define FirBIN2 GPIO_Pin_15

# define SecAIN1 GPIO_Pin_8
# define SecAIN2 GPIO_Pin_9
# define SecBIN1 GPIO_Pin_4
# define SecBIN2 GPIO_Pin_5

/*��ʼ��GPIO���������С����I/O��*/
void  GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIOB_InitStructure;//��������GPIO_InitTypeDef���͵Ľṹ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //����GPIOB����ʱ��
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
	
    GPIOB_InitStructure.GPIO_Pin = FirAIN1 | FirAIN2 | FirBIN1 | FirBIN2 | SecAIN1 | SecAIN2 | SecBIN1 | SecBIN2;             //ѡ��Ҫ���Ƶ�GPIOB����
    GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       //��������ģʽΪ����ʽ���
    GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //���������ٶ�Ϊ50MHz

    GPIO_Init(GPIOB, &GPIOB_InitStructure);                 //���ó�ʼ��GPIOB
    GPIO_SetBits(GPIOB, FirAIN1 | FirAIN2 | FirBIN1 | FirBIN2 | SecAIN1 | SecAIN2 | SecBIN1 | SecBIN2);                       //�����رյ��
}

/*��ʼ��TIM8��ʹ����PC6~9����PWM*/
void TIM8_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);//ʹ�ܶ�ʱ��3ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIO
	//��ʼ��TIM8

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler = psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM8 Channel2 PWMģʽ
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset ;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low ; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM8, &TIM_OCInitStructure); //����Tָ���Ĳ�����ʼ������TIM8 OC2
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable); //ʹ��TIM8��CCR2�ϵ�Ԥװ�ؼĴ���
	//TIM_SetCompare1(TIM8,474);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset ;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low ; //�������:TIM����Ƚϼ��Ը�
	TIM_OC2Init(TIM8, &TIM_OCInitStructure); //����Tָ���Ĳ�����ʼ������TIM8 OC2
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable); //ʹ��TIM8��CCR2�ϵ�Ԥװ�ؼĴ���
	//TIM_SetCompare2(TIM8,700);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset ;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low ; //�������:TIM����Ƚϼ��Ը�
	TIM_OC3Init(TIM8, &TIM_OCInitStructure); //����Tָ���Ĳ�����ʼ������TIM8 OC2
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable); //ʹ��TIM8��CCR2�ϵ�Ԥװ�ؼĴ���
	//TIM_SetCompare3(TIM8,900);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset ;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low ; //�������:TIM����Ƚϼ��Ը�
	TIM_OC4Init(TIM8, &TIM_OCInitStructure); //����Tָ���Ĳ�����ʼ������TIM8 OC2
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable); //ʹ��TIM8��CCR2�ϵ�Ԥװ�ؼĴ���
	//TIM_SetCompare4(TIM8,1200);
	
	TIM_Cmd(TIM8, ENABLE); //��ʹ��TIM8
	TIM_CtrlPWMOutputs(TIM8,ENABLE);
}

void Motor_go(int v_1, int v_2, int v_3)
{

	if(v_1 < 0)
	{
		GPIO_ResetBits(GPIOB, SecBIN1);                     
		GPIO_SetBits(GPIOB, SecBIN2);  
		TIM_SetCompare4(TIM8, speed_limit(v_1));
	}
		//AIN2 = 1,			AIN1 = 0;
	else if(v_1 > 0)
	{
		GPIO_SetBits(GPIOB, SecBIN1);                     
		GPIO_ResetBits(GPIOB, SecBIN2);  
		TIM_SetCompare4(TIM8, speed_limit(v_1));
	}
	else if(v_1 == 0)
	{
		GPIO_SetBits(GPIOB, SecBIN1);                     
		GPIO_SetBits(GPIOB, SecBIN2);  
		TIM_SetCompare4(TIM8, 0);
	}
	
	if(v_2 < 0)
	{
		GPIO_ResetBits(GPIOB, FirBIN1);                     
		GPIO_SetBits(GPIOB, FirBIN2);  
		TIM_SetCompare2(TIM8, speed_limit(v_2));
	}
		//AIN2 = 1,			AIN1 = 0;
	else if(v_2 > 0)
	{
		GPIO_SetBits(GPIOB, FirBIN1);                     
		GPIO_ResetBits(GPIOB, FirBIN2);  
		TIM_SetCompare2(TIM8, speed_limit(v_2));
	}
	else if(v_2 == 0)
	{
		GPIO_SetBits(GPIOB, FirBIN1);                     
		GPIO_ResetBits(GPIOB, FirBIN2);  
		TIM_SetCompare2(TIM8, 0);
	}
	
	if(v_3 > 0)
	{
		GPIO_ResetBits(GPIOB, SecAIN1);                     
		GPIO_SetBits(GPIOB, SecAIN2);  
		TIM_SetCompare3(TIM8, speed_limit(v_3));
	}
		//AIN2 = 1,			AIN1 = 0;
	else if(v_3 < 0)
	{
		GPIO_SetBits(GPIOB, SecAIN1);                     
		GPIO_ResetBits(GPIOB, SecAIN2);  
		TIM_SetCompare3(TIM8, speed_limit(v_3));
	}
	else if(v_3 == 0)
	{
		GPIO_SetBits(GPIOB, SecAIN1);                     
		GPIO_SetBits(GPIOB, SecAIN2);  
		TIM_SetCompare3(TIM8, 0);
	}
}

		
int speed_limit(int speed)
{
	if(speed > 0)
	{
		if(speed > 7199)
		{
			return 7199;
		}
		else
		{
			return speed;
		}
	}
	else if(speed < 0)
	{
		if(speed < -7199)
		{
			return 7199;
		}
		else
		{
			return -speed;
		}
	}
	else
	{
		return speed;
	}
}
