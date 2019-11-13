#include "stdAfx.h"

# define FirAIN1 GPIO_Pin_12
# define FirAIN2 GPIO_Pin_13
# define FirBIN1 GPIO_Pin_14
# define FirBIN2 GPIO_Pin_15

# define SecAIN1 GPIO_Pin_8
# define SecAIN2 GPIO_Pin_9
# define SecBIN1 GPIO_Pin_4
# define SecBIN2 GPIO_Pin_5

/*初始化GPIO输出，配置小车的I/O口*/
void  GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIOB_InitStructure;//定义两个GPIO_InitTypeDef类型的结构体
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //开启GPIOB外设时钟
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
	
    GPIOB_InitStructure.GPIO_Pin = FirAIN1 | FirAIN2 | FirBIN1 | FirBIN2 | SecAIN1 | SecAIN2 | SecBIN1 | SecBIN2;             //选择要控制的GPIOB引脚
    GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       //配置引脚模式为推挽式输出
    GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //设置引脚速度为50MHz

    GPIO_Init(GPIOB, &GPIOB_InitStructure);                 //调用初始化GPIOB
    GPIO_SetBits(GPIOB, FirAIN1 | FirAIN2 | FirBIN1 | FirBIN2 | SecAIN1 | SecAIN2 | SecBIN1 | SecBIN2);                       //启动关闭电机
}

/*初始化TIM8，使其在PC6~9产生PWM*/
void TIM8_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);//使能定时器3时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能GPIO外设和AFIO复用功能模块时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO
	//初始化TIM8

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler = psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM8 Channel2 PWM模式
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset ;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low ; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM8, &TIM_OCInitStructure); //根据T指定的参数初始化外设TIM8 OC2
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable); //使能TIM8在CCR2上的预装载寄存器
	//TIM_SetCompare1(TIM8,474);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset ;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low ; //输出极性:TIM输出比较极性高
	TIM_OC2Init(TIM8, &TIM_OCInitStructure); //根据T指定的参数初始化外设TIM8 OC2
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable); //使能TIM8在CCR2上的预装载寄存器
	//TIM_SetCompare2(TIM8,700);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset ;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low ; //输出极性:TIM输出比较极性高
	TIM_OC3Init(TIM8, &TIM_OCInitStructure); //根据T指定的参数初始化外设TIM8 OC2
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable); //使能TIM8在CCR2上的预装载寄存器
	//TIM_SetCompare3(TIM8,900);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset ;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low ; //输出极性:TIM输出比较极性高
	TIM_OC4Init(TIM8, &TIM_OCInitStructure); //根据T指定的参数初始化外设TIM8 OC2
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable); //使能TIM8在CCR2上的预装载寄存器
	//TIM_SetCompare4(TIM8,1200);
	
	TIM_Cmd(TIM8, ENABLE); //不使能TIM8
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
