#include "stdAfx.h"

PID V1_pid= {4.3, 0.16, 0};         //�����ֵkp��ki��kd
PID V2_pid= {4.3, 0.16, 0};         //�����ֵkp��ki��kd
PID V3_pid= {4.3, 0.16, 0};         //�����ֵkp��ki��kd

int V1_output, V2_output, V3_output;

float angle_rate;
void TIM6_IRQHandler(void)  //TIM6�ж� ������������� PID����
{
	//IMU solvement
	static float increase_angle, angle_sum;
	static float angle_last = 0;
	static int sum_init = 1;
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);  //���TIMx���жϴ�����λ:TIM �ж�Դ 
		
		//��ȡ����������ֵ
		Encoder_1 = (short)TIM2->CNT;
		Encoder_2 = (short)TIM3->CNT;
		Encoder_3 = (short)TIM4->CNT;
		Encoder_4 = (short)TIM5->CNT;
		
		TIM2->CNT = 0;	
		TIM3->CNT = 0;
		TIM4->CNT = 0;
		TIM5->CNT = 0;
		
		//���Ƶ��̵��ٶȣ�X�ٶȣ�Y�ٶȣ�W�ٶȽ��㣬�����V1_cal, V2_cal, V3_cal 
		MovingSpeed(SBUS_Ch[0]*3, SBUS_Ch[1]*3, (float)SBUS_Ch[3]*-0.18);
		
		//ң��Ҫ���
		if(V1_cal != 0)
		{
			V1_output = speed_pid(&V1_pid, V1_cal, Encoder_4);
		}
		else
		{
			V1_output = 0;
			V1_pid_init();
		}
		if(V3_cal != 0)
		{
			V2_output = speed_pid(&V2_pid, V3_cal, Encoder_2);
		}
		else
		{
			V2_output = 0;
			V2_pid_init();
		}
		if(V2_cal != 0)
		{
			V3_output = speed_pid(&V3_pid, V2_cal, Encoder_3);
		}
		else
		{
			V3_output = 0;
			V3_pid_init();
		}
		//���Ƶ��̵��ٶȣ�X�ٶȣ�Y�ٶȣ�W�ٶ�
		Motor_go(-V1_output, -V2_output, -V3_output);

		//������
		if(sum_init == 1)
		{
			angle_last = angle_z;
			sum_init = 0;
		}
		increase_angle = angle_z - angle_last;
		if(increase_angle >= 300)
		{
			increase_angle = increase_angle - 360;
		}
		else if(increase_angle <= -300)
		{
			increase_angle = increase_angle + 360;
		}
		else
		{
			increase_angle = increase_angle;
		}
		angle_sum = angle_sum + increase_angle;
		angle_rate = angle_sum / 360;
		angle_last = angle_z;
	}
}

void V1_pid_init(void)
{
	V1_pid.error_intergrate = 0;
	V1_pid.error_last = 0;
	V1_pid.error_next = 0;
	V1_pid.error_now = 0;
	V1_pid.error_sum = 0;
	V1_pid.increment_speed = 0;
	V1_pid.target_speed = 0;
	V1_pid.error_delta = 0;
	
	V1_pid.actual_speed = 0;
	V1_pid.current_speed = 0;
}

void V2_pid_init(void)
{
	V2_pid.error_intergrate = 0;
	V2_pid.error_last = 0;
	V2_pid.error_next = 0;
	V2_pid.error_now = 0;
	V2_pid.error_sum = 0;
	V2_pid.increment_speed = 0;
	V2_pid.target_speed = 0;
	V2_pid.error_delta = 0;
	
	V2_pid.actual_speed = 0;
	V2_pid.current_speed = 0;
}

void V3_pid_init(void)
{
	V3_pid.error_intergrate = 0;
	V3_pid.error_last = 0;
	V3_pid.error_next = 0;
	V3_pid.error_now = 0;
	V3_pid.error_sum = 0;
	V3_pid.increment_speed = 0;
	V3_pid.target_speed = 0;
	V3_pid.error_delta = 0;
	
	V3_pid.actual_speed = 0;
	V3_pid.current_speed = 0;
}
