#ifndef __IMU_H
#define __IMU_H
#include "stdio.h"	
#include "stdAfx.h"
#include "stm32f10x.h"

void IMU_Init(u32 bound);
#define USART3_REC_LEN  		200  	//�����������ֽ��� 200
#define EN_USART3_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

struct GYROs
{
	float ax;	//���ٶ�
	float ay;
	float az;
	
	float yaw;	//�Ƕ�
	float roll;
	float pitch;
	
	float yaw_init;	//��ʼ�Ƕ�
	float roll_init;
	float pitch_init;
	
	float yaw_bias;	//��ֵ
	float roll_bias;
	float pitch_bias;
};

extern float angle_x, angle_y, angle_z;
extern struct GYROs gyro;

#endif
