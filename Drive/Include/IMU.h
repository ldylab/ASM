#ifndef __IMU_H
#define __IMU_H
#include "stdio.h"	
#include "stdAfx.h"
#include "stm32f10x.h"

void IMU_Init(u32 bound);
#define USART3_REC_LEN  		200  	//定义最大接收字节数 200
#define EN_USART3_RX 			1		//使能（1）/禁止（0）串口1接收

struct GYROs
{
	float ax;	//加速度
	float ay;
	float az;
	
	float yaw;	//角度
	float roll;
	float pitch;
	
	float yaw_init;	//初始角度
	float roll_init;
	float pitch_init;
	
	float yaw_bias;	//差值
	float roll_bias;
	float pitch_bias;
};

extern float angle_x, angle_y, angle_z;
extern struct GYROs gyro;

#endif
