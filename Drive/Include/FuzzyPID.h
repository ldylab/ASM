#ifndef __FUZZYPID_H
#define __FUZZYPID_H
#include "stdAfx.h"

//PID fuzzy(float e,float ec);
//PID�����趨�����ṹ��
typedef struct{
	float Kp;
	float Ki;
	float Kd;
    float target_speed;
    float set_point;
    float current_speed;
    int error_last;
    int error_next;
    int error_intergrate;
    int actual_speed;
    float error_now;
    float error_delta;
    float error_sum;
    float increment_speed; //����������ٶ�
    
    //���뿹���ֱ���
    float maximun;
    float minimun;
    
    //���ֵ�����С���޶�
    float output_maximun;
    float output_minimun;
    
    //������ת��һ���Ƕ�
    float angle_now;
    float angle_target;
    
}PID;

float speed_pid(PID *pid, float target_speed, float current_speed);
float MyAbs(float input_num);
float StepInProcessing(PID *vPID,float step, int realspeed, int targetspeed);
float posi_pid(PID *pid, float target, float current);
void PosiPID_Init(void);
#endif
