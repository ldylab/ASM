#include "stdAfx.h"

//ģ�����������趨
#define IS_Kp 1
#define IS_Ki 2
#define IS_Kd 3
 
#define NL   -3
#define NM	 -2
#define NS	 -1
#define ZE	 0
#define PS	 1
#define PM	 2
#define PL	 3

//����APP�ĵ�������
extern double K_P_E, K_I_E, K_D_E;

//ģ�������
static const float fuzzyRuleKp[7][7]={
	PL,	PL,	PM,	PM,	PS,	PS,	ZE,
	PL,	PL,	PM,	PM,	PS,	ZE,	ZE,
	PM,	PM,	PM,	PS,	ZE,	NS,	NM,
	PM,	PS,	PS,	ZE,	NS,	NM,	NM,
	PS,	PS,	ZE,	NS,	NS,	NM,	NM,
	ZE,	ZE,	NS,	NM,	NM,	NM,	NL,
	ZE,	NS,	NS,	NM,	NM,	NL,	NL
};
 
static const float fuzzyRuleKi[7][7]={
	NL,	NL,	NL,	NM,	NM,	ZE,	ZE,
	NL,	NL,	NM,	NM,	NS,	ZE,	ZE,
	NM,	NM,	NS,	NS,	ZE,	PS,	PS,
	NM,	NS,	NS,	ZE,	PS,	PS,	PM,
	NS,	NS,	ZE,	PS,	PS,	PM,	PM,
	ZE,	ZE,	PS,	PM,	PM,	PL,	PL,
	ZE,	ZE,	PS,	PM,	PL,	PL,	PL
};
 
static const float fuzzyRuleKd[7][7]={
	PS,	PS,	ZE,	ZE,	ZE,	PL,	PL,
	NS,	NS,	NS,	NS,	ZE,	NS,	PM,
	NL,	NL,	NM,	NS,	ZE,	PS,	PM,
	NL,	NM,	NM,	NS,	ZE,	PS,	PM,
	NL,	NM,	NS,	NS,	ZE,	PS,	PS,
	NM,	NS,	NS,	NS,	ZE,	PS,	PS,
	PS,	ZE,	ZE,	ZE,	ZE,	PL,	PL
};

//ģ��PID�Ĳ�������
PID fuzzy(float e,float ec)
{
     float etemp,ectemp;
     float eLefttemp,ecLefttemp;
     float eRighttemp ,ecRighttemp;
 
     int eLeftIndex,ecLeftIndex;
     int eRightIndex,ecRightIndex;
     PID      fuzzy_PID;
     etemp = e > 3.0 ? 0.0 : (e < - 3.0 ? 0.0 : (e >= 0.0 ? (e >= 2.0 ? 2.5: (e >= 1.0 ? 1.5 : 0.5)) : (e >= -1.0 ? -0.5 : (e >= -2.0 ? -1.5 : (e >= -3.0 ? -2.5 : 0.0) ))));
 
     //һ�����Ǻ�����ģ����
     eLeftIndex = (int)e;
     eRightIndex = eLeftIndex;
     eLeftIndex = (int)((etemp-0.5) + 3);        //[-3,3] -> [0,6]
     eRightIndex = (int)((etemp+0.5) + 3);
 
     eLefttemp =etemp == 0.0 ? 0.0:((etemp+0.5)-e);
     eRighttemp=etemp == 0.0 ? 0.0:( e-(etemp-0.5));
 
     ectemp = ec > 3.0 ? 0.0 : (ec < - 3.0 ? 0.0 : (ec >= 0.0 ? (ec >= 2.0 ? 2.5: (ec >= 1.0 ? 1.5 : 0.5)) : (ec >= -1.0 ? -0.5 : (ec >= -2.0 ? -1.5 : (ec >= -3.0 ? -2.5 : 0.0) ))));
 
     ecLeftIndex = (int)((ectemp-0.5) + 3);        //[-3,3] -> [0,6]
     ecRightIndex = (int)((ectemp+0.5) + 3);
 
     ecLefttemp =ectemp == 0.0 ? 0.0:((ectemp+0.5)-ec);
     ecRighttemp=ectemp == 0.0 ? 0.0:( ec-(ectemp-0.5));
 
/*************************************��ģ��*************************************/
//��ģ����������趨
	 fuzzy_PID.Kp = (eLefttemp * ecLefttemp *  fuzzyRuleKp[eLeftIndex][ecLeftIndex]                   
                    + eLefttemp * ecRighttemp * fuzzyRuleKp[eLeftIndex][ecRightIndex]
                    + eRighttemp * ecLefttemp * fuzzyRuleKp[eRightIndex][ecLeftIndex]
                    + eRighttemp * ecRighttemp * fuzzyRuleKp[eRightIndex][ecRightIndex]);
 
	 fuzzy_PID.Ki = (eLefttemp * ecLefttemp * fuzzyRuleKi[eLeftIndex][ecLeftIndex]
                    + eLefttemp * ecRighttemp * fuzzyRuleKi[eLeftIndex][ecRightIndex]
                    + eRighttemp * ecLefttemp * fuzzyRuleKi[eRightIndex][ecLeftIndex]
                    + eRighttemp * ecRighttemp * fuzzyRuleKi[eRightIndex][ecRightIndex]);
 
	 fuzzy_PID.Kd = (eLefttemp * ecLefttemp *    fuzzyRuleKd[eLeftIndex][ecLeftIndex]
                    + eLefttemp * ecRighttemp * fuzzyRuleKd[eLeftIndex][ecRightIndex]
                    + eRighttemp * ecLefttemp * fuzzyRuleKd[eRightIndex][ecLeftIndex]
                    + eRighttemp * ecRighttemp * fuzzyRuleKd[eRightIndex][ecRightIndex]);
                    
     return fuzzy_PID;
}

//λ��ʽPIDFuzzy����
float posi_pid(PID *pid, float target, float current)
{
	PID OUT = {0, 0, 0}; //��ʼ��ģ��PID
    
    pid->target_speed = target;
    pid->current_speed = current;
 
		pid->error_now = pid->target_speed - pid->current_speed;             //Ŀ��ֵ - ʵ��ֵ = ���ֵ
		pid->error_delta = pid->error_now - pid->error_last;            //���仯�� ����Pd�ĵ���
		pid->error_sum += pid->error_now;                 //���ֵ���ۼ�
		pid->error_last = pid->error_now;                //��һ�ε����ֵ
	//OUT = fuzzy(pid->error_now, pid->error_delta);      //ģ�����Ƶ���  kp��ki��kd
    
    //����APP��������
    //pid->Kp = K_P_E;
    //pid->Kd = K_D_E;
    //pid->Ki = K_I_E;
    
    //�������Ҫiֵ����ģ��PID��iֵҲ������
    if(pid->Ki == 0)
    {
        pid->set_point = (pid->Kp+OUT.Kp)*pid->error_now + (pid->Kd+OUT.Kd)*pid->error_delta;
    }
    else if(pid->Ki != 0)
    {
        pid->set_point = (pid->Kp+OUT.Kp)*pid->error_now + (pid->Kd+OUT.Kd)*pid->error_delta + (pid->Ki + OUT.Ki)*pid->error_sum;
    }
    
    //�������ֵ���޶�
    if(pid->set_point > pid->output_maximun)
    {
        pid->set_point = pid->output_maximun;
    }
    else if(pid->set_point < pid->output_minimun)
    {
        pid->set_point = pid->output_minimun;
    }
	return pid->set_point;
}


//����ʽPIDFuzzy����
float speed_pid(PID *pid, float target_speed, float current_speed)
{   
    //��ò���
		PID OUT = {0, 0, 0};
    pid->target_speed = target_speed;
    pid->current_speed = current_speed;
    pid->maximun = target_speed + 5;
    pid->minimun = target_speed - 5;
    //������Ϊ�˼����KP, KI, KD�Ķ�Ӧ�Ĵ�С
		pid->error_now = pid->target_speed - pid->current_speed;             //Ŀ��ֵ - ʵ��ֵ = ���ֵ(e)
		pid->error_delta = pid->error_now - pid->error_next;            //���仯��
    OUT = fuzzy(pid->error_now, pid->error_delta);      //ģ�����Ƶ���  kp��ki��kd
    pid->error_intergrate = 0;
    
    //�����ֱ���PID����
    if(pid->current_speed > pid->maximun)
    {
        if(pid->error_now <= 0)
        {
            pid->error_intergrate = pid->error_now;
        }
    }
    else if(pid->current_speed < pid->minimun)
    {
        if(pid->error_now >= 0)
        {
            pid->error_intergrate = pid->error_now;
        }
    }
    else
    {
        pid->error_intergrate = pid->error_now;
    }
   
    //���ַ���
    if(MyAbs(pid->error_now) <= MyAbs(target_speed - 20))
    {
        pid->increment_speed = (pid->Kp+OUT.Kp)*(pid->error_now - pid->error_next) + (pid->Ki+OUT.Ki)*pid->error_intergrate+\
        (pid->Kd+OUT.Kd)*(pid->error_now - 2*pid->error_next + pid->error_last);//�������㹫ʽ  
    }
    else
    {
        pid->increment_speed = (pid->Kp+OUT.Kp)*(pid->error_now - pid->error_next) +\
        (pid->Kd+OUT.Kd)*(pid->error_now - 2*pid->error_next + pid->error_last);//�������㹫ʽ 
				pid->error_intergrate = 0;
			
    }
    
		pid->actual_speed += pid->increment_speed;                 //���ֵ���ۼ�
    pid->error_last = pid->error_next;//��һ�ε���  
    pid->error_next = pid->error_now;

    return pid->actual_speed; 
}

//����ľ���ֵ����
float MyAbs(float input_num)
{
    if(input_num <= 0)
    {
        return -input_num;
    }
    else
    {
        return input_num;
    }
}

//���룺������Ҫ���Ƶ�PID�Ľṹ�壬����һ�α仯�Ĳ���ֵ������Ŀǰ���ٶ�ֵ����������Ҫ��Ŀ���ٶ�ֵ
//������������ٶ�ֵ��ʵ��ƽ���仯�����е�kFactor����ƽ���仯�ķ���
float StepInProcessing(PID *vPID,float step, int realspeed, int targetspeed)
{
    float speedoutput;
    float speed_differ = targetspeed - realspeed;
    float kFactor=0.0;
    
    if(fabs(speed_differ) <= step)
    {
        speedoutput = targetspeed;
    }
    
    else
    {
        if(speed_differ > 0)
        {
            kFactor = 1.0;
        }
        else if(speed_differ < 0)
        {
            kFactor = -1.0;
        }
        else
        {
            kFactor=0.0;
        }
        speedoutput = realspeed + kFactor*step;
    }
    
    return speedoutput;
}
