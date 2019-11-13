#include "stdAfx.h"

int V1_cal, V2_cal, V3_cal;

void MovingSpeed(float Vx, float Vy, float angularVell)
{
	static int Max_speed;
	static float Con_Speed_rate;
	int AFA = 60;  // 轮子之间的夹角
	int L = 15;  // 轮子到中心的半径的距离
	int theta = 0;  // 与Y轴的夹角
	V1_cal = (float)(-cos((AFA + theta) / 180.0f * 3.1415926f) * Vx - sin((theta + AFA) / 180.0f * 3.1415926f) * Vy + L * angularVell);

	V2_cal = (float)(cos(theta / 180.0f * 3.1415926f) * Vx + sin(theta / 180.0f * 3.1415926f) * Vy      + L * angularVell);

	V3_cal = (float)(-cos((AFA - theta) / 180.0f * 3.1415926f) * Vx + sin((AFA - theta) / 180.0f * 3.1415926f) * Vy + L * angularVell);
	
	Max_speed = Max_one(Max_one(V1_cal, V2_cal), V3_cal);
	
	if(myabs(Max_speed) >= 3500.0)
	{
			Con_Speed_rate = (3500 / (float)Max_speed);
			if(Con_Speed_rate < 0)
			{
					Con_Speed_rate = -Con_Speed_rate;
			}
	}
	else
	{
			Con_Speed_rate = 1;
	}
	
	V1_cal = V1_cal*Con_Speed_rate;
	V2_cal = V2_cal*Con_Speed_rate;
	V3_cal = V3_cal*Con_Speed_rate;
	
}

int Max_one(int a, int b)
{
    if(myabs(a) >= myabs(b))
    {
        return myabs(a);
    }
    else
    {
        return myabs(b);
    }
}

int myabs(int a)  //取绝对值函数
{
    int temp;
    if(a < 0)  temp = -a;
    else temp = a;
    return temp;
}
