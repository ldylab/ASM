#ifndef  __PICONTROL_H
#define  __PICONTROL_H
#include "stm32f10x.h"
extern int V1_output, V2_output, V3_output;
void V1_pid_init(void);
void V2_pid_init(void);
void V3_pid_init(void);

extern float angle_rate;
#endif

