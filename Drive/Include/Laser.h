#ifndef __LASER_H
#define __LASER_H
#include "stdio.h"	
#include "stdAfx.h"

void uart4_init(u32 bound);
void FindPosi(void);
void ResetArray(char array[]);

extern int dis_1, dis_2, dis_3, dis_4;

#endif


