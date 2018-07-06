/**
  ******************************************************************************
  * @file   	ServoUtil.c
  * @author  	Quyen Truong
  * @version 	V1.0
  * @Project	Spider Bot
  * @email		quyentruong.scorpion@gmail.com
  ******************************************************************************
  */

#include "ServoUtil.h"

volatile Spider_Leg_t legpos[LEG_NUMBERS];

const uint16_t HomePos[NUMBERS_SERVO] = {1500, 2275, 1500, 2500, 1500, 725, 1500, 500, 1500, 2275, 1500, 2500, 1500, 725, 1500, 500};
const uint16_t StartPos[NUMBERS_SERVO] = {1501, 1948, 2165, 1968, 1499, 1052, 835, 1032, 1501, 1948, 2165, 1968, 1499, 1052, 835, 1032};

void SpiderInit(void)
{
	uint8_t i, j, pos;
	for(i = 0; i < LEG_NUMBERS; i++)
	{
		for(j = 0; j < LEG_JOINT; j++)
		{
			legpos[i].x[j] = 0;
			legpos[i].y[j] = 0;
			legpos[i].z[j] = 0;
			pos = i * LEG_JOINT + j;
			legpos[i].angle[j] = HomePos[pos];
			legpos[i].dir[j] = 1;
		}
	}
}

void SpiderTouch(void)
{
	uint8_t i, j, pos;
	for(i = 0; i < LEG_NUMBERS; i++)
	{
		for(j = 0; j < LEG_JOINT; j++)
		{
			pos = i * LEG_JOINT + j;
			legpos[i].angle[j] = StartPos[pos];
		}
	}
}
