/**
  ******************************************************************************
  * @file   	ServoUtil.h
  * @author  	Quyen Truong
  * @version 	V1.0
  * @Project	Spider Bot
  * @email		quyentruong.scorpion@gmail.com
  ******************************************************************************
  */

#ifndef _SERVO_UTIL_H_
#define _SERVO_UTIL_H_

#include <stdint.h>

#define NUMBERS_SERVO			16
#define LEG_NUMBERS				04
#define LEG_JOINT					04
#define NUMBER_DOT				8
#define PULSE_BASE				500
#define PULSE_MAX					2500
#define BLU								01.0		/* Pulse */
#define MAX_ROTATE_SPEED	3600.0 	/* 0.1 x degree/s */
#define DEG_PER_PULSE			0.09		/* Degree */

typedef enum
{
	LEG_ST = 0,
	LEG_ND,
	LEG_RD,
	LEG_TH,
	LEG_ALL,
} Leg_Order;

typedef struct Spider_Leg
{
	float x[NUMBER_DOT];
	float y[NUMBER_DOT];
	float z[NUMBER_DOT];
	int8_t dir[LEG_JOINT];
	uint16_t angle[LEG_JOINT];
} Spider_Leg_t;

typedef struct Spider_Body
{
	float x[LEG_NUMBERS];
	float y[LEG_NUMBERS];
	float z[LEG_NUMBERS];
} Spider_Body_t;

extern volatile Spider_Leg_t legpos[LEG_NUMBERS];

void SpiderInit(void);
void SpiderTouch(void);
//void Move16Servo(uint16_t *data, uint16_t speed);
//void MoveLegsServo(uint16_t *data, uint16_t speed, Leg_Order leg);

#endif
