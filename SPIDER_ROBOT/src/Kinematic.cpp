/**
  ******************************************************************************
  * @file   	Kinematic.cpp
  * @author  	Quyen Truong
  * @version 	V1.0
  * @Project	Spider Bot
  * @email		quyentruong.scorpion@gmail.com
  ******************************************************************************
  */

#include <string.h>
#include "Kinematic.h"
#include <stdlib.h>
#include <stdio.h>

#define MAX_IK_LOOP			10000
#define DELTATHETA			(DEG_PER_PULSE) * (DEG_TO_RAD)
#define MAX_SERVO_PULSE 	2000

#define Z_LEG_START			-100.0
#define Z_LEG_MIN			-180.0
#define Z_LEG_MAX			-60.0
#define BODY_YAW_START		90
#define BODY_YAW_MIN		45
#define BODY_YAW_MAX		135

const float maxangle[LEG_JOINT] = {PI, PI, PI, PI};
const float minangle[LEG_JOINT] = {0, 0, 0.523, 0};
const int8_t revert[NUMBERS_SERVO] = {0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0};
const float legrotatemap[LEG_NUMBERS] = {(7*PI)/4, (3*PI)/4, (3*PI)/4, (7*PI)/4};
const int8_t legyaxismap[LEG_NUMBERS] = {-1, 1, -1, 1};

static float *CalForwardKinematic(float radangle[4]);
static uint16_t FloatToPulse(float radangle);
static uint16_t *BodyYawProcess(float yaw);
static uint16_t *BodyRollProcess(float roll);
static uint16_t *BodyPitchProcess(float pitch);

bool ForwardKinematic4Dof(Leg_Order leg)
{
	float anglerad[LEG_JOINT];
	float module;
	uint16_t temp;
	uint8_t pos;
	if(LEG_ALL == leg) return false;

	/* First Joint */
	pos = leg * LEG_JOINT + 0;
	if(revert[pos])
		temp = MAX_SERVO_PULSE - (legpos[leg].angle[0] - PULSE_BASE);
	else
		temp = legpos[leg].angle[0] - PULSE_BASE;
	anglerad[0] = (float)temp * DEG_PER_PULSE * DEG_TO_RAD;
	legpos[leg].x[0] = 0;
	legpos[leg].y[0] = 0;
	legpos[leg].z[0] = 0;

	legpos[leg].x[1] = FIRST_JOINT_L1 * cos(anglerad[0]);
	legpos[leg].y[1] = FIRST_JOINT_L1 * sin(anglerad[0]);
	legpos[leg].z[1] = 0;

	legpos[leg].x[2] = legpos[leg].x[1] + FIRST_JOINT_L2 * cos(anglerad[0] + FIRST_JOINT_ANG2);
	legpos[leg].y[2] = legpos[leg].y[1] + FIRST_JOINT_L2 * sin(anglerad[0] + FIRST_JOINT_ANG2);
	legpos[leg].z[2] = 0;

	legpos[leg].x[3] = legpos[leg].x[2];
	legpos[leg].y[3] = legpos[leg].y[2];
	legpos[leg].z[3] = FIRST_JOINT_L3;

	/* Second Joint */
	pos = leg * LEG_JOINT + 1;
	if(revert[pos])
		temp = MAX_SERVO_PULSE - (legpos[leg].angle[1] - PULSE_BASE);
	else
		temp = legpos[leg].angle[1] - PULSE_BASE;
	anglerad[1] = (float)temp * DEG_PER_PULSE * DEG_TO_RAD;
	module = SECOND_JOINT_L1 * cos(anglerad[1] + SECOND_JOINT_ANG1);
	legpos[leg].x[4] = legpos[leg].x[3] + module * cos(anglerad[0] + FIRST_JOINT_ANG2);
	legpos[leg].y[4] = legpos[leg].y[3] + module * sin(anglerad[0] + FIRST_JOINT_ANG2);
	legpos[leg].z[4] = legpos[leg].z[3] + SECOND_JOINT_L1 * sin(anglerad[1] + SECOND_JOINT_ANG1);

	module = SECOND_JOINT_L2 * cos(anglerad[1] + SECOND_JOINT_ANG1 + SECOND_JOINT_ANG2);
	legpos[leg].x[5] = legpos[leg].x[4] + module * cos(anglerad[0] + FIRST_JOINT_ANG2);
	legpos[leg].y[5] = legpos[leg].y[4] + module * sin(anglerad[0] + FIRST_JOINT_ANG2);
	legpos[leg].z[5] = legpos[leg].z[4] + SECOND_JOINT_L2 * sin(anglerad[1] + SECOND_JOINT_ANG1 + SECOND_JOINT_ANG2);

	/* Third Joint */
	pos = leg * LEG_JOINT + 2;
	if(revert[pos])
		temp = MAX_SERVO_PULSE - (legpos[leg].angle[2] - PULSE_BASE);
	else
		temp = legpos[leg].angle[2] - PULSE_BASE;
	anglerad[2] = (float)temp * DEG_PER_PULSE * DEG_TO_RAD;
	module = THIRD_JOINT_L * cos(anglerad[1] + SECOND_JOINT_ANG1 + SECOND_JOINT_ANG2 + anglerad[2] + THIRD_JOINT_ANG);
	legpos[leg].x[6] = legpos[leg].x[5] + module * cos(anglerad[0] + FIRST_JOINT_ANG2);
	legpos[leg].y[6] = legpos[leg].y[5] + module * sin(anglerad[0] + FIRST_JOINT_ANG2);
	legpos[leg].z[6] = legpos[leg].z[5] + THIRD_JOINT_L * sin(anglerad[1] + SECOND_JOINT_ANG1 + SECOND_JOINT_ANG2 + anglerad[2] + THIRD_JOINT_ANG);

	/* Fourth Joint */
	pos = leg * LEG_JOINT + 3;
	if(revert[pos])
		temp = MAX_SERVO_PULSE - (legpos[leg].angle[3] - PULSE_BASE);
	else
		temp = legpos[leg].angle[3] - PULSE_BASE;
	anglerad[3] = (float)temp * DEG_PER_PULSE * DEG_TO_RAD;
	module = FOURTH_JOINT_L * cos(anglerad[1] + SECOND_JOINT_ANG1 + SECOND_JOINT_ANG2 + anglerad[2] + THIRD_JOINT_ANG +  anglerad[3] + FOURTH_JOINT_ANG);
	legpos[leg].x[7] = legpos[leg].x[6] + module * cos(anglerad[0] + FIRST_JOINT_ANG2);
	legpos[leg].y[7] = legpos[leg].y[6] + module * sin(anglerad[0] + FIRST_JOINT_ANG2);
	legpos[leg].z[7] = legpos[leg].z[6] + FOURTH_JOINT_L * sin(anglerad[1] + SECOND_JOINT_ANG1 + SECOND_JOINT_ANG2 + anglerad[2] + THIRD_JOINT_ANG + anglerad[3] + FOURTH_JOINT_ANG);

	return true;
}

void ForwardAllLeg(void)
{
	uint16_t i;
	for(i = 0; i < LEG_NUMBERS; i++)
		ForwardKinematic4Dof((Leg_Order)i);
}

float *CalForwardKinematic(float radangle[4])
{
	static float ret[3] = {0, 0, 0};

	float module;
	float x[NUMBER_DOT], y[NUMBER_DOT], z[NUMBER_DOT];

	/* First Joint */
	x[0] = 0;
	y[0] = 0;
	z[0] = 0;

	x[1] = FIRST_JOINT_L1 * cos(radangle[0]);
	y[1] = FIRST_JOINT_L1 * sin(radangle[0]);
	z[1] = 0;

	x[2] = x[1] + FIRST_JOINT_L2 * cos(radangle[0] + FIRST_JOINT_ANG2);
	y[2] = y[1] + FIRST_JOINT_L2 * sin(radangle[0] + FIRST_JOINT_ANG2);
	z[2] = 0;

	x[3] = x[2];
	y[3] = y[2];
	z[3] = FIRST_JOINT_L3;

	/* Second Joint */
	module = SECOND_JOINT_L1 * cos(radangle[1] + SECOND_JOINT_ANG1);
	x[4] = x[3] + module * cos(radangle[0] + FIRST_JOINT_ANG2);
	y[4] = y[3] + module * sin(radangle[0] + FIRST_JOINT_ANG2);
	z[4] = z[3] + SECOND_JOINT_L1 * sin(radangle[1] + SECOND_JOINT_ANG1);

	module = SECOND_JOINT_L2 * cos(radangle[1] + SECOND_JOINT_ANG1 + SECOND_JOINT_ANG2);
	x[5] = x[4] + module * cos(radangle[0] + FIRST_JOINT_ANG2);
	y[5] = y[4] + module * sin(radangle[0] + FIRST_JOINT_ANG2);
	z[5] = z[4] + SECOND_JOINT_L2 * sin(radangle[1] + SECOND_JOINT_ANG1 + SECOND_JOINT_ANG2);

	/* Third Joint */
	module = THIRD_JOINT_L * cos(radangle[1] + SECOND_JOINT_ANG1 + SECOND_JOINT_ANG2 + radangle[2] + THIRD_JOINT_ANG);
	x[6] = x[5] + module * cos(radangle[0] + FIRST_JOINT_ANG2);
	y[6] = y[5] + module * sin(radangle[0] + FIRST_JOINT_ANG2);
	z[6] = z[5] + THIRD_JOINT_L * sin(radangle[1] + SECOND_JOINT_ANG1 + SECOND_JOINT_ANG2 + radangle[2] + THIRD_JOINT_ANG);

	/* Fourth Joint */
	module = FOURTH_JOINT_L * cos(radangle[1] + SECOND_JOINT_ANG1 + SECOND_JOINT_ANG2 + radangle[2] + THIRD_JOINT_ANG +  radangle[3] + FOURTH_JOINT_ANG);
	x[7] = x[6] + module * cos(radangle[0] + FIRST_JOINT_ANG2);
	y[7] = y[6] + module * sin(radangle[0] + FIRST_JOINT_ANG2);
	z[7] = z[6] + FOURTH_JOINT_L * sin(radangle[1] + SECOND_JOINT_ANG1 + SECOND_JOINT_ANG2 + radangle[2] + THIRD_JOINT_ANG + radangle[3] + FOURTH_JOINT_ANG);

	/* Revese x and y for later use */
	ret[0] = y[7]; // <== reverse
	ret[1] = x[7]; // <== reverse
	ret[2] = z[7];

	return ret;
}

uint16_t *InverseKinematic4Dof(Leg_Order leg, float x, float y, float z)
{
	uint16_t i, j, pos;
	float IKangle[4] = {0,0,0,0};
	static uint16_t ret[4] = {INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE}; // init return invalid numbers
	float	module;
	float totalerr;
	float error[2];
	float curangle[4];
	float curcoor[3];
	float precoor[3];
	float target[2];
	float rate[3][2];
	float temp[2];
	float sign[2];
	float change;
	float *p;
	float tempang;
	uint16_t t;

	/* Calculate the first angle (theta0) */
	module = sqrt(x * x + y * y);
	tempang = acos(x / module);
	if(y < 0) tempang = -tempang;
	IKangle[0] = acos(x / module) + acos(FIRST_JOINT_L1 / module);

	if(IKangle[0] > maxangle[0] || IKangle[0] < minangle[0])
		return ret; /* Cannot reach to the target */

	/* Calculate others angle */
	/* Initialize parameters */
	curangle[0] = IKangle[0];
	target[0] = x;
	target[1] = z;
	for(i = 0; i < (LEG_JOINT - 1); i++)
	{
		pos = leg * LEG_JOINT + i + 1;
		if(revert[pos])
			t = MAX_SERVO_PULSE - (legpos[leg].angle[i+1] - PULSE_BASE);
		else
			t = legpos[leg].angle[i + 1] - PULSE_BASE;
		curangle[i + 1] = t * DEG_PER_PULSE * DEG_TO_RAD;
	}

	for(i = 0; i < MAX_IK_LOOP; i++)
	{
		p = CalForwardKinematic(curangle);
		memcpy(curcoor, p, (3 * sizeof(float)));

		totalerr = 0;
		for(j = 0; j < 2; j++)
		{
			error[j] = target[j] - curcoor[j + 1];
			totalerr += error[j] * error[j];
			if(error[j] < 0)
				sign[j] = -1;
			else
				sign[j] = 1;
		}
		totalerr = totalerr / 2.0;
		if(totalerr <= 0.01)
			break;

		memcpy(precoor, curcoor, (3 * sizeof(float)));
		for(j = 0; j < (LEG_JOINT - 1); j++)
		{
			curangle[j + 1] += DELTATHETA;
			p = CalForwardKinematic(curangle);
			memcpy(curcoor, p, (3 * sizeof(float)));
			/* x diff */
			temp[0] = curcoor[1] - precoor[1];
			/* z diff */
			temp[1] = curcoor[2] - precoor[2];
			/* rate x */
			rate[j][0] = temp[0] / (temp[0] + temp[1]);
			/* rate z */
			rate[j][1] = temp[1] / (temp[0] + temp[1]);
			curangle[j + 1] -= DELTATHETA;
		}

		for(j = 0; j < (LEG_JOINT - 1); j++)
		{
			change = (sign[0] * DELTATHETA * rate[j][0]) + (sign[1] * DELTATHETA * rate[j][1]);
			curangle[j + 1] += change;
			curangle[j + 1] = fmodf(curangle[j + 1] , (2 * PI));
			if(curangle[j + 1] > maxangle[j + 1])
				curangle[j + 1] = maxangle[j + 1];
			else if(curangle[j + 1] < minangle[j + 1])
				curangle[j + 1] = minangle[j + 1];
		}
	}
	/* Finish calculate for IK */
	for(j = 0; j < LEG_JOINT; j++)
	{
		pos = leg * LEG_JOINT + j;
		ret[j] = FloatToPulse(curangle[j]);
		if(revert[pos])
			ret[j] = MAX_SERVO_PULSE - ret[j];
	}

	return ret;
}

uint16_t FloatToPulse(float radangle)
{
	float temp, mod;
	uint16_t ret;

	temp = (radangle * RAD_TO_DEG) / DEG_PER_PULSE;
	mod = fmodf(temp, 1.0);
	ret = temp;
	if(mod > 0.5)
		ret += 1;

	return ret;
}

uint16_t *BodyRotateProcess(float roll, float pitch, float yaw, BD_Rotate type)
{
	static uint16_t ret[16] = {INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE};
	switch(type)
	{
	case BODY_ROLL:
		return BodyRollProcess(roll);
		break;
	case BODY_PITCH:
		return BodyPitchProcess(pitch);
		break;
	case BODY_YAW:
		return BodyYawProcess(yaw);
		break;
	}

	return ret;
}

uint16_t *BodyTransfer(float delx, float dely, float delz)
{
	uint8_t i, k, pos;
	uint16_t *angle;
	float x, y, z, mxy, legdelx, legdely, temp, alpha;
	static uint16_t ret[16] = {INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE};

	if((delx != 0) || (dely != 0))
	{
		mxy = sqrt(delx * delx + dely * dely);
		alpha = acos(delx / mxy);
		if(dely < 0) alpha = -alpha;
	}
	for(i = 0; i < LEG_NUMBERS; i++)
	{
		legdelx = legdely = 0;
		if((delx != 0) || (dely != 0))
		{
			/* Map x, y body coordinate to x, y leg cordinate */
			alpha = alpha * legyaxismap[i];
			legdelx = mxy * cos(alpha + legrotatemap[i]);
			legdely = mxy * sin(alpha + legrotatemap[i]);
			alpha = alpha * legyaxismap[i];
		}

		/* calculate x */
		x = legpos[i].x[7] - legdelx;
		/* calculate y */
		y = legpos[i].y[7] - legdely;
		temp = sqrt(x * x + y * y);
		printf("temp: %f\n", temp);
		if(temp < 55 || temp > 150) return ret;

		z = legpos[i].z[7] - delz;
		printf("z: %f\n", z);
		if(z > Z_LEG_MAX)
			z = Z_LEG_MAX;
		else if(z < Z_LEG_MIN)
			z = Z_LEG_MIN;
		angle = InverseKinematic4Dof((Leg_Order)(i), x, y, z);
		for(k = 0; k < LEG_JOINT; k++)
		{
			pos = i * LEG_JOINT + k;
			legpos[i].angle[k] = angle[k] + PULSE_BASE;
			ret[pos] = legpos[i].angle[k];
		}
	}

	return ret;
}

uint16_t *BodyYawProcess(float yaw)
{
	static uint16_t ret[16] = {INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE};
	uint8_t i, j, pos;
	float radangle;

	if((yaw < BODY_YAW_MIN) || (yaw > BODY_YAW_MAX)) return ret;

	/* init return value */
	for(i = 0; i < LEG_NUMBERS; i++)
	{
		for(j = 0; j < LEG_JOINT; j++)
		{
			pos = i * LEG_JOINT + j;
			ret[pos] = legpos[i].angle[j];
		}
	}
	radangle = yaw * DEG_TO_RAD;
	for(i = 0; i < LEG_NUMBERS; i++)
	{
		pos = i * LEG_JOINT;
		ret[pos] = FloatToPulse(radangle);
//		if(revert[pos])
//			ret[pos] = MAX_SERVO_PULSE - ret[pos];
		ret[pos] += PULSE_BASE;
		legpos[i].angle[0] = ret[pos];
	}

	return ret;
}

uint16_t *BodyRollProcess(float roll)
{
	static uint16_t ret[16] = {INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE};
	uint8_t i, j, pos;
	float deltaz, radangle, z;
	uint16_t *angle;

	/* init return value */
	for(i = 0; i < LEG_NUMBERS; i++)
	{
		for(j = 0; j < LEG_JOINT; j++)
		{
			pos = i * LEG_JOINT + j;
			ret[pos] = legpos[i].angle[j];
		}
	}

	radangle = roll * DEG_TO_RAD;
	deltaz = CEN_TO_LEG * sin(radangle);

	/* Calculate for leg 1, 2*/
	for(i = 0; i < (LEG_NUMBERS - 2); i++)
	{
		z = legpos[i].z[7] - deltaz;
		if(z > Z_LEG_MAX)
			z = Z_LEG_MAX;
		else if(z < Z_LEG_MIN)
			z = Z_LEG_MIN;
		angle = InverseKinematic4Dof((Leg_Order)(i), legpos[i].x[7], legpos[i].y[7], z);
		for(j = 0; j < LEG_JOINT; j++)
		{
			pos = i * LEG_JOINT + j;
			legpos[i].angle[j] = angle[j] + PULSE_BASE;
			ret[pos] = legpos[i].angle[j];
		}
	}

	/* Calculate for leg 3, 4 */
	for(i = 2; i < LEG_NUMBERS; i++)
	{
		z = legpos[i].z[7] + deltaz;
		if(z > Z_LEG_MAX)
			z = Z_LEG_MAX;
		else if(z < Z_LEG_MIN)
			z = Z_LEG_MIN;
		angle = InverseKinematic4Dof((Leg_Order)(i), legpos[i].x[7], legpos[i].y[7], z);
		for(j = 0; j < LEG_JOINT; j++)
		{
			pos = i * LEG_JOINT + j;
			legpos[i].angle[j] = angle[j] + PULSE_BASE;
			ret[pos] = legpos[i].angle[j];
		}
	}

	return ret;
}

uint16_t *BodyPitchProcess(float pitch)
{
	static uint16_t ret[16] = {INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE};
	uint8_t i, j, pos;
	float deltaz, radangle, z;
	uint16_t *angle;
	const Leg_Order leg[4] = {LEG_ST, LEG_TH, LEG_ND, LEG_RD};
	Leg_Order l;

	/* init return value */
	for(i = 0; i < LEG_NUMBERS; i++)
	{
		for(j = 0; j < LEG_JOINT; j++)
		{
			pos = i * LEG_JOINT + j;
			ret[pos] = legpos[i].angle[j];
		}
	}

	radangle = pitch * DEG_TO_RAD;
	deltaz = CEN_TO_LEG * sin(radangle);

	/* Calculate for leg 1, 4*/
	for(i = 0; i < (LEG_NUMBERS - 2); i++)
	{
		l = leg[i];
		z = legpos[l].z[7] - deltaz;
		if(z > Z_LEG_MAX)
			z = Z_LEG_MAX;
		else if(z < Z_LEG_MIN)
			z = Z_LEG_MIN;
		angle = InverseKinematic4Dof((Leg_Order)(l), legpos[l].x[7], legpos[l].y[7], z);
		for(j = 0; j < LEG_JOINT; j++)
		{
			pos = l * LEG_JOINT + j;
			legpos[l].angle[j] = angle[j] + PULSE_BASE;
			ret[pos] = legpos[l].angle[j];
		}
	}

	/* Calculate for leg 2, 3 */
	for(i = 2; i < LEG_NUMBERS; i++)
	{
		l = leg[i];
		z = legpos[l].z[7] + deltaz;
		if(z > Z_LEG_MAX)
			z = Z_LEG_MAX;
		else if(z < Z_LEG_MIN)
			z = Z_LEG_MIN;
		angle = InverseKinematic4Dof((Leg_Order)(l), legpos[l].x[7], legpos[l].y[7], z);
		for(j = 0; j < LEG_JOINT; j++)
		{
			pos = l * LEG_JOINT + j;
			legpos[l].angle[j] = angle[j] + PULSE_BASE;
			ret[pos] = legpos[l].angle[j];
		}
	}

	return ret;
}

uint16_t *LegMove(float delx, float dely, float delz, Leg_Order leg)
{
	uint8_t i, k, pos;
	uint16_t *angle;
	float x, y, z, temp;
	static uint16_t ret[16] = {INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE,INVALID_ANGLE};

	for(i = 0; i < LEG_NUMBERS; i++)
	{
		for(k = 0; k < LEG_JOINT; k++)
		{
			pos = i * LEG_JOINT + k;
			ret[pos] = legpos[i].angle[k];
		}
	}
	/* calculate x */
	x = legpos[leg].x[7] + delx;
	/* calculate y */
	y = legpos[leg].y[7] + dely;
	temp = sqrt(x * x + y * y);
	printf("temp: %f\n", temp);
	if(temp < 55 || temp > 150) return ret;

	z = legpos[leg].z[7] + delz;
	if(z > Z_LEG_MAX)
		z = Z_LEG_MAX;
	else if(z < Z_LEG_MIN)
		z = Z_LEG_MIN;
	angle = InverseKinematic4Dof(leg, x, y, z);
	for(k = 0; k < LEG_JOINT; k++)
	{
		pos = leg * LEG_JOINT + k;
		legpos[leg].angle[k] = angle[k] + PULSE_BASE;
		ret[pos] = legpos[leg].angle[k];
	}

	return ret;
}
void testIK(void)
{
	uint16_t i, k;
	uint16_t *angle;

	for(k = 0; k < LEG_NUMBERS; k++)
	{
		angle = InverseKinematic4Dof((Leg_Order)(k), 80.0 , 20.0 , -180.0);
		for(i = 0; i < LEG_JOINT; i++)
		{
			legpos[k].angle[i] = angle[i] + PULSE_BASE;
		}
	}

	for(k = 0; k < LEG_NUMBERS; k++)
	{
		angle = InverseKinematic4Dof((Leg_Order)(k), 80.0 , 20.0 , -60.0);
		for(i = 0; i < LEG_JOINT; i++)
		{
			legpos[k].angle[i] = angle[i] + PULSE_BASE;
		}
	}

	for(k = 0; k < LEG_NUMBERS; k++)
	{
		angle = InverseKinematic4Dof((Leg_Order)(k), 80.0 , 20.0 , -190.0);
		for(i = 0; i < LEG_JOINT; i++)
		{
			legpos[k].angle[i] = angle[i] + PULSE_BASE;
		}
	}
}
