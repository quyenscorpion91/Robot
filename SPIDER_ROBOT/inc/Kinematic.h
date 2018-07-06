/**
  ******************************************************************************
  * @file   	Kinematic.h
  * @author  	Quyen Truong
  * @version 	V1.0
  * @Project	Spider Bot
  * @email		quyentruong.scorpion@gmail.com
  ******************************************************************************
  */

#ifndef _KINEMATIC_H_
#define _KINEMATIC_H_

#include <math.h>
#include <stdint.h>
#include "ServoUtil.h"

#define	PI							3.14
#define DEG_TO_RAD			PI/180
#define RAD_TO_DEG			180/PI

#define INVALID_ANGLE		5000

/* Mechanical Leg Parameters */
#define FIRST_JOINT_L1		20.0	//mm
#define FIRST_JOINT_L2		35.0	//mm
#define FIRST_JOINT_L3		-20.0	//mm FIRST_JOINT_L3 = 20 * sin(-90) = -20
#define FIRST_JOINT_ANG1	0
#define FIRST_JOINT_ANG2	(-PI/2)
#define FIRST_JOINT_ANG3	(-PI/2)
#define SECOND_JOINT_L1		54.0 //mm
#define SECOND_JOINT_L2		35.0 //mm
#define SECOND_JOINT_ANG1	(-PI/2)
#define SECOND_JOINT_ANG2	(PI/4)
#define THIRD_JOINT_L			65.0 //mm
#define THIRD_JOINT_ANG		(-PI)
#define FOURTH_JOINT_L		95.0 //mm
#define FOURTH_JOINT_ANG	(-PI/2)
/* Mechanical Body Parameter */
#define CEN_TO_LEG				(108/2) //mm
#define LEG1_ANGLE				(PI/4)

typedef enum
{
	BODY_ROLL = 0,
	BODY_PITCH,
	BODY_YAW,
} BD_Rotate;

bool ForwardKinematic4Dof(Leg_Order leg);
uint16_t *InverseKinematic4Dof(Leg_Order leg, float x, float y, float z);
uint16_t *BodyTransfer(float delx, float dely, float delz);
uint16_t *BodyRotateProcess(float roll, float pitch, float yaw, BD_Rotate type);
uint16_t *LegMove(float x, float y, float z, Leg_Order leg);
void ForwardAllLeg(void);
void testIK(void);

#endif

