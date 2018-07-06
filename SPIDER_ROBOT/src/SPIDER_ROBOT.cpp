//============================================================================
// Name        : SPIDER_ROBOT.cpp
// Author      : Quyen Truong
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <Kinematic.h>
#include "ComPort.h"
#include <windows.h>

using namespace std;

static void SendToRoBot(char *buff, uint16_t size);
static char *ConvertToCtrlBuff(uint16_t *data, uint16_t size, uint16_t speed);
static void PrintLegCoor(void);
static void RobotBodyMoveTest(float speed);

int main() {
	unsigned int mode;
	float sc, delx, dely, delz;
	uint16_t *data;
	unsigned int leg;
	char *send;
	char start[10] = {0x53, 0x50, 0x44, 0x42, 0x00, 0x00, 0x12, 0x34, 0x56, 0x78};

	SpiderInit();
	SpiderTouch();
	ForwardAllLeg();
	PrintLegCoor();
	while(1)
	{
		cout << "Please input mode: \n";
		scanf("%u", &mode);
		cout << "your input is: " << mode << endl;

		switch(mode)
		{
		case 0:
			cout << "exit programe\n";
			exit(0);
			break;
		case 1:
			cout << "body transfer mode: \n";
			cout << "please input delta x:";
			scanf("%f", &delx);
			cout << "please input delta y:";
			scanf("%f", &dely);
			cout << "please input delta z:";
			scanf("%f", &delz);
			data = BodyTransfer(delx, dely, delz);
			ForwardAllLeg();
			PrintLegCoor();
			send = ConvertToCtrlBuff(data, 16, 300);
			SendToRoBot(send, 44);
			free(send);
			break;
		case 2:
			cout << "body rotate roll: \n";
			cout << "please input roll angle in degree:";
			scanf("%f", &sc);
			data = BodyRotateProcess(sc, 0, 0, BODY_ROLL);
			ForwardAllLeg();
			PrintLegCoor();
			send = ConvertToCtrlBuff(data, 16, 400);
			SendToRoBot(send, 44);
			free(send);
			break;
		case 3:
			cout << "body rotate pitch: \n";
			cout << "please input pitch angle in degree:";
			scanf("%f", &sc);
			data = BodyRotateProcess(0, sc, 0, BODY_PITCH);
			ForwardAllLeg();
			PrintLegCoor();
			send = ConvertToCtrlBuff(data, 16, 400);
			SendToRoBot(send, 44);
			free(send);
			break;
		case 4:
			cout << "body rotate yaw: \n";
			cout << "please input yaw angle in degree:";
			scanf("%f", &sc);
			data = BodyRotateProcess(0, 0, sc, BODY_YAW);
			ForwardAllLeg();
			PrintLegCoor();
			send = ConvertToCtrlBuff(data, 16, 400);
			SendToRoBot(send, 44);
			free(send);
			break;
		case 5:
			cout << "send start to your robot \n";
			SpiderTouch();
			ForwardAllLeg();
			PrintLegCoor();
			SendToRoBot(start, 10);
			break;
		case 6:
			cout << "body move test: \n";
			cout << "please input test speed:";
			scanf("%f", &sc);
			RobotBodyMoveTest(sc);
			break;
		case 7:
			cout << "move leg \n";
			cout << "input your leg number\n";
			scanf("%u", &leg);
			if(leg >= LEG_ALL) break;
			cout << "please input delta x:";
			scanf("%f", &delx);
			cout << "please input delta y:";
			scanf("%f", &dely);
			cout << "please input delta z:";
			scanf("%f", &delz);
			data = LegMove(delx, dely, delz, (Leg_Order)leg);
			ForwardAllLeg();
			PrintLegCoor();
			send = ConvertToCtrlBuff(data, 16, 400);
			SendToRoBot(send, 44);
			free(send);
			break;
		case 8:
			cout << "move serial: \n";
			delx = 0;
			for(int i = 0; i < 20; i++)
			{
				delx += 2;
				data = BodyTransfer(delx, 0, 0);
				ForwardAllLeg();
//				PrintLegCoor();
				send = ConvertToCtrlBuff(data, 16, 500);
				SendToRoBot(send, 44);
			}
			break;

		}
	}
	return 0;
}

void RobotBodyMoveTest(float speed)
{
	uint16_t spd = (uint16_t)speed;
	char start[10] = {0x53, 0x50, 0x44, 0x42, 0x00, 0x00, 0x12, 0x34, 0x56, 0x78};
	uint16_t *data;
	char *send;

	cout << "move body test with speed: " << spd << endl;
	cout << "start send \n";
	SpiderTouch();
	ForwardAllLeg();
	PrintLegCoor();
	SendToRoBot(start, 10);

	cout << "move z down 35 \n";
	data = BodyTransfer(0, 0, -35);
	ForwardAllLeg();
	PrintLegCoor();
	send = ConvertToCtrlBuff(data, 16, spd);
	SendToRoBot(send, 44);
	free(send);

	Sleep(500);

//	cout << "move to start \n";
//	SpiderTouch();
//	ForwardAllLeg();
//	PrintLegCoor();
//	SendToRoBot(start, 10);

	cout << "move z up 35 + 40 \n";
	data = BodyTransfer(0, 0, 75);
	ForwardAllLeg();
	PrintLegCoor();
	send = ConvertToCtrlBuff(data, 16, spd);
	SendToRoBot(send, 44);
	free(send);

	Sleep(500);

	cout << "move to start \n";
	SpiderTouch();
	ForwardAllLeg();
	PrintLegCoor();
	SendToRoBot(start, 10);

	Sleep(1500);

	cout << "body rotate roll 40 degree \n";
	data = BodyRotateProcess(40, 0, 0, BODY_ROLL);
	ForwardAllLeg();
	PrintLegCoor();
	send = ConvertToCtrlBuff(data, 16, spd);
	SendToRoBot(send, 44);
	free(send);

	Sleep(500);

//	cout << "move to start \n";
//	SpiderTouch();
//	ForwardAllLeg();
//	PrintLegCoor();
//	SendToRoBot(start, 10);

	cout << "body rotate roll -80 degree \n";
	data = BodyRotateProcess(-80, 0, 0, BODY_ROLL);
	ForwardAllLeg();
	PrintLegCoor();
	send = ConvertToCtrlBuff(data, 16, spd);
	SendToRoBot(send, 44);
	free(send);

	Sleep(500);

	cout << "move to start \n";
	SpiderTouch();
	ForwardAllLeg();
	PrintLegCoor();
	SendToRoBot(start, 10);

	Sleep(1500);

	cout << "body rotate pitch 30 degree \n";
	data = BodyRotateProcess(0, 30, 0, BODY_PITCH);
	ForwardAllLeg();
	PrintLegCoor();
	send = ConvertToCtrlBuff(data, 16, spd);
	SendToRoBot(send, 44);
	free(send);

	Sleep(500);

//	cout << "move to start \n";
//	SpiderTouch();
//	ForwardAllLeg();
//	PrintLegCoor();
//	SendToRoBot(start, 10);

	cout << "body rotate pitch -60 degree \n";
	data = BodyRotateProcess(0, -60, 0, BODY_PITCH);
	ForwardAllLeg();
	PrintLegCoor();
	send = ConvertToCtrlBuff(data, 16, spd);
	SendToRoBot(send, 44);
	free(send);

	Sleep(500);

	cout << "move to start \n";
	SpiderTouch();
	ForwardAllLeg();
	PrintLegCoor();
	SendToRoBot(start, 10);

	Sleep(1500);

	cout << "body rotate yaw 115 degree \n";
	data = BodyRotateProcess(0, 0, 125, BODY_YAW);
	ForwardAllLeg();
	PrintLegCoor();
	send = ConvertToCtrlBuff(data, 16, 500);
	SendToRoBot(send, 44);
	free(send);

	Sleep(500);

	cout << "body rotate yaw 65 degree \n";
	data = BodyRotateProcess(0, 0, 65, BODY_YAW);
	ForwardAllLeg();
	PrintLegCoor();
	send = ConvertToCtrlBuff(data, 16, 500);
	SendToRoBot(send, 44);
	free(send);

	Sleep(500);

	cout << "move to start \n";
	SpiderTouch();
	ForwardAllLeg();
	PrintLegCoor();
	SendToRoBot(start, 10);

	cout << "done test \n";
}

void PrintLegCoor(void)
{
	uint16_t i;
	for(i = 0; i < LEG_NUMBERS; i++)
	{
		cout << "leg[" << i << "].x: " << legpos[i].x[7] << endl;
		cout << "leg[" << i << "].y: " << legpos[i].y[7] << endl;
		cout << "leg[" << i << "].z: " << legpos[i].z[7] << endl;
	}
}

void SendToRoBot(char *buff, uint16_t size)
{
	HANDLE h = openSerialPort("COM7", B9600, one, off);
	char readbuffer[100];
	int timeout;
	int bytesRead = 0;

	//write test
	int bytesWritten = writeToSerialPort(h, buff, size);
	printf("%d Bytes were written\n",bytesWritten);
	//read something
//	int bytesRead = readFromSerialPort(h, readbuffer, 99);

	timeout = 2;
	while(bytesRead == 0)
	{
		bytesRead = readFromSerialPort(h, readbuffer, 99);
		readbuffer[bytesRead] = 0;
		Sleep(100);
		timeout--;
		printf("timeout: %d\n", timeout);
		if(0 >= timeout)
			break;
	}
	printf("%d Bytes were read:%s\n", bytesRead, readbuffer);
	closeSerialPort(h);
}

char *ConvertToCtrlBuff(uint16_t *data, uint16_t size, uint16_t speed)
{
	uint16_t i, pos, temp;
	char *ret;
	uint16_t sizebuf = (size * 2) + 12;
	ret = (char*)malloc(sizebuf);
	char const protocol[6] = {0x53, 0x50, 0x44, 0x42, 0x00, 0x01};
	char const checksum[4] = {0x12, 0x34, 0x56, 0x78};

//	printf("buf size: %u\n", sizebuf);
	pos = 0;
	for(i = 0; i < 6; i++)
	{
		ret[i] = protocol[i];
//		printf("ret[%u]: %02X\n", pos, ret[pos] & 0xff);
		pos++;
	}
	for(i = 0; i < size; i++)
	{
		temp = data[i] - PULSE_BASE;
		ret[pos] = temp >> 8;
//		printf("ret[%u]: %02X\n", pos, ret[pos] & 0xff);
		pos++;
		ret[pos] = (char)(temp & 0xFF);
//		printf("ret[%u]: %02X\n", pos, ret[pos] & 0xff);
		pos++;
	}
	ret[pos] = speed >> 8;
//	printf("ret[%u]: %02X\n", pos, ret[pos] & 0xff);
	pos++;
	ret[pos] = (char)(speed & 0xFF);
//	printf("ret[%u]: %02X\n", pos, ret[pos] & 0xff);
	pos++;
	for(i = 0; i < 4; i++)
	{
		ret[pos] = checksum[i];
//		printf("ret[%u]: %02X\n", pos, ret[pos] & 0xff);
		pos++;
	}

	return ret;
}

//#include <stdio.h>
//#include "serialport.h"
//int main(void)
//{
//	HANDLE h = openSerialPort("COM5",B9600,one,off);
//	char sendbuffer[] = "test";
//	char readbuffer[100];
//	//write test
//	int bytesWritten = writeToSerialPort(h,sendbuffer,strlen(sendbuffer));
//	printf("%d Bytes were written\n",bytesWritten);
//	//read something
//	int bytesRead = readFromSerialPort(h,readbuffer,99);
//	readbuffer[bytesRead]=0;
//	printf("%d Bytes were read:%s\n",bytesRead,readbuffer);
//	closeSerialPort(h);
//    return 0;
//}
