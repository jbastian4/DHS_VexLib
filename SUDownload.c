#pragma config(Sensor, in1,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  lEnc,           sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  rEnc,           sensorQuadEncoder)
#pragma config(Motor,  port1,           lbDrive,       tmotorVex393HighSpeed_HBridge, openLoop)
#pragma config(Motor,  port8,           lfDrive,       tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port9,           rfDrive,       tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          rbDrive,       tmotorVex393HighSpeed_HBridge, openLoop, reversed)
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "superuser.c"
#include "Vex_Competition_Includes.c"
int cumBias;
int debug;
void pre_auton()
{
  bStopTasksBetweenModes = true;
  /*SensorType[gyro] = sensorNone;
	for(int i = 0; i<2000; i++)
	{
		cumBias += SensorValue[gyro];
		wait1Msec(1);
	}

	debug = cumBias / 2000;*/
}
task autonomous()
{
	SensorScale[gyro] = 134;
	wait1Msec(250);
	 SensorBias[gyro] = 1851;
	 wait1Msec(250);
	startTask (unity3);
	unityStraight(80,true,true);
}
task usercontrol()
{
  while (true)
  {
		setMotors(lDrive, vexRT[Ch3]);
		setMotors(rDrive, vexRT[Ch2]);
  }
}
