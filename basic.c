#ifndef basic
#define basic

//#region definitions

#define wheelDiameter 4
#define lDrivePort1 port1
#define lDrivePort2 port2
#define rDrivePort1 port3
#define rDrivePort2 port4
#define lEncPort dgtl1
#define rEncPort dgtl3
#define gyroPort in1

//#endregion

//#region drivestraight

void setDriveMotors(int lPower, int rPower = -128)
{
  if (rPower == -128)
    rPower = lPower;

  //Left drive motors
  if(lPower <= 127) {
  motor[lDrivePort1] = lpower; //set left drive motors like this
  motor[lDrivePort2] = lpower;
  }

  //Right drive motors
  if(rPower <= 127) {
  motor[rDrivePort1] = rpower; //set right drive motors like this
  motor[rDrivePort2] = rpower;
  }
}

void driveController(int distance, int power = 100)
 // Creating a function to make sure the robot drives forward.
{
	int direction = sgn(distance);
	distance = abs(distance);

 // telling it to do this while you are driving to the desired distance
	while(abs(SensorValue[rEncPort]) < distance)
	{
		 // Checking if the right drive is faster than the left drive
		if (abs(SensorValue[rEncPort]) > abs(SensorValue[lEncPort]))
		{
			setDriveMotors(direction * (power + 12), direction * (power - 50));
		}
		// Checking if the left drive is faster than the right drive
		else if (abs(SensorValue[lEncPort]) > abs(SensorValue[rEncPort]))
		{
			setDriveMotors(direction * (power - 15), direction * power + 4);
		}
		else //if the values are the same, keeps motor powers equal
		{
			setDriveMotors(direction * power, direction * power);
		}
	}
		setDriveMotors(0); // stops motors when finished with function
}

void driveStraight(float inches, int power)
{   //Auton function to move forward by itself
	int ticks = (inches * 360)/(PI * wheelDiameter);
	  //Clear Encoders
  SensorValue[lEncPort] = 0;
  SensorValue[rEncPort] = 0;
		//Drive
	driveStraight(ticks, power);
    //sudden stop to correct overshooting the value
	setDriveMotors(-15 * sgn(ticks));
	wait1Msec(100);
}

//#endregion

//#region turning

void encTurn(int counts, int power = 100)
{
	//Clear Encoders
  SensorValue[rEncPort] = 0;
  SensorValue[leftEncoder] = 0;

	while((fabs(SensorValue[rEncPort]) + fabs(SensorValue[lEncPort]))/2 > fabs(counts))
	{
		setDriveMotors(power * sgn(counts), power * -sgn(counts));
	}
  //stop to prevent overshoot
  setDriveMotors(127 * -sgn(counts), 127 * sgn(counts));
  wait1Msec(100);
  setDriveMotors(0);
}

void gyroTurn(int counts, int power = 100)
{
  //clear gyro
  SensorValue[gyroPort] = 0;

  while(fabs(SensorValue[gyroPort]) <= fabs(counts))
  {
    setDriveMotors(power * sgn(counts), power * -sgn(counts));
  }

  //stop to prevent overshoot
  setDriveMotors(127 * -sgn(counts), 127 * sgn(counts));
  wait1Msec(100);
  setDriveMotors(0);
}

//#endregion


#endif
