#ifndef advanced
#define advanced


//#region User Variables

//define Sensors
#define gyroPort in1
#define lEncPort dgtl1
#define rEncPort dgtl3

//define motors
#define lDrivePort1 port3
#define lDrivePort2 port10
#define rDrivePort1 port2
#define rDrivePort2 port1

//miscellaneous values
#define wheelDiameter 4
#define dontHog 25
#define stopError 17
#define stopTime 200

//Encoder PID Values
#define lEnc_Kp 0.8
#define lEnc_Ki .001// if you dont want an i keep it 0
#define lEnc_Kd 0.03

#define rEnc_Kp 0.45
#define rEnc_Ki .001// if you dont want an i keep it 0
#define rEnc_Kd 0.03

//Gyro PID Values
#define gyro_Kp 0.35
#define gyro_ki .001// if you dont want an i keep it 0
#define gyro_Kd 1

//Drive ramp values
int rampInterval = 3;
int RampingChange = 10;
int initalRamp = 20;
int lEncRampBias = 0;
int rEncRampBias = 0;
int RP;
int P;
int lEncPrevPower;
int rEncPrevPower;
//#endregion

//#region System Variables
int driveMode = 0; //0 is drivestraight, 1 is turn
int direction = 0; //0 is stopped, 1 is forward/right, -1 is backward/left
int distance = 0; //drivestraight distance or turn angle
int countsToInches(float value) //converts drive encoder counts into inches
{
  return (value * 360)/(PI * wheelDiameter);
}

//Encoder PID Values

int   lEncRequestedValue;
float lEncErr;//proportional error
float lEncPrevErr; //prop error from previous loop
float lEncInt;//integral error
float lEncDer;//derivative error
long lEncPrevTime;
float lEncDt; //difference in time
float lEncCurrentValue;
byte lEncOutput;

int   rEncRequestedValue;
float rEncErr;//proportional error
float rEncPrevErr; //prop error from previous loop
float rEncInt;//integral error
float rEncDer;//derivative error
long rEncPrevTime;
float rEncDt; //difference in time
float rEncCurrentValue;
byte  rEncOutput;

//Gyro PID values
int   gyroRequestedValue;
float gyroErr;//proportional error
float gyroPrevErr; //prop error from previous loop
float gyroInt;//integral error
float gyroDer;//derivative error
long gyroPrevTime;
float gyroDt; //difference in time
float gyroCurrentValue;
byte  gyroOutput;
//#endregion
//#region Main Functions
void driveWaity(int distance)
{
  int ticks = fabs(countsToInches(distance));
  while(fabs(SensorValue[lEnc]) <= ticks - stopError){}
  wait1Msec(stopTime);
  ticks = 0;
}
void turnwaity(int degrees)
  {
    while(fabs(SensorValue[gyroPort]) <= fabs(degrees) - 50){}
    wait1Msec(stopTime);
  }
void unityStraight(int distance, bool waity = false, bool correct = false) //for correction to work properly waity must be true
{
  driveMode = 2;
  if (correct){
  SensorValue[gyroPort] = 0;}
  SensorValue[rEncPort] = 0;
  SensorValue[lEncPort] = 0;
  int ticks = fabs(countsToInches(distance));
  lEncRequestedValue = ticks;
  rEncRequestedValue = ticks;
  driveMode = 0;
  if(waity)  {
    wait1Msec(stopTime);
    driveWaity(distance);
  }
  if (correct){
    driveMode = 1;
    turnwaity(0);
    wait1Msec(stopTime);
  }
}

void unityTurn(int degrees,bool waity=false)
{
  driveMode = 2;
  SensorValue[gyroPort] = 0;
  gyroRequestedValue = degrees;
  driveMode = 1;
  if(waity)
  {
    turnwaity(degrees);
  }

}
int driveRamp(int RequestedPower,int Power,int sidebias=0 )
{
  RP = RequestedPower;
  P = Power;
		if (abs(RP)>abs(P) && RP!=0)
		{
			if(abs(P)<initalRamp)	P=initalRamp;
			else P = abs(P) + RampingChange + sidebias;
			P = abs(P) * sgn(RP);
		}
		else	P=RP;
    return P;
		wait1Msec(rampInterval);
}
void setLDriveMotors(int power)
{
	motor[lDrivePort1] = power;
	motor[lDrivePort2] = power;
}
void setRDriveMotors(int power)
{
	motor[rDrivePort1] = power;
	motor[rDrivePort2] = power;
}
//#endregion

//#region PID Functions
 void lEncController()
 {
   lEncCurrentValue = SensorValue[lEncPort];

   lEncErr = lEncRequestedValue - lEncCurrentValue;
   lEncInt = lEncInt + lEncErr;
   lEncDer = lEncErr - lEncPrevErr;
   lEncDt = nPgmTime - lEncPrevTime;

   lEncOutput = (lEnc_Kp * lEncErr) + (lEnc_Ki * lEncInt * lEncDt) + (lEnc_Kd * lEncDer / lEncDt);

   lEncPrevErr = lEncErr;
   lEncPrevTime = nPgmTime;
   lEncPrevPower = driveRamp(lEncOutput,lEncPrevPower,lEncRampBias);
   setLDriveMotors(lEncPrevPower);
 }

 void rEncController()
 {
   rEncCurrentValue = SensorValue[rEncPort];

   rEncErr = rEncRequestedValue - rEncCurrentValue;
   rEncInt = rEncInt + rEncErr;
   rEncDer = rEncErr - rEncPrevErr;
   rEncDt = nPgmTime - rEncPrevTime;

   rEncOutput = (rEnc_Kp * rEncErr) + (rEnc_Ki * rEncInt * rEncDt) + (rEnc_Kd * rEncDer / rEncDt);

   rEncPrevErr = rEncErr;
   rEncPrevTime = nPgmTime;
   rEncPrevPower = driveRamp(rEncOutput,rEncPrevPower,rEncRampBias);
   setRDriveMotors(rEncPrevPower);

 }

 void gyroController()
 {
   gyroCurrentValue = SensorValue[gyroPort];

   gyroErr = gyroRequestedValue - gyroCurrentValue;
   gyroInt = gyroInt + gyroErr;
   gyroDer = gyroErr - gyroPrevErr;
   gyroDt = nPgmTime - gyroPrevTime;

   gyroOutput = (gyro_Kp * gyroErr) + (gyro_ki * gyroInt * gyroDt) + (gyro_Kd * gyroDer / gyroDt);

   gyroPrevErr = gyroErr;
   gyroPrevTime = nPgmTime;
 }

//#endregion



//#region Tasks
task unity2()
{
  SensorValue[rEncPort] = 0;
  SensorValue[lEncPort] = 0;
  SensorValue[gyroPort] = 0;
  while(true)
  {
    if (driveMode == 0)
    {
      lEncController();
      rEncController();
    }
    else if(driveMode == 1)
    {
     gyroController();
    }
    else
    {
     setLDriveMotors(0);
     setRDriveMotors(0);
    }
  }
}
//#endregion


#endif
