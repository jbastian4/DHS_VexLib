#ifndef advanced
#define advanced


//#region User Variables

//define sensors
#define gyroPort in1
#define lEncPort dgtl1
#define rEncPort dgtl3

//define motors
#define lDrivePort1 port1
#define lDrivePort2 port2
#define rDrivePort1 port3
#define rDrivePort2 port4

//miscellaneous values
#define wheelDiameter 4
#define dontHog 25
#define stopError 17
#define stopTime 200

//Encoder PID Values
#define lEnc_Kp 0.8
#define lEnc_Kd 0.03

#define rEnc_Kp 0.45
#define rEnc_Kd 0.03

//Gyro PID Values
#define gyro_Kp 0.35
#define gyro_Kd 1

//Drive ramp values
int rampInterval = 3;
int lNormalRampSpeed = 10;
int rNormalRampSpeed = 10;
int lHighRampSpeed = 25;
int rHighRampSpeed = 25;
int deadband = 10;
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
int  lEncRequestedValue;
int  rEncRequestedValue;
int  gyroRequestedValue;

float lEncD;
float lEncP;
float lastlEncError;
float lEncDF;

float  lEncSensorCurrentValue;
float  lEncError;
float  lEncDrive;

float rEncD;
float rEncP;
float lastrEncError;
float rEncDF;

float  rEncSensorCurrentValue;
float  rEncError;
float  rEncDrive;

//Gyro PID values
float gyroD;
float gyroP;
float lastGyroError;
float gyroDF;

float  gyroCurrentValue;
float  gyroError;
float  gyroDrive;
//#endregion

//#region PID Functions

//#endregion

//#region Main Functions

//#region Tasks

//#endregion




#endif
