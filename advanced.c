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
#define lEnc_Ki 0
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

int   lEncRequestedValue;
float lEncErr;//proportional error
float lEncPrevErr; //prop error from previous loop
float lEncInt;//integral error
float lEncDer;//derivative error
float lEncPrevTime;
float lEncDt; //difference in time
float lEncCurrentValue;
byte lEncOutput;

int   rEncRequestedValue;
float rEncErr;//proportional error
float rEncPrevErr; //prop error from previous loop
float rEncInt;//integral error
float rEncDer;//derivative error
float rEncPrevTime;
float rEncDt; //difference in time
float rEncCurrentValue;
byte  rEncOutput;

//Gyro PID values
int   gyroRequestedValue;
float gyroErr;//proportional error
float gyroPrevErr; //prop error from previous loop
float gyroInt;//integral error
float gyroDer;//derivative error
float gyroPrevTime;
float gyroDt; //difference in time
float gyroCurrentValue;
byte  gyroOutput;
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
 }

 void gyroController()
 {
   gyroCurrentValue = SensorValue[gyroPort];

   gyroErr = gyroRequestedValue - gyroCurrentValue;
   gyroInt = gyroInt + gyroErr;
   gyroDer = gyroErr - gyroPrevErr;
   gyroDt = nPgmTime - gyroPrevTime;

   gyroOutput = (gyro_Kp * gyroErr) + (gyro_Ki * gyroInt * gyroDt) + (gyro_Kd * gyroDer / gyroDt);

   gyroPrevErr = gyroErr;
   gyroPrevTime = nPgmTime;
 }
//#endregion

//#region Main Functions

//#region Tasks

//#endregion




#endif
