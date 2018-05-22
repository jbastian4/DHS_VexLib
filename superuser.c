#ifndef superuser
#define superuser

//#region Variables

//define Sensors
#define gyroPort in1
#define lEncPort dgtl1
#define rEncPort dgtl3

//miscellaneous values
#define wheelDiameter 4
#define stopError 17
#define stopTime 200

//Encoder PID Values
#define lEnc_Kp 0.8
#define lEnc_Ki .000001// if you dont want an i keep it 0, otherwise still very small
#define lEnc_Kd 0.03

#define rEnc_Kp 0.45
#define rEnc_Ki .000001// if you dont want an i keep it 0
#define rEnc_Kd 0.03

//Gyro PID Values
#define gyro_Kp 0.35
#define gyro_Ki .000001// if you dont want an i keep it 0
#define gyro_Kd 1

string driveMode = "None";

//#endregion

//#region Structs

typedef struct {
  tMotor motors[10];
  int numMotors;

  bool ramping;
  byte currentPower;
  int rampInterval;
  int normalRampSpeed;
  int highRampSpeed;

} motorGroup;

typedef struct {

  float kp;
  float ki;
  float kd;

  tSensors sensor; //gyro, encoder or potentiometer

  int   requestedValue;
  float err;//proportional error
  float prevErr; //prop error from previous loop
  float intg;//integral error
  float der;//derivative error
  float prevTime;
  float dt; //difference in time
  float currentValue;

} pidGroup;

	motorGroup lDrive;
  motorGroup rDrive;
  pidGroup lDrivePID;
  pidGroup rDrivePID;
  pidGroup gyroPID;

  motorGroup motorNone;
  pidGroup pidNone;

//#endregion

//#region Initialization Functions
void initializeMotorGroup(motorGroup *group, int numMotors, tMotor *motors, bool ramp = false)
{
  group->ramping = false;
  group->numMotors = numMotors;

  for (int i=0; i < group->numMotors; i++)
		group->motors[i] = motors[i];
}

void configureRamping(motorGroup *group, int normalRampSpeed, int highRampSpeed)
{
  group->ramping = true;
  group->normalRampSpeed = normalRampSpeed;
  group->highRampSpeed = highRampSpeed;
}

void attachSensor(pidGroup *PID, tSensors sensor)
{
  PID->sensor = sensor;
}

void initializePIDGroup(pidGroup *pid, float kp, float ki, float kd)
{
  pid->kp = kp;
  pid->ki = kd;
  pid->kd = kd;
}

void initializeAll()
{

  #define NUM_LDRIVE_MOTORS 2
	tMotor lDriveMotors[NUM_LDRIVE_MOTORS] = { port1, port2};

  #define NUM_RDRIVE_MOTORS 2
	tMotor rDriveMotors[NUM_RDRIVE_MOTORS] = { port3, port4};

  initializeMotorGroup(lDrive, NUM_LDRIVE_MOTORS, lDriveMotors, true);
  initializeMotorGroup(rDrive, NUM_RDRIVE_MOTORS, rDriveMotors, true);

  initializePIDGroup(lDrivePID, lEnc_Kp, lEnc_Ki, lEnc_Kd);
  initializePIDGroup(rDrivePID, rEnc_Kp, rEnc_Ki, rEnc_Kd);
  initializePIDGroup(gyroPID, gyro_Kp, gyro_Ki, gyro_Kd);

  attachSensor(lDrivePID, lEncPort);
  attachSensor(rDrivePID, rEncPort);
  attachSensor(gyroPID, gyroPort);

  configureRamping(lDrive, 7, 20);
  configureRamping(rDrive, 7, 20);
}

//#endregion

//#region PID Functions
void setMotors(motorGroup *group, byte power)
{
  for (int i=0; i<group->numMotors; i++) //set motors
		motor[group->motors[i]] = power;

  group->currentPower = power;
}

void pidRequest(pidGroup *pid, int request)
{
  pid->requestedValue = request;
}

byte pid(pidGroup *pid)
{
  pid->currentValue = SensorValue[pid->sensor];

  pid->err = pid->requestedValue - pid->currentValue;
  pid->intg = pid->intg + pid->err;
  pid->der = pid->err - pid->prevErr;
  pid->dt = nPgmTime - pid->prevTime;
  if(pid->dt < 1)
    pid->dt = 1;

  return ((pid->kp * pid->err) + (pid->ki * pid->intg * pid->dt) + (pid->kd * pid->der / pid->dt));

  pid->prevErr = pid->err;
  pid->prevTime = nPgmTime;
}

void runPIDLoop(motorGroup *group, pidGroup *pidG, bool driveCorrection, pidGroup *otherPID = pidNone)
{
  byte desiredPower = pid(pidG);

  if(driveCorrection)
  {
    if (fabs(pidG->currentValue)>fabs(SensorValue[otherPID->sensor]))
      desiredPower = desiredPower-((fabs(pidG->currentValue) - fabs(SensorValue[otherPID->sensor]))/2*sgn(desiredPower));
  }

  if(group->ramping)
  {
    if (abs(desiredPower)>abs(group->currentPower) && desiredPower!=0)
    {
      if(abs(group->currentPower) < group->highRampSpeed)
        desiredPower = group->highRampSpeed;
      else	desiredPower = abs(group->currentPower) + group->normalRampSpeed;
      group->currentPower = abs(group->currentPower) * sgn(desiredPower);
    }
    else	group->currentPower = desiredPower;
  }

  setMotors(group, desiredPower);
}

//#endregion

//#region Main Functions

int inchesToCounts(float value) //converts drive encoder counts into inches
{
  return (value * 360)/(PI * wheelDiameter);
}

void driveWaity(int distance)
{
  int ticks = fabs(inchesToCounts(distance));
  while(fabs(SensorValue[lEncPort]) <= ticks - stopError){}
  wait1Msec(stopTime);
  ticks = 0;
}

void turnWaity(int degrees)
{
  while(fabs(SensorValue[gyroPort]-degrees) > 50){}
  wait1Msec(stopTime);
}

void unityStraight(int distance, bool waity = false, bool correct = false) //for correction to work properly waity must be true
{
  driveMode = "None";
  if(correct)
    SensorValue[gyroPort] = 0;
  SensorValue[rEncPort] = 0;
  SensorValue[lEncPort] = 0;
  int ticks = fabs(inchesToCounts(distance));
  pidRequest(lDrivePID, ticks);
  pidRequest(rDrivePID, ticks);
  driveMode = "Straight";
  if(waity){
    driveWaity(distance);
    if (correct){
      driveMode = "Turn";
      pidRequest(gyroPID, 0);
      turnWaity(0);
      wait1Msec(stopTime);
    }
  }
}

void unityTurn(int degrees, bool waity = false)
{
  driveMode = "None";
  SensorValue[gyroPort] = 0;
  pidRequest(gyroPID, degrees);
  driveMode = "Turn";
  if(waity)
    turnWaity(degrees);
}

task unity3()
{
  initializeAll();
  while(true)
  {
    if(driveMode == "None") {}
    else if(driveMode == "Straight")
    {
      runPIDLoop(lDrive, lDrivePID, true, rDrivePID);
      runPIDLoop(rDrive, rDrivePID, true, lDrivePID);
    }
    else if(driveMode == "Turn")
    {
      setMotors(lDrive, pid(gyroPID));
      setMotors(rDrive, -pid(gyroPID));
    }
    else
    {
      setMotors(lDrive, 0);
      setMotors(rDrive, 0);
    }
  }
}

//#endregion

#endif
