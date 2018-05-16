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
#define lEnc_Ki .001// if you dont want an i keep it 0
#define lEnc_Kd 0.03

#define rEnc_Kp 0.45
#define rEnc_Ki .001// if you dont want an i keep it 0
#define rEnc_Kd 0.03

//Gyro PID Values
#define gyro_Kp 0.35
#define gyro_ki .001// if you dont want an i keep it 0
#define gyro_Kd 1

//#endregion

//#region Structs

typedef struct {
  tMotor motors[10];
  int numMotors;

  bool hasEncoder, hasPotentiometer;
  tSensors encoder, potentiometer;

  bool ramping = false;
  byte currentPower;
  int rampInterval;
  int normalRampSpeed;
  int highRampSpeed;

} motorGroup;

typedef struct {

  float kp;
  float ki;
  float kd;

  int   requestedValue;
  float err;//proportional error
  float prevErr; //prop error from previous loop
  float intg;//integral error
  float der;//derivative error
  float prevTime;
  float dt; //difference in time
  float currentValue;

} pidGroup;

typedef struct {
  motorGroup *leftMot;
  pidGroup *leftPID;
  motorGroup *rightMot;
  pidGroup *rightPID;

  bool hasGyro;
  tSensors gyro;
  pidGroup *gyroPID;

} driveGroup;

//#endregion

//#region Initialization Functions
void initializeMotorGroup(motorGroup *group, int numMotors, *tMotor motors)
{
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

void attachSensor(motorGroup *group, tSensors sensor)
{
  switch (SensorType[sensor]) {
		case sensorPotentiometer:
			group->hasPotentiometer = true;
			group->potentiometer = sensor;
			break;
		case sensorQuadEncoder:
			group->hasEncoder = true;
			group->encoder = sensor;
			SensorValue[sensor] = 0;
			break;
	}
}

void initializePIDGroup(pidGroup *pid, float kp, float ki, float kd)
{
  group->kp = kp;
  group->ki = kd;
  group->kd = kd;
}

void initializeDriveGroup(driveGroup *drive, motorGroup *leftMot, pidGroup *leftPID, motorGroup *rightMot, pidGroup *rightPID, tSensors gyro = sensorNone, pidGroup *turnPID)
{
  drive->leftMot = leftMot;
  drive->leftPID = leftPID;
  drive->rightMot = rightMot;
  drive->rightPID = rightPID;

  if(SensorType[gyro] = SensorGyro) {
    drive->hasGyro = true;
    drive->gyro = gyro;
    drive->gyroPID = turnPID;
  }
}

//#endregion

//#region PID Functions
void setMotors(motorGroup *group, byte power)
{
  for (int i=0; i<group->numMotors; i++) //set motors
		motor[group->motors[i]] = power;
}

void pidRequest(pidGroup *pid, int request)
{
  pid->requestedValue = request;
}

byte pid(pidGroup *pid)
{
  pid->currentValue = SensorValue[gyroPort];

  pid->err = pid->requestedValue - pid->currentValue;
  pid->intg = pid->intg + gyroErr;
  pid->der = pid->err - pid->prevErr;
  pid->dt = nPgmTime - pid->prevTime;

  return ((pid->kp * pid->err) + (pid->ki * pid->intg * pid->dt) + (pid->kd * pid->der / pid->dt));

  pid->prevErr = pid->err;
  pid->prevTime = nPgmTime;
}

void runPIDLoop(motorGroup *group, pidGroup *pid)
{
  byte desiredPower = pid(pid);

  if(group->ramping)
  {
    if (abs(desiredPower)>abs(group->currentPower) && desiredPower!=0)
    {
      if(abs(group->currentPower) < group->highRampSpeed)
        desiredPower = highRampSpeed;
      else	desiredPower = abs(group->currentPower) + group->normalRampSpeed;
      group->currentPower = abs(group->currentPower) * sgn(desiredPower);
    }
    else	group->currentPower = desiredPower;
  }

  setMotors(group, desiredPower);
  group->currentPower = desiredPower;
}

//#endregion

//#region Initialization

void initializeAll()
{
  motorGroup lDrive;
  motorGroup rDrive;
  pidGroup lDrivePID;
  pidGroup rDrivePID;
  pidGroup gyroPID;
  driveGroup Drive;

  #define NUM_LDRIVE_MOTORS 2
	tMotor lDriveMotors[NUM_LDRIVE_MOTORS] = { port1, port2};

  #define NUM_RDRIVE_MOTORS 2
	tMotor rDriveMotors[NUM_RDRIVE_MOTORS] = { port3, port4};

  initializeMotorGroup(lDrive, NUM_LDRIVE_MOTORS, lDriveMotors);
  initializeMotorGroup(rDrive, NUM_RDRIVE_MOTORS, rDriveMotors);

  initializePIDGroup(lDrivePID, lEnc_Kp, lEnc_Ki, lEnc_Kd);
  initializePIDGroup(rDrivePID, rEnc_Kp, rEnc_Ki, rEnc_Kd);
  initializePIDGroup(gyroPID, gyro_Kp, gyro_Ki, gyro_Kd);

  initializeDriveGroup(Drive, lDrive, lDrivePID, rDrive, rDrivePID, gyroPort, gyroPID);

  attachSensor(lDrive, lEncPort);
  attachSensor(rDrive, rEncPort);

  configureRamping(lDrive, 7, 20);
  configureRamping(rDrive, 7, 20);
}

//#endregion

//#region functions

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
    while(fabs(SensorValue[gyroPort]) <= fabs(degrees) - 50){}
    wait1Msec(stopTime);
  }

void unityStraight(int distance, bool waity = false, bool correct = false) //for correction to work properly waity must be true
{
  driveMode = 2;
  if(correct)
    SensorValue[gyroPort] = 0;
  SensorValue[rEncPort] = 0;
  SensorValue[lEncPort] = 0;
  int ticks = fabs(inchesToCounts(distance));
  lEncRequestedValue = ticks;
  rEncRequestedValue = ticks;
  driveMode = 0;
  if(waity)  {
    wait1Msec(stopTime);
    driveWaity(distance);
    if (correct){
      driveMode = 1;
      turnwaity(0);
      wait1Msec(stopTime);
    }
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

//#endregion
