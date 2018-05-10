#ifndef superuser
#define superuser

//#region Structs

typedef struct {
  tMotor motors[10];
  int numMotors;

  bool hasEncoder, hasPotentiometer;
  tSensors encoder, potentiometer;

  bool ramping = false;
  byte currentPower;
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
} driveGroup;

//#endregion

//#region Initialization Functions
void initializeMotorGroup(motorGroup *group, int numMotors, *tMotor motors)
{
  group->numMotors = numMotors;

  for (int i=0; i < group->numMotors; i++)
		group->motors[i] = motors[i];
}

void configureRamping(motorGroup *group, normalRampSpeed, highRampSpeed)
{
  group->ramping = true;
  group->rampInterval = rampInterval;
  group->normalRampSpeed = normalRampSpeed;
  group->highRampSpeed = highRampSpeed;
  group->deadband = deadband;
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

void initializeDriveGroup(driveGroup *drive, motorGroup *leftMot, pidGroup *leftPID, motorGroup *rightMot, pidGroup *leftPID, tSensors gyro = sensorNone)
{
  drive->leftMot = leftMot;
  drive->leftPID = leftPID;
  drive->rightMot = rightMot;
  drive->rightPID = rightPID;

  if(SensorType[gyro] = SensorGyro) {
    drive->hasGyro = true;
    drive->gyro = gyro;
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
  drive Drive;

}

//#region
