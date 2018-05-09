#ifndef superuser
#define superuser

typedef struct {
  tMotor motors[10];
  int numMotors;

  tSensor encoder;
} motorGroup;

typedef struct {

  float kp 0.8
  float ki 0
  float kd 0.03

  int   requestedValue;
  float err;//proportional error
  float prevErr; //prop error from previous loop
  float intg;//integral error
  float der;//derivative error
  float prevTime;
  float dt; //difference in time
  float currentValue;
  byte output;

} pidGroup;

void initializeMotorGroup(motorGroup *group, int numMotors, *tMotor motors, tSensors encoder)
{
  group->numMotors = numMotors;

  for (int i=0; i < group->numMotors; i++)
		group->motors[i] = motors[i];
}

void initializePIDGroup(pidGroup *group, float kp, float ki, float kd)
{
  group->kp = kp;
  group->ki = kd;
  group->kd = kd;
}

void setMotors(motorGroup *group, byte power)
{
  for (int i=0; i<group->numMotors; i++) //set motors
		motor[group->motors[i]] = power;
}

byte pid(pidGroup *pid, int request)
{
  gyroCurrentValue = SensorValue[gyroPort];

  pid->err = pid->RequestedValue - pid->currentValue;
  pid->intg = pid->intg + gyroErr;
  pid->der = pid->err - pid->prevErr;
  pid->dt = nPgmTime - pid->prevTime;

  return ((pid->kp * pid->err) + (pid->ki * pid->intg * pid->dt) + (pid->kd * pid->der / pid->dt));

  pid->prevErr = pid->err;
  pid->prevTime = nPgmTime;
}
