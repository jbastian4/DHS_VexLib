#ifndef superuser
#define superuser

void setDriveMotors(int lPower, int rPower = -128)
{
  if (rPower == -128)
    rPower = lPower;

  //Left drive motors
  if(lPower <= 127) {
  motor[ldrive] = lpower; //set left drive motors like this
  }

  //Right drive motors
  if(rPower <= 127) {
  motor[rdrive] = rpower; //set right drive motors like this
  }
}



#endif
