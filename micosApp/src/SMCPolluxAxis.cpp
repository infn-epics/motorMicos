
#include <epicsThread.h>
#include "SMCpolluxDriver.h"
#include <string.h>
#define TERMINATOR "\n"

std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\n\r\f\v");
    size_t end   = s.find_last_not_of(" \t\n\r\f\v");
    return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
}

/** Creates a new SMCpolluxAxis object.
  * \param[in] pC Pointer to the SMCpolluxController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
SMCpolluxAxis::SMCpolluxAxis(SMCpolluxController *pC, int axis,int physaddr)
  : asynMotorAxis(pC, axis),axisid(physaddr),pC_(pC)
{  
  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "creating asyn axis %d physical axis %d\n",axis,physaddr);

  printf("creating asyn axis %d physical axis %d\n",axis,physaddr);
  sprintf(pC_->outString_, "%i nversion" TERMINATOR, (physaddr ));
  pC_->writeReadController();
  version=trim(std::string((char*)&pC_->inString_));
  if(version.size()==0){
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
    "## WARNING version empty asyn axis %d physical axis %d, it may not work.. check connection and axis ids\n",axis,physaddr);
  }
  sprintf(pC_->outString_, "%i getpitch" TERMINATOR, (physaddr ));
  pC_->writeReadController();
  pitch_ = atof( (char *) &pC_->inString_ );


  sprintf(pC_->outString_, "%i getpolepairs" TERMINATOR, (physaddr ));
  pC_->writeReadController();
  polePairs_ = atoi( (char *) &pC_->inString_ );

  sprintf(pC_->outString_, "%i getclperiod" TERMINATOR, (physaddr ));
  pC_->writeReadController();
  clPeriod_ = atof( (char *) &pC_->inString_ );

  switch (motorForm_)
  {
    case 0:
      // Stepper motor
      axisRes_ = pitch_ / ( 4.0 * polePairs_);
      break;

    case 1:
      // Linear or torque motor
      axisRes_ = clPeriod_;
      break;

    default:
      // For now assume clPeriod_ works for other motor forms
      axisRes_ = clPeriod_;
      break;
  }

  /* Enable gain support so that the CNEN field can be used to send
     the init command to clear a motor fault for stepper motors, even
     though they lack closed-loop support. */
  setIntegerParam(pC_->motorStatusGainSupport_, 1);

  // Determine the travel limits (will change after homing)
  sprintf(pC_->outString_, "%i getnlimit" TERMINATOR, (physaddr ));
  pC_->writeReadController();
  sscanf(pC_->inString_, "%lf %lf", &negTravelLimit_, &posTravelLimit_);

}
/** Change the axis resolution
  * \param[in] newResolution The new resolution
  */
asynStatus SMCpolluxAxis::changeResolution(double newResolution)
{
  axisRes_ = newResolution;
  
  return asynSuccess;
}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void SMCpolluxAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisid);
    fprintf(fp, "  version \"%s\"\n", version.c_str());

    fprintf(fp, "    motorForm %d\n", motorForm_);
    fprintf(fp, "    pitch %f\n", pitch_);
    fprintf(fp, "    polePairs %d\n", polePairs_);
    fprintf(fp, "    clPeriod %f\n", clPeriod_);
    fprintf(fp, "    axisRes %f\n", axisRes_);
    fprintf(fp, "    lowLimitConfig %d\n", lowLimitConfig_);
    fprintf(fp, "    highLimitConfig %d\n", highLimitConfig_);
    fprintf(fp, "    posTravelLimit %f\n", posTravelLimit_);
    fprintf(fp, "    negTravelLimit %f\n", negTravelLimit_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus SMCpolluxAxis::sendAccelAndVelocity(double acceleration, double velocity) 
{
  asynStatus status;
  // static const char *functionName = "SMCpolluxAxis::sendAccelAndVelocity";

  // Send the velocity
  sprintf(pC_->outString_, "%f %i snv", fabs(velocity * axisRes_), (axisid));
  status = pC_->writeController();

  // Send the acceleration
  // acceleration is in units/sec/sec
  sprintf(pC_->outString_, "%f %i sna" TERMINATOR, fabs(acceleration * axisRes_), (axisid));
  status = pC_->writeController();
  return status;
}


asynStatus SMCpolluxAxis::move(double position, int relative, double baseVelocity, double slewVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "SMCpolluxAxis::move";

  status = sendAccelAndVelocity(acceleration, slewVelocity);
  
  if (relative) {
    sprintf(pC_->outString_, "%f %i nr" TERMINATOR, (position * axisRes_), (axisid));
  } else {
    sprintf(pC_->outString_, "%f %i nm" TERMINATOR, (position * axisRes_), (axisid));
  }
  status = pC_->writeController();
  return status;
}

asynStatus SMCpolluxAxis::home(double baseVelocity, double slewVelocity, double acceleration, int forwards)
{
  asynStatus status;
  // static const char *functionName = "SMCpolluxAxis::home";

  status = sendAccelAndVelocity(acceleration, slewVelocity);

  if (forwards) {
    sprintf(pC_->outString_, "%i nrm" TERMINATOR, (axisid));
  } else {
    sprintf(pC_->outString_, "%i ncal" TERMINATOR, (axisid));
  }
  status = pC_->writeController();
  return status;
}

asynStatus SMCpolluxAxis::moveVelocity(double baseVelocity, double slewVelocity, double acceleration)
{
  asynStatus status;
  static const char *functionName = "SMCpolluxAxis::moveVelocity";

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s: baseVelocity=%f, slewVelocity=%f, acceleration=%f\n",
    functionName, baseVelocity, slewVelocity, acceleration);
  
  /* SMC hydra does not have jog command. Move to a limit*/
  if (slewVelocity > 0.) {
    status = sendAccelAndVelocity(acceleration, slewVelocity);
    sprintf(pC_->outString_, "%f %i nm" TERMINATOR, posTravelLimit_, (axisid));
  } else {
    status = sendAccelAndVelocity(acceleration, (slewVelocity * -1.0));
    sprintf(pC_->outString_, "%f %i nm" TERMINATOR, negTravelLimit_, (axisid));
  }
  status = pC_->writeController();
  return status;
}

asynStatus SMCpolluxAxis::stop(double acceleration )
{
  asynStatus status;
  //static const char *functionName = "SMCpolluxAxis::stop";

  // Set stop deceleration (will be overridden by accel if accel is higher)
  sprintf(pC_->outString_, "%f %i ssd" TERMINATOR, fabs(acceleration * axisRes_), (axisid));
  status = pC_->writeController();

  sprintf(pC_->outString_, "%i nabort" TERMINATOR, (axisid));
  status = pC_->writeController();
  return status;
}

asynStatus SMCpolluxAxis::setPosition(double position)
{
  asynStatus status;
  //static const char *functionName = "SMCpolluxAxis::setPosition";

  // The argument to the setnpos command is the distance from the current position of the
  // desired origin, which is why the position needs to be multiplied by -1.0
  sprintf(pC_->outString_, "%f %i setnpos" TERMINATOR, (position * axisRes_ * -1.0), (axisid));
  status = pC_->writeController();
  return status;
}

asynStatus SMCpolluxAxis::setClosedLoop(bool closedLoop)
{
  asynStatus status = asynSuccess;
  int regulatorMode;
  //static const char *functionName = "SMCpolluxAxis::setClosedLoop";

  switch (motorForm_)
  {
    case 0:
      // Stepper motor with encoder
    case 1:
      // Linear or torque motor

      // Get the regulator mode (0 = open-loop, 1 = standard, 2 = adaptive)
      pC_->getIntegerParam(axisid, pC_->SMCpolluxRegulatorMode_, &regulatorMode);
      
      if (closedLoop) {
        // enable closed-loop control
        sprintf(pC_->outString_, "%i %i setcloop" TERMINATOR, regulatorMode, (axisid));
        status = pC_->writeController();
        
        // reinit so the closed-loop setting takes effect (this powers on the motor)
        sprintf(pC_->outString_, "%i init" TERMINATOR, (axisid));
        status = pC_->writeController();

        // a delay is required after the init command is sent
        epicsThreadSleep(0.2);
      } else {
        // disable closed-loop control
        sprintf(pC_->outString_, "%i motoroff" TERMINATOR, (axisid));
        status = pC_->writeController();
      }
      
      break;

    default:
      // Do nothing
      break;
  }
  
  return status;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus SMCpolluxAxis::poll(bool *moving)
{ 
  int done;
  int driveOn;
  int lowLimit;
  int highLimit;
  int ignoreLowLimit;
  int ignoreHighLimit;
  int axisStatus=-1;
  double position=0.0;
  asynStatus comStatus;

  static const char *functionName = "SMCpolluxAxis::poll";

  // Read the current motor position
  sprintf(pC_->outString_, "%i np" TERMINATOR, (axisid));
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is a double
  position = atof( (char *) &pC_->inString_);
  setDoubleParam(pC_->motorPosition_, (position / axisRes_) );
  setDoubleParam(pC_->motorEncoderPosition_, (position / axisRes_) );
  
  // Read the status of this motor
  sprintf(pC_->outString_, "%i nst" TERMINATOR, (axisid));
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is an int
  axisStatus = atoi( (char *) &pC_->inString_);
  
  // Check the moving bit
  done = !(axisStatus & 0x1);
  setIntegerParam(pC_->motorStatusDone_, done);
  setIntegerParam(pC_->motorStatusMoving_, !done);
  *moving = done ? false:true;

  // Read the commanded velocity and acceleration
  sprintf(pC_->outString_, "%i gnv" TERMINATOR, (axisid));
  comStatus = pC_->writeReadController();
  
  sprintf(pC_->outString_, "%i gna" TERMINATOR, (axisid));
  comStatus = pC_->writeReadController();

  // Check the limit bit (0x40)
  if (axisStatus & 0x40)
  {
    asynPrint(this->pasynUser_, ASYN_TRACEIO_DRIVER, 
      "%s: axis %i limit indicator active.\n",
      functionName, (axisid));
      
    // query limits?
  }

  // Check the e-stop bit (0x80)
  if (axisStatus & 0x80)
  {
    asynPrint(this->pasynUser_, ASYN_TRACEIO_DRIVER, 
      "%s: axis %i emergency stopped.\n",
      functionName, (axisid));
  }
  
  // Check the e-stop switch active bit (0x200)
  if (axisStatus & 0x200)
  {
    asynPrint(this->pasynUser_, ASYN_TRACEIO_DRIVER, 
      "%s: axis %i emergency stop switch active.\n",
      functionName, (axisid));
    setIntegerParam(pC_->motorStatusProblem_, 1);
  }
  else{
    setIntegerParam(pC_->motorStatusProblem_, 0);
  }

  // Check the device busy bit (0x400)
  if (axisStatus & 0x400)
  {
    asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, 
      "%s: axis %i device is busy - move commands discarded.\n",
      functionName, (axisid));
  }

  // Read the limit status
  // Note: calibration switch = low limit; range measure switch = high limit
  // also need to read the switch confiruation to see if limits are ignored"

  // Read switch confiruation
  // Bit 0:	polarity (0 = NO, 1 = NC)
  // Bit 1:	mask (0 = enabled, 1 = disabled)
  sprintf(pC_->outString_, "%i getsw" TERMINATOR, (axisid));
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  sscanf(pC_->inString_, "%i %i" TERMINATOR, &lowLimitConfig_, &highLimitConfig_);
  ignoreLowLimit = lowLimitConfig_ & 0x2;
  ignoreHighLimit = highLimitConfig_ & 0x2;
  
  // Read status of switches 0=inactive 1=active
  sprintf(pC_->outString_, "%i getswst" TERMINATOR, (axisid));
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is of the form "0 0"
  sscanf(pC_->inString_, "%i %i" TERMINATOR, &lowLimit, &highLimit);
  //
  if (ignoreLowLimit)
    setIntegerParam(pC_->motorStatusLowLimit_, 0);
  else
    setIntegerParam(pC_->motorStatusLowLimit_, lowLimit);

  if (ignoreHighLimit)
    setIntegerParam(pC_->motorStatusHighLimit_, 0);
  else
    setIntegerParam(pC_->motorStatusHighLimit_, highLimit);

  /*setIntegerParam(pC_->motorStatusAtHome_, limit);*/

  // Check the drive power bit (0x100)
  driveOn = (axisStatus & 0x100) ? 0 : 1;
  setIntegerParam(pC_->motorStatusPowerOn_, driveOn);
  setIntegerParam(pC_->motorStatusProblem_, 0);

  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

