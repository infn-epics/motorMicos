/*
FILENAME... SMCpolluxDriver.cpp
USAGE...    Motor driver support for the Micos SMC pollux controller.

Note: This driver was tested with the Micos SMC pollux CM and 
      motor forms 0 (stepper) and 1 (linear).

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>
#include <asynOctetSyncIO.h>
#include <epicsExport.h>

#include "SMCPolluxDriver.h"

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

/** Creates a new SMCpolluxController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SMCpolluxPortName  The name of the drvAsynSerialPort that was created previously to connect to the SMC pollux controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
SMCpolluxController::SMCpolluxController(const char *portName, const char *SMCpolluxPortName, int numAxes, 
                                 double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_SMCpollux_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  SMCpolluxAxis *pAxis;
  static const char *functionName = "SMCpolluxController::SMCpolluxController";

  // Create controller-specific parameters
  createParam(SMCpolluxRegulatorModeString, asynParamInt32, &SMCpolluxRegulatorMode_);

  /* Connect to SMC pollux controller */
  status = pasynOctetSyncIO->connect(SMCpolluxPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to SMC pollux controller\n",
      functionName);
  }
  int map[2]={7,8};
  for (axis=0; axis<numAxes; axis++) {
    
    pAxis = new SMCpolluxAxis(this, axis,map[axis]);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Change the resolution of an axis
  * \param[in] axisNo The index of the axis
  * \param[in] newResolution The new resolution
  */
asynStatus SMCpolluxController::changeResolution(int axisNo, double newResolution)
{
  SMCpolluxAxis* pAxis;
  asynStatus status;
  
  pAxis = this->getAxis(axisNo);
  
  status = pAxis->changeResolution(newResolution);
  
  return status;
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void SMCpolluxController::report(FILE *fp, int level)
{
  fprintf(fp, "SMC pollux motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an SMCpolluxAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
SMCpolluxAxis* SMCpolluxController::getAxis(asynUser *pasynUser)
{
  return static_cast<SMCpolluxAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an SMCpolluxAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
SMCpolluxAxis* SMCpolluxController::getAxis(int axisNo)
{
  return static_cast<SMCpolluxAxis*>(asynMotorController::getAxis(axisNo));
}

/** Creates a new SMCpolluxController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SMCpolluxPortName  The name of the drvAsynIPPPort that was created previously to connect to the SMC pollux controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int SMCpolluxCreateController(const char *portName, const char *SMCpolluxPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  SMCpolluxController *pSMCpolluxController
    = new SMCpolluxController(portName, SMCpolluxPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  pSMCpolluxController = NULL;
  return(asynSuccess);
}

/** Specify a new resolution for an SMC pollux axis.
  * Configuration command, called directly or from iocsh
  * \param[in] SMCpolluxPortName  The name of the drvAsynIPPPort that was created previously to connect to the SMC pollux controller 
  * \param[in] axisNo            Index of the desired axis 
  * \param[in] newResolution     The new resolution of the specified axis
  */
extern "C" int SMCpolluxChangeResolution(const char *SMCpolluxPortName, int axisNo, double newResolution)
{
  SMCpolluxController *pC;
  static const char *functionName = "SMCpolluxChangeResolution";
  
  pC = (SMCpolluxController*) findAsynPortDriver(SMCpolluxPortName);
  if (!pC) {
    printf("SMCpolluxDriver.cpp:%s: Error port %s not found\n",
           functionName, SMCpolluxPortName);
    return asynError;
  }
  
  pC->lock();
  pC->changeResolution(axisNo, newResolution);
  pC->unlock();
  
  return(asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg SMCpolluxCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg SMCpolluxCreateControllerArg1 = {"SMC pollux port name", iocshArgString};
static const iocshArg SMCpolluxCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg SMCpolluxCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg SMCpolluxCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const SMCpolluxCreateControllerArgs[] = {&SMCpolluxCreateControllerArg0,
                                                             &SMCpolluxCreateControllerArg1,
                                                             &SMCpolluxCreateControllerArg2,
                                                             &SMCpolluxCreateControllerArg3,
                                                             &SMCpolluxCreateControllerArg4};
static const iocshFuncDef SMCpolluxCreateControllerDef = {"SMCpolluxCreateController", 5, SMCpolluxCreateControllerArgs};
static void SMCpolluxCreateControllerCallFunc(const iocshArgBuf *args)
{
  SMCpolluxCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static const iocshArg SMCpolluxChangeResolutionArg0 = {"SMC pollux port name", iocshArgString};
static const iocshArg SMCpolluxChangeResolutionArg1 = {"Axis number", iocshArgInt};
static const iocshArg SMCpolluxChangeResolutionArg2 = {"Axis resolution", iocshArgDouble};
static const iocshArg * const SMCpolluxChangeResolutionArgs[] = {&SMCpolluxChangeResolutionArg0,
                                                             &SMCpolluxChangeResolutionArg1,
                                                             &SMCpolluxChangeResolutionArg2};
static const iocshFuncDef SMCpolluxChangeResolutionDef = {"SMCpolluxChangeResolution", 3, SMCpolluxChangeResolutionArgs};
static void SMCpolluxChangeResolutionCallFunc(const iocshArgBuf *args)
{
  SMCpolluxChangeResolution(args[0].sval, args[1].ival, args[2].dval);
}

static void SMCpolluxRegister(void)
{
  iocshRegister(&SMCpolluxCreateControllerDef, SMCpolluxCreateControllerCallFunc);
  iocshRegister(&SMCpolluxChangeResolutionDef, SMCpolluxChangeResolutionCallFunc);
}

extern "C" {
epicsExportRegistrar(SMCpolluxRegister);
}
