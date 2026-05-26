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

#include <stdio.h>
#include <iocsh.h>
#include <epicsThread.h>
#include <asynOctetSyncIO.h>
#include <epicsExport.h>

#include "SMCPolluxDriver.h"

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

static SMCpolluxController* pSMCpolluxController=NULL;
/** Creates a new SMCpolluxController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SMCpolluxPortName  The name of the drvAsynSerialPort that was created previously to connect to the SMC pollux controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  * \param[in] map    asyn to physical axis map if 0 takes asyn=physical

  */
SMCpolluxController::SMCpolluxController(const char *portName, const char *SMCpolluxPortName, int numAxes, 
                                 double movingPollPeriod, double idlePollPeriod, int* axismap, int debugLevel, const char *configFile)
  :  asynMotorController(portName, numAxes, NUM_SMCpollux_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0),  // Default priority and stack size
     debugLevel_(debugLevel)
{
  asynStatus status;
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
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
    "creating asyn pollux controller naxis %d map 0x%p polling period %f idleperiod %f\n",numAxes,axismap,movingPollPeriod,idlePollPeriod);

  for (int ax=0; ax<numAxes; ax++) {
    int phys=ax+1;
    if (axismap!=NULL) {
      phys=axismap[ax];
    }

    axis[ax] = new SMCpolluxAxis(this, ax,phys);
  }

  // Load and apply axis configuration file if provided
  if (configFile != NULL && configFile[0] != '\0') {
    FILE *fp = fopen(configFile, "r");
    if (fp == NULL) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
        "%s: cannot open config file '%s'\n", functionName, configFile);
    } else {
      char lineBuf[256];
      while (fgets(lineBuf, sizeof(lineBuf), fp) != NULL) {
        // Strip trailing newline/carriage-return
        size_t len = strlen(lineBuf);
        while (len > 0 && (lineBuf[len-1] == '\n' || lineBuf[len-1] == '\r'))
          lineBuf[--len] = '\0';

        // Skip blank lines and comment lines starting with '#'
        if (len == 0 || lineBuf[0] == '#') continue;

        // Find the '%' placeholder that represents the axis number
        char *pct = strchr(lineBuf, '%');
        if (pct == NULL) continue;  // no axis placeholder — skip

        // Everything after '%' (plus optional space) is the command verb
        // Build the command for each axis: "<physaddr> <verb>"
        char *verb = pct + 1;
        // skip optional leading space after '%'
        while (*verb == ' ') verb++;

        for (int ax=0; ax<numAxes; ax++) {
          int phys = ax + 1;
          if (axismap != NULL) phys = axismap[ax];

          // Build the part before '%' (may contain a numeric literal like "1.000")
          // Replace '%' with the physical axis number
          char cmdBuf[256];
          int prefixLen = (int)(pct - lineBuf);
          // trim trailing space from prefix
          while (prefixLen > 0 && lineBuf[prefixLen-1] == ' ') prefixLen--;

          if (prefixLen > 0) {
            // Line of the form: "<value> % <verb>"  →  "<value> <phys> <verb>"
            snprintf(cmdBuf, sizeof(cmdBuf), "%.*s %d %s\n", prefixLen, lineBuf, phys, verb);
          } else {
            // Line of the form: "% <verb>"  →  "<phys> <verb>"
            snprintf(cmdBuf, sizeof(cmdBuf), "%d %s\n", phys, verb);
          }

          strncpy(outString_, cmdBuf, sizeof(outString_)-1);
          outString_[sizeof(outString_)-1] = '\0';

          if (debugLevel_ >= 2) {
            printf("[Config] Axis %d: %s", phys, cmdBuf);
          }

          writeController();
          // Small delay so controller can process each command
          epicsThreadSleep(0.05);
        }
      }
      fclose(fp);
    }
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

/** Wrapper for writeController with debug logging */
asynStatus SMCpolluxController::writeController()
{
  asynStatus status;
  
  if (debugLevel_ >= 3) {
    // Try to extract axis ID from command (usually first token)
    int axisId = -1;
    sscanf(outString_, "%d", &axisId);
    if (axisId > 0) {
      printf("[Pollux CMD Axis %d] >> %s", axisId, outString_);
    } else {
      printf("[Pollux CMD] >> %s", outString_);
    }
  }
  
  status = asynMotorController::writeController();
  
  if (status != asynSuccess && debugLevel_ >= 1) {
    int axisId = -1;
    sscanf(outString_, "%d", &axisId);
    if (axisId > 0) {
      printf("[Pollux ERROR Axis %d] Failed to send command: %s", axisId, outString_);
    } else {
      printf("[Pollux ERROR] Failed to send command: %s", outString_);
    }
  }
  
  return status;
}

/** Wrapper for writeReadController with debug logging */
asynStatus SMCpolluxController::writeReadController()
{
  asynStatus status;
  
  if (debugLevel_ >= 3) {
    // Try to extract axis ID from command (usually first token)
    int axisId = -1;
    sscanf(outString_, "%d", &axisId);
    if (axisId > 0) {
      printf("[Pollux CMD Axis %d] >> %s", axisId, outString_);
    } else {
      printf("[Pollux CMD] >> %s", outString_);
    }
  }
  
  status = asynMotorController::writeReadController();
  
  if (debugLevel_ >= 3) {
    int axisId = -1;
    sscanf(outString_, "%d", &axisId);
    if (axisId > 0) {
      printf("[Pollux RESP Axis %d] << %s", axisId, inString_);
    } else {
      printf("[Pollux RESP] << %s", inString_);
    }
  }
  
  if (status != asynSuccess && debugLevel_ >= 1) {
    int axisId = -1;
    sscanf(outString_, "%d", &axisId);
    if (axisId > 0) {
      printf("[Pollux ERROR Axis %d] Command failed: %s\n", axisId, outString_);
    } else {
      printf("[Pollux ERROR] Command failed: %s\n", outString_);
    }
  }
  
  return status;
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
                                   int movingPollPeriod, int idlePollPeriod, int *axmap, int debugLevel, const char *configFile)
{
  pSMCpolluxController = new SMCpolluxController(portName, SMCpolluxPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000., axmap, debugLevel, configFile);
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
static const iocshArg SMCpolluxCreateControllerArg5 = {"Axis map (comma separated)", iocshArgString};
static const iocshArg SMCpolluxCreateControllerArg6 = {"Debug level (0=off, 1=errors, 2=status every 5s, 3=commands, 4=verbose)", iocshArgInt};
static const iocshArg SMCpolluxCreateControllerArg7 = {"Config file (optional, e.g. MP21.init.txt)", iocshArgString};

static const iocshArg * const SMCpolluxCreateControllerArgs[] = {&SMCpolluxCreateControllerArg0,
                                                             &SMCpolluxCreateControllerArg1,
                                                             &SMCpolluxCreateControllerArg2,
                                                             &SMCpolluxCreateControllerArg3,
                                                             &SMCpolluxCreateControllerArg4,
                                                             &SMCpolluxCreateControllerArg5,
                                                             &SMCpolluxCreateControllerArg6,
                                                             &SMCpolluxCreateControllerArg7
                                                            };
static const iocshFuncDef SMCpolluxCreateControllerDef = {"SMCpolluxCreateController", 8, SMCpolluxCreateControllerArgs};
static void SMCpolluxCreateControllerCallFunc(const iocshArgBuf *args)
{
  int map[MAX_SMCpollux_AXES]={0};
  int numAxes = args[2].ival;
  int i = 0;
  char *token=NULL;
  int debugLevel = (args[6].ival >= 0) ? args[6].ival : 0;  // Default to 0 if not specified
  const char *configFile = (args[7].sval != NULL && args[7].sval[0] != '\0') ? args[7].sval : NULL;
  
  if (args[5].sval !=NULL && *args[5].sval!=0){
    char *input = strdup(args[5].sval);
    token = strtok(input, ",");
    while (token && i < numAxes) {
      map[i++] = atoi(token);
      token = strtok(NULL, ",");
    }
    free(input);
  }
  SMCpolluxCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, (i==0)?NULL:map, debugLevel, configFile);
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
