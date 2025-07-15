/*
FILENAME...   SMCpolluxDriver.h
USAGE...      Motor driver support for the Micos SMC hydra controller.

*/

#ifndef __SMCpollux__
#define __SMCpollux__

#include "SMCPolluxAxis.h"

#define MAX_SMCpollux_AXES 16

// Controller-specific parameters
#define NUM_SMCpollux_PARAMS 1

/** drvInfo strings for extra parameters that the SMC Hydra controller supports */
#define SMCpolluxRegulatorModeString "SMCpollux_REGULATOR_MODE"

class epicsShareClass SMCpolluxController : public asynMotorController {
public:
  SMCpolluxController(const char *portName, const char *SMCpolluxPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  SMCpolluxAxis* getAxis(asynUser *pasynUser);
  SMCpolluxAxis* getAxis(int axisNo);
  asynStatus changeResolution(int axisNo, double newResolution);

protected:
  int SMCpolluxRegulatorMode_;    /** Regulator mode parameter index */

friend class SMCpolluxAxis;
};

#endif

