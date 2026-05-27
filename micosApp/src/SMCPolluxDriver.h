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
  SMCpolluxAxis* axis[MAX_SMCpollux_AXES] = {nullptr};
public:
  SMCpolluxController(const char *portName, const char *SMCpolluxPortName, int numAxes, double movingPollPeriod, double idlePollPeriod, int *map=NULL, int debugLevel=0, const char *configFile=NULL);

  void report(FILE *fp, int level);
  SMCpolluxAxis* getAxis(asynUser *pasynUser);
  SMCpolluxAxis* getAxis(int axisNo);
  asynStatus changeResolution(int axisNo, double newResolution);
  void setSendVelAccel(int enable) { sendVelAccel_ = enable ? 1 : 0; }
  asynStatus writeController();
  asynStatus writeReadController();

protected:
  int SMCpolluxRegulatorMode_;    /** Regulator mode parameter index */
  int debugLevel_;                /** Debug level: 0=off, 1=errors, 2=commands, 3=verbose */
  int sendVelAccel_;              /** 1=send velocity/accel on every move (default), 0=use controller-initialized values */

friend class SMCpolluxAxis;
};

#endif

