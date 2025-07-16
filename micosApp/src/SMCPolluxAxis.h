#ifndef __SMCpollux_AXIS__
#define __SMCpollux_AXIS__

#include <cstdlib>
#include <cmath>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

class epicsShareClass SMCpolluxAxis : public asynMotorAxis
{
  protected:
  int axisid;
  std::string version;
public:
  /* These are the methods we override from the base class */
  SMCpolluxAxis(class SMCpolluxController *pC, int axis,int physaddr);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setClosedLoop(bool closedLoop);
  asynStatus changeResolution(double newResolution);

private:
  SMCpolluxController *pC_;          /* Pointer to the asynMotorController to which this axis belongs.
                                      Abbreviated because it is used very frequently */
  asynStatus sendAccelAndVelocity(double accel, double velocity);
  int motorForm_;
  double pitch_;
  int polePairs_;
  double clPeriod_;
  double axisRes_;
  double mres_;
  int lowLimitConfig_;
  int highLimitConfig_;
  double posTravelLimit_;
  double negTravelLimit_;

friend class SMCpolluxController;
};

#endif
