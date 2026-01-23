# motorMicos
EPICS motor drivers for the following [Micos](https://www.pi-usa.us) controllers: 
- MoCo dc controller
- SMC Hydra controller
- SMC Corvus controller
- SMC Pollux controller
- SMC Taurus controller

[![Build Status](https://github.com/epics-motor/motorMicos/actions/workflows/ci-scripts-build.yml/badge.svg)](https://github.com/epics-motor/motorMicos/actions/workflows/ci-scripts-build.yml)
<!--[![Build Status](https://travis-ci.org/epics-motor/motorMicos.png)](https://travis-ci.org/epics-motor/motorMicos)-->

motorMicos is a submodule of [motor](https://github.com/epics-modules/motor).  When motorMicos is built in the ``motor/modules`` directory, no manual configuration is needed.

motorMicos can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorMicos contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.

## SMC Pollux Controller

The SMC Pollux controller driver supports multiple axes with configurable axis mapping and includes comprehensive debug capabilities.

### Creating a Controller

```
SMCpolluxCreateController(portName, asynPort, numAxes, movingPollPeriod, idlePollPeriod, axisMap, debugLevel)
```

**Parameters:**
- `portName` - The name of the asyn motor port to be created
- `asynPort` - The name of the drvAsynIPPort that connects to the controller
- `numAxes` - Number of axes (1-16)
- `movingPollPeriod` - Poll period in ms when any axis is moving
- `idlePollPeriod` - Poll period in ms when all axes are idle
- `axisMap` - Comma-separated string mapping asyn addresses to physical axis IDs (e.g., "1,2,3,4,5,6")
- `debugLevel` - Optional debug output level (default: 0)
  - **0** = Off (no debug output)
  - **1** = Errors only
  - **2** = Status every 5 seconds (position, status, moving state)
  - **3** = Commands and responses (with axis ID)
  - **4** = Verbose (includes level 3 + polling details)

**Example:**
```
# Create controller with 6 axes, status output every 5s
drvAsynIPPortConfigure("polluxPort", "192.168.1.100:4001", 0, 0, 1)
SMCpolluxCreateController("pollux1", "polluxPort", 6, 100, 500, "1,2,3,4,5,6", 2)
```

### Debug Output Examples

**Level 2** (Status every 5s):
```
[Status] Axis 1: pos=2.0000, status=0x40, moving=0, done=1, driveOn=1
[Status] Axis 2: pos=3.3000, status=0x40, moving=0, done=1, driveOn=1
```

**Level 3** (Commands):
```
[Pollux CMD Axis 1] >> 1 npos
[Pollux RESP Axis 1] << 2.0000
[Pollux CMD Axis 2] >> 2 nst
[Pollux RESP Axis 2] << 64
```

**Level 4** (Verbose):
```
[Pollux CMD Axis 1] >> 1 npos
[Pollux RESP Axis 1] << 2.0000
Axis 1: Poll #10, pos=2.0000, moving=0
```
