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
SMCpolluxCreateController(portName, asynPort, numAxes, movingPollPeriod, idlePollPeriod, axisMap, debugLevel, configFile)
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
  - **2** = Status every 5 seconds (position, velocity/accel set values, status, moving state)
  - **3** = Commands and responses (with axis ID)
  - **4** = Verbose (includes level 3 + polling details)
- `configFile` - Optional path to an axis initialization file (see [Axis Configuration File](#axis-configuration-file))

**Example:**
```
# Create controller with 2 axes, status output every 5s, with config file
drvAsynIPPortConfigure("polluxPort", "192.168.1.100:4001", 0, 0, 1)
SMCpolluxCreateController("pollux1", "polluxPort", 2, 100, 500, "1,2", 2, "/path/to/MP21.init.txt")
```

### Axis Configuration File

An optional text file can be provided to initialize axis parameters at startup (velocity, acceleration, switch config, etc.). Each line contains a Pollux command where `%` is replaced by the physical axis number for each axis.

**File format:**
```
# Lines starting with # are comments; blank lines are ignored.
# <value> % <command>   sets a value
# % <command>           issues a command with no leading value
1 % setaxis
1.000 % setpitch
5.000000 % snv
50.000 % sna
50.000 % setnstopdecel
```

The file is applied to **all** axes in sequence after the controller and axes are constructed. With `debugLevel >= 2` each sent command is printed.

### Velocity / Acceleration Unit Scaling

Velocity and acceleration are passed from the EPICS motor record in **steps/s** (steps/s²).
The driver converts to controller native units (mm/s, mm/s²) by multiplying by `MRES`:

```
controller_velocity [mm/s] = EPICS_velocity [steps/s] × MRES [mm/step]
```

This ensures correct scaling for any `MRES` value (e.g. `MRES=0.0001` for 0.1 µm resolution).

### Controlling Velocity/Acceleration Sending

By default the driver sends the velocity and acceleration to the controller before every move.
To rely instead on values pre-loaded at startup (from a config file or controller NVM):

```
SMCpolluxSetSendVelAccel("pollux1", 0)   # disable — use initialized values
SMCpolluxSetSendVelAccel("pollux1", 1)   # re-enable (default)
```

This command can be placed in the startup script or issued from the iocsh prompt at any time.
When disabled with `debugLevel >= 2`, the values that *would* have been sent are still logged.

### Debug Output Examples

**Level 2** (Status every 5s + set events):
```
axis 1 startup velocity 5.0000
axis 1 startup acceleration 50.0000
[Set] Axis 1: vel=1.000000 acc=0.500000 (controller vel=0.000100 acc=0.000050, mres=0.000100) sendVelAccel=1
[Status] Axis 1: pos=2.0000, res=0.000100, status=0x40, moving=0, done=1, driveOn=1
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
