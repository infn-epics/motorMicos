#!../../bin/linux-x86_64/micos

< envPaths


## Register all support components
dbLoadDatabase "../../dbd/micos.dbd"
micos_registerRecordDeviceDriver pdbbase


#epicsEnvSet(P, "EUAPS:FPMMIR")
epicsEnvSet(P, "EUAPS:CTRL:FPRCK1:W:CTR")
## motorUtil (allstop & alldone)
# dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P):")

##
drvAsynIPPortConfigure("polluxPort","scflameprmoxa001.lnf.infn.it:4001", 0, 0, 1)
#  {P,       M,      DTYP,    PORT,   ADDR,  DESC,	EGU,  DIR,  VMAX,  VELO,  VBAS,  ACCL,  BDST,     BVEL,  BACC,  MRES,   PREC,  DHLM,       DLLM,   INIT}
#  {micos:,  m3, "asynMotor", pollux1,  0,    "m3",   degrees,  Pos,  15.,   3.,    .05,    .5,    0,       1.0,    2,	0.01,     2,   175,       -175,     ""}
asynReport 1
#asynSetTraceMask("polluxPort",0,0x3F)
#asynSetTraceIOMask("polluxPort",0,0x3F)
#asynSetTraceMask("pollux1",0,0x3F)
#asynSetTraceIOMask("pollux1",0,0x3F)

#SMCpolluxCreateController("pollux1", "polluxPort", 6, 100, 500,"1,2,3,4,5,6", 3) ## Debug: 0=off, 1=errors, 2=status every 5s, 3=commands, 4=verbose
SMCpolluxCreateController("pollux1", "polluxPort", 2, 100, 500,"1,2", 3, "/path/to/MP21.init.txt") ## Debug: 0=off, 1=errors, 2=status every 5s, 3=commands, 4=verbose

# asynSetTraceMask("pollux1",0,0x3F)
# asynSetTraceIOMask("pollux1",0,0x3F)

# dbLoadRecords("/epics/support/motor/db/asyn_motor.db","P=$(P),M=:MOT001,DTYP=asynMotor,PORT=pollux1,ADDR=0,DIR=Pos,DESC=Motor001,EGU=mm,VMAX=5.,VELO=1.,VBAS=.5,ACCL=.5,BDST=0,BVEL=1.0,BACC=2,MRES=0.0001,PREC=4,DHLM=12,DLLM=0,INIT=")
# dbLoadRecords("/epics/support/motor/db/asyn_motor.db","P=$(P),M=:MOT002,DTYP=asynMotor,PORT=pollux1,ADDR=1,DIR=Pos,DESC=Motor002,EGU=mm,VMAX=5.,VELO=1.,VBAS=.5,ACCL=.5,BDST=0,BVEL=1.0,BACC=2,MRES=0.0001,PREC=4,DHLM=12,DLLM=0,INIT=")
# dbLoadRecords("/epics/support/motor/db/asyn_motor.db","P=$(P),M=:MOT003,DTYP=asynMotor,PORT=pollux1,ADDR=2,DIR=Pos,DESC=Motor003,EGU=mm,VMAX=5.,VELO=1.,VBAS=.5,ACCL=.5,BDST=0,BVEL=1.0,BACC=2,MRES=0.0001,PREC=4,DHLM=12,DLLM=0,INIT=")
# dbLoadRecords("/epics/support/motor/db/asyn_motor.db","P=$(P),M=:MOT004,DTYP=asynMotor,PORT=pollux1,ADDR=3,DIR=Pos,DESC=Motor004,EGU=mm,VMAX=5.,VELO=1.,VBAS=.5,ACCL=.5,BDST=0,BVEL=1.0,BACC=2,MRES=0.0001,PREC=4,DHLM=12,DLLM=0,INIT=")
# dbLoadRecords("/epics/support/motor/db/asyn_motor.db","P=$(P),M=:MOT005,DTYP=asynMotor,PORT=pollux1,ADDR=4,DIR=Pos,DESC=Motor005,EGU=mm,VMAX=5.,VELO=1.,VBAS=.5,ACCL=.5,BDST=0,BVEL=1.0,BACC=2,MRES=0.0001,PREC=4,DHLM=12,DLLM=0,INIT=")
# dbLoadRecords("/epics/support/motor/db/asyn_motor.db","P=$(P),M=:MOT006,DTYP=asynMotor,PORT=pollux1,ADDR=5,DIR=Pos,DESC=Motor006,EGU=mm,VMAX=5.,VELO=1.,VBAS=.5,ACCL=.5,BDST=0,BVEL=1.0,BACC=2,MRES=0.0001,PREC=4,DHLM=12,DLLM=0,INIT=")

dbLoadRecords("/epics/support/motor/db/asyn_motor.db","P=$(P),M=:MOT001,DTYP=asynMotor,PORT=pollux1,ADDR=0,DIR=Pos,DESC=Motor001,EGU=mm,VMAX=5.,VELO=1.,VBAS=.5,ACCL=.5,BDST=0,BVEL=1.0,BACC=2,MRES=1,PREC=4,DHLM=12,DLLM=0,INIT=")
dbLoadRecords("/epics/support/motor/db/asyn_motor.db","P=$(P),M=:MOT002,DTYP=asynMotor,PORT=pollux1,ADDR=1,DIR=Pos,DESC=Motor002,EGU=mm,VMAX=5.,VELO=1.,VBAS=.5,ACCL=.5,BDST=0,BVEL=1.0,BACC=2,MRES=1,PREC=4,DHLM=12,DLLM=0,INIT=")

# dbLoadRecords("/epics/support/motor/db/asyn_motor.db","P=$(P),M=:MOT001,DTYP=asynMotor,PORT=pollux1,ADDR=0,DIR=Pos,DESC=Motor001,EGU=mm,VMAX=5.,VELO=1.,VBAS=.5,ACCL=.5,BDST=0,BVEL=1.0,BACC=2,MRES=0.0001,PREC=4,DHLM=12,DLLM=0,INIT=")
# dbLoadRecords("/epics/support/motor/db/asyn_motor.db","P=$(P),M=:MOT002,DTYP=asynMotor,PORT=pollux1,ADDR=1,DIR=Pos,DESC=Motor002,EGU=mm,VMAX=5.,VELO=1.,VBAS=.5,ACCL=.5,BDST=0,BVEL=1.0,BACC=2,MRES=0.0001,PREC=4,DHLM=12,DLLM=0,INIT=")


iocInit

## motorUtil (allstop & alldone)
# motorUtilInit("$(P)")