#!../../bin/linux-x86_64/micos

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/micos.dbd"
micos_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

epicsEnvSet(P, "EUAPS:DIAG:MICOS")
## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P):")

##
drvAsynIPPortConfigure("hydraPort","192.168.190.64:4001", 0, 0, 0)
#  {P,       M,      DTYP,    PORT,   ADDR,  DESC,	EGU,  DIR,  VMAX,  VELO,  VBAS,  ACCL,  BDST,     BVEL,  BACC,  MRES,   PREC,  DHLM,       DLLM,   INIT}
#  {micos:,  m3, "asynMotor", Hydra1,  0,    "m3",   degrees,  Pos,  15.,   3.,    .05,    .5,    0,       1.0,    2,	0.01,     2,   175,       -175,     ""}

SMCpolluxCreateController("Hydra1", "hydraPort", 2, 100, 500)
#SMChydraCreateController("Hydra1", "hydraPort", 10, 100, 500)

# dbLoadRecords("../../../../db/SMChydraAxis.db","P=$(P):,R=m0,DTYP=asynMotor,PORT=Hydra1,ADDR=0,VAL=0")
# dbLoadRecords("../../../../db/SMChydraAxis.db","P=$(P):,R=m1,DTYP=asynMotor,PORT=Hydra1,ADDR=1,VAL=0")

dbLoadRecords("/epics/support/motor/db/asyn_motor.db","P=$(P),M=m0,DTYP=asynMotor,PORT=Hydra1,ADDR=0,DIR=Pos,DESC=M0,EGU=degrees,VMAX=15.,VELO=3.,VBAS=.05,ACCL=.5,BDST=0,BVEL=1.0,BACC=2,MRES=001,PREC=2,DHLM=175,DLLM=-175,INIT=")
dbLoadRecords("/epics/support/motor/db/asyn_motor.db","P=$(P),M=m1,DTYP=asynMotor,PORT=Hydra1,ADDR=1,DIR=Pos,DESC=M1,EGU=degrees,VMAX=15.,VELO=3.,VBAS=.05,ACCL=.5,BDST=0,BVEL=1.0,BACC=2,MRES=001,PREC=2,DHLM=175,DLLM=-175,INIT=")

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("$(P)")