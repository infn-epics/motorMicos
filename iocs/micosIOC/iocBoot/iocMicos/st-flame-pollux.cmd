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
drvAsynIPPortConfigure("polluxPort","192.168.190.64:4001", 0, 0, 0)
#  {P,       M,      DTYP,    PORT,   ADDR,  DESC,	EGU,  DIR,  VMAX,  VELO,  VBAS,  ACCL,  BDST,     BVEL,  BACC,  MRES,   PREC,  DHLM,       DLLM,   INIT}
#  {micos:,  m3, "asynMotor", pollux1,  0,    "m3",   degrees,  Pos,  15.,   3.,    .05,    .5,    0,       1.0,    2,	0.01,     2,   175,       -175,     ""}
asynReport 1
#asynSetTraceMask("polluxPort",0,0x3F)
#asynSetTraceIOMask("polluxPort",0,0x3F)
#asynSetTraceMask("pollux1",0,0x3F)
#asynSetTraceIOMask("pollux1",0,0x3F)
SMCpolluxCreateController("pollux1", "polluxPort", 2, 1, 1,"7,8") ## last argument is the map between asyn addr and physical axis id 0->7 1->8
# asynSetTraceMask("pollux1",0,0x3F)
# asynSetTraceIOMask("pollux1",0,0x3F)

dbLoadRecords("/epics/support/motor/db/asyn_motor.db","P=$(P),M=:m0,DTYP=asynMotor,PORT=pollux1,ADDR=0,DIR=Pos,DESC=M0,EGU=degrees,VMAX=5.,VELO=3.,VBAS=.5,ACCL=.5,BDST=0,BVEL=1.0,BACC=2,MRES=001,PREC=2,DHLM=12,DLLM=0,INIT=")
dbLoadRecords("/epics/support/motor/db/asyn_motor.db","P=$(P),M=:m1,DTYP=asynMotor,PORT=pollux1,ADDR=1,DIR=Pos,DESC=M1,EGU=degrees,VMAX=5.,VELO=3.,VBAS=.5,ACCL=.5,BDST=0,BVEL=1.0,BACC=2,MRES=001,PREC=2,DHLM=12,DLLM=0,INIT=")

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("$(P)")