#!../../bin/darwin-x86/fzoomExample

## You may have to change fzoomExample to something else
## everywhere it appears in this file

cd "../.."

# Load binaries on architectures that need to do so.
# VXWORKS_ONLY, LINUX_ONLY and RTEMS_ONLY are macros that resolve
# to a comment symbol on architectures that are not the current
# build architecture, so they can be used liberally to do architecture
# specific things. Alternatively, you can include an architecture
# specific file.
#$(VXWORKS_ONLY)ld < bin/$(ARCH)/fzoomExample.munch

## Register all support components
dbLoadDatabase("dbd/fzoomExample.dbd")
fzoomExample_registerRecordDeviceDriver(pdbbase)

# portName, hostInfo, priority, noAutoConnect, noProcessEos
#drvAsynIPPortConfigure("P0", "172.23.241.5:7025", 0,0,1)
drvAsynSerialPortConfigure("P0", "/dev/ttyUSB0", 0,0,1)
asynSetOption("P0", 0, "baud",    "9600")
asynSetOption("P0", 0, "bits",    "8")
asynSetOption("P0", 0, "parity",  "none")
asynSetOption("P0", 0, "stop",    "2")

drvFzoomAsynConfigure("FZOOM0","P0") 

## Load record instances

dbLoadRecords("db/fzoomExample.db")
dbLoadRecords("db/asynRecord.db","P=RJQ35657-TEST-01:,R=ASYN,PORT=P0,ADDR=0,OMAX=0,IMAX=200")

## Set this to see messages from mySub
#mySubDebug 1

iocInit()

asynSetTraceMask("FZOOM0", 0, 0xFF)
asynSetTraceIOMask("FZOOM0", 0, 0xFF)
asynSetTraceMask("P0", 0, 0xFF)
asynSetTraceIOMask("P0", 0, 0xFF)
## Start any sequence programs
#seq sncExample,"user=xxx"
