TOP=../..

include $(TOP)/configure/CONFIG

# ------------------------
# Build an IOC Application
# ------------------------

PROD_IOC = fzoomExample

# fzoomExample.dbd will be installed into <top>/dbd
DBD += fzoomExample.dbd

# fzoomExample.dbd will be created from these files
fzoomExample_DBD += base.dbd
fzoomExample_DBD += asyn.dbd
fzoomExample_DBD += drvAsynSerialPort.dbd
fzoomExample_DBD += drvAsynIPPort.dbd
fzoomExample_DBD += fzoomSupport.dbd

# fzoomExample_registerRecordDeviceDriver.cpp will be created
# fzoomExample.dbd
fzoomExample_SRCS += fzoomExample_registerRecordDeviceDriver.cpp

# These two lines are needed for non-vxWorks builds, such as Linux
fzoomExample_SRCS_DEFAULT += fzoomExampleMain.cpp
fzoomExample_SRCS_vxWorks += -nil-

# Add locally compiled object code
#fzoomExample_SRCS +=

# The following adds object code from base/src/vxWorks
fzoomExample_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

fzoomExample_LIBS += asyn
fzoomExample_LIBS += fzoom

# We need to link this IOC Application against the EPICS Base libraries
fzoomExample_LIBS += $(EPICS_BASE_IOC_LIBS)

DATA += asynRecordGui
DATA += fzoomExampleGui

include $(TOP)/configure/RULES
