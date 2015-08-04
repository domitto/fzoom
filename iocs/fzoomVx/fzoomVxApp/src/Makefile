TOP=../..

#
# test application to run on ioc258
#
include $(TOP)/configure/CONFIG

# ------------------------
# Build an IOC Application
# ------------------------

PROD_IOC_vxWorks = fzoomVx

# fzoomVx.dbd will be installed into <top>/dbd
DBD += fzoomVx.dbd

# fzoomVx.dbd will be created from these files
fzoomVx_DBD += base.dbd
fzoomVx_DBD += asyn.dbd
fzoomVx_DBD += drvAsynSerialPort.dbd
fzoomVx_DBD += drvIpac.dbd
fzoomVx_DBD += DLS8515.dbd
fzoomVx_DBD += utility.dbd
fzoomVx_DBD += fzoomSupport.dbd

# fzoomVx_registerRecordDeviceDriver.cpp will be created
# fzoomVx.dbd
fzoomVx_SRCS += fzoomVx_registerRecordDeviceDriver.cpp

# These two lines are needed for non-vxWorks builds, such as Linux
fzoomVx_SRCS_DEFAULT += fzoomVxMain.cpp
fzoomVx_SRCS_vxWorks += -nil-

# Add locally compiled object code
#fzoomVx_SRCS +=

# The following adds object code from base/src/vxWorks
fzoomVx_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# This line says that this IOC Application depends on the
# xxx Support Module
#fzoomVx_LIBS += xxx

# We need to link this IOC Application against the EPICS Base libraries
fzoomVx_LIBS += Ipac
fzoomVx_LIBS += DLS8515
fzoomVx_LIBS += utility
fzoomVx_LIBS += fzoom
fzoomVx_LIBS += asyn
fzoomVx_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
