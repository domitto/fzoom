TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = fzoom
# fzoom.dbd will be created and installed
DBD += fzoom.dbd

# fzoom.dbd will be made up from these files:
fzoom_DBD += base.dbd

# Include dbd files from all support applications:
#fzoom_DBD += xxx.dbd

# Add all the support libraries needed by this IOC
#fzoom_LIBS += xxx

# fzoom_registerRecordDeviceDriver.cpp derives from fzoom.dbd
fzoom_SRCS += fzoom_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
fzoom_SRCS_DEFAULT += fzoomMain.cpp
fzoom_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#fzoom_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
fzoom_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

