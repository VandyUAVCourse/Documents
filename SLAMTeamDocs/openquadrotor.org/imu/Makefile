-include ../global.mk

OBJS= MTComm.o xsensDriver.o imubuffer.o
APPS= xsens_imu xsens_imu_test imubuffer_test


LDFLAGS+= -lqc_utils_global -lqc_ipcInterfaces -lqc_ipc
CPPFLAGS+= -DFSLINE 


-include ../build_tools/Makefile.generic-shared-object

