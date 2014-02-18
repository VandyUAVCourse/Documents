-include ../global.mk

OBJS= laserutils.o 
APPS= hokuyo_urg hokuyo_urg_test hokuyo_urg_timestamp_test
COBJS= urg_device_driver.o

LDFLAGS+= -lqc_utils_global -lqc_ipc -lqc_ipcInterfaces
CPPFLAGS+= -DFSLINE 


-include ../build_tools/Makefile.generic-shared-object

