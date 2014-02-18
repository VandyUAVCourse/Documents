-include ../global.mk

OBJS= gumstix2copter.o 
APPS= flightcontrol_reader flightcontrol_reader_test offboard_controler


LDFLAGS+= -lqc_laser -lqc_imu -lqc_utils_global -lqc_libserial-0.5.2 -lqc_ipcInterfaces -lqc_configSender -lqc_ipc -lqc_logger
CPPFLAGS+= -DFSLINE 


-include ../build_tools/Makefile.generic-shared-object

