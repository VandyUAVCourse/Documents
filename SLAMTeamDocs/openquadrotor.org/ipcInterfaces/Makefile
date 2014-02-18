-include ../global.mk

OBJS= qc_ipc_interface.o qc_laser_interface.o qc_imu_interface.o qc_odometry_interface.o 
OBJS+= qc_flightcontrol_interface.o qc_config_interface.o
APPS= 
COBJS=  

LDFLAGS+= -lqc_ipc
CPPFLAGS+= -DFSLINE 


-include ../build_tools/Makefile.generic-shared-object

