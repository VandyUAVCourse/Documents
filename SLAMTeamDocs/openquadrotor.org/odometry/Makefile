-include ../global.mk

OBJS= lasertransformator.o
APPS= lasertransformator_test odometry_laser_imu_sender odometry_laser_imu_sender_test velocity_sender
COBJS=  

LDFLAGS+= -lqc_math_stuff -lqc_laser -lqc_imu -lqc_cmatcher -lqc_configSender
CPPFLAGS+= -DFSLINE 
#for lasertransformator_test
LDFLAGS+= $(QT_LIB) -L$(QGLVIEWER_LIB) -lGLU -lGL -lQGLViewer
CPPFLAGS+= $(QT_INCLUDE)


-include ../build_tools/Makefile.generic-shared-object

