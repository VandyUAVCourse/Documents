-include ../global.mk

OBJS= ipcParamReader.o
COBJS= 
APPS= configSender configSender_test ipcParamReader_test


LDFLAGS+= -lqc_ipcInterfaces -lqc_utils_global -lqc_ipc 
CPPFLAGS+= -DFSLINE 


-include ../build_tools/Makefile.generic-shared-object

