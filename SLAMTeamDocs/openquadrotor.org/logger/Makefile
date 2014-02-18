-include ../global.mk

OBJS= playerutils.o
APPS= logger player logcleaner


LDFLAGS+= -lqc_utils_global -lqc_ipcInterfaces -lqc_ipc
CPPFLAGS+= -DFSLINE 


-include ../build_tools/Makefile.generic-shared-object

