-include ../global.mk

OBJS= localizemap.o motionmodel.o localizer.o pointmap.o 

APPS= localize_test pgm2lmap clf2lmap clfmerge clffilter localization



LDFLAGS  += -lm -lqc_math_stuff -lqc_laser -lqc_utils_global -lqc_ipc -lqc_ipcInterfaces -lqc_ipc -lqc_cmatcher
-include ../build_tools/Makefile.generic-shared-object

