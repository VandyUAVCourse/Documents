-include ../global.mk

COBJS+= ipc.o queryResponse.o marshall.o timer.o list.o behaviors.o \
        tcModError.o datamsg.o formatters.o hash.o idtable.o key.o \
        primFmttrs.o reg.o sendMsg.o tcaMem.o tcaRef.o comModule.o com.o \
        globalM.o globalMUtil.o strList.o modLogging.o modVar.o resMod.o \
        parseFmttrs.o lex.o printData.o comServer.o dispatch.o msgTap.o \
        recvMsg.o res.o tcerror.o logging.o globalS.o centralIO.o \
        globalVar.o multiThread.o

APPS+= central 


LDFLAGS+= 
CPPFLAGS+= -DFSLINE -Wall -Wstrict-prototypes -Wnested-externs -Wmissing-prototypes -Wmissing-declarations -O3 -DREDHAT_52 -DREDHAT_6 -DREDHAT_71 -g -fPIC


-include ../build_tools/Makefile.generic-shared-object

