CPPFLAGS+=

ifeq ($(DEBUG), 1)
CXXFLAGS += -g -O0 -Wall
CFLAGS   += -g -O0 -Wall
else
#CXXFLAGS += -O3 -Wall -ffast-math -fno-inline
CXXFLAGS += -DNDEBUG -O3 -Wall -ffast-math
CFLAGS   += -DNDEBUG -O3 -Wall -ffast-math
endif

PROFILE= false
