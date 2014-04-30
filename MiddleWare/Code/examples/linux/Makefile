# This is a work in progress!
CXX		:= g++
CXXFLAGS	:= -g -Wall -Wextra -pedantic-errors
CPPFLAGS	:= -std=c++0x

target		:= exampleSerialWrapper

sources		:= Serial_Wrapper.cpp

objects		:= $(sources:.cpp=.o)
depends		:= $(sources:.cpp=.dep)

LIBS		= -llibserial

$(target): $(objects) 
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $^ -o $@

include $(depends)
