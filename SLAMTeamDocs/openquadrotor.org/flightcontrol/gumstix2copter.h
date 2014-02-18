#ifndef _GUMSTIX_TO_COPTER_H_
#define _GUMSTIX_TO_COPTER_H_

#include <iostream>
#include <string>
#include <ipcMessages/qc_flightcontrol_messages.h>
#include <libserial-0.5.2/SerialStream.h>
#include <cstdlib>
using namespace LibSerial ;
using namespace std;

///ROLL PITC +- 10

#define MAX_ROLL 40 
#define MIN_ROLL -40 
#define MAX_PITCH 40
#define MIN_PITCH -40
#define MAX_YAW 40
#define MIN_YAW -40
#define MAX_THRUST 25
#define MIN_THRUST -25


class Gumstix2Copter {
	public:
		Gumstix2Copter(string device, bool debug=false);
		bool sendCommand(gumstix_2_copter_message msg);
		void close();
	private:
		SerialStream fd;
		bool debug;
};

#endif // _GUMSTIX_TO_COPTER_H_
