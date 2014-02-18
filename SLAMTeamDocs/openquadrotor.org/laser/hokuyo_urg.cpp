#include "urg_device_driver.h"
#include <utils_global/cmdargsreader.h>
#include <ipcInterfaces/qc_ipc_interface.h>
#include <ipcInterfaces/qc_laser_interface.h>
#include <iostream>
#include <string>
#include <math.h>
#include <signal.h>

using namespace std;
HokuyoURG urg;
qc_laser_laser_message glmsg;
volatile int stop = 0;

void add_time(const int s_start, const int us_start, int delta_ms, int* res_s, int* res_us){
	*res_us = (us_start + delta_ms);
	*res_s = s_start + *res_us/ 1000000;
	*res_us = *res_us % 1000000;
}

void sigterm_handler(int sgn __attribute__ ((unused))){
	stop = 1;
}

void errorless(int status, string info){
	if (!status){
		///error
		fprintf(stderr,"ERROR: status of %s is %i - Aborting\n\n",info.c_str(),status);
		exit(1);
	}
}

int main (int argc, char* argv[]){
	signal(SIGINT,sigterm_handler);
	glmsg.ranges = 0;
	CmdArgsReader commandline(argc, argv);
	if (argc < 2){
		std::cerr << "Usage: " << argv[0] << std::endl;
		std::cerr << "                         -d | -device <pathToDevice>  {path to laser}" << std::endl;
		std::cerr << "                         -i | -id <id of the laser> (valid 1 or 2, default:1)" << std::endl;
		exit(1);
	}
	
	///connect to ipc
	if (qc_ipc_connect(argv[0]) <= 0){
		exit(1);
	}
		
	string device = "";
	int laserid = 1;
	commandline.getStringValue("-d","-device",&device);
	commandline.getIntValue("-i","-id",&laserid);
	if (commandline.printUnusedArguments())
		exit(1);
	
  
	errorless(hokuyo_open(&urg,device.c_str()),"hokuyo_open");
	errorless(hokuyo_init(&urg),"hokuyo_init");
	//errorless(hokuyo_reset(&urg),"hokuyo_reset");
	hokuyo_reset(&urg);
	hokuyo_reset(&urg);
	hokuyo_reset(&urg);
	
	///calibrate
	int deltaMS;
	errorless(hokuyo_estimateDeltaT(&urg, &deltaMS),"calibration");
	///
	hokuyo_reset(&urg);
	struct timeval basisTime; ///after adjustment: just add hokuyo time to this one!
	errorless(hokuyo_estimateStartingTime(&urg,&basisTime),"hokuyo_estimateStartingTime");
	///now we have the basis time: to get the actual timestamp: add time of hokuyo to get
	///the estimated local timestamp
	
	///the internal ocsillatior of the hokuyo is not accurate enough.
	///after about 20 sec we have an offset of about 50ms!
	///assuming this bias is linear - try tto estimate the true oscillation of the hokuyo!
	//double hokuyo_time_multiplicator = 0.99718478;
	
	hokuyo_startContinuous(&urg, 44, 725, 0);
	///workaround: max_beams+1 in order to have proper processing of data
	///but only 682 beams will be send over ipc!
	glmsg.ranges = new int [682+1];  
	glmsg.timestamp_hokuyo_init_sec = basisTime.tv_sec;
	glmsg.timestamp_hokuyo_init_sec = basisTime.tv_usec;
	int k = 0;
	while (!stop){
		k++;
		if (!(k % 10))
			std::cerr << "." << std::flush;
		if (hokuyo_getQCLaserMessage(&urg, &glmsg, &deltaMS)){
			if (laserid == 1)
				qc_laser_publish_laser1_message(&glmsg);
			else
				qc_laser_publish_laser2_message(&glmsg);
		}
		IPC_listen(40);
		usleep(10000);
	}
	hokuyo_stopContinuous(&urg);
	hokuyo_close(&urg);
	if (glmsg.ranges){
		delete[] glmsg.ranges;
		glmsg.ranges = 0;
	}
	return 0;
}

