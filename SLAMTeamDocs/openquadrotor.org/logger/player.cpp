#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <ipcMessages/qc_imu_messages.h>
#include <ipcMessages/qc_laser_messages.h>
#include <ipcMessages/qc_odometry_messages.h>
#include <ipcMessages/qc_flightcontrol_messages.h>
#include <ipcInterfaces/qc_ipc_interface.h>
#include <ipcInterfaces/qc_imu_interface.h>
#include <ipcInterfaces/qc_laser_interface.h>
#include <ipcInterfaces/qc_odometry_interface.h>
#include <ipcInterfaces/qc_flightcontrol_interface.h>
#include <utils_global/cmdargsreader.h>
#include <sys/time.h>
#include "playerutils.h"
using namespace std;

ifstream in;

qc_laser_laser_message lmsg1;
qc_laser_laser_message lmsg2;
qc_imu_imu_message imsg;
qc_odometry_odometry_message omsg;
qc_odometry_odometry_laserpoints_message olmsg;
qc_odometry_velocity_message vmsg;
qc_flightcontrol_flightcontrol_message fcmsg;
qc_flightcontrol_offboard_command_message ocmsg;

///FIXME multiple jpg picture message missing
///FIXME how to handle wrong file format? simple abort?

double first_ts;
bool first_message;
double slowdown_factor = 1.0;
double message_version = 1.0;
int msg_counter = 0;
int line_counter = 0;
double system_start_time = 0;
struct timeval tmm;
bool wait_active = false;

void sleepUntilTimestamp(int s, int us){
	double currentTime = s + 1e-6*us;
	if (first_message){
		first_message = false;
		first_ts = currentTime;
		gettimeofday(&tmm,NULL);
		system_start_time = tmm.tv_sec + 1e-6 * tmm.tv_usec;
		return;
	} 
	
	double deltaSleep = slowdown_factor * (currentTime - first_ts);
	gettimeofday(&tmm, NULL);
	double current_system_time = tmm.tv_sec + 1e-6 * tmm.tv_usec;
	double send_msg_system_time = system_start_time + deltaSleep;
	double sleepTime_u = 1e6 * (send_msg_system_time - current_system_time) - 100;
	msg_counter++;
	if (sleepTime_u > 0)
		usleep((int)sleepTime_u);
	first_message = wait_active;
}

void getFileFormat(istringstream& lis){
	string tag;
	lis >> tag; // [
	lis >> tag; // "File
	lis >> tag; // Format
	lis >> tag;  // Version
	lis >> tag; //<version number>
	message_version = atof(tag.c_str());
	lis >> tag; // ]
}

void sendLaser1Message(istringstream& lis){
	PlayerUtils::readLaserMessage(lmsg1, lis);
	if (!qc_laser_publish_laser1_message(&lmsg1))
		exit(-1);
	std::cerr << "[L1]" << std::flush;
}


void sendLaser2Message(istringstream& lis){
	PlayerUtils::readLaserMessage(lmsg2, lis);
	sleepUntilTimestamp(lmsg2.timestamp_sec, lmsg2.timestamp_usec);
	if (!qc_laser_publish_laser2_message(&lmsg2))
		exit(-1);
	std::cerr << "[L2]" << std::flush;
}


void sendOdometryMessage(istringstream& lis){
	PlayerUtils::readOdometryMessage(omsg, lis);
	sleepUntilTimestamp(omsg.timestamp_sec, omsg.timestamp_usec);
	if(!qc_odometry_publish_odometry_message(&omsg))
		exit(-1);
	std::cerr << "O" << std::flush;
}


void sendOdometryLaserpointsMessage(istringstream& lis){
	PlayerUtils::readOdometryLaserpointsMessage(olmsg, lis);
	sleepUntilTimestamp(olmsg.timestamp_sec, olmsg.timestamp_usec);
	if (!qc_odometry_publish_odometry_laserpoints_message(&olmsg))
		exit(-1);
	std::cerr << "[OL]" << std::flush;
}

void sendVelocityMessage(istringstream& lis){
	PlayerUtils::readVelocityMessage(vmsg, lis);
	sleepUntilTimestamp(vmsg.timestamp_sec, vmsg.timestamp_usec);
	if(!qc_odometry_publish_velocity_message(&vmsg))
		exit(-1);
	std::cerr << "V" << std::flush;
}


void sendIMUMessage(istringstream& lis){
	PlayerUtils::readImuMessage(imsg, lis);
	sleepUntilTimestamp(imsg.timestamp_sec, imsg.timestamp_usec);
	if (!qc_imu_publish_imu_message(&imsg))
		exit(-1);
	std::cerr << "I" << std::flush;
}


void sendFlightcontrolMessage(istringstream& lis){
	PlayerUtils::readFlightControlMessage(fcmsg, lis);
	sleepUntilTimestamp(fcmsg.timestamp_sec, fcmsg.timestamp_usec);
	if(!qc_flightcontrol_publish_flightcontrol_message(&fcmsg))
		exit(-1);
	std::cerr << "F" << std::flush;
}

void sendOffboardCommandMessage(istringstream& lis){
	PlayerUtils::readOffboardCommandMessage(ocmsg, lis);
	sleepUntilTimestamp(ocmsg.timestamp_sec, ocmsg.timestamp_usec);
	if(!qc_flightcontrol_publish_offboard_command_message(&ocmsg))
		exit(-1);
	std::cerr << "C" << std::flush;
}

int main (int argc, char* argv[]){
	if (argc < 2){
		std::cerr << "Usage= " << argv[0] << std::endl;
		std::cerr << "          -i | -input <inputfilename>" << std::endl;
		std::cerr << "          -sf | -slowdownfactor <double> (wait <value>*real_time between messages) [default = 1.0], if < 1.0 its faster" << std::endl;
		std::cerr << "          -nl1 | -nolaser1 (discard laser1 messages)" << std::endl;
		std::cerr << "          -nl2 | -nolaser2 (discard laser2 messages)" << std::endl;
		std::cerr << "          -ni | -noimu (discard imu messages)" << std::endl;
		std::cerr << "          -no | -noodometry (discard odometry messages)" << std::endl;
		std::cerr << "          -nol| -noodometrylaser (discard odometry_laserpoint_messages)" << std::endl;
		std::cerr << "          -nv | -novelocity (discard velocity messages)" << std::endl;
		std::cerr << "          -nf | -noflightcontrol (discard flightcontrol messages)" << std::endl;
		std::cerr << "          -noc | -nooffboardcommand (discard offboard command messages)" << std::endl;
		std::cerr << "          -w[l1|l2|i|o|ol|v|f|oc] | -wait[laser1|laser2|imu|odometry|odometrylaser|velocity|flightcontrol|offboardcommand] (stop after sending the message and wait for a return" << std::endl;
		exit(1);
	}
	
	lmsg1.num_ranges = 0;
	lmsg1.ranges = 0;
	lmsg2.num_ranges = 0;
	lmsg2.ranges = 0;
	omsg.num_z_readings = 0;
	omsg.zr = 0;
	olmsg.num_z_readings = 0;
	olmsg.zr = 0;
	olmsg.num_laser_readings = 0;
	olmsg.lx = 0;
	olmsg.ly = 0;
	string filename = "";
	bool nolaser1 = false;
	bool nolaser2 = false;
	bool noimu = false;
	bool noodometry = false;
	bool noodometrylaser = false;
	bool noflightcontrol = false;
	bool novelocity = false;
	bool nooffboardcommand = false;
	
	bool waitlaser1 = false;
	bool waitlaser2 = false;
	bool waitimu = false;
	bool waitodometry = false;
	bool waitodometrylaser = false;
	bool waitflightcontrol = false;
	bool waitvelocity = false;
	bool waitoffboardcommand = false;
	char cinbuffer[10];
	
	
	
	first_message = true;
	CmdArgsReader commandline(argc, argv);
	commandline.getStringValue("-i","-input",&filename);
	commandline.getDoubleValue("-sf","-slowdownfactor",&slowdown_factor);
	commandline.getFlagValue("-nl1","-nolaser1",&nolaser1);
	commandline.getFlagValue("-nl2","-nolaser2",&nolaser2);
	commandline.getFlagValue("-ni","-noimu",&noimu);
	commandline.getFlagValue("-no","-noodometry",&noodometry);
	commandline.getFlagValue("-nol","-noodometrylaser",&noodometrylaser);
	commandline.getFlagValue("-nf","-noflightcontrol",&noflightcontrol);
	commandline.getFlagValue("-nv","-novelocity",&novelocity);
	commandline.getFlagValue("-noc","-nooffboardcommand",&nooffboardcommand);
	
	commandline.getFlagValue("-wl1","-waitlaser1", &waitlaser1);
	commandline.getFlagValue("-wl2","-waitlaser2", &waitlaser2);
	commandline.getFlagValue("-wi","-waitimu", &waitimu);
	commandline.getFlagValue("-wo","-waitodometry", &waitodometry);
	commandline.getFlagValue("-wol","-waitodometrylaser", &waitodometrylaser);
	commandline.getFlagValue("-wf","-waitflightcontrol",&waitflightcontrol);
	commandline.getFlagValue("-wv","-waitvelocity",&waitvelocity);
	commandline.getFlagValue("-woc","-waitoffboardcommand",&waitoffboardcommand);
	
	wait_active = waitlaser1 || waitlaser2 || waitimu || waitodometry || waitodometrylaser || waitflightcontrol || waitvelocity || waitoffboardcommand;
	
	if (slowdown_factor < 0){
		std::cerr << "#Warning: slowdownfactor < 0! setting to 1.0" << std::endl;
	}
	
	if (commandline.printUnusedArguments()){
		exit(-1);
	}
	
	
	if (filename == ""){
		std::cerr << "#unspecified input filename!" << std::endl;
		exit(-1);
	}
	
	in.open(filename.c_str());
	
	if (!in){
		std::cerr << "#Error: unable to open file \"" << filename << "\"" << std::endl;
		exit(-1);
	}
	
	if (qc_ipc_connect(argv[0]) <= 0){
		exit(-1);
	}
	
	///read log file, send messages in with constant "wait"
	int buffersize = 100000;
	string tag;
	while (in.good()){
		
		line_counter++;
		char buffer[buffersize];
		in.getline(buffer,buffersize);
		istringstream lis(buffer);
		lis >> tag;
		if (tag == "LASER1"){
			if (nolaser1)
				continue;
			sendLaser1Message(lis);
			if (waitlaser1)
				cin.getline(cinbuffer,10);
		}
		if (tag == "LASER2"){
			if (nolaser2)
				continue;
			sendLaser2Message(lis);
			if (waitlaser2)
				cin.getline(cinbuffer,10);
		}
		if (tag == "IMU"){
			if (noimu)
				continue;
			sendIMUMessage(lis);
			if (waitimu)
				cin.getline(cinbuffer,10);
		}
		if (tag == "ODOMETRY"){
			if (noodometry)
				continue;
			sendOdometryMessage(lis);
			if (waitodometry)
				cin.getline(cinbuffer,10);
		}
		if (tag == "ODOMETRY_LASERPOINTS"){
			if (noodometrylaser)
				continue;
			sendOdometryLaserpointsMessage(lis);
			if (waitodometrylaser)
				cin.getline(cinbuffer,10);
		}
		if (tag == "VELOCITY"){
			if (novelocity)
				continue;
			sendVelocityMessage(lis);
			if (waitvelocity)
				cin.getline(cinbuffer,10);
		}
		if (tag == "FLIGHTCONTROL"){
			if (noflightcontrol)
				continue;
			sendFlightcontrolMessage(lis);
			if (waitflightcontrol)
				cin.getline(cinbuffer,10);
		}
		if (tag == "OFFBOARDCOMMAND"){
			if (nooffboardcommand)
				continue;
			sendOffboardCommandMessage(lis);
			if (waitoffboardcommand)
				cin.getline(cinbuffer,10);
		}
		if (tag == "#Messages"){
			getFileFormat(lis);
		}
	}
	
	if (lmsg1.ranges){
		delete[] lmsg1.ranges;
		lmsg1.ranges = 0;
	}
	if (lmsg2.ranges){
		delete[] lmsg1.ranges;
		lmsg1.ranges = 0;
	}
	if (omsg.zr){
		delete[] omsg.zr;
		omsg.zr = 0;
	}
	if (olmsg.zr){
		delete[] olmsg.zr;
		olmsg.zr = 0;
	}
	if (olmsg.lx){
		delete[] olmsg.lx;
		olmsg.lx = 0;
	}
	if (olmsg.ly){
		delete[] olmsg.ly;
		olmsg.ly = 0;
	}
	
	return 0;
}


