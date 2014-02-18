#include <iostream>
#include <fstream>
#include <string>
#include <assert.h>
#include <values.h>
#include <sys/time.h>
#include <sstream>
///
#include <cmatcher/cmatcher.hh>
#include "localizer.hh"
#include <math_stuff/transformation3.hh>
///
#include <signal.h>
#include <utils_global/cmdargsreader.h>
#include <utils_global/linefilereader.h>
#include <ipcMessages/qc_odometry_messages.h>
#include <ipcInterfaces/qc_odometry_interface.h>
#include <ipcInterfaces/qc_ipc_interface.h>
#include <ipc/ipc.h>

using namespace std;

bool stop = false;
qc_odometry_odometry_laserpoints_message olmsg;
qc_odometry_odometry_laserpoints_message inolmsg;
double lastts = 0;
double currentts = 0;
double firstts = 0;
bool newMeasurement = false;

int numParsedReadings = 0;
struct timeval tvstart;
struct timeval tvend;
double avgtime = 0;

void sigterm_handler(int sgn __attribute__ ((unused))){
	stop = true;
}

void qc_odometry_odometry_laserpoints_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &inolmsg, sizeof(inolmsg));
	///deep copy of the data
	//i need only x,y,z, roll, pitch, yaw, and the laserpoint data
	olmsg.x = inolmsg.x;
	olmsg.y = inolmsg.y;
	olmsg.z = inolmsg.z;
	olmsg.num_z_readings = 0;
	olmsg.zr = 0;
	olmsg.roll = inolmsg.roll;
	olmsg.pitch = inolmsg.pitch;
	olmsg.yaw = inolmsg.yaw;
	olmsg.q0 = inolmsg.q0;
	olmsg.q1 = inolmsg.q1;
	olmsg.q2 = inolmsg.q2;
	olmsg.q3 = inolmsg.q3;
	if (olmsg.num_laser_readings != 0 && olmsg.num_laser_readings != inolmsg.num_laser_readings){
		delete[] olmsg.lx;
		delete[] olmsg.ly;
		olmsg.lx = 0;
		olmsg.ly = 0;
	}
	olmsg.num_laser_readings = inolmsg.num_laser_readings;
	if (!olmsg.lx)
		olmsg.lx = new float[inolmsg.num_laser_readings];
	if (!olmsg.ly)
		olmsg.ly = new float[inolmsg.num_laser_readings];
	for (int i=0; i<olmsg.num_laser_readings; i++){
		olmsg.lx[i] = inolmsg.lx[i];
		olmsg.ly[i] = inolmsg.ly[i];
	} 		
	olmsg.timestamp_sec = inolmsg.timestamp_sec;
	olmsg.timestamp_usec = inolmsg.timestamp_usec;
	///
	IPC_freeByteArray(callData);
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&inolmsg);
}

bool getNextOfflineMessage (LineFileReader& ilog){
	///get next projected laser message
	string tag;
	while (ilog.isGood()){
		istringstream& lis = *(ilog.getNextLine());
		lis >> tag;
		if (tag == "ODOMETRY_LASERPOINTS"){
			lis >> olmsg.x;
			lis >> olmsg.y;
			lis >> olmsg.z;
			int newSize = 0;
			lis >> newSize;
			if (olmsg.num_z_readings != newSize && olmsg.zr){
				delete[] olmsg.zr;
				olmsg.zr = 0;
			}
			olmsg.num_z_readings = newSize;
			if (!olmsg.zr && olmsg.num_z_readings > 0){
				olmsg.zr = new float[olmsg.num_z_readings];
			}
			for (int i=0; i<olmsg.num_z_readings; i++)
				lis >> olmsg.zr[i];
			lis >> olmsg.q0;
			lis >> olmsg.q1;
			lis >> olmsg.q2;
			lis >> olmsg.q3;
			lis >> olmsg.roll;
			lis >> olmsg.pitch;
			lis >> olmsg.yaw;
			newSize = 0;
			lis >> newSize;
			if (olmsg.num_laser_readings != newSize && olmsg.lx){
				delete[] olmsg.lx;
				delete[] olmsg.ly;
				olmsg.lx = 0;
				olmsg.ly = 0;
			}
			olmsg.num_laser_readings = newSize;
			if (!olmsg.lx)
				olmsg.lx = new float[olmsg.num_laser_readings];
			if (!olmsg.ly)
				olmsg.ly = new float[olmsg.num_laser_readings];
			for (int i=0; i<olmsg.num_laser_readings; i++){
				lis >> olmsg.lx[i];
				lis >> olmsg.ly[i];
			}
			lis >> olmsg.timestamp_sec;
			lis >> olmsg.timestamp_usec;
			newMeasurement = true;
			return true;
		}
	}
	return false;
}

void dumpStatus(char* filename, Localizer& l, LocalizeMap& background){
	LocalizeMap image(background);
	for (int i=0; i<(int)l.particles.size(); i++){
		IVector2 p=l.localizeMap.world2map(DVector2(l.particles[i].pose.x(), l.particles[i].pose.y()));
		image.cell(p).occupancy=0.7;
		image.cell(p).visited=1;
	}

	IVector2Vector ib;
	l.remapScan(ib, l.bestPose()); 
	ofstream os(filename);
	image.saveToPPM(os,ib,l.localizeMap.world2map(DVector2(l.bestPose().x(), l.bestPose().y())));
	os.close();
}

int main (int argc, char* argv[]){		
	string filename_log = "";
	string filename_map = "";
	int nparticles = 500;
	bool help = false;
	bool disableOutput = false;
	bool syncForVideo = false;
	bool offline_mode = false;
	
	///FIXME read parameters from config file ??
	//BEGIN LOCALIZER PART
	double sqLinearUpdate = 0.1 * 0.1;
	double sqAngularUpdate = 0.15 * 0.15;
	LocalizerParameters params;
	MotionModel motionModel;
	Localizer localizer;
	motionModel.ff = 0.5;
	motionModel.fs = 0.5;
	motionModel.fr = 0.5;
	motionModel.ss = 0.5;
	motionModel.sr = 0.5;
	motionModel.rr = 0.75; //0.3
	motionModel.sx = 1;//1;
	motionModel.sy = 1;//1;
	motionModel.sth = 0.5;//0.5;
	params.distanceMapThreshold = 0.3; //10 //0.3
	params.dynamicRestart = true; ///evtl set true
	params.minWeight = 0.01;
	params.maxLocalizationRange = 5.5; ///evtl 20
	params.linearUpdate = 0.05;
	params.angularUpdate = 0.1;
	localizer.params = &params;
	localizer.motionModel = &motionModel;
	//END LOCALIZER PART
	
	
	
	
	CmdArgsReader commandline(argc, argv);
	commandline.getStringValue("-f","-filename",&filename_log);
	commandline.getStringValue("-l","-localizeMap",&filename_map);
	commandline.getIntValue("-p","-numParticles",&nparticles);
	commandline.getFlagValue("-h","-help",&help);
	commandline.getFlagValue("-do","-disableOutput",&disableOutput);
	commandline.getFlagValue("-sv","-syncForVideo",&syncForVideo);
	if (help || argc < 2){
		std::cerr << "Usage=  "<< argv[0] << std::endl;
		std::cerr << "          -l | -localizeMap <filename> (localize map)" << std::endl;
		std::cerr << "         [-f | -filename <filename>] (for offline mode)" << std::endl;
		std::cerr << "         [-p | -numParticles <int>] (use num particles)" << std::endl;
		std::cerr << "         [-do | -disableOutput] (disable output pics)" << std::endl;
		std::cerr << "         [-sv | -syncForVideo] (try to dump a frame each 0.1 sec) "<< std::endl;
		std::cerr << "         [-h | -help] (distplay this help message " << std::endl;
		return 0;
	}
	///FIXME read parameters from config file before the cmdargsreader
	
	
	
	if (commandline.printUnusedArguments())
		return 0;
	
	if (filename_log == ""){ ///online mode
		offline_mode = false;
		if (qc_ipc_connect(argv[0]) <= 0)
			return 0;
		qc_odometry_subscribe_odometry_laserpoints_message(qc_odometry_odometry_laserpoints_message_handler, 2, NULL);
	} else 
		offline_mode = true;
		
	ifstream ilmap(filename_map.c_str());
	if (!ilmap){
		cerr << "#Unable to open localizemap \"" << filename_map << "\"" << endl;
		return 0;
	}

	LineFileReader ilog;
	if (offline_mode){
		if (!ilog.openFile(filename_log.c_str())){
			cerr << "#Unable to open logfile \"" << filename_log << "\"" << endl;
			return 0;
		}
	}

	std::cerr << "=====LOCALIZER====== " << std::endl;
	std::cerr << "#LocalizeMap= " << filename_map << endl;
	std::cerr << "#Particles= " << nparticles<< endl;

	cerr << "#Loading localize-map..." << flush;
	LocalizeMap om;
	ilmap >> om;
	ilmap.close();
	cerr << "done" << endl;
	
	localizer.init(om, nparticles);
	LocalizeMap image (localizer.localizeMap);
	
	
	signal(SIGINT,sigterm_handler);
	olmsg.num_z_readings = 0;
	olmsg.zr = 0;
	olmsg.num_laser_readings = 0;
	olmsg.lx = 0;
	olmsg.ly = 0;
	bool first = true;
	DTransformation2 currentOdometry;
	DTransformation2 lastOdometry;
	bool processReading = false;
	uint localizer_count = 0;
	while (!stop){
		processReading = false;
		if (newMeasurement){
			newMeasurement = false;
			if (first){
				lastOdometry = currentOdometry;
				firstts = olmsg.timestamp_sec + 1e-6 * olmsg.timestamp_usec;
			}
			currentOdometry = DTransformation2(olmsg.x, olmsg.y, olmsg.yaw);
			if (!first)
				processReading = true;
			first = false;
			lastts= currentts;
			currentts = olmsg.timestamp_sec + 1e-6 * olmsg.timestamp_usec;
		}
		if (processReading){
			bool updated = false;
			DPose2 delta = (lastOdometry.inv() * currentOdometry).toPoseType();
			double dtrans = delta.x() * delta.x() + delta.y() * delta.y();
			double dtheta = delta.theta() * delta.theta();
			double ts = currentts - firstts;
			if (dtrans > sqLinearUpdate || dtheta > sqAngularUpdate){
				gettimeofday(&tvstart, NULL);
				///update localizer
				lastOdometry = currentOdometry;
				localizer.updateMotion(delta);
				///get mappedPoints2d;
				vector<DVector2> mappedPoints2d(olmsg.num_laser_readings);
				for (int i=0; i<olmsg.num_laser_readings; i++)
					mappedPoints2d[i] = DVector2(olmsg.lx[i], olmsg.ly[i]);
				updated = localizer.updateObservation(mappedPoints2d, ts, false);
				gettimeofday(&tvend, NULL);
				avgtime += (tvend.tv_sec + 1e-6 * tvend.tv_usec) - (tvstart.tv_sec + 1e-6 * tvstart.tv_usec);
				numParsedReadings++;
				if (!disableOutput && !syncForVideo){
					char dumpName[1000];
					sprintf(dumpName, "ldump-%05d.ppm", localizer_count++);
					dumpStatus(dumpName, localizer, image);
					if (! (localizer_count%10))
						cerr << "." << flush;
				}
			} 
			if (!disableOutput && syncForVideo){
				while ((currentts - lastts) >= 0.1){
					char dumpName[1000];
					sprintf(dumpName, "ldump-%05d.ppm", localizer_count++);
					dumpStatus(dumpName, localizer, image);
					lastts += 0.1;
					std::cerr << "f" << std::flush;
				}
			}
		}
		if (offline_mode){
			if (!getNextOfflineMessage(ilog))
				break;
			usleep(1000);
		} else
			IPC_listen(1);
	}

	if (numParsedReadings > 0){
		avgtime = avgtime / numParsedReadings;
		std::cerr << "Avg Iteration time= " << avgtime*1e3 << " ms" << std::endl;
	}
	
	
	
	
	if (!offline_mode)
		qc_odometry_unsubscribe_odometry_laserpoints_message(qc_odometry_odometry_laserpoints_message_handler);
	return 0;
}

