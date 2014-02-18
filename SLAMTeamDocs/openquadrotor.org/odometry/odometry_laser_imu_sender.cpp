#include <ipcMessages/qc_imu_messages.h>
#include <ipcMessages/qc_laser_messages.h>
#include <ipcMessages/qc_odometry_messages.h>
#include <imu/imubuffer.h>
#include <laser/laserutils.h>
#include <ipc/ipc.h>
#include <iostream>
#include <signal.h>
#include <vector>
#include <math_stuff/transformation2.hh>

#include "lasertransformator.h"
#include <ipcInterfaces/qc_imu_interface.h>
#include <ipcInterfaces/qc_laser_interface.h>
#include <ipcInterfaces/qc_odometry_interface.h>
#include <ipcInterfaces/qc_ipc_interface.h>
#include <utils_global/cmdargsreader.h>
#include <configSender/ipcParamReader.h>
#include <cmatcher/histcmatcher.h>
#include <cmatcher/histmrcmatcher.h>
#include <cmatcher/mrcmatcher.h>
#include <sys/time.h>

using namespace std;

qc_imu_imu_message imsg;
qc_laser_laser_message ilmsg;
qc_laser_laser_message lmsg;
bool newLaserMeasurement = false;
bool stop = false;
IMUBuffer imubuffer(100); ///max 100 messages = 1 sec
LaserUtils lu;
bool imu_init = false;
bool v_init = false;
///if laser1 is planar, this is the rotation of the imu.
//Quaternion<double> laser2imu;((1.61/180.)*M_PI, (1.47/180.)*M_PI, 0);
Quaternion<double> imu2laser;// = laser2imu.inverse();
struct timeval tvstart;
struct timeval tvend;
double avgtime = 0;
int numScans = 0;

void sigterm_handler(int sgn __attribute__ ((unused))){
	stop = true;
}

void qc_imu_imu_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &imsg, sizeof(imsg));
	IPC_freeByteArray(callData);
	if (err != IPC_OK)
		fprintf(stderr, "#Error!\n");
	imubuffer.addMessage(imsg);
	imu_init = true;
}

void qc_laser_laser1_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &ilmsg, sizeof(ilmsg));
	IPC_freeByteArray(callData);
	lu.deepMessageCopy(ilmsg,lmsg);
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&ilmsg);
	newLaserMeasurement = true && imu_init;
}



int main (int argc, char* argv[]){
	bool help = false;
	double sm_search_DX = 0.1; ///search radius x
	double sm_search_DY = 0.1; ///search radius y
	double sm_search_DTH = 0.1; ///search radius theta
	int buffersize = -1;
	int worldsize = 12;
	double resolution = 0.04;
	int numMatchers = 3;
	CmdArgsReader commandline(argc, argv);
	commandline.getFlagValue("-h","-help",&help);
	commandline.getIntValue("-b","-buffersize",&buffersize);
	commandline.getIntValue("-w","-worldsize",&worldsize);
	commandline.getDoubleValue("-dx",&sm_search_DX);
	commandline.getDoubleValue("-dy",&sm_search_DY);
	commandline.getDoubleValue("-dth",&sm_search_DTH);
	commandline.getDoubleValue("-r","-resolution",&resolution);
	commandline.getIntValue("-nm","-numMatchers",&numMatchers);
	
	if (commandline.printUnusedArguments())
		return 0;
	if (help){
		std::cerr << "Usage= " << argv[0] << std::endl;
		std::cerr << "     -b | -buffersize (buffersize of the SM, <= 0 == use all, default = 30)" << std::endl;
		std::cerr << "     -w | -worldsize (worldsize of the SM, default = 10)" << std::endl;
		std::cerr << "     -dx (search radius in X dimension, default: 0.1)" << std::endl;
		std::cerr << "     -dy (search radius in Y dimension, default: 0.1)" << std::endl;
		std::cerr << "     -dth (search radius in THETA dimension, default: 0.1)" << std::endl;
		std::cerr << "     -r | -resolution (resolution of the scanmatcher: default 0.04)" << std::endl;
		std::cerr << "     -nm| -numMatchers (num Hirachical Matchers: default 3)" << std::endl;
		std::cerr << "     -h | -help (display this help)" << std::endl;
		return 0;
	}
	if (qc_ipc_connect(argv[0]) <= 0)
		exit(-1);
	signal(SIGINT,sigterm_handler);
	///--->retrieve parameters
	double distance_laser2mirror = 0;
	double distance_laser2ground = 0;
	int heightFirstBeam = 0;
	int heightSkipBeamStart = 0;
	int heightSkipBeamStop = 0;
	int heightLastBeam = 0;
	vector<double> weights(numMatchers);
	double n = 1;
	weights[numMatchers-1] = 1;
	for (int i=numMatchers-2; i>=0; i--){
		weights[i] = 0.5 * weights[i+1];
		n+= weights[i];
	}
	for (uint i=0; i<weights.size(); i++){
		weights[i] = (1./n) * weights[i];
		//std::cerr << weights[i] << " ";
	}
	//std::cerr << std::endl;


	///---->retrieve parameters
	IPCParamReader paramReader;
	double _tx,_ty,_tz,_tq0,_tq1,_tq2,_tq3;
	if (!paramReader.getDoubleValue("global_laser2imu", 7, &_tx, &_ty, &_tz, &_tq0, &_tq1, &_tq2, &_tq3))
		return 0;
	imu2laser = Quaternion<double>(_tq0,_tq1,_tq2,_tq3).inverse();
	if (!paramReader.getDoubleValue("laser_distance_parameters", 2, &distance_laser2mirror, &distance_laser2ground))
		return 0;
	if (!paramReader.getIntValue("laser_mirror_parameters", 4, &heightFirstBeam, &heightSkipBeamStart, &heightLastBeam, &heightSkipBeamStop))
		return 0;
	///<---retrieve parameters
	
	
	std::cerr << "Parameters: " << std::endl;
	std::cerr << "=== WORLD === " << std::endl;
	std::cerr << "   BufferSize= "<< buffersize << std::endl;
	std::cerr << "   WorldSize= " << worldsize << std::endl;
	std::cerr << "   SearchX = " << sm_search_DX << std::endl;
	std::cerr << "   SearchY = " << sm_search_DY << std::endl;
	std::cerr << "   SearchTheta = " << sm_search_DTH << std::endl;
	std::cerr << "   Resolution= " << resolution << std::endl;
	std::cerr << "   NumMatchers= " << numMatchers << std::endl;
	std::cerr << "   Weights= ";
	for (uint i=0; i<weights.size(); i++)
		std::cerr << weights[i] << " ";
	std::cerr << std::endl;
	
	std::cerr << "=== LASER === " << std::endl;
	std::cerr << "   Laser2Imu " << _tx << " " <<  _ty << " " <<  _tz << " " << _tq0 << " " << _tq1 << " " << _tq2 << " " << _tq3 << std::endl;
	std::cerr << "   MirrorParams= " << heightFirstBeam << " " << heightSkipBeamStart << " " << heightLastBeam << " " << heightSkipBeamStop << std::endl;
	
	
	ilmsg.ranges = 0;
	ilmsg.num_ranges = 0;
	lmsg.ranges = 0;
	lmsg.num_ranges = 0;
	qc_odometry_odometry_message omsg;
	qc_odometry_odometry_laserpoints_message olmsg;
	omsg.num_z_readings = 0;
	omsg.zr = 0;
	olmsg.num_z_readings = 0;
	olmsg.zr = 0;
	olmsg.num_laser_readings = 0;
	olmsg.lx = 0;
	olmsg.ly = 0;
	vector<double> ranges;
	vector<double> angles;
	
	vector<DVector2> front_beams;
	vector<DVector3> height_beams;
	
	
	//?FIXME: scanmatcher at lvl i should take his own estimated pose?
	///old histcmatcher with resolution of 0.01 cm is much more stable than the new mrscanmatcher.
	
	
	
	//HistCMatcher matcher(buffersize, resolution, (M_PI/720.), 0.2, 0.4, worldsize); 
	//MultiResolutionCMatcher matcher(numMatchers, resolution, worldsize);
	HistMRCMatcher matcher(buffersize, numMatchers, resolution, (M_PI/720.), 0.2, 0.4, worldsize);
	
	
	///buffersize, gridsize, angular resolution, fillingdist, minscore, worldsize
	
	LaserTransformator transformator;
	
	int count = 0;
	double alpha = 0;
	
	DTransformation2 currentOdometry(0,0,0);
	DTransformation2 ipc_currentOdometry(0,0,0);
	DTransformation2 oldOdometry(0,0,0);
	DTransformation2 lastDelta(0,0,0);
	bool first = true;
	int iteration = 0;
	
	qc_imu_subscribe_imu_message(qc_imu_imu_message_handler,10,NULL);
	qc_laser_subscribe_laser1_message(qc_laser_laser1_message_handler,10,NULL);
	
	double min_t = 0.1 * 0.1;
	if (sm_search_DX * sm_search_DX + sm_search_DY * sm_search_DY < min_t)
		min_t = sm_search_DX * sm_search_DX + sm_search_DY * sm_search_DY - 2 * resolution;
	
	double min_r = 0.15 * 0.15;
	if (sm_search_DTH * sm_search_DTH < min_r)
		min_r = sm_search_DTH * sm_search_DTH - 0.05;
	while (!stop){
		if (newLaserMeasurement){
			gettimeofday(&tvstart,NULL);
			newLaserMeasurement = false;
			///first -> search for latest IMU reading
			/// -0.02 because of the delay between laser and imu (at least 20ms)
			double laser_ts = lmsg.timestamp_sec + 1e-6 * lmsg.timestamp_usec - 0.02;
			qc_imu_imu_message interpolated_imsg;
			Quaternion<double> qimu;
			if (imubuffer.getInterpolatedIMUMessage(interpolated_imsg, laser_ts)) {
				qimu = Quaternion<double> (interpolated_imsg.q0, interpolated_imsg.q1, interpolated_imsg.q2, interpolated_imsg.q3);
				///FIXME
				qimu = imu2laser * qimu;
			} else {
				std::cerr << "#No IMU message found, eventually IMU stalled ?!" << std::endl;
				qimu = Quaternion<double>(1,0,0,0);
			}
			///calculate ranges and angles of laser
			ranges.resize(lmsg.num_ranges);
			angles.resize(lmsg.num_ranges);
			count = 0;
			alpha = lmsg.startAngle - lmsg.incrementAngle;
			for (int i=0; i<lmsg.num_ranges; i++){
				alpha += lmsg.incrementAngle;
				if (i >= heightFirstBeam && i<= heightLastBeam)
					continue;
				if (lmsg.ranges[i] < 20) ///discard max readings
					continue;
				ranges[count] = lmsg.ranges[i] * 1e-3;
				angles[count] = alpha;
				count++;
			}
			ranges.resize(count);
			angles.resize(count);			
			transformator.calculate2dLaserPoints(ranges, angles, front_beams, qimu, true);
			///now we have the front laser beams.
			oldOdometry = currentOdometry;
			DPose2 initialGuess = (currentOdometry * lastDelta).toPoseType();
			if (first){
				initialGuess = currentOdometry.toPoseType();
				matcher.integrateScan(front_beams, initialGuess, true);
				first = false;
				///only register front_beams, forget about height.
				continue;
			} 
			///search in an area of 0.3m in x and y direction, 0.2 in angular direction
			///constant velocity model for initial guess
			matcher.scanMatch (front_beams, initialGuess, sm_search_DX, sm_search_DY, sm_search_DTH);
			///get matcher results
			///check if scan matching succeeded
			///if moved  -> integrate scanMatch
			const vector<MatcherResult*> results = matcher.results();
			MatcherResult r;
			MatcherResult tmp_r;
			r.pose = currentOdometry;
			bool scanmatching_failed = false;
			if (!results.size()){
				std::cerr << "***** SCAN MATCHING FAILED! ***** (IT= " << iteration << ")" << std::endl;
				scanmatching_failed = true;
			} else {
				///FIXME multi resolution scanmatchin result:
				///which weights to use?
				r.pose = results.at(0)->pose;
				//std::cout << r.pose.toPoseType().x() << " " << r.pose.toPoseType().y() << " ";
				
				r.pose.translationVector[0] = weights[0] * r.pose.translationVector[0];
				r.pose.translationVector[1] = weights[0] * r.pose.translationVector[1];
				int maxUseable = 0;
				for (int i=1; i<numMatchers; i++){
					const vector<MatcherResult*> results_i = matcher.results(i);
					if (results_i.size()>0){
						r.pose.translationVector[0] += weights[i] * results_i[0]->pose.translationVector[0];
						r.pose.translationVector[1] += weights[i] * results_i[0]->pose.translationVector[1];
						r.pose.rotationMatrix[0][0] = results_i[0]->pose.rotationMatrix[0][0];
						r.pose.rotationMatrix[0][1] = results_i[0]->pose.rotationMatrix[0][1];
						r.pose.rotationMatrix[1][0] = results_i[0]->pose.rotationMatrix[1][0];
						r.pose.rotationMatrix[1][1] = results_i[0]->pose.rotationMatrix[1][1];
						//std::cout << results_i[0]->pose.toPoseType().x() << " " << results_i[0]->pose.toPoseType().y() << " ";
						maxUseable = i;
					}else{
						std::cerr << "***** SCAN MATCHING FAILED! ***** (IT= " << iteration << ") at level " << i << std::endl;
						r.pose.translationVector[0] += weights[i] * results[0]->pose.translationVector[0];
						r.pose.translationVector[1] += weights[i] * results[0]->pose.translationVector[1];
					}
				}
				///HACK
				const vector<MatcherResult*> results_i = matcher.results(maxUseable);
				tmp_r = r;
				r.pose= results_i[0]->pose;
				///
				//std::cout << std::endl;
				
				scanmatching_failed = false;
			}
			if (!scanmatching_failed){
				///HACK
				currentOdometry = r.pose;
				ipc_currentOdometry = tmp_r.pose;
			} else {
				/*
				///assume constant velocity model when SM failed!
				currentOdometry = currentOdometry * lastDelta;
				*/
				///do not assume constant velocity model!
			}
			gettimeofday(&tvend,NULL);
			avgtime += tvend.tv_sec - tvstart.tv_sec + 1e-6 * (tvend.tv_usec - tvstart.tv_usec);
			numScans++;
			lastDelta = oldOdometry.inv() * currentOdometry;
			DPose2 p = lastDelta.toPoseType();
			if (scanmatching_failed || (p.x() * p.x() + p.y() * p.y() > min_t || p.theta() * p.theta() > min_r)){
				DPose2 cp = currentOdometry.toPoseType();
				matcher.integrateScan(front_beams, cp, false);
			}
			///HACK
			//DPose2 odom = currentOdometry.toPoseType();
			DPose2 odom = ipc_currentOdometry.toPoseType();
			Quaternion<double> nq (qimu.toAngles().x(), qimu.toAngles().y(), odom.theta());
			DVector3 an = nq.toAngles();
			
			omsg.x = odom.x();
			omsg.y = odom.y();			
			omsg.q0 = nq.w;
			omsg.q1 = nq.x;
			omsg.q2 = nq.y;
			omsg.q3 = nq.z;
			omsg.roll = an.roll();
			omsg.pitch = an.pitch();
			omsg.yaw = an.yaw();
			
			olmsg.x = odom.x();
			olmsg.y = odom.y();
			olmsg.q0 = nq.w;
			olmsg.q1 = nq.x;
			olmsg.q2 = nq.y;
			olmsg.q3 = nq.z;
			olmsg.roll = an.roll();
			olmsg.pitch = an.pitch();
			olmsg.yaw = an.yaw();
			if (olmsg.lx){
				delete[] olmsg.lx;
				olmsg.lx = 0;
			}
			if (olmsg.ly){
				delete[] olmsg.ly;
				olmsg.ly = 0;
			}
			olmsg.num_laser_readings = (int)front_beams.size();
			olmsg.lx = new float[front_beams.size()];
			olmsg.ly = new float[front_beams.size()];
			for (uint i=0; i<front_beams.size(); i++){
				olmsg.lx[i] = front_beams[i].x();
				olmsg.ly[i] = front_beams[i].y();
			}
			///front beams finished
			///perform height calculation
			ranges.resize(lmsg.num_ranges);
			angles.resize(lmsg.num_ranges);
			count = 0;
			alpha = lmsg.startAngle + (heightFirstBeam+heightSkipBeamStart-1) * lmsg.incrementAngle;
			for (int i=heightFirstBeam+heightSkipBeamStart; i<lmsg.num_ranges && i < (heightLastBeam-heightSkipBeamStop); i++){
				alpha += lmsg.incrementAngle;
				if (lmsg.ranges[i] < 20)
					continue;
				ranges[count] = lmsg.ranges[i] * 1e-3;
				angles[count] = alpha;
				count++;
			}
			ranges.resize(count);
			angles.resize(count);
			transformator.calculate3dMirrorPoints(ranges, angles, height_beams, qimu, distance_laser2mirror, true);
			if (omsg.zr){
				delete[] omsg.zr;
				omsg.zr = 0;
			}
			omsg.num_z_readings = (int)height_beams.size();
			float meanHeight = 0;
			if (omsg.num_z_readings > 0){
				omsg.zr = new float[omsg.num_z_readings];
				for (int i=0; i<omsg.num_z_readings; i++){
					omsg.zr[i] = fabs(height_beams[i].z());
					meanHeight += fabs(height_beams[i].z());
				}
				meanHeight = meanHeight / (float) omsg.num_z_readings;
			}
			meanHeight = meanHeight - distance_laser2ground;
			omsg.z = meanHeight;
		
			if (olmsg.zr){
				delete[] olmsg.zr;
				olmsg.zr = 0;
			}
			olmsg.num_z_readings = (int)height_beams.size();
			if (olmsg.num_z_readings > 0){
				olmsg.zr = new float[olmsg.num_z_readings];
				for (int i=0; i<olmsg.num_z_readings; i++)
					olmsg.zr[i] = fabs(height_beams[i].z());
			}
			olmsg.z = meanHeight;
			///height calculation done
			///publish
			if (!(iteration%10))
				std::cerr << "." << std::flush;
			/*
			if (omsg.num_z_readings > 0)
				std::cerr << "ODOM= " << omsg.x << " " << omsg.y << " " << omsg.z[0] << " " << (nq.toAngles().x()/M_PI)*180. << " " << (nq.toAngles().y()/M_PI)*180. << " " << (nq.toAngles().z()/M_PI)*180. << std::endl;
			else 
				std::cerr << "ODOM= " << omsg.x << " " << omsg.y << " " << "n/a" << " " << (nq.toAngles().x()/M_PI)*180. << " " << (nq.toAngles().y()/M_PI)*180. << " " << (nq.toAngles().z()/M_PI)*180. << std::endl;
			*/
			omsg.timestamp_sec =  lmsg.timestamp_sec;
			omsg.timestamp_usec = lmsg.timestamp_usec;
			olmsg.timestamp_sec =  lmsg.timestamp_sec;
			olmsg.timestamp_usec = lmsg.timestamp_usec;
			qc_odometry_publish_odometry_message(&omsg);
			qc_odometry_publish_odometry_laserpoints_message(&olmsg);
			///delete imu readings
			imubuffer.deleteUntil(laser_ts - 0.5);
			iteration++;
			IPC_listen(1);
		}
		IPC_listen(1);
	}
	if (numScans > 0)
		std::cerr << "Avg time=  "<< (avgtime / numScans) * 1e3 << " ms "  << std::endl;
	return 0;
}
