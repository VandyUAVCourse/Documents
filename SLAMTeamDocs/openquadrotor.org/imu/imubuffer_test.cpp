#include "imubuffer.h"
#include <ipcMessages/qc_imu_messages.h>
#include <math_stuff/transformation3.hh>
#include <cstdlib>

void printMsg(const qc_imu_imu_message& msg, const int i){
	std::cerr << "Message No= " << i << std::endl;
	std::cerr << "ACC= " << msg.accX << " " << msg.accY << " " << msg.accZ << std::endl;
	std::cerr << "MAG= " << msg.magX << " " << msg.magY << " " << msg.magZ << std::endl;
	std::cerr << "GYRO= " << msg.gyroX << " " << msg.gyroY << " " << msg.gyroZ << std::endl;
	Quaternion<double> q(msg.q0, msg.q1, msg.q2, msg.q3);
	DVector3 angles = q.toAngles();
	std::cerr << "R_P_Y= " << (angles.x()/M_PI)*180 << " " << (angles.y()/M_PI)*180 << " " << (angles.z()/M_PI)*180 << std::endl;
	std::cerr << "TS= " << msg.timestamp_sec + 1e-6 * msg.timestamp_usec << std::endl;
	std::cerr << std::endl;
	
}

int main (int argc, char* argv[]){
	int NMSG = 10;
	qc_imu_imu_message msg[NMSG];
	IMUBuffer imubuffer(20);
	
	for (int i=0; i<NMSG; i++){
		msg[i].accX = i*10;
		msg[i].accY = i*10;
		msg[i].accZ = i*10;
		msg[i].magX = i*10;
		msg[i].magY = i*10;
		msg[i].magZ = i*10;
		msg[i].gyroX = i*10;
		msg[i].gyroY = i*10;
		msg[i].gyroZ = i*10;
		double roll = M_PI * 2 * i/(double)NMSG;
		double pitch = 0; //M_PI * 2 * (i+5)/(double)NMSG;
		double yaw = 0; //M_PI * 2 * (i-5)/(double)NMSG;
		Quaternion<double> q(roll,pitch,yaw);
		msg[i].q0 = q.w;
		msg[i].q1 = q.x;
		msg[i].q2 = q.y;
		msg[i].q3 = q.z;
		msg[i].timestamp_sec = i;
		msg[i].timestamp_usec = abs((long)(rand() % 1000000));
		printMsg(msg[i],i);
		imubuffer.addMessage(msg[i]);
	}
	
	///test
	qc_imu_imu_message m;
	double testTS = 7.76049;
	std::cerr << "######### TEST ##########" << std::endl;
	std::cerr << " searching for: " << testTS << std::endl;
	imubuffer.getClosestIMUMessage(m, testTS);
	printMsg(m,-1);
	imubuffer.getInterpolatedIMUMessage(m, testTS);
	printMsg(m,-1);
	
	testTS = 17.332;
	std::cerr << "######### TEST ##########" << std::endl;
	std::cerr << " searching for: " << testTS << std::endl;
	imubuffer.getClosestIMUMessage(m, testTS);
	printMsg(m,-1);
	imubuffer.getInterpolatedIMUMessage(m, testTS);
	printMsg(m,-1);
	
	imubuffer.deleteUntil(6.);
	testTS = 6.1;
	std::cerr << "######### TEST ##########" << std::endl;
	std::cerr << " searching for: " << testTS << std::endl;
	imubuffer.getClosestIMUMessage(m, testTS);
	printMsg(m,-1);
	imubuffer.getInterpolatedIMUMessage(m, testTS);
	printMsg(m,-1);
	
	testTS = 2.1;
	std::cerr << "######### TEST ##########" << std::endl;
	std::cerr << " searching for: " << testTS << std::endl;
	imubuffer.getClosestIMUMessage(m, testTS);
	printMsg(m,-1);
	imubuffer.getInterpolatedIMUMessage(m, testTS);
	printMsg(m,-1);
	
}

