#include <ipcMessages/qc_odometry_messages.h>
#include <ipcMessages/qc_imu_messages.h>
#include <ipcInterfaces/qc_odometry_interface.h>
#include <ipcInterfaces/qc_ipc_interface.h>
#include <ipcInterfaces/qc_imu_interface.h>
#include <iostream>
#include <math_stuff/transformation3.hh>

qc_imu_imu_message imsg;
qc_odometry_odometry_message omsg;
qc_odometry_odometry_message prev_omsg;
qc_odometry_velocity_message vmsg;
qc_odometry_velocity_message prev_vmsg;
bool newImuMessageAvailable = false;
int msgcounter = 0;
bool validData = false;


///handler needed for odom message and velocity message
void qc_imu_imu_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &imsg, sizeof(imsg));
	newImuMessageAvailable = true;
	IPC_freeByteArray(callData);
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&imsg);
}

void qc_odometry_odometry_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	prev_omsg = omsg;
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &omsg, sizeof(omsg));
	if (newImuMessageAvailable){
		newImuMessageAvailable = false;
		prev_vmsg = vmsg;
		vmsg.timestamp_sec = omsg.timestamp_sec;
		vmsg.timestamp_usec = omsg.timestamp_usec;
		if (msgcounter > 2)
			validData = true;
		if (validData){
			double dtv = omsg.timestamp_sec - prev_omsg.timestamp_sec + 1e-6 * (omsg.timestamp_usec - prev_omsg.timestamp_usec);
			double dta = vmsg.timestamp_sec - prev_vmsg.timestamp_sec + 1e-6 * (vmsg.timestamp_usec - prev_vmsg.timestamp_usec);
			vmsg.x = omsg.x;
			vmsg.y = omsg.y;
			vmsg.z = omsg.z;
			vmsg.q0 = omsg.q0;
			vmsg.q1 = omsg.q1;
			vmsg.q2 = omsg.q2;
			vmsg.q3 = omsg.q3;
			vmsg.roll = omsg.roll;
			vmsg.pitch = omsg.pitch;
			vmsg.yaw = omsg.yaw;
			Transformation3<double> delta;
			Transformation3<double> p0(DVector3(prev_omsg.x, prev_omsg.y, prev_omsg.z), Quaternion<double>(prev_omsg.q0, prev_omsg.q1, prev_omsg.q2, prev_omsg.q3));
			Transformation3<double> p1(DVector3(omsg.x, omsg.y, omsg.z), Quaternion<double>(omsg.q0, omsg.q1, omsg.q2, omsg.q3));
			delta = p0.inv() * p1;
			///velocity && acceleration based on omsg
			if (dtv > 1e-7){
				dtv = 1./dtv;
				vmsg.vx = delta.translationVector.x() * dtv;
				vmsg.vy = delta.translationVector.y() * dtv;
				vmsg.vz = delta.translationVector.z() * dtv;
				if (dta > 1e-7){
					///take into account the different orientations of the velocities!
					p0.translationVector = DVector3(0,0,0);
					p0.rotationQuaternion = Quaternion<double>(prev_vmsg.q0, prev_vmsg.q1, prev_vmsg.q2, prev_vmsg.q3);
					p0.translationVector = p0 * DVector3(prev_vmsg.vx, prev_vmsg.vy, prev_vmsg.vz);
					
					p1.translationVector = DVector3(0,0,0);
					p1.rotationQuaternion = Quaternion<double>(vmsg.q0, vmsg.q1, vmsg.q2, vmsg.q3);
					p1.translationVector = p1 * DVector3(vmsg.vx, vmsg.vy, vmsg.vz);
					delta = p0.inv() * p1;
					dta = 1./dta;
					vmsg.avx = delta.translationVector.x() * dta;
					vmsg.avy = delta.translationVector.y() * dta;
					vmsg.avz = delta.translationVector.z() * dta;
				} else {
					vmsg.avx = vmsg.avy = vmsg.avz = 0;
				}
			} else {
				vmsg.avx = vmsg.avy = vmsg.avz = 0;
				vmsg.vx = vmsg.vy = vmsg.vz = 0;
			}
			
			///acceleration based on imu data
			///FIXME
			
			
			
			///acceleration based on tan roll and tan pitch
			Quaternion<double> q(imsg.q0, imsg.q1, imsg.q2, imsg.q3);
			DVector3 an = q.toAngles();
			vmsg.axtanp = tan(an.pitch()) * 9.81;
			vmsg.aytanr = tan(an.roll()) * 9.81;
			
			
			
			qc_odometry_publish_velocity_message(&vmsg);
		}
	}
	IPC_freeByteArray(callData);
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&omsg);
	msgcounter++;
	if (!(msgcounter%10))
		std::cerr << "." << std::flush;
}




int main (int argc, char* argv[]){
	
	if (qc_ipc_connect(argv[0]) <= 0)
		return 0;
	qc_imu_subscribe_imu_message(qc_imu_imu_message_handler, 1, NULL);
	qc_odometry_subscribe_odometry_message(qc_odometry_odometry_message_handler, 1, NULL);
	IPC_dispatch();
	
	return 0;
}
