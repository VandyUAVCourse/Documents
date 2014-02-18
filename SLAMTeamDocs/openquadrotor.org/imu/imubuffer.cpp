#include "imubuffer.h"

IMUBuffer::IMUBuffer(int _bufferSize){
	bufferSize = _bufferSize;
	timestamps.clear();
	msgBuffer.clear();
}

void IMUBuffer::addMessage(qc_imu_imu_message& msg){
	msgBuffer.push_back(msg);
	timestamps.push_back(msg.timestamp_sec + 1e-6*msg.timestamp_usec);
	if (bufferSize > 0 && (int) msgBuffer.size() > bufferSize){
		msgBuffer.pop_front();
		timestamps.pop_front();
	}
}

bool IMUBuffer::getClosestIMUMessage(qc_imu_imu_message& msg, double timestamp){
	///search for the timestamp closest to the desired one
	if (msgBuffer.size() == 0)
		return false;
	std::list<qc_imu_imu_message>::iterator it = msgBuffer.begin();
	std::list<double>::iterator ts = timestamps.begin();
	
	///assuming the list is ordered ascending w.r.t. time
	double deltaT = 1e10;
	bool found = false;
	while (it != msgBuffer.end()){
		if (fabs(*ts - timestamp) < deltaT){
			deltaT = fabs(*ts - timestamp);
			msg = *it;
			found = true;
		} else {
			return found;
		}
		it++;
		ts++;
	}
	return false;
}


bool IMUBuffer::getInterpolatedIMUMessage(qc_imu_imu_message& msg, double timestamp){
	if (msgBuffer.size() == 0)
		return false;
	///assuming the list is ordered ascending w.r.t. time
	std::list<qc_imu_imu_message>::iterator it = msgBuffer.begin();
	std::list<double>::iterator ts = timestamps.begin();
	///find first point (just before timestamp, therefore *ts must be smaller)
	qc_imu_imu_message before = msg; ///avoid warning
	qc_imu_imu_message after = msg; ///avaoid warning
	
	
	double deltaT1 = 1e10;
	bool found1 = false;
	double found1TS = 0;
	while (it != msgBuffer.end()){
		if (*ts <= timestamp && (timestamp - *ts) < deltaT1){
			deltaT1 = timestamp - *ts;
			before = *it;
			found1TS = *ts;
			found1 = true;
		} else {
// 			it++;
// 			ts++;
			break;
		}
		it++;
		ts++;
	}
	
	double deltaT2 = 1e10;
	bool found2 = false;
	double found2TS = 0;
	while (it != msgBuffer.end()){
		if (*ts < timestamp){ ///shouldnt happen.. for security
			it++;
			ts++;
#ifdef DEBUG
			std::cerr << "##WARNING: searching for 2nd imu message and *ts < timestamp!!!!" << std::endl;
#endif
		} else {
			if (timestamp <= *ts && (*ts - timestamp) < deltaT2){
				deltaT2 = *ts - timestamp;
				after = *it;
				found2TS = *ts;
				found2 = true;
			} else {
				break;
			}
			it++;
			ts++;
		}
		
	}
// 	std::cerr << "Interpolating between " << found1TS << " and " << found2TS << std::endl;
	if (!found1 && !found2)
		return false;
	if (found1 && !found2){
		msg = before;
		return true;
	}
	if (!found1 && found2){
		msg = after;
		return true;
	}
	///two messages found
	///interpolate magnetometers, accelerometers and gyros
	///slerp the quaternion
	msg.timestamp_sec = (long) timestamp;
	msg.timestamp_usec = (long) ((timestamp - msg.timestamp_sec) * 1e6);
	double lambda;
	timestamp = timestamp - found1TS;
	found2TS = found2TS - found1TS;
	lambda = timestamp / found2TS;
	lambda = lambda > 1 ? 1 : lambda;
	lambda = lambda < 0 ? 0 : lambda;
	double r_lambda = 1 - lambda;
	
	///linear interpolation for acc, mag and gyro
	msg.accX = r_lambda * before.accX + lambda * after.accX;
	msg.accY = r_lambda * before.accY + lambda * after.accY;
	msg.accZ = r_lambda * before.accZ + lambda * after.accZ;
	
	msg.magX = r_lambda * before.magX + lambda * after.magX;
	msg.magY = r_lambda * before.magY + lambda * after.magY;
	msg.magZ = r_lambda * before.magZ + lambda * after.magZ;
	
	msg.gyroX = r_lambda * before.gyroX + lambda * after.gyroX;
	msg.gyroY = r_lambda * before.gyroY + lambda * after.gyroY;
	msg.gyroZ = r_lambda * before.gyroZ + lambda * after.gyroZ;
	///slerp for quaternions;
	
	
	Quaternion<double> q1(before.q0, before.q1, before.q2, before.q3);
	Quaternion<double> q2(after.q0, after.q1, after.q2, after.q3);
	Quaternion<double> q = slerp(q1,q2,lambda);
	msg.q0 = q.w;
	msg.q1 = q.x;
	msg.q2 = q.y;
	msg.q3 = q.z;
	return true;
}


void IMUBuffer::deleteUntil (double timestamp, bool deleteAll){
	if (deleteAll){
		timestamps.clear();
		msgBuffer.clear();
	}
	while (timestamps.size() > 0 && timestamp > timestamps.front()){
		timestamps.pop_front();
		msgBuffer.pop_front();
	}
}
