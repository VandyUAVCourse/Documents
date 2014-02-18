#ifndef _IMU_BUFFER_H_
#define _IMU_BUFFER_H_

#include <list>
#include <ipcMessages/qc_imu_messages.h>
#include <math_stuff/transformation3.hh>

#define DEBUG

struct IMUBuffer{
	public:
		///if bufferSize < 1 all messages are stored.
		///then the user should invoke clearUntil in order to keep memory storage feasible
		IMUBuffer(int bufferSize);
		void addMessage(qc_imu_imu_message& imsg);
		///the closest message (closest to timestamp) is returned
		///false if no message available
		bool getClosestIMUMessage(qc_imu_imu_message& msg, double timestamp);
		
		///the 2 closest messages (closest to timestamp) are found
		///then a new message is simulated by interpolation
		///if only one message is found, this message is returned (-> closest)
		///false if no message available
		bool getInterpolatedIMUMessage(qc_imu_imu_message& msg, double timestamp);
		
		///
		void deleteUntil (double timestamp, bool deleteAll = false);
	
	private:
		std::list<qc_imu_imu_message> msgBuffer;
		std::list<double> timestamps;
		int bufferSize;
};
#endif // _IMU_BUFFER_H_
