#ifndef _PLAYER_UTILS_H_
#define _PLAYER_UTILS_H_

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>


#include <ipcMessages/qc_imu_messages.h>
#include <ipcMessages/qc_laser_messages.h>
#include <ipcMessages/qc_odometry_messages.h>
#include <ipcMessages/qc_flightcontrol_messages.h>
using std::istringstream;
using std::string;
class PlayerUtils{
	public:
	static void readLaserMessage(qc_laser_laser_message& lmsg, istringstream& lis);
	static void readOdometryMessage(qc_odometry_odometry_message& omsg, istringstream& lis);
	static void readOdometryLaserpointsMessage(qc_odometry_odometry_laserpoints_message& olmsg, istringstream& lis);
	static void readImuMessage(qc_imu_imu_message& imsg, istringstream& lis);
	static void readVelocityMessage(qc_odometry_velocity_message& vmsg, istringstream& lis);
	static void readFlightControlMessage(qc_flightcontrol_flightcontrol_message& fcmsg, istringstream& lis);
	static void readOffboardCommandMessage(qc_flightcontrol_offboard_command_message& ocmsg, istringstream& lis);
};
#endif // _PLAYER_UTILS_H_
