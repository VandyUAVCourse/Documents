#include "playerutils.h"

void PlayerUtils::readLaserMessage(qc_laser_laser_message& lmsg, istringstream& lis){
	lis >> lmsg.startStep;
	lis >> lmsg.stopStep;
	lis >> lmsg.startAngle;
	lis >> lmsg.incrementAngle;
	lis >> lmsg.clusterCount;
	lis >> lmsg.status;
	lis >> lmsg.internal_timestamp_hokuyo;
	lis >> lmsg.timestamp_hokuyo_init_sec;
	lis >> lmsg.timestamp_hokuyo_init_usec;
	int newSize = 0;
	lis >> newSize;
	if (lmsg.num_ranges != newSize){
		delete[] lmsg.ranges;
		lmsg.ranges = 0;
	}
	lmsg.num_ranges = newSize;
	if (!lmsg.ranges)
		lmsg.ranges = new int[lmsg.num_ranges];
	for (int i=0; i<lmsg.num_ranges; i++)
		lis >> lmsg.ranges[i];
	lis >> lmsg.timestamp_sec;
	lis >> lmsg.timestamp_usec;
}

void PlayerUtils::readOdometryMessage(qc_odometry_odometry_message& omsg, istringstream& lis){
	lis >> omsg.x;
	lis >> omsg.y;
	lis >> omsg.z;
	int newSize = 0;
	lis >> newSize;
	if (omsg.num_z_readings != newSize && omsg.zr){
		delete[] omsg.zr;
		omsg.zr = 0;
	}
	omsg.num_z_readings = newSize;
	if (!omsg.zr && omsg.num_z_readings > 0){
		omsg.zr = new float[omsg.num_z_readings];
	}
	for (int i=0; i<omsg.num_z_readings; i++)
		lis >> omsg.zr[i];
	lis >> omsg.q0;
	lis >> omsg.q1;
	lis >> omsg.q2;
	lis >> omsg.q3;
	lis >> omsg.roll;
	lis >> omsg.pitch;
	lis >> omsg.yaw;
	lis >> omsg.timestamp_sec;
	lis >> omsg.timestamp_usec;
}

void PlayerUtils::readOdometryLaserpointsMessage(qc_odometry_odometry_laserpoints_message& olmsg, istringstream& lis){
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
}

void PlayerUtils::readVelocityMessage(qc_odometry_velocity_message& vmsg, istringstream& lis){
	lis >> vmsg.x;
	lis >> vmsg.y;
	lis >> vmsg.z;
	lis >> vmsg.q0;
	lis >> vmsg.q1;
	lis >> vmsg.q2;
	lis >> vmsg.q3;
	lis >> vmsg.roll;
	lis >> vmsg.pitch;
	lis >> vmsg.yaw;
	lis >> vmsg.vx;
	lis >> vmsg.vy;
	lis >> vmsg.vz;
	lis >> vmsg.avx;
	lis >> vmsg.avy;
	lis >> vmsg.avz;
	lis >> vmsg.aix;
	lis >> vmsg.aiy;
	lis >> vmsg.aiz;
	lis >> vmsg.axtanp;
	lis >> vmsg.aytanr;
	lis >> vmsg.timestamp_sec;
	lis >> vmsg.timestamp_usec;
}

void PlayerUtils::readImuMessage(qc_imu_imu_message& imsg, istringstream& lis){
	lis >> imsg.accX;
	lis >> imsg.accY;
	lis >> imsg.accZ;
	lis >> imsg.q0;
	lis >> imsg.q1;
	lis >> imsg.q2;
	lis >> imsg.q3;
	lis >> imsg.magX;
	lis >> imsg.magY;
	lis >> imsg.magZ;
	lis >> imsg.gyroX;
	lis >> imsg.gyroY;
	lis >> imsg.gyroZ;
	lis >> imsg.timestamp_sec;
	lis >> imsg.timestamp_usec;
}

void PlayerUtils::readFlightControlMessage(qc_flightcontrol_flightcontrol_message& fcmsg, istringstream& lis){
	lis >> fcmsg.message_version;
	
	lis >> fcmsg.integral_nick;
	lis >> fcmsg.integral_roll;
	lis >> fcmsg.mean_acceleration_nick;
	lis >> fcmsg.mean_acceleration_roll;
	lis >> fcmsg.gyro_yaw;
	lis >> fcmsg.height_value;
	lis >> fcmsg.height_integral;
	lis >> fcmsg.mixture_value_acceleration;
	lis >> fcmsg.compass_value;
	lis >> fcmsg.battery_voltage;

	lis >> fcmsg.rc_connection;
	lis >> fcmsg.stick_roll;
	lis >> fcmsg.engine_front;
	lis >> fcmsg.engine_back;
	lis >> fcmsg.engine_left;
	lis >> fcmsg.engine_right;
	lis >> fcmsg.mean_acceleration_z;
	lis >> fcmsg.stick_yaw;
	lis >> fcmsg.stick_thrust;
	lis >> fcmsg.stick_pitch;

	lis >> fcmsg.servo;
	lis >> fcmsg.nick;
	lis >> fcmsg.roll;
	lis >> fcmsg.autonomous_enabled;
	lis >> fcmsg.gcm_failures;
	lis >> fcmsg.gcm_delta_roll;
	lis >> fcmsg.gcm_delta_pitch;
	lis >> fcmsg.gcm_delta_yaw;
	lis >> fcmsg.gcm_delta_thrust;
	lis >> fcmsg.measurement_roll;

	lis >> fcmsg.calc_stick_pitch;
	lis >> fcmsg.calc_stick_roll;
	lis >> fcmsg.timestamp_sec;
	lis >> fcmsg.timestamp_usec;
}

void PlayerUtils::readOffboardCommandMessage(qc_flightcontrol_offboard_command_message& ocmsg, istringstream& lis){
	lis >> ocmsg.cmd_delta_roll;
	lis >> ocmsg.cmd_delta_roll_duration;
	lis >> ocmsg.cmd_delta_pitch;
	lis >> ocmsg.cmd_delta_pitch_duration;
	lis >> ocmsg.cmd_delta_yaw;
	lis >> ocmsg.cmd_delta_yaw_duration;
	lis >> ocmsg.cmd_delta_thrust;
	lis >> ocmsg.cmd_delta_thrust_duration;
	lis >> ocmsg.timestamp_sec;
	lis >> ocmsg.timestamp_usec;
}

