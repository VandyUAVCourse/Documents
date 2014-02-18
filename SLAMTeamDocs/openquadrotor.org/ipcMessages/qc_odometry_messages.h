#ifndef _QC_ODOMETRY_MESSAGES_H
#define _QC_ODOMETRY_MESSAGES_H


struct qc_odometry_odometry_message {
	float x;
	float y;
	float z;
	int num_z_readings;
	float* zr;
	float q0; ///real part of rotational quaternion
	float q1; /// q1-q3 imaginery part
	float q2;
	float q3;
	float roll; ///the rotation in roll pitch yaw
	float pitch;
	float yaw;
	long timestamp_sec;
	long timestamp_usec;
};

#define QC_ODOMETRY_ODOMETRY_MESSAGE_NAME "qc_odometry_odometry_message"
#define QC_ODOMETRY_ODOMETRY_MESSAGE_FMT  "{float, float, float, int, <float:4>, float, float, float, float, float, float, float, long, long}"
#define QC_ODOMETRY_ODOMETRY_MESSAGE_TYPE IPC_VARIABLE_LENGTH


struct qc_odometry_odometry_laserpoints_message {
	float x;
	float y;
	float z;
	int num_z_readings;
	float* zr;
	float q0; ///real part of rotational quaternion
	float q1; /// q1-q3 imaginery part
	float q2;
	float q3;
	float roll; ///the rotation in roll pitch yaw
	float pitch;
	float yaw;
	int num_laser_readings;
	float* lx;
	float* ly;
	long timestamp_sec;
	long timestamp_usec;
};

#define QC_ODOMETRY_ODOMETRY_LASERPOINTS_MESSAGE_NAME "qc_odometry_odometry_laserpoints_message"
#define QC_ODOMETRY_ODOMETRY_LASERPOINTS_MESSAGE_FMT  "{float, float, float, int,  <float:4>, float, float, float, float, float, float, float, int, <float:13>, <float:13>, long, long}"
#define QC_ODOMETRY_ODOMETRY_MESSAGE_LASERPOINTS_TYPE IPC_VARIABLE_LENGTH

struct qc_odometry_velocity_message {
	float x;
	float y;
	float z;
	float q0;
	float q1;
	float q2;
	float q3;
	float roll; ///the rotation in roll pitch yaw
	float pitch;
	float yaw;
	float vx;
	float vy;
	float vz;
	float avx; ///acceleration based on two velocity informations
	float avy;
	float avz;
	float aix; ///accleration based on imu acceleration values (compensated for gravity)
	float aiy;
	float aiz;
	float axtanp; ///acceleration estimated based on the current angle pitch 
	float aytanr; ///same as above but on roll angle
	long timestamp_sec;
	long timestamp_usec;
};

#define QC_ODOMETRY_VELOCITY_MESSAGE_NAME "qc_odometry_velocity_message"
#define QC_ODOMETRY_VELOCITY_MESSAGE_FMT "{float, float, float, float, float, float, float, float, float, float, float, float, float, float, float, float, float, float, float, float, float, long, long}"


#endif // _QC_ODOMETRY_MESSAGES_H
