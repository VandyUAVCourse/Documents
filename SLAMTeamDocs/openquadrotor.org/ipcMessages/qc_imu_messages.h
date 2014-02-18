#ifndef _QC_IMU_MESSAGES_H
#define _QC_IMU_MESSAGES_H


struct qc_imu_imu_message {
	float accX;    ///< acceleration in x direction
	float accY;    ///< acceleration in y direction
	float accZ;    ///< acceleration in z direction
	float q0;      ///< Quaternion describing the rotation, w elem
	float q1;      ///< Quaternion describing the rotation, x elem
	float q2;      ///< Quaternion describing the rotation, y elem
	float q3;      ///< Quaternion describing the rotation, z elem
	float magX;    ///< magnetic field vector
	float magY;    ///< magnetic field vector
	float magZ;    ///< magnetic field vector
	float gyroX;   ///< gyro rotvel in radians/s
	float gyroY;   ///< gyro rotvel in radians/s
	float gyroZ;   ///< gyro rotvel in radians/s
	long timestamp_sec;
	long timestamp_usec;
};

#define QC_IMU_IMU_MESSAGE_NAME "qc_imu_imu_message"
#define QC_IMU_IMU_MESSAGE_FMT  "{float, float, float, float, float, float, float, float, float, float, float, float, float, long, long}"
#define QC_IMU_IMU_MESSAGE_TYPE IPC_VARIABLE_LENGTH


#endif // _QC_IMU_MESSAGES_H
