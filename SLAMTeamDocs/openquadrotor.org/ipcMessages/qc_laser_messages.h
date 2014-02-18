#ifndef _QC_LASER_MESSAGES_H_
#define _QC_LASER_MESSAGES_H_

#define URG_MAX_BEAMS 768
#define URG_BUFSIZE 8192


typedef struct qc_laser_laser_message {
	int startStep;
	int stopStep;
	float startAngle;
	float incrementAngle;
	int clusterCount;
	int status;
	int internal_timestamp_hokuyo; ///internal counter of the hokuyo
	long timestamp_hokuyo_init_sec; ///timestamp of the host, when hokuyo was enabled.
	long timestamp_hokuyo_init_usec;
	int num_ranges;
	int *ranges;
	long timestamp_sec;
	long timestamp_usec;
} qc_laser_laser_message;


#define QC_LASER_LASER1_MESSAGE_NAME "qc_laser_laser1_message"
#define QC_LASER_LASER1_MESSAGE_FMT "{int, int, float, float, int, int, int, long, long, int, <int:10>, long, long}"

#define QC_LASER_LASER2_MESSAGE_NAME "qc_laser_laser2_message"
#define QC_LASER_LASER2_MESSAGE_FMT "{int, int, float, float, int, int, int, long, long, int, <int:10>, long, long}"


#endif // _QC_LASER_MESSAGES_H_
