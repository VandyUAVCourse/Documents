#ifndef _URG_DEVICE_DRIVER_H_
#define _URG_DEVICE_DRIVER_H_


#include <sys/time.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <ipcMessages/qc_laser_messages.h>
#include <utils_global/timeval_ops.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG 1

typedef struct HokuyoRangeReading{
	int timestamp_hokuyo;
	int status;
	int n_ranges;
	int ranges[URG_MAX_BEAMS];
	int startStep;
	int endStep;
	int clusterCount;
	struct timeval timestamp;
} HokuyoRangeReading;

typedef struct HokuyoURG{
	int fd;
	int isProtocol2;
	int isContinuous;
	int isInitialized;
} HokuyoURG;

// opens the urg, returns <=0 on failure
int hokuyo_open(HokuyoURG* urg, const char* filename);

// initializes the urg and sets it to the new scip2.0 protocol
// returns <=0 on failure
int hokuyo_init(HokuyoURG* urg);

// reads a packet into the buffer
int hokuyo_readPacket(HokuyoURG* urg, char* buf, int bufsize, int faliures);

// starts the continuous mode
int hokuyo_startContinuous(HokuyoURG* urg, int startStep, int endStep, int clusterCount);

// stop the continuous mode
int hokuyo_stopContinuous(HokuyoURG* urg);

//reset device-> clock is set to 0 of hokuyo
int hokuyo_reset(HokuyoURG* urg);

//close device
int hokuyo_close(HokuyoURG* urg);

//parse laser message
//void hokuyo_parseReading(HokuyoRangeReading* r, char* buffer);

int hokuyo_getQCLaserMessage(HokuyoURG* urg, qc_laser_laser_message* msg, int* diffMS);



///try to estimate difference between onboard clock and hokuyo clock:
///return (double) starting time of hokuyo in local clock frame
///return (int): status: 0== failed, 1==success
///note: this should be done after a reset, since the internal clock iof the hokuyo is reset
int hokuyo_estimateStartingTime(HokuyoURG* urg, struct timeval* startTime);

int hokuyo_estimateDeltaT(HokuyoURG* urg, int* deltaMS);
	

#ifdef __cplusplus
}
#endif

#endif
