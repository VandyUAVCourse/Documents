#include "laserutils.h"

void LaserUtils::deepMessageCopy(const qc_laser_laser_message& src, qc_laser_laser_message& dest){
	if (src.num_ranges != 0){
		if (dest.ranges == 0){
			dest.ranges = new int [src.num_ranges];
		} else {
			if (dest.num_ranges != src.num_ranges){
				delete[] dest.ranges;
				dest.ranges = new int [src.num_ranges];
			}
		}
	}
	dest.num_ranges = src.num_ranges;
	if (src.num_ranges != 0){
		for (int i=0; i<src.num_ranges; i++)
			dest.ranges[i] = src.ranges[i];
	}
	dest.startStep = src.startStep;
	dest.stopStep = src.stopStep;
	dest.startAngle = src.startAngle;
	dest.incrementAngle = src.incrementAngle;
	dest.clusterCount = src.clusterCount;
	dest.status = src.status;
	dest.internal_timestamp_hokuyo = src.internal_timestamp_hokuyo;
	dest.timestamp_hokuyo_init_sec = src.timestamp_hokuyo_init_sec;
	dest.timestamp_hokuyo_init_usec = src.timestamp_hokuyo_init_usec;
	dest.timestamp_sec = src.timestamp_sec;
	dest.timestamp_usec = src.timestamp_usec;
}



