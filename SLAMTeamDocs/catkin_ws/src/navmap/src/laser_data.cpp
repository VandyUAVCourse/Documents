// Alex Meyer
// NavMap
// 3/22/2014

#include <string>
#include <iostream>
#include <cmath>
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

const std::string LASER_DATA = "/scan";
const std::string LASER_NORM = "/laser_norm";
const std::string LASER_UP = "/laser_up";
const std::string LASER_DOWN = "/laser_down";
const int QUEUE_SIZE = 10;
const int REFRESH_RATE = 30; // in hz
const double RADIUS_TO_MIRROR_REFLECT_POINT = .088; // In meters
const double HOKUYO_MIN_ANG = -2.08621382713;
const double HOKUYO_MAX_ANG = 2.08007788658;
const double PI = 3.1415926;
const double HOKUYO_MEASUREMENT_ANGLE = HOKUYO_MAX_ANG - HOKUYO_MIN_ANG;
const double MIRROR_LOCATION = 2.0943951; // In radians, 0 is front, center LRF, measured to outside edge of mirror
const double MIRROR_UP_ANGLE = 45; // In degrees, 0 is towards LRF
const double MIRROR_DOWN_ANGLE = 135; // In degrees, 0 is towards LRF
const double SQUARE_MIRROR_SIZE = .024; // In meters
const double MOUNT_EXTRA_ON_EACH_SIDE_OF_MIRROR = .024; // In meters
const int READING_VALUE_TRASH = 10;

class SubscribeAndPublishSplitLaserInfo {
public:
	SubscribeAndPublishSplitLaserInfo() {
		laserSub = nodeHandle.subscribe(LASER_DATA, QUEUE_SIZE, 
				&SubscribeAndPublishSplitLaserInfo::laserReadingCallback, this);
		upPub = nodeHandle.advertise<sensor_msgs::LaserScan>(LASER_UP, QUEUE_SIZE);
		downPub = nodeHandle.advertise<sensor_msgs::LaserScan>(LASER_DOWN, QUEUE_SIZE);
		normPub = nodeHandle.advertise<sensor_msgs::LaserScan>(LASER_NORM, QUEUE_SIZE);
		ros::Rate loop_rate(REFRESH_RATE);
	}
	
	void laserReadingCallback(const sensor_msgs::LaserScan reading)
	{
		double radiansPerReadingValue = (HOKUYO_MAX_ANG - HOKUYO_MIN_ANG) / reading.ranges.size();
		double anglePerMirror = 2 * atan((SQUARE_MIRROR_SIZE / 2) / RADIUS_TO_MIRROR_REFLECT_POINT);
		double angleBlockedPerMount = 2 * atan((MOUNT_EXTRA_ON_EACH_SIDE_OF_MIRROR / 2)
					 / RADIUS_TO_MIRROR_REFLECT_POINT);
		int readingValuesPerMirror = floor(anglePerMirror / radiansPerReadingValue);
		int readingValuesBlockedPerMount = floor(angleBlockedPerMount / radiansPerReadingValue);
		int readingValuesPerNorm = floor(reading.ranges.size() - 
				2 * (readingValuesPerMirror + readingValuesBlockedPerMount));
		double up_ang_min = HOKUYO_MIN_ANG;
		double up_ang_max = HOKUYO_MIN_ANG + anglePerMirror;
		double down_ang_min = HOKUYO_MAX_ANG - anglePerMirror;
		double down_ang_max = HOKUYO_MAX_ANG;
		double norm_ang_min = up_ang_max + angleBlockedPerMount;
		double norm_ang_max = down_ang_min - angleBlockedPerMount;
		sensor_msgs::LaserScan up, down, norm;
		setupLaserScan(reading, up, "laser_up", up_ang_min, up_ang_max, readingValuesPerMirror);
		setupLaserScan(reading, down, "laser_down", down_ang_min, down_ang_max, readingValuesPerMirror);
		setupLaserScan(reading, norm, "laser_norm", norm_ang_min, norm_ang_max, readingValuesPerNorm);
		int curIndexOfCurrent = 0;
		for(int i = 0; i < readingValuesPerMirror; ++i) {
			up.ranges[curIndexOfCurrent++] = reading.ranges[i] - RADIUS_TO_MIRROR_REFLECT_POINT;	
		}
		curIndexOfCurrent = 0;
		for(int i = reading.ranges.size() - readingValuesPerMirror; i < reading.ranges.size(); ++i)
			down.ranges[curIndexOfCurrent++] = reading.ranges[i] - RADIUS_TO_MIRROR_REFLECT_POINT;
		curIndexOfCurrent = 0;
		for(int i = readingValuesPerMirror + readingValuesBlockedPerMount;
				 i < reading.ranges.size() - (readingValuesPerMirror + readingValuesBlockedPerMount);
				 ++i) {
			norm.ranges[curIndexOfCurrent++] = reading.ranges[i];
		}
		upPub.publish(up);
		downPub.publish(down);
		normPub.publish(norm);
	}

	void setupLaserScan(const sensor_msgs::LaserScan& reading,
				sensor_msgs::LaserScan& msg, 
				std::string frame_id, 
				double angle_min, 
				double angle_max, 
				int num_readings) {
		msg.header.stamp = reading.header.stamp;
		msg.header.frame_id = frame_id;
		msg.angle_min = angle_min;
		msg.angle_max = angle_max;
		msg.angle_increment = reading.angle_increment;
		msg.time_increment = reading.time_increment;
		msg.range_min = reading.range_min;
		msg.range_max = reading.range_max;
		msg.ranges.resize(num_readings);
	}

private:
	ros::NodeHandle nodeHandle;
	ros::Subscriber laserSub;
	ros::Publisher upPub;
	ros::Publisher downPub;
	ros::Publisher normPub;	
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_pub_sub");
    SubscribeAndPublishSplitLaserInfo splitPub;
    ros::spin();
    return 0;
}
