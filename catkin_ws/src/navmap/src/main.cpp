#include "ros/ros.h"

#include "pose_estimator/point2d.h"
#include "pose_estimator/tree2d.h"
#include "pose_estimator/icp.h"
#include "pose_estimator/pose_estimator_msg.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "laser_geometry/laser_geometry.h"
#include "imu_broadcast/attitude.h"

#include <iostream>
#include <vector>
#include <cstdlib>

const std::string IMU_DATA_IN = "/imu_attitude";
const std::string LASER_NORM_IN = "/laser_norm";
const int QUEUE_SIZE = 10;

//may need to change these variables to get faster or more accurate solutions
int depth_of_tree;  
int iterations;

int curCount = 0;

// Publisher
ros::Publisher pub;
ros::Subscriber imuSub;

std::vector<point2d_t> reference;

double imu_roll;
double imu_pitch;

laser_geometry::LaserProjection projector_;

Eigen::Matrix3f set_roll(float roll) {
    Eigen::Matrix3f rot;
	rot(0, 0) = cos(roll);
	rot(0, 1) = 0;
	rot(0, 2) = sin(roll);
	rot(1, 0) = 0;
	rot(1, 1) = 1;
	rot(1, 2) = 0;
	rot(2, 0) = -sin(roll);
	rot(2, 1) = 0;
	rot(2, 2) = cos(roll);
    return rot;
}

Eigen::Matrix3f set_pitch(float pitch) {
    Eigen::Matrix3f rot;
	rot(0, 0) = cos(pitch);
	rot(0, 1) = -sin(pitch);
	rot(0, 2) = 0;
	rot(1, 0) = sin(pitch);
	rot(1, 1) = cos(pitch);
	rot(1, 2) = 0;
	rot(2, 0) = 0;
	rot(2, 1) = 0;
	rot(2, 2) = 1;
    return rot;
}

void imuReadingCallback(const imu_broadcast::attitude attitude) {
    imu_roll = attitude.roll;
    imu_pitch = attitude.pitch;
}

void adjustByIMUData(sensor_msgs::PointCloud& cloud) {
	for(int i = 0; i < cloud.points.size(); ++i) {
		geometry_msgs::Point32 curPoint = cloud.points[i];
		Eigen::Vector3f curVec(curPoint.x, curPoint.y, curPoint.z);
		curVec = set_pitch(imu_pitch) * set_roll(imu_roll) * curVec;
		cloud.points[i].x = curVec[0];
		cloud.points[i].y = curVec[1];
		cloud.points[i].z = curVec[2];
	}
}


void laser_receive(const sensor_msgs::LaserScan::ConstPtr& reading) {
    sensor_msgs::PointCloud cloud;
    projector_.projectLaser(*reading, cloud);
	adjustByIMUData(cloud);

	std::vector<point2d_t> target;
    
	for(int i = 0; i < cloud.points.size(); ++i) {
		point2d_t pt;
		pt.x = cloud.points[i].x;
		pt.y = cloud.points[i].y;
		target.push_back(pt);
	} 

	// build tree for search
	if(reference.size() != 0) {
		tree2d_t *tree = tree2d_build(reference.begin(), reference.end(), X, depth_of_tree);
		if (!tree) {
			std::cout << "failed!" << std::endl;
			return;
		}

		reference = target;

		// find pose
		pose_t pose;
		pose.p.x = 0;
		pose.p.y = 0;
		pose.yaw = 0;
		pose = icp_align(tree, target, pose, iterations);
		std::cout << pose << std::endl;
		
		pose_estimator::pose_estimator_msg outMsg;
		outMsg.x = pose.p.x;
		outMsg.y = pose.p.y;
		outMsg.yaw = pose.yaw;
		
		pub.publish(outMsg);
		
		// free tree
		tree2d_free(&tree);
		if (tree) {
			std::cout << "failed!" << std::endl;
			return;
		}
	} else {
		reference = target;
	}
}

int main(int argc, char** argv)
{
	imu_roll = 0;
	imu_pitch = 0;
    // Initialize ros
    ros::init(argc, argv, "pose_estimator");
    ros::NodeHandle n;
    
    // Retrieve parameters
    if (!n.getParam("pose_estimator_depth", depth_of_tree)) {
        depth_of_tree = 100;
        n.setParam("pose_estimator_depth", depth_of_tree);
    }
    
    if (!n.getParam("pose_estimator_iterations", iterations)) {
        iterations = 200;
        n.setParam("pose_estimator_iterations", iterations);
    }

	imuSub = n.subscribe(IMU_DATA_IN, QUEUE_SIZE, imuReadingCallback);

    ros::Subscriber sub = n.subscribe(LASER_NORM_IN, QUEUE_SIZE, laser_receive);
    
    pub = n.advertise<pose_estimator::pose_estimator_msg>("pose_estimate_odom",
                                                          1000);
    
    // Run message loops
    ros::spin();

    return 0;
}
