#include "ros/ros.h"

#include "pose_estimator/point2d.h"
#include "pose_estimator/tree2d.h"
#include "pose_estimator/icp.h"
#include "pose_estimator/pose_estimator_msg.h"
#include "pose_estimator/proj_2_5d_msg.h"

#include <iostream>
#include <vector>
#include <cstdlib>


//may need to change these variables to get faster or more accurate solutions
int depth_of_tree;  
int iterations;

// Publisher
ros::Publisher pub;

/**
 * Receive the 2.5d projection data
 */
void proj_receive(pose_estimator::proj_2_5d_msg msg) {
	/////CALL FROM NOLAN 2D to get vector of reference and target point clouds
	std::vector<point2d_t> reference, target;
    
    for (pose_estimator::proj_2_5d_msg::_refPoints_type::iterator it = msg.refPoints.begin();
         it != msg.refPoints.end(); ++it) {
        point2d_t pt;
        pt.x = it->x;
        pt.y = it->y;
        
        reference.push_back(pt);
    }
    
    for (pose_estimator::proj_2_5d_msg::_targetPoints_type::iterator it = msg.targetPoints.begin();
         it != msg.targetPoints.end(); ++it) {
        point2d_t pt;
        pt.x = it->x;
        pt.y = it->y;
        
        target.push_back(pt);
    }
    

	// build tree for search
	tree2d_t *tree = tree2d_build(reference.begin(), reference.end(), X, depth_of_tree);
	if (!tree) {
		std::cout << "failed!" << std::endl;
		return;
	}


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
}

int main(int argc, char** argv)
{
    // Initialize ros
    ros::init(argc, argv, "pose_estimator");
    ros::NodeHandle n;
    
    // Retrieve parameters
    if (!n.getParam("pose_estimator_depth", depth_of_tree)) {
        depth_of_tree = 5;
        n.setParam("pose_estimator_depth", depth_of_tree);
    }
    
    if (!n.getParam("pose_estimator_iterations", iterations)) {
        iterations = 10;
        n.setParam("pose_estimator_iterations", iterations);
    }
    
    ros::Subscriber sub = n.subscribe("proj_2_5_d", 1, proj_receive);
    
    pub = n.advertise<pose_estimator::pose_estimator_msg>("pose_estimate_odom",
                                                          1000);
    
    // Run message loops
    ros::spin();

    return 0;
}
