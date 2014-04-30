/**
 * Generates IMU, pose, and slam messages for testing
 */

#include "navmap/kalman_filters.h"
#include "ros/ros.h"
#include "navmap/pose_ukf_msg.h"
#include "geometry_msgs/Twist.h"

using namespace navmap;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "test_node");
    
    ros::NodeHandle n;
    
    ros::Publisher imuPub = n.advertise<geometry_msgs::Twist>("imu_data", 1000);
    ros::Publisher slamPub = 
                        n.advertise<navmap::pose_ukf_msg>("slam_data", 1000);
    ros::Publisher posePub = 
                    n.advertise<navmap::pose_ukf_msg>("pose_est_data", 1000);
    
    ros::Rate loop_rate(100);
    
    
    unsigned int i = 0;
    while (ros::ok()) {
        geometry_msgs::Twist imuMsg;
        memset(&imuMsg, 0, sizeof(geometry_msgs::Twist));
        imuMsg.linear.x = 0.0;
        imuMsg.linear.z = 9.8;
        //imuMsg.angular.x = 0.5;
        imuPub.publish(imuMsg);
        
        if ((i % 5) == 0) {
            navmap::pose_ukf_msg msg;
            msg.x += 1.0;
            msg.y += 1.0;
            msg.z += 1.0;
            //msg.roll -= .001;
            
            posePub.publish(msg);
        }
        
        if ((i % 10) == 0) {
            static navmap::pose_ukf_msg msg;
            msg.x += 1.0;
            msg.y += 1.0;
            msg.z += 1.0;
            //msg.roll -= .001;
            //if (msg.roll < -3.14) { msg.roll = 3.14; }
            
            slamPub.publish(msg);
        }
        
        ros::spinOnce();
        
        i++;
        
        loop_rate.sleep();
    }

}
