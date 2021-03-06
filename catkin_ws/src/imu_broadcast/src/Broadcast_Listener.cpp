#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void broadcastCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Broadcast received: ", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "broadcast_listener");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("msg_broadcast", 1000, broadcastCallback);

  ros::spin();

  return 0;
}
