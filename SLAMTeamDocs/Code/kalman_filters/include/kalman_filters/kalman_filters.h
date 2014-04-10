#ifndef __KALMAN_FILTERS_H__
#define __KALMAN_FILTERS_H__

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "kalman_filters/state_ukf_msg.h"
#include "kalman_filters/pose_ukf_msg.h"
#include <Eigen/Dense>

namespace kalman_filters {

    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;

    typedef Eigen::Matrix<double, 15, 1> Vector15d;
    typedef Eigen::Matrix<double, 15, 15> Matrix15d;


    /** Functions used by the filters. */
     
    /**
     * Performs the operation x (+) u using the pose update function from
     * [Smith 1990]. x and u are 6DOF vectors, i.e., 
     * (x, y, z, roll, pitch, yaw).
     */
    void poseUpdate(Vector6d& result, const Vector6d& x, const Vector6d& u);
    
    /**
     * Given roll, pitch, and yaw, determine the ZYX rotation matrix.
     */
    void rpyToRotationMatrixZYX(Eigen::Matrix3d& out,
                                const Eigen::Vector3d& rpy);
                                
    /**
     * Computes the derivative of the quaternion representing orientation given
     * the angular velocity from the IMU.
     */
    Eigen::Quaterniond quat_deriv(const Eigen::Vector3d& angularVel,
                                  const Eigen::Quaterniond& q);
}

#endif
