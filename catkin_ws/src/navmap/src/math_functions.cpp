#include "navmap/kalman_filters.h"

namespace navmap {


/** Some mathematical functions used in the filters. */


/**
 * Performs the operation x (+) u using the pose update function from
 * [Smith 1990]. x and u are 6DOF vectors, i.e., (x, y, z, roll, pitch, yaw).
 */
void poseUpdate(Vector6d& result,
                const Vector6d& x,
                const Vector6d& u) {
    Eigen::Matrix3d r1, r2, r3;
    rpyToRotationMatrixZYX(r1, x.tail<3>());
    rpyToRotationMatrixZYX(r2, u.tail<3>());
    
    r3 = r1 * r2;
    
    // Compute x,y,z
    Eigen::Vector3d xyz = r1 * u.head<3>() + x.head<3>();
    result.head<3>() = xyz.head<3>();
    
    // Compute roll, pitch, yaw
    // Smith 1990 represents yaw as phi and roll as psi, unlike Kumar et al.
    // which represent yaw as psi and roll as phi
    result(5) = atan2(r3(1,0), r3(0,0)); // phi (yaw)
    
    result(4) = atan2(-r3(2,0), r3(0,0) * cos(result(5)) + 
                                r3(1,0) * sin(result(5))); // theta (pitch)

    result(3) = atan2(r3(0,2) * sin(result(5)) - r3(1,2) * cos(result(5)),
                      -r3(0,1) * sin(result(5)) + 
                      r3(1,1) * cos(result(5))); // psi (roll)
}

/**
 * Computes the derivative of the quaternion representing orientation given
 * the angular velocity from the IMU.
 */
Eigen::Quaterniond quat_deriv(const Eigen::Vector3d& angularVel,
                              const Eigen::Quaterniond& q) {
    // Equation 105 on pg. 12 of
    // http://www-users.cs.umn.edu/~trawny/Publications/Quaternions_3D.pdf
   
    // Multiply by 1/2 factor inside of angular velocity quaternion
    Eigen::Quaterniond angularVelQuat(0.0, 0.5 * angularVel(0),
                                           0.5 * angularVel(1),
                                           0.5 * angularVel(2));
    
    return angularVelQuat * q;
}

/**
 * Given roll, pitch, and yaw, determine the ZYX rotation matrix.
 */
void rpyToRotationMatrixZYX(Eigen::Matrix3d& out,
                            const Eigen::Vector3d& rpy) {
    // Note: using notation from Smith 1990 paper, which is different from
    // Kumar papers, i.e., phi is yaw instead of psi
    double psi = rpy[0]; // Roll
    double theta = rpy[1]; // Pitch
    double phi = rpy[2]; // Yaw
    
    out(0,0) = cos(phi) * cos(theta);
    out(1,0) = sin(phi) * cos(theta);
    out(2,0) = -sin(theta);
    
    out(0,1) = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);
    out(1,1) = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
    out(2,1) = cos(theta) * sin(psi);
    
    out(0,2) = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);
    out(1,2) = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
    out(2,2) = cos(theta) * cos(psi);
}

} // End namespace
