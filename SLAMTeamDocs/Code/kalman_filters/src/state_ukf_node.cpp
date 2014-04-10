#include "kalman_filters/kalman_filters.h"
#include <eigen3/Eigen/Cholesky>

namespace kalman_filters {

/**
 * Constructor.
 */
StateUKF::StateUKF() {

    // Retrieve parameters
    if (!mN.getParam("state_ukf_alpha", mAlpha)) {
        mAlpha = 1.0;
        mN.setParam("state_ukf_alpha", mAlpha);
    }
    
    if (!mN.getParam("state_ukf_beta", mBeta)) {
        mBeta = 2.0;
        mN.setParam("state_ukf_beta", mBeta);
    }
    
    if (!mN.getParam("state_ukf_kappa", mKappa)) {
        mKappa = 0.0;
        mN.setParam("state_ukf_kappa", mKappa);
    }
    
    if (!mN.getParam("state_ukf_process_error", mProcessError)) {
        mProcessError = 0.1;
        mN.setParam("state_ukf_process_error", mProcessError);
    }
    
    if (!mN.getParam("state_ukf_measurement_error", mMeasurementError)) {
        mMeasurementError = 0.1;
        mN.setParam("state_ukf_measurement_error", mMeasurementError);
    }
    

    if (!mN.getParam("state_ukf_accel_bias_error", mAccelBiasError)) {
        mAccelBiasError = 0.0;
        mN.setParam("state_ukf_accel_bias_error", mAccelBiasError);
    }
    
    if (!mN.getParam("state_ukf_ang_vel_bias_error", mAngVelBiasError)) {
        mAngVelBiasError = 0.0;
        mN.setParam("state_ukf_ang_vel_bias_error", mAngVelBiasError);
    }
    
    double frequency = 100.0;
    
    if (!mN.getParam("state_ukf_process_frequency", frequency)) {
        mN.setParam("state_ukf_process_frequency", frequency);
    }
    
    // Initialize the state and covariance
    mState.fill(0.0);
    mCov = Matrix15d::Identity() * mProcessError;
    mCov(9,9) = mCov(10,10) = mCov(11,11) = mAccelBiasError;
    mCov(12,12) = mCov(13,13) = mCov(14,14) = mAngVelBiasError;
    
    // Use the bias values if set, otherwise leave as 0
    if (!mN.getParam("state_ukf_accel_bias_x", mState(9))) {
        mN.setParam("state_ukf_accel_bias_x", 0);
    }
    
    if (!mN.getParam("state_ukf_accel_bias_y", mState(10))) {
        mN.setParam("state_ukf_accel_bias_y", mState(10));
    }
    
    if (!mN.getParam("state_ufk_accel_bias_z", mState(11))) {
        mN.setParam("state_ukf_accel_bias_z", mState(11));
    }
    
    if (!mN.getParam("state_ukf_ang_vel_bias_x", mState(12))) {
        mN.setParam("state_ukf_ang_vel_bias_x", mState(12));
    }
    
    if (!mN.getParam("state_ukf_ang_vel_bias_y", mState(13))) {
        mN.setParam("state_ukf_ang_vel_bias_y", mState(13));
    }
    
    if (!mN.getParam("state_ufk_ang_vel_bias_z", mState(14))) {
        mN.setParam("state_ukf_ang_vel_bias_z", mState(14));
    }
    
    mAvailableControl = mAvailableMeasurement = false;

    // Create the state publisher
    mPub = mN.advertise<state_ukf::state_ukf_msg>("state_ukf", 1000);
    
    // Create the subscribers to the IMU data nd the pose estimate UKF data
    mImuSub = mN.subscribe("imu_data", 1, &StateUKF::imu_data_receiver, this);
    mPoseSub = mN.subscribe("pose_ukf", 1,
                            &StateUKF::pose_est_receiver, this);
    
    // Create timer used for main loop
    mLoop = mN.createTimer(ros::Duration(1.0 / frequency),
                           &StateUKF::main_loop, this);
                           
    mLastUpdate = ros::Time::now();

}


/**
 * Timer callback. Used to output updated state estimate.
 */
void StateUKF::main_loop(const ros::TimerEvent& e) {
    ros::Time currentTime = ros::Time::now();
    
    double deltaT = (currentTime - mLastUpdate).toSec();

    if (mAvailableControl) {
        // Run process step
        predictionStepState(deltaT);

        mAvailableControl = false;
    }
    
    if (mAvailableMeasurement) {
        // Run measurement update
        measurementUpdateState();

        mAvailableMeasurement = false;
    }
    
    state_ukf::state_ukf_msg msg;
    msg.x = mState(0);
    msg.y = mState(1);
    msg.z = mState(2);
    msg.roll = mState(3);
    msg.pitch = mState(4);
    msg.yaw = mState(5);
    msg.velX = mState(6);
    msg.velY = mState(7);
    msg.velZ = mState(8);

    mPub.publish(msg);
    
    mLastUpdate = currentTime;
}


/**
 * Receiver for data from the IMU.
 */
void StateUKF::imu_data_receiver(const geometry_msgs::TwistConstPtr& msg) {
    mControl << msg->linear.x, msg->linear.y, msg->linear.z,
                msg->angular.x, msg->angular.y, msg->angular.z;
    
    mAvailableControl = true;
}

/**
 * Receiver for data from the pose estimate UKF.
 */
void StateUKF::pose_est_receiver(const state_ukf::pose_ukf_msgConstPtr& msg) {
    mMeasurement << msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw;
    
    mAvailableMeasurement = true;
}


/**
 * Update the state of the system by numerically integrating the dynamic
 * equations from "On the consistency of vision-aided inertial navigation"
 * (equations 7,8).
 */
void StateUKF::stateUpdate(Vector15d& result, const Vector15d& x,
                           const Vector6d& u, double deltaT) const {
    
    // Adjust the linear acceleration and angular velocity measurements
    // using the bias values
    Eigen::Vector3d accelMeasurement = u.head<3>() - x.segment<3>(9);
    Eigen::Vector3d angVelMeasurement = u.tail<3>() - x.segment<3>(12);

    // Gravity
    const Eigen::Vector3d gravity(0.0, 0.0, -9.8);
    
    // Obtain body to world rotation matrix
    Eigen::Matrix3d bodyToWorld;

    rpyToRotationMatrixZYX(bodyToWorld, x.segment<3>(3));
    
    // Quaternion representation of orientation from body to world
    Eigen::Quaterniond q(bodyToWorld);
        
    /**
     * Use 4th-order Runge-Kutta integration to numerically integrate the
     * motion equations
     */
    
    Eigen::Vector3d kv1 = bodyToWorld * accelMeasurement + gravity;

    Eigen::Vector3d kx1 = x.segment<3>(6);
    
    
    Eigen::Quaterniond kq1 = quat_deriv(angVelMeasurement, q);
    Eigen::Quaterniond intermediate(q.w() + kq1.w() * deltaT / 2.0,
                                    q.x() + kq1.x() * deltaT / 2.0,
                                    q.y() + kq1.y() * deltaT / 2.0,
                                    q.z() + kq1.z() * deltaT / 2.0);
    
    intermediate.normalize();
    
    Eigen::Vector3d kv2 = intermediate.toRotationMatrix() * accelMeasurement +
                          gravity;
    
    Eigen::Vector3d kx2 = x.segment<3>(6) + 0.5 * deltaT * kv1;
    
    Eigen::Quaterniond kq2 = quat_deriv(angVelMeasurement, intermediate);
    intermediate.w() = q.w() + kq2.w() * deltaT / 2.0;
    intermediate.x() = q.x() + kq2.x() * deltaT / 2.0;
    intermediate.y() = q.y() + kq2.y() * deltaT / 2.0;
    intermediate.z() = q.z() + kq2.z() * deltaT / 2.0;
    
    intermediate.normalize();
    
    Eigen::Vector3d kv3 = intermediate.toRotationMatrix() * accelMeasurement +
                          gravity;
    
    Eigen::Vector3d kx3 = x.segment<3>(6) + 0.5 * deltaT * kv2;
    
    Eigen::Quaterniond kq3 = quat_deriv(angVelMeasurement, intermediate);
    intermediate.w() = q.w() + kq3.w() * deltaT;
    intermediate.x() = q.x() + kq3.x() * deltaT;
    intermediate.y() = q.y() + kq3.y() * deltaT;
    intermediate.z() = q.z() + kq3.z() * deltaT;
    
    intermediate.normalize();
    
    Eigen::Vector3d kv4 = intermediate.toRotationMatrix() * accelMeasurement + 
                          gravity;
    
    Eigen::Vector3d kx4 = x.segment<3>(6) + 0.5 * deltaT * kv3;
    
    Eigen::Quaterniond kq4 = quat_deriv(angVelMeasurement, intermediate);
    
    q.w() += deltaT * (kq1.w() + 2.0 * kq2.w() + 2.0 * kq3.w() + kq4.w()) / 6.0;
    q.x() += deltaT * (kq1.x() + 2.0 * kq2.x() + 2.0 * kq3.x() + kq4.x()) / 6.0;
    q.y() += deltaT * (kq1.y() + 2.0 * kq2.y() + 2.0 * kq3.y() + kq4.y()) / 6.0;
    q.z() += deltaT * (kq1.z() + 2.0 * kq2.z() + 2.0 * kq3.z() + kq4.z()) / 6.0;
    
    q.normalize();
    
    // Update orientation
    // Info on converting quaternion to ZYX angles:
    // http://www.mathworks.com/help/aeroblks/quaternionstorotationangles.html
    
    result(3) = atan2(2.0 * (q.y() * q.z() + q.w() * q.x()), q.w() * q.w() - 
                                                             q.x() * q.x() -
                                                             q.y() * q.y() +
                                                             q.z() * q.z());
    result(4) = asin(-2.0 * (q.x() * q.z() - q.w() * q.y()));
    result(5) = atan2(2.0 * (q.x() * q.y() + q.w() * q.z()), q.w() * q.w() +
                                                             q.x() * q.x() -
                                                             q.y() * q.y() -
                                                             q.z() * q.z());
    
    // Update position and velocity
    result.head<3>() = x.head<3>() + 
                       deltaT * (kx1 + 2.0 * kx2 + 2.0 * kx3 + kx4) / 6.0;
    
    result.segment<3>(6) = x.segment<3>(6) + 
                           deltaT * (kv1 + 2.0 * kv2 + 2.0 * kv3 + kv4) / 6.0;

    // Copy biases
    result.tail<6>() = x.tail<6>();
}

/**
 * State Estimation UKF
 *
 * Process model takes as input the IMU linear acceleration and angular velocity
 * - behaves according to discretized model in [17] of reference from Shen
 *
 * Measurement takes as input the estimated 6-DOF pose from Pose Estimation UKF
 * and performs linear update. Exactly the same as other measurement, except
 * account for entries do not care about
 */

void StateUKF::predictionStepState(double deltaT) {
    
    Vector15d prev = mState;
    Matrix15d prevCov = mCov;
                                   
    // Use non-augmented version, since only additive noise is used. Because
    // relative measurements are not being used, do not need to augment states
    
    // Right now, state is 6dof, velocity in body frame, bias of accelerometer,
    // bias of gyroscope

    
    // Assuming additive noise, so the state vector is unaugmented. Thus, the
    // length is just the length of the state vector
    int L = 15;
    
    double lambda = mAlpha * mAlpha * (L + mKappa) - L;
    double gamma = sqrt(L + lambda);
    
    // Calculate the weight values
    double w0_m = lambda / (L + lambda);
    double w0_c = w0_m + 1 - mAlpha * mAlpha + mBeta;
    double wi = 1.0 / (2 * (L + lambda));
    
    /**** PROCESS NOISE ****/
    Matrix15d Q = Matrix15d::Identity() * mProcessError;
    Q(9,9) = Q(10,10) = Q(11,11) = mAccelBiasError;
    Q(12,12) = Q(13,13) = Q(14,14) = mAngVelBiasError;
    /***********************/
   
    
    Eigen::LLT<Matrix15d> llt(prevCov);
    Matrix15d sqrtMatrix = llt.matrixL();
    
    Eigen::Matrix<double, 15, 31> sigmaPoints;
    
    sigmaPoints.col(0) = prev;
    
    for (int i = 0; i < 2 * L; i++) {
        if (i < L) {
            sigmaPoints.col(i + 1) = prev + gamma * sqrtMatrix.col(i % L);
        } else {
            sigmaPoints.col(i + 1) = prev - gamma * sqrtMatrix.col(i % L);
        }
    }
    
    
    // Compute the predictions using the sigma points
    Eigen::Matrix<double, 15, 31> predictions;
    
    for (int i = 0; i < (2 * L + 1); i++) {
        Vector15d result;
        stateUpdate(result, sigmaPoints.col(i), mControl, deltaT);
        
        predictions.col(i) = result;
    }
    
    // Find the mean of the predictions
    mState = w0_m * predictions.col(0);
    
    for (int i = 1; i < (2 * L + 1); i++) {
        mState += wi * predictions.col(i);
    }
    
    // Find the covariance of the predictions    
    mCov = Q + w0_c * (predictions.col(0) - mState) * 
                      (predictions.col(0) - mState).transpose();
    
    for (int i = 1; i < (2 * L + 1); i++) {
        mCov += wi * (predictions.col(i) - mState) *
                     (predictions.col(i) - mState).transpose();
    }
}

/**
 * Update the estimated state using the estimated pose as the measurement.
 * The estimated pose is a 6DOF representation of the position and orientation
 * of the vehicle.
 * Note: measurement should be in same coordinate frame as estimate
 */
void StateUKF::measurementUpdateState() {   
    
    Vector15d est = mState;
    Matrix15d covEst = mCov;
    
    // Set H to pull out the 6DOF pose from the 15-dimension state estimate
    // (effectively take the first 6 elements of the state estimate)
    Eigen::Matrix<double, 6, 15> H;
    H.fill(0.0);
    H(0, 0) = H(1, 1) = H(2, 2) = H(3, 3) = H(4, 4) = H(5, 5) = 1.0;
    
    // Measurement covariance
    Matrix6d R = Matrix6d::Identity() * mMeasurementError;
    
                       
    Eigen::Matrix<double, 15, 6> K = covEst * H.transpose() *
                                     (H * covEst * H.transpose() + R).inverse();
    
    mState = est + K * (mMeasurement - H * est);
    
    mCov = (Matrix15d::Identity() - K * H) * covEst;
}

} // End namespace


int main(int argc, char* argv[]) {
    // Initialize ros
    ros::init(argc, argv, "state_ukf_node");
    state_ukf::StateUKF ukf;
    
    // Run node
    ros::spin();
    
    return 0;
}
