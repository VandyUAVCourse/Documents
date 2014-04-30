#ifndef __STATE_UKF_NODE_H__
#define __STATE_UKF_NODE_H__

#include "navmap/kalman_filters.h"
#include "imu_broadcast/raw_imu.h"


namespace navmap {

/**
 * Class to implement the state estimation UKF.
 */
class StateUKF {

public:
    
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;

    typedef Eigen::Matrix<double, 15, 1> Vector15d;
    typedef Eigen::Matrix<double, 15, 15> Matrix15d;
    
    
    /**
     * Constructor.
     */
    StateUKF();
    
    /**
     * Destructor.
     */
    ~StateUKF() {}

private:
    
    /**
     * Timer callback. Used to output updated state estimate.
     */
    void main_loop(const ros::TimerEvent& e);
    
    /**
     * Receiver for data from the IMU.
     */
    void imu_data_receiver(const imu_broadcast::raw_imu& msg);

    /**
     * Receiver for data from the pose estimate UKF.
     */
    void pose_est_receiver(const navmap::pose_ukf_msgConstPtr& msg);                                 
    
    
    /**
     * Update the state of the system by numerically integrating the dynamic
     * equations from "On the consistency of vision-aided inertial navigation"
     * (equations 7,8).
     */
    void stateUpdate(Vector15d& result, const Vector15d& x, const Vector6d& u,
                     double deltaT) const;
                     
    
    
    /**
     * State Estimation UKF
     *
     * Process model takes as input the IMU linear acceleration and angular 
     * velocity - behaves according to discretized model in [17] of reference 
     * from Shen
     */
    void predictionStepState(double deltaT);
    
    /**
     * Update the estimated state using the estimated pose as the measurement.
     * The estimated pose is a 6DOF representation of the position and orientation
     * of the vehicle.
     * Note: measurement should be in same coordinate frame as estimate
     */
    void measurementUpdateState();
    
    /***************************************/
    /************** VARIABLES **************/
    /***************************************/
    
    // Handle to node
    ros::NodeHandle mN;
    
    // Publisher
    ros::Publisher mPub;
    
    // Subscribers to IMU data and pose estimate UKF data
    ros::Subscriber mImuSub;
    ros::Subscriber mPoseSub;
    
    // Main loop timer
    ros::Timer mLoop;
    
    /** Parameters **/
    double mAlpha;
    double mBeta;
    double mKappa;
    
    double mProcessError;
    double mAccelBiasError;
    double mAngVelBiasError;
    double mMeasurementError;
    
    // Number of steps to delay a measurement. For example, if set to 2,
    // a received measurement will be applied to the state 2 time steps ago
    unsigned int mMeasurementDelay;
    
    /** Kalman filter state/covariance **/
    Vector15d mState;
    Matrix15d mCov;
    
    /** Kalman filter control/measurement **/
    Vector6d mControl;
    Vector6d mMeasurement;
    bool mAvailableControl;
    bool mAvailableMeasurement;
    
    ros::Time mLastUpdate;
};

} // End namespace

#endif
