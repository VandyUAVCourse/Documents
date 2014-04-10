#ifndef __POSE_UKF_NODE_H__
#define __POSE_UKF_NODE_H__

#include "kalman_filters/kalman_filters.h"
#include <deque>
#include <boost/shared_ptr.hpp>

namespace kalman_filters {

/**
 * Class to implement the pose estimation UKF.
 */
class PoseUKF {

public:
    
    /**
     * Constructor.
     */
    PoseUKF();
    
    /**
     * Destructor.
     */
    ~PoseUKF() {}

private:
    
    /**
     * Timer callback. Used to output updated state estimate.
     */
    void main_loop(const ros::TimerEvent& e);
    
    /**
     * Receiver for data from the pose estimator.
     */
    void pose_est_receiver(const state_ukf::pose_ukf_msgConstPtr& msg);

    /**
     * Receiver for data from SLAM.
     */
    void slam_receiver(const state_ukf::pose_ukf_msgConstPtr& msg);
                     
    
    /**
     * Pose Estimation UKF
     *
     * Process model takes as input the incremental 6-DOF motion of the pose
     * estimator.
     */
    void predictionStepPose(const Vector6d& control);

    /**
     * Update the estimated pose using a pose measurement from SLAM. The pose
     * estimate is just the estimated 6DOF pose from SLAM.
     * Note: measurement should be in same coordinate frame as estimate
     */
    void measurementUpdatePose();

    
    
    /***************************************/
    /************** VARIABLES **************/
    /***************************************/
    
    // Handle to node
    ros::NodeHandle mN;
    
    // Publisher
    ros::Publisher mPub;
    
    // Subscribers to pose estimate data and SLAM data
    ros::Subscriber mPoseSub;
    ros::Subscriber mSlamSub;
    
    // Main loop timer
    ros::Timer mLoop;
    
    /** Parameters **/
    double mAlpha;
    double mBeta;
    double mKappa;
    
    double mProcessError;
    double mMeasurementError;
    
    // Number of steps to delay a measurement. For example, if set to 2,
    // a received measurement will be applied to the state 2 time steps ago
    int mMeasurementDelay;
    
    /** Kalman filter state/covariance **/
    std::deque< boost::shared_ptr<Vector6d> > mStateBuffer;
    std::deque< boost::shared_ptr<Matrix6d> > mCovBuffer;
    
    /** Kalman filter control/measurement **/
    std::deque< boost::shared_ptr<Vector6d> > mControlBuffer;
    Vector6d mMeasurement;
    
    bool mAvailableControl;
    bool mAvailableMeasurement;
};

} // End namespace

#endif
