#include "kalman_filters/pose_ukf_node.h"
#include <eigen3/Eigen/Cholesky>

namespace kalman_filters {

/**
 * Constructor.
 */
PoseUKF::PoseUKF() {
    
    // Retrieve parameters
    if (!mN.getParam("pose_ukf_alpha", mAlpha)) {
        mAlpha = 1.0;
        mN.setParam("pose_ukf_alpha", mAlpha);
    }
    
    if (!mN.getParam("pose_ukf_beta", mBeta)) {
        mBeta = 2.0;
        mN.setParam("pose_ukf_beta", mBeta);
    }
    
    if (!mN.getParam("pose_ukf_kappa", mKappa)) {
        mKappa = 0.0;
        mN.setParam("pose_ukf_kappa", mKappa);
    }
    
    if (!mN.getParam("pose_ukf_process_error", mProcessError)) {
        mProcessError = 0.1;
        mN.setParam("pose_ukf_process_error", mProcessError);
    }
    
    if (!mN.getParam("pose_ukf_measurement_error", mMeasurementError)) {
        mMeasurementError = 0.1;
        mN.setParam("pose_ukf_measurement_error", mMeasurementError);
    }
    
    if (!mN.getParam("pose_ukf_measurement_delay_count", mMeasurementDelay)) {
        mMeasurementDelay = 2;
        mN.setParam("pose_ukf_measurement_delay_count", mMeasurementDelay);
    }
    
    double frequency = 20.0;
    
    if (!mN.getParam("pose_ukf_process_frequency", frequency)) {
        mN.setParam("pose_ukf_process_frequency", frequency);
    }
    
    // Initialize the state and covariance   
    boost::shared_ptr<Vector6d> initState(new Vector6d);
    boost::shared_ptr<Matrix6d> initCov(new Matrix6d);
    
    initState->fill(0.0);
    *initCov = Matrix6d::Identity() * mProcessError;
    
    mStateBuffer.push_back(initState);
    mCovBuffer.push_back(initCov);
    
    
    mAvailableControl = mAvailableMeasurement = false;
    
    // Create the state publisher
    mPub = mN.advertise<kalman_filters::pose_ukf_msg>("pose_ukf", 1000);
    
    // Create the subscribers to the IMU data nd the pose estimate UKF data
    mPoseSub = mN.subscribe("pose_est_data", 1,
                            &PoseUKF::pose_est_receiver, this);
    mSlamSub = mN.subscribe("slam_data", 1,
                            &PoseUKF::slam_receiver, this);
    
    // Create timer used for main loop
    mLoop = mN.createTimer(ros::Duration(1.0 / frequency),
                           &PoseUKF::main_loop, this);
}


/**
 * Timer callback. Used to output updated state estimate.
 */
void PoseUKF::main_loop(const ros::TimerEvent& e) {
    
    if (mAvailableControl) {
        // Run process step
        predictionStepPose(*mControlBuffer.back());
        
        // Get rid of out of date control vectors
        while (mControlBuffer.size() > mMeasurementDelay) {
            mControlBuffer.pop_front();
        }
        
        mAvailableControl = false;
    }
    
    if (mAvailableMeasurement) {
        // Because measurements are assumed to be delayed, only perform
        // measurement update if history goes back far enough
        if (mStateBuffer.size() > mMeasurementDelay) {
            // Remove all but the oldest stored state and propagate the
            // state to the current time after making the measurement update
            while (mStateBuffer.size() > 1) {
                mStateBuffer.pop_back();
            }
            
            // Run measurement update
            measurementUpdatePose();
            
            // Propagate state to the current time
            for (int i = 0; i < mMeasurementDelay; i++) {
                predictionStepPose(*mControlBuffer[i]);
            }
        }

        mAvailableMeasurement = false;
    }
    
    kalman_filters::pose_ukf_msg msg;
    
    const Vector6d& state = *mStateBuffer.back();
    
    msg.x = state(0);
    msg.y = state(1);
    msg.z = state(2);
    msg.roll = state(3);
    msg.pitch = state(4);
    msg.yaw = state(5);

    mPub.publish(msg);
}


/**
 * Receiver for data from the IMU.
 */
void PoseUKF::pose_est_receiver(const kalman_filters::pose_ukf_msgConstPtr& msg) {
    
    boost::shared_ptr<Vector6d> control(new Vector6d);
    *control << msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw;
    
    mControlBuffer.push_back(control);
    
    mAvailableControl = true;
}

/**
 * Receiver for data from the pose estimate UKF.
 */
void PoseUKF::slam_receiver(const kalman_filters::pose_ukf_msgConstPtr& msg) {
    mMeasurement << msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw;
    
    mAvailableMeasurement = true;
}


/**
 * Pose Estimation UKF
 *
 * Process model takes as input the incremental 6-DOF motion of the pose
 * estimator.
 */
void PoseUKF::predictionStepPose(const Vector6d& control) {

    const Vector6d& prev = *mStateBuffer.back();
    const Matrix6d& prevCov = *mCovBuffer.back();
    
    boost::shared_ptr<Vector6d> state(new Vector6d);
    boost::shared_ptr<Matrix6d> cov(new Matrix6d);
          
    // Assuming additive noise, so the state vector is unaugmented. Thus, the
    // length is just the length of the state vector
    int L = 6;
    
    double lambda = mAlpha * mAlpha * (L + mKappa) - L;
    double gamma = sqrt(L + lambda);
    
    // Calculate the weight values
    double w0_m = lambda / (L + lambda);
    double w0_c = w0_m + 1 - mAlpha * mAlpha + mBeta;
    double wi = 1.0 / (2 * (L + lambda));
    
    /**** PROCESS NOISE ****/
    Matrix6d Q = Matrix6d::Identity() * mProcessError;
    /***********************/
    
    
    Eigen::LLT<Matrix6d> llt(prevCov);
    Matrix6d sqrtMatrix = llt.matrixL();
    
    Eigen::Matrix<double, 6, 13> sigmaPoints;
    
    sigmaPoints.col(0) = prev;
    
    for (int i = 0; i < 2 * L; i++) {
        if (i < L) {
            sigmaPoints.col(i + 1) = prev + gamma * sqrtMatrix.col(i % L);
        } else {
            sigmaPoints.col(i + 1) = prev - gamma * sqrtMatrix.col(i % L);
        }
    }
    
    
    // Compute the predictions using the sigma points
    Eigen::Matrix<double, 6, 13> predictions;
    
    for (int i = 0; i < (2 * L + 1); i++) {
        Vector6d result;
        poseUpdate(result, sigmaPoints.col(i), control);

        predictions.col(i) = result;
    }
    
    // Find the mean of the predictions
    *state = w0_m * predictions.col(0);
    
    for (int i = 1; i < (2 * L + 1); i++) {
        *state += wi * predictions.col(i);
    }
    
    // Find the covariance of the predictions
    *cov = Q + w0_c * (predictions.col(0) - *state) * 
                    (predictions.col(0) - *state).transpose();
    
    for (int i = 1; i < (2 * L + 1); i++) {
        *cov += wi * (predictions.col(i) - *state) *
                    (predictions.col(i) - *state).transpose();
    }
    
    mStateBuffer.push_back(state);
    mCovBuffer.push_back(cov);
    
    // If exceeded buffer size, remove the oldest state
    if (mStateBuffer.size() > (mMeasurementDelay + 1)) {
        mStateBuffer.pop_front();
    }
    
    if (mCovBuffer.size() > (mMeasurementDelay + 1)) {
        mCovBuffer.pop_front();
    }
}

/**
 * Update the estimated pose using a pose measurement from SLAM. The pose
 * estimate is just the estimated 6DOF pose from SLAM.
 * Note: measurement should be in same coordinate frame as estimate
 */
void PoseUKF::measurementUpdatePose() {
    
    Vector6d& est = *mStateBuffer.front();
    Matrix6d& covEst = *mCovBuffer.front();
    
    Matrix6d H = Matrix6d::Identity();
    
    // Measurement covariance
    Matrix6d R = Matrix6d::Identity() * mMeasurementError;
    
                       
    Matrix6d K = covEst * H.transpose() 
                        * (H * covEst * H.transpose() + R).inverse();
    
    est = est + K * (mMeasurement - H * est);

    covEst = (Matrix6d::Identity() - K * H) * covEst;
}


} // End namespace


int main(int argc, char* argv[]) {
    // Initialize ros
    ros::init(argc, argv, "pose_ukf_node");
    kalman_filters::PoseUKF ukf;
    
    // Run node
    ros::spin();
    
    return 0;
}
