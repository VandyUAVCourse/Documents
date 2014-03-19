/**
 * Implements the Kalman filter used for altitude estimation.
 *
 * Note: due to the previty of this code, it is not intended to stand as its own
 *       file. This code should be inserted in the file where it is called.
 */

#include <eigen3/Eigen/Dense>

/**
 * Implements the Kalman filter step on line 4 of the Multilevel SLAM algorithm
 * in the Algorithms document
 *
 * @param altEst - the estimated altitude will be returned in this variable
 * @param velEst - the estimated z velocity will be returned in this variable
 * @param covEst - the estimated covariance matrix will be returned in this
                   variable
 * @param prevAlt - the altitude estimate at the previous time step
 * @param prevVel - the z velocity estimate at the previous time step
 * @param prevCov - the covariance matrix at the previous time step
 * @param deltaT - the amount of time that has passed since the previous step
 * @param imuAz - the z acceleration from the IMU
 * @param imuSigmaZ - the uncertainty of the z acceleration from the IMU
 */
void predictionStep(double& altEst, double& velEst, Eigen::Matrix2d& covEst,
                    double prevAlt, double prevVel,
                    const Eigen::Matrix2d& prevCov,
                    double deltaT, double imuAz, double imuSigmaZ) {
                    
    Eigen::Matrix2d A;
    A << 1.0, deltaT, 0.0, 1.0;
    
    Eigen::Matrix2d Q;
    Q << imuSigmaZ, 0.0, 0.0, imuSigmaZ; // Process covariance matrix - could be
                                         // tuned if necessary
    
    
    altEst = prevAlt + deltaT * prevVel + 0.5 * deltaT * deltaT * imuAz;
    velEst = prevVel + deltaT * imuAz;
    
    covEst = A * prevCov * A.transpose() + Q;
}

/**
 * Implements the Kalman filter step on line 8 of the Multilevel SLAM algorithm
 * in the Algorithms document
 * @param alt - the corrected altitude will be returned in this variable
 * @param vel - the velocity will be returned in this variable
 * @param cov - the corrected covariance matrix will be returned in this
 *              variable
 * @param altEst - the altitude estimate from the predictionStep method
 * @param velEst - the velocity estimate from the predictionStep method
 * @param covEst - the covariance matrix estimate from the predictionStep method
 * @param measurement - the value of the altitude measurement from the LRF
 * @param measureSigma - the uncertainty of the measurement from the LRF
 */
void measurementUpdate(double& alt, double& vel, Eigen::Matrix2d& cov,
                       double altEst, double velEst,
                       const Eigen::Matrix2d& covEst,
                       double measurement, double measureSigma) {
                       
    Eigen::RowVector2d H(1.0, 0.0);
    
    Eigen::Vector2d s_hat(altEst, velEst); 
                       
    Eigen::Vector2d K = covEst * H.transpose() 
                        / (H * covEst * H.transpose() + measureSigma);
    
    Eigen::Vector2d s = s_hat + K * (measurement - H * s_hat);
    
    alt = s[0];
    vel = s[1];
    
    cov = (Eigen::MatrixXd::Identity(2,2) - K * H) * covEst;
}
