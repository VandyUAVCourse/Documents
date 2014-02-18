#ifndef _IMUPARAMETERS_HH_
#define _IMUPARAMETERS_HH_


struct IMUParameters{
  double rollGyroCov, pitchGyroCov, yawGyroCov;
  double magXCov, magYCov, magZCov;
  double accXCov, accYCov, accZCov;
  double rollCov, pitchCov, yawCov;
  IMUParameters();
};

#endif
