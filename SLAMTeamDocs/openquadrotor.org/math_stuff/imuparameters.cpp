#include "imuparameters.hh"



IMUParameters::IMUParameters(){
  rollGyroCov=10, pitchGyroCov=10, yawGyroCov=10;
  magXCov=10, magYCov=10, magZCov=10;
  accXCov=10, accYCov=10, accZCov=10;
  rollCov=1, pitchCov=1, yawCov=10;
}

