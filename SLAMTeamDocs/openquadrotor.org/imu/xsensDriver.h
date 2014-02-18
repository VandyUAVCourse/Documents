#ifndef XSENS_H
#define XSENS_H


#include <string>
#include <utils_global/timeval_ops.h>
#include <ipcMessages/qc_imu_messages.h>
#include <ipcInterfaces/qc_imu_interface.h>
#include "MTComm.h"

enum XSENS_ORIENTATION_FORMAT {
   XSENS_NO_ORIENTATION_DATA,
   XSENS_ORIENTATION_QUATERNION_FORMAT,
   XSENS_ORIENTATION_EULER_FORMAT,
   XSENS_ORIENTATION_MATRIX_FORMAT,
};

class Xsens {
   //-----STATIC-----
   protected: static const char* className;

   public:
      //-----CONSTRUCTOR&DESTRUCTOR-----
      Xsens(const char* device="/dev/ttyUSB0"); // testMode=true: No real xsens-device neccessary. Sends just zeros.
      ~Xsens();
      //-----METHODS-----
      void init(XSENS_ORIENTATION_FORMAT orientationFormat=XSENS_ORIENTATION_QUATERNION_FORMAT, bool getAccelerationData=false, bool getGyroData=false, bool getMagneticData=false);
      void useAMD(bool active=true); // Use or don't use AMD (Adapt to Magnetic Disturbances filter).

      void fillMessage(qc_imu_imu_message& msg, bool updateData = true);

      bool hadReadError() {return _readError;}
   
   protected:
      //-----METHODS-----
      void getNewData();

      //-----VARIABLES-----
      std::string _device;
      bool _readError;
      CMTComm mtcomm;
      bool _doAMD;
      XSENS_ORIENTATION_FORMAT _orientationFormat;
      bool _getAccelerationData, _getGyroData, _getMagneticData;
      int _outputFormat, _outputMode;
      float fdata[18];
      short datalen;
      unsigned char data[MAXMSGLEN];

			struct timeval lastDataTime;
			struct timeval startTime;
      //double lastDataTime;
      //double startTime;
      
      unsigned short samplecounter;
      int _lastSampleCounter;
      
};

#endif
