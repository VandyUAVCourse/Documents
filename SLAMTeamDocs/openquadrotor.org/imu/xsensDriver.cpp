#include "xsensDriver.h"

#include <iostream>
#include <fstream>
#include <cmath>

// Initialization of statics:
const char* Xsens::className = "Xsens";

#define DEBUG_XSENS     0
#define DEBUG_TIMESTAMP 0

Xsens::Xsens(const char* device) :
   _device(device), _readError(false), 
   _orientationFormat(XSENS_NO_ORIENTATION_DATA), _getAccelerationData(false), _getGyroData(false), _getMagneticData(false), 
   _outputFormat(0), _outputMode(0)
{
}

Xsens::~Xsens() {
  mtcomm.close();
}

void Xsens::init(XSENS_ORIENTATION_FORMAT orientationFormat, bool getAccelerationData, bool getGyroData, bool getMagneticData) {

  _orientationFormat = orientationFormat;
  _getAccelerationData=getAccelerationData; _getGyroData=getGyroData; _getMagneticData=getMagneticData;

  mtcomm.close();  // First close device

  // Set OutputFormat
  switch(_orientationFormat) {
    case XSENS_NO_ORIENTATION_DATA:
      _outputFormat = 0;
      _outputMode = 0;
      break;
    case XSENS_ORIENTATION_QUATERNION_FORMAT:
      _outputFormat = OUTPUTSETTINGS_ORIENTMODE_QUATERNION;
      _outputMode= OUTPUTMODE_ORIENT;
      break;
    case XSENS_ORIENTATION_EULER_FORMAT:
      _outputFormat = OUTPUTSETTINGS_ORIENTMODE_EULER;
      _outputMode= OUTPUTMODE_ORIENT;
      break;
    case XSENS_ORIENTATION_MATRIX_FORMAT:
      _outputFormat = OUTPUTSETTINGS_ORIENTMODE_MATRIX;
      _outputMode = OUTPUTMODE_ORIENT;
      break;
    default:
      std::cerr << "Xsens: invalid orientation format selected." << std::endl << "Exiting" << std::endl;
      _outputFormat = 0;
      _outputMode = 0;
      break;
  }

  _outputFormat |= OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;

  _outputFormat |= OUTPUTSETTINGS_CALIBMODE_MASK;  // Deactivate acc, gyro and mag
  if (getAccelerationData || getGyroData || getMagneticData)
    _outputMode |= OUTPUTMODE_CALIB;  // Activate output possibility for acc, gyro and mag
  if (getAccelerationData)
    _outputFormat &= ~OUTPUTSETTINGS_CALIBMODE_ACC_MASK;  // Activate acc
  if (getGyroData)
    _outputFormat &= ~OUTPUTSETTINGS_CALIBMODE_GYR_MASK;  // Activate gyro
  if (getMagneticData)
    _outputFormat &= ~OUTPUTSETTINGS_CALIBMODE_MAG_MASK;  // Activate magnetic

  if (_outputMode == 0) {
    std::cout << className << "::" << __func__ << ": ERROR: Neither asked for orientation nor other data!\n";
    exit(0);
  }

  // Start of initialization
  unsigned long tmpOutputMode, tmpOutputSettings;
  unsigned short tmpDataLength;
  unsigned short int numDevices;

  bool success = true;
  // Open and initialize serial port
  success = success && mtcomm.openPort(_device.c_str())==MTRV_OK;
  if (!success)
    printf("Cannot open COM port %s\n", _device.c_str());
  else {
    // Put MTi/MTx in Config State
    success = success && mtcomm.writeMessage(MID_GOTOCONFIG)==MTRV_OK;
    if (!success)
      printf("mtcomm.writeMessage: ERROR. No XSENS device connected\n");
    else {
      // Get current settings and check if Xbus Master is connected
      success = success && mtcomm.getDeviceMode(&numDevices)==MTRV_OK;
      if (!success) {
        printf("mtcomm.getDeviceMode: ERROR. Error initializing XSENS Device\n");
        if (numDevices <= 1)
          printf("MTi / MTx has not been detected\nCould not get device mode\n");
        else
          printf("Not just MTi / MTx connected to Xbus\nCould not get all device modes\n");
      }
      else {
        // Set output mode and output settings
        success = success && mtcomm.setDeviceMode(_outputMode, _outputFormat, BID_MT)==MTRV_OK;
        if (!success)
          printf("Could not set (all) device mode(s)\n");
      }
    }
  }

  // Something went wrong. Trying again.
  if (!success) {
    //std::cout << "Something went wrong. Trying again "<<std::flush;
    int count=1;
    while ((count<=10) && !success) {
      count++;
      mtcomm.close();
      usleep(100000);
      success = mtcomm.openPort(_device.c_str())==MTRV_OK;
      success = success && mtcomm.writeMessage(MID_GOTOCONFIG)==MTRV_OK;
      success = success && mtcomm.getDeviceMode(&numDevices)==MTRV_OK;
      success = success && mtcomm.setDeviceMode(_outputMode, _outputFormat, BID_MT)==MTRV_OK;
      std::cout << "." << std::flush;
    }
    if (success) {
      std::cout << " success!\n";
    }
    else {
      std::cout << " failure!\n";
      exit(0);
    }
  }

  mtcomm.getMode(tmpOutputMode, tmpOutputSettings, tmpDataLength, BID_MASTER);
  if (DEBUG_XSENS) printf("The current output mode is : %ld settings are: %ld\n",tmpOutputMode, tmpOutputSettings);

  // Set the delay
  // SETTING  Description
  // Bit 16-0 Delay value
  // Valid value is 590 (20 usec) to 294912 (10 msec)
  unsigned long value=600;
  //mtcomm->setSetting(MID_SETTRANSMITDELAY , value, LEN_TRANSMITDELAY);
  mtcomm.reqSetting(MID_REQTRANSMITDELAY , value);
  if (DEBUG_XSENS) printf("TransDelay= %ld\n",value);

  useAMD();  // Set default value

  // Put MTi/MTx in Measurement State
  mtcomm.setTimeOut(0);  // No timeout
  mtcomm.writeMessage(MID_GOTOMEASUREMENT);
  ///FIXME startTime = carmen_get_time();
	gettimeofday(&startTime, NULL);
  _lastSampleCounter = -1;

  if (DEBUG_XSENS) printf("MTi / MTx has been configured\n");
}

// Use or don't use AMD (Adapt to Magnetic Disturbances filter).
void Xsens::useAMD(bool active) {
  if (active)
    mtcomm.setSetting(MID_SETAMD, AMDSETTING_ENABLED, LEN_AMD);
  else
    mtcomm.setSetting(MID_SETAMD, AMDSETTING_DISABLED, LEN_AMD);

  _doAMD = active;
}

void Xsens::fillMessage(qc_imu_imu_message& msg, bool updateData)
{
  if (updateData) getNewData();
	msg.timestamp_sec = lastDataTime.tv_sec;
  msg.timestamp_usec = lastDataTime.tv_usec;

  if (   (_outputMode & OUTPUTMODE_ORIENT)==0
      || (_outputFormat & OUTPUTSETTINGS_ORIENTMODE_MASK)!=OUTPUTSETTINGS_ORIENTMODE_QUATERNION)
  {
    std::cout << className << "::" << __func__ << ": Not initialized for quaternions!\n";
    exit(0);
  }
  mtcomm.getValue(VALUE_ORIENT_QUAT, fdata, data, BID_MT);
  if (DEBUG_XSENS)
    printf("Quaternion (QUAD): q0=%0.3f, q1=%0.3f, q2=%0.3f, q3=%0.3f\n",fdata[0], fdata[1], fdata[2], fdata[3]);
  msg.q0 = fdata[0];
  msg.q1 = fdata[1];
  msg.q2 = fdata[2];
  msg.q3 = fdata[3];

  if (_getAccelerationData) {
    if (   (_outputMode & OUTPUTMODE_CALIB)==0
        || (_outputFormat & OUTPUTSETTINGS_CALIBMODE_ACC_MASK) != 0)
    {
      std::cout << className << "::" << __func__ << ": Not initialized for acceleration data!\n";
      exit(0);
    }
    mtcomm.getValue(VALUE_CALIB_ACC, fdata, data, BID_MT);
    if (DEBUG_XSENS)
      printf("Acceleration (ACC): accX=%0.2f, accY=%0.2f, accZ=%0.2f\n", fdata[0], fdata[1], fdata[2]);

    msg.accX = fdata[0];
    msg.accY = fdata[1];
    msg.accZ = fdata[2];

  } else {
    msg.accX = msg.accY = msg.accZ = 0.0;
  }

  if (_getGyroData) {
    if (   (_outputMode & OUTPUTMODE_CALIB)==0
        || (_outputFormat & OUTPUTSETTINGS_CALIBMODE_GYR_MASK) != 0)
    {
      std::cout << className << "::" << __func__ << ": Not initialized for gyro data!\n";
      exit(0);
    }
    mtcomm.getValue(VALUE_CALIB_GYR, fdata+3, data, BID_MT);
    if (DEBUG_XSENS)
      printf("Gyroscope (GYR): gyroX=%0.2f, gyroY=%0.2f, gyroZ=%0.2f\n", fdata[3], fdata[4], fdata[5]);
    msg.gyroX = fdata[3];
    msg.gyroY = fdata[4];
    msg.gyroZ = fdata[5];
  } else {
    msg.gyroX = msg.gyroY = msg.gyroZ = 0.0;
  }

  if (_getMagneticData) {
    mtcomm.getValue(VALUE_CALIB_MAG, fdata+6, data, BID_MT);
    if (DEBUG_XSENS)
      printf("Compass (MAG): compX=%0.2f, compY=%0.2f, compZ=%0.2f\n", fdata[6], fdata[7], fdata[8]);
    msg.magX = fdata[6];
    msg.magY = fdata[7];
    msg.magZ = fdata[8];
  } else {
    msg.magX = msg.magY = msg.magZ = 0.0;
  }
}

void Xsens::getNewData() {
  // We want the most recent data. No idea, how to do that properly...
  //mtcomm.flush();
  //------------------------------------------------------------------

  int readReturnValue = mtcomm.readDataMessage(data, datalen);

  if (readReturnValue != MTRV_OK) {
    //std::cout << "Problem while reading from device\n";
    std::cout << "x" << std::flush;
    _readError = true;
    return;
  }

  // Get sample count for timestamp calculation
  mtcomm.getValue(VALUE_SAMPLECNT, samplecounter, data, BID_MASTER);
  if (_lastSampleCounter<0) samplecounter = 0;  // bug workaround
  if (_lastSampleCounter > samplecounter) {  // Overflow of the unsigned short?
		struct timeval overflow;
		overflow.tv_sec = 655;
		overflow.tv_usec = 360000;
		tvu_add(&startTime, &startTime, &overflow);
		//startTime = startTime + 655.36;///
  }
  _lastSampleCounter = samplecounter;
  //lastDataTime = startTime + 0.01 * samplecounter;
	tvu_add_ms(&lastDataTime,&startTime, 10 * samplecounter);
  _readError = false;

	struct timeval now;
	struct timeval diff;
	gettimeofday(&now, NULL);
	tvu_subtract(&diff,&now,&lastDataTime);
	if (tvu_timeval_to_double(&diff) > 0.1) {  // Data too old (buffered data is read -> flush and read again
    //std::cout << "Flushing!\n";
    mtcomm.flush();  // Flush old data
    getNewData();
  }

  // debug timstamp
  if (DEBUG_TIMESTAMP) {
    static unsigned int numRead = 0;
		if (numRead % 50 == 0){
			struct timeval debug_tv;
			fprintf(stderr,"now= %d (sec)  %d (usec)\n",(int)now.tv_sec, (int)now.tv_usec);
			fprintf(stderr,"xsens= %d (sec)  %d (usec)\n",(int)lastDataTime.tv_sec, (int)lastDataTime.tv_usec);
			fprintf(stderr,"diff= %d (sec) %d (usec)\n",(int)diff.tv_sec, (int)diff.tv_usec);
			fprintf(stderr,"samplecounter= %d\n",samplecounter);
			fprintf(stderr,"start= %d (sec)  %d (usec)\n",(int)startTime.tv_sec, (int)startTime.tv_usec);
			tvu_subtract(&debug_tv, &now, &startTime);
			fprintf(stderr,"startToNow= %d (sec)  %d (usec)\n",(int)debug_tv.tv_sec, (int)debug_tv.tv_usec);
			fprintf(stderr,"\n\n");
		}
    ++numRead;
  }
}
