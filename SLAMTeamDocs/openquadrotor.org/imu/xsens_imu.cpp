#include <iostream>
#include <string>
using std::cout;


#include "xsensDriver.h"
#include <utils_global/cmdargsreader.h>
#include <math_stuff/transformation3.hh>
#include <ipcInterfaces/qc_ipc_interface.h>
int main(int argc, char *argv[])
{
	std::string device;
  int getAccelerationData = 1;
  int getGyroData = 1;
  int getMagneticData = 1;
  int doAMD = 1;
	
	if (argc < 2){
		std::cerr << "Usage: " << argv[0] << std::endl;
		std::cerr << "                         -d | -device <pathToDevice>  {path to IMU}" << std::endl;
		std::cerr << "                         -u | -upsidedown {if the imu is mounted upside down} " << std::endl;
		exit(1);
	}
	
	///connect to ipc
	if (qc_ipc_connect(argv[0]) <= 0){
		exit(1);
	}

	CmdArgsReader commandline(argc,argv);
	bool upsidedown = false;
	commandline.getStringValue("-d","-device",&device);
	commandline.getFlagValue("-u","-upsidedown",&upsidedown);
	
	if (commandline.printUnusedArguments()){
		exit(1);
	}
  printf("adapt_to_magnetic_disturbance_filter %d\n", doAMD);

  //double lastIsAliveTimestamp = 0.;
  qc_imu_imu_message msg;
  int hasErrors = 0;

  Xsens xsens(device.c_str());
  xsens.init(XSENS_ORIENTATION_QUATERNION_FORMAT, getAccelerationData, getGyroData, getMagneticData);
  xsens.useAMD(doAMD);

	int k =0;
  while(true) {
    xsens.fillMessage(msg);
    hasErrors = xsens.hadReadError();

		if (!hasErrors){
			if (upsidedown){
				Quaternion<double> qm(msg.q0,msg.q1,msg.q2,msg.q3);
				Quaternion<double> ud(M_PI,0,0);
				Quaternion<double> corr = qm * ud.inverse();
				msg.q0 = corr.re();
				msg.q1 = corr.im().x();
				msg.q2 = corr.im().y();
				msg.q3 = corr.im().z();
			}
      	qc_imu_publish_imu_message(&msg);
			k++;
			if (!(k%100))
				fprintf(stderr,".");
		}
		else
			fprintf(stderr,"E");
		IPC_listen(7);
		}
}

