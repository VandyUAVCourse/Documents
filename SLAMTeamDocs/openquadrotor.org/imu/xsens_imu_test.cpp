#include <ipcInterfaces/qc_ipc_interface.h>
#include <ipcInterfaces/qc_imu_interface.h>
#include <ipcMessages/qc_imu_messages.h>
#include <ipc/ipc.h>
#include <cmath>
#include <utils_global/timeval_ops.h>
#include <cstdlib>

#define CALCANGLES
#ifdef CALCANGLES
#include <math_stuff/transformation3.hh>
#endif

qc_imu_imu_message imumsg;
using namespace std;

void qc_imu_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	fprintf(stderr, "I");
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &imumsg, sizeof(imumsg));
	if (err != IPC_OK)
		fprintf(stderr, "#Error!\n");
	
	struct timeval t;
	t.tv_sec = imumsg.timestamp_sec;
	t.tv_usec = imumsg.timestamp_usec;
	//struct timeval now;
	//gettimeofday(&now, NULL);
	//tvu_subtract(&t,&now,&t);
	
	fprintf(stderr,"### ImuMsg (%f)\n",tvu_timeval_to_double(&t));
	fprintf(stderr,"# accX= %f\n",imumsg.accX);
	fprintf(stderr,"# accY= %f\n",imumsg.accY);
	fprintf(stderr,"# accZ= %f\n",imumsg.accZ);
	fprintf(stderr,"# q0= %f\n",imumsg.q0);
	fprintf(stderr,"# q1= %f\n",imumsg.q1);
	fprintf(stderr,"# q2= %f\n",imumsg.q2);
	fprintf(stderr,"# q3= %f\n",imumsg.q3);
	fprintf(stderr,"# magX= %f\n",imumsg.magX);
	fprintf(stderr,"# magY= %f\n",imumsg.magY);
	fprintf(stderr,"# magZ= %f\n",imumsg.magZ);
	fprintf(stderr,"# gyroX= %f\n",imumsg.gyroX);
	fprintf(stderr,"# gyroY= %f\n",imumsg.gyroY);
	fprintf(stderr,"# gyroZ= %f\n",imumsg.gyroZ);
#ifdef CALCANGLES
	Quaternion<double> q(imumsg.q0,imumsg.q1,imumsg.q2,imumsg.q3);
	DVector3 o = q.toAngles();
 	fprintf(stderr,"# roll= %f (deg)\n",o.x()* (180./M_PI));
 	fprintf(stderr,"# pitch= %f (deg)\n",o.y() * (180./M_PI));
 	fprintf(stderr,"# yaw= %f (deg)\n\n",o.z() * (180./M_PI));
	///corrected acceleration
	fprintf(stderr,"# ACCX= %f\n", -1*(imumsg.accX + cos(o.roll())*sin(o.pitch())*9.8));
	fprintf(stderr,"# ACCY= %f\n", -1*(imumsg.accY - sin(o.roll())*9.8));
#endif
	IPC_freeByteArray(callData);
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&imumsg);
	
}


int main (int argc, char* argv[]){

	if (qc_ipc_connect(argv[0]) <= 0){
		exit(1);
	}
	qc_imu_subscribe_imu_message(qc_imu_message_handler,10,NULL);
	
	IPC_dispatch();
	
	return 0;
}
