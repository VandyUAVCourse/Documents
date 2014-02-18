#include <sys/time.h>
#include <ipcMessages/qc_flightcontrol_messages.h>
#include <ipcInterfaces/qc_flightcontrol_interface.h>
#include <ipcInterfaces/qc_ipc_interface.h>
#include <cstdlib>


qc_flightcontrol_flightcontrol_message fcmsg;
using namespace std;
void qc_flightcontrol_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	fprintf(stderr, "F");
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &fcmsg, sizeof(fcmsg));
	if (err != IPC_OK)
		fprintf(stderr, "#Error!\n");
	
	struct timeval t;
	t.tv_sec = fcmsg.timestamp_sec;
	t.tv_usec = fcmsg.timestamp_usec;
	struct timeval now;
	gettimeofday(&now, NULL);
	//tvu_subtract(&t,&now,&t);
   fprintf(stderr,"### FC-Msg (%ld) Format= %.2f\n", now.tv_sec, fcmsg.message_version/100.);
	fprintf(stderr,"#BatteryVoltage= %d\n",fcmsg.battery_voltage);
	fprintf(stderr,"#RC_Connection= %d\n",fcmsg.rc_connection);
	fprintf(stderr,"#Combined Thrust= %d\n",fcmsg.mixture_value_acceleration);
	fprintf(stderr,"#Swicth Value= %d\n",fcmsg.autonomous_enabled);
	fprintf(stderr,"#gcm_failures= %d\n",fcmsg.gcm_failures);
	fprintf(stderr,"#gcm_delta_roll= %d\n",fcmsg.gcm_delta_roll);
	fprintf(stderr,"#gcm_delta_pitch= %d\n",fcmsg.gcm_delta_pitch);
	fprintf(stderr,"#gcm_delta_yaw= %d\n",fcmsg.gcm_delta_yaw);
	fprintf(stderr,"#gcm_delta_thrust= %d\n",fcmsg.gcm_delta_thrust);
	fprintf(stderr,"#stick_roll= %d\n",fcmsg.stick_roll);
	fprintf(stderr,"#stick_pitch= %d\n",fcmsg.stick_pitch);
	fprintf(stderr,"#stick_yaw= %d\n",fcmsg.stick_yaw);
	fprintf(stderr,"#stick_thrust= %d\n",fcmsg.stick_thrust);
	fprintf(stderr,"#calc_stick_roll= %d\n",fcmsg.calc_stick_roll);
	fprintf(stderr,"#calc_stick_pitch= %d\n\n",fcmsg.calc_stick_pitch);
	
	IPC_freeByteArray(callData);
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&fcmsg);
}


int main (int argc, char* argv[]){
	if (qc_ipc_connect(argv[0]) <= 0){
		exit(1);
	}
	qc_flightcontrol_subscribe_flightcontrol_message(qc_flightcontrol_message_handler,10,NULL);
	
	IPC_dispatch();
	
	return 0;
}
