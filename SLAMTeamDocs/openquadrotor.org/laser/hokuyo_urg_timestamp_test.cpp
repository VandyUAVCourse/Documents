#include <ipcMessages/qc_laser_messages.h>
#include <ipcInterfaces/qc_ipc_interface.h>
#include <ipcInterfaces/qc_laser_interface.h>
#include <ipc/ipc.h>
#include <cmath>
#include <sys/time.h>
#include <cstdlib>
int counter = 0;
qc_laser_laser_message lasermsg;
double global_timestamp;
using namespace std;
void qc_laser_laser1_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	if (lasermsg.ranges){
		delete[] lasermsg.ranges;
		lasermsg.ranges = 0;
	}
	fprintf(stderr, "L");
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &lasermsg, sizeof(lasermsg));
	double ts1 = lasermsg.timestamp_sec + 1e-6 * lasermsg.timestamp_usec - global_timestamp;
	double ts2 = lasermsg.timestamp_hokuyo_init_sec + 1e-6 * lasermsg.timestamp_hokuyo_init_usec - global_timestamp;
	double ts3 = lasermsg.internal_timestamp_hokuyo * 1e-3;
	std::cout << counter << " " << ts1 << " " << ts2 << " " << ts3 << std::endl;
	counter++;
	
}


int main (int argc, char* argv[]){
	lasermsg.ranges = 0;
	if (qc_ipc_connect(argv[0]) <= 0){
		exit(1);
	}
	struct timeval startTime;
	gettimeofday(&startTime,NULL);
	global_timestamp = startTime.tv_sec + 1e-6 * startTime.tv_usec;
	qc_laser_subscribe_laser1_message(qc_laser_laser1_message_handler,10);
	
	IPC_dispatch();
	
	return 0;
}
